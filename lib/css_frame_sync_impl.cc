/* -*- c++ -*- */
/*
 * Copyright 2025 zerosignal.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "css_frame_sync_impl.h"
#include <gnuradio/io_signature.h>
#include <gnuradio/fft/fft.h>
#include <gnuradio/fft/window.h>
#include <volk/volk.h>
#include <gnuradio/types.h> // For gr_complex
#include <cmath>
#include <numeric> // For std::accumulate
#include <algorithm> // For std::max_element
#include <cstdio> // For printf (debugging)

namespace gr {
namespace cssmods {

using input_type = gr_complex;
using output_type = gr_complex;
css_frame_sync::sptr css_frame_sync::make(int sf, double bw, int zero_padding_ratio)
{
    return gnuradio::make_block_sptr<css_frame_sync_impl>(sf, bw, zero_padding_ratio);
}


/*
 * The private constructor
 */
css_frame_sync_impl::css_frame_sync_impl(int sf, double bw, int zero_padding_ratio)
    : gr::sync_block("css_frame_sync",
                     gr::io_signature::make(
                         1 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
                     gr::io_signature::make(
                         1 /* min outputs */, 1 /*max outputs */, sizeof(output_type))),
        d_sf(sf),
        d_bw(bw),
        d_zero_padding_ratio(zero_padding_ratio),
        d_sample_num(2 * (1 << sf)), // Corresponds to matlab self.sample_num
        d_bin_num((1 << sf) * zero_padding_ratio), // Corresponds to matlab self.bin_num
        d_fft_len(d_sample_num * zero_padding_ratio), // Corresponds to matlab self.fft_len
        d_state(STATE_IDLE),
        d_current_search_pos(0),
        d_preamble_bin(0),
        d_cfo(0.0)
{
    // Initialize FFT object
    d_fft = std::make_shared<gr::fft::fft_complex_fwd>(d_fft_len); // false for forward FFT
    // Generate nominal chirps (cfo=0) based on the determined parameters
    // fs = 2*bw, tdelta = (1 << sf) / bw, h_in=0, cfo=0, tscale=0
    double effective_fs_for_chirp_gen = 2.0 * d_bw;
    double symbol_duration = (double)(1 << d_sf) / d_bw; // Nominal duration based on BW and SF
    // Check if round(effective_fs_for_chirp_gen * symbol_duration) matches d_sample_num
    // This should be round(2*bw * (2^sf)/bw) = round(2*2^sf) = 2*2^sf = d_sample_num
    // This mapping seems consistent with the matlab sample_num definition.
    d_upchirp = generate_lora_chirp(
        true, d_sf, d_bw, effective_fs_for_chirp_gen, 0, 0, symbol_duration, 0);
    d_downchirp = generate_lora_chirp(
        false, d_sf, d_bw, effective_fs_for_chirp_gen, 0, 0, symbol_duration, 0);
    if (d_upchirp.size() != (size_t)d_sample_num || d_downchirp.size() != (size_t)d_sample_num) {
        fprintf(stderr, "Warning: Generated chirp length (%zu) does not match d_sample_num (%d).\n", d_upchirp.size(), d_sample_num);
            // Handle error or adjust chirp generation parameters if possible
            // For now, proceed but this indicates a potential mismatch in parameters
            // Or the generate_lora_chirp needs to be adjusted.
            // If sample_num comes from elsewhere (e.g. oversampling factor),
            // then tdelta could be (1<<sf)/bw and fs = sample_num / tdelta.
            // fs = (2*(1<<sf)) / ((1<<sf)/bw) = 2*bw. This confirms the 2*bw effective rate.
    } else {
        // Good, chirp length matches
    }
    // Set history to require enough data for the largest dechirp/sync step
    // Sync involves looking back 4 symbols + current symbol = 5 symbols + refinement
    // Let's set a history large enough for the CFO calculation step, which looks back 4 symbols
    // plus the current symbol for dechirp. Total samples needed are around 5 * d_sample_num.
    // However, work processes incrementally. The state machine handles spanning across calls.
    // Let's set history = d_sample_num to ensure at least one symbol's worth of data
    // is always available relative to the current processing point when needed.
    // A larger history might be safer if data availability becomes an issue near boundaries.
    // Let's use 5 * d_sample_num to be safe for the CFO calculation step within one call if possible.
    set_history(5 * d_sample_num);
}

/*
 * Our virtual destructor.
 */
css_frame_sync_impl::~css_frame_sync_impl() {}


// Dechirp helper function
std::pair<float, int>
css_frame_sync_impl::dechirp(const gr_complex *input_buffer, size_t buffer_offset_in_buffer, bool is_up)
{
    // Need d_sample_num samples starting at input_buffer + buffer_offset_in_buffer
    // Need d_fft_len samples for FFT input
    std::vector<gr_complex> dechirped_symbol(d_sample_num);
    std::vector<gr_complex> fft_input(d_fft_len, 0); // Padded with zeros
    // Check if enough data is available in the buffer from the offset
    // This check should ideally happen *before* calling dechirp from work,
    // but adding a safeguard here.
    // The data needed is input_buffer[buffer_offset_in_buffer ... buffer_offset_in_buffer + d_sample_num - 1]
    // The size of the available buffer relative to input_buffer start is not known here.
    // Assume the caller (work) guarantees buffer_offset_in_buffer + d_sample_num <= total_items_in_buffer
    // This assumption is okay if work checks bounds correctly using nitems_read(0) and ninput_items[0].
    const std::vector<gr_complex>& chirp_to_use = is_up ? d_upchirp : d_downchirp;
    // Apply dechirping and copy to FFT input
    for (int i = 0; i < d_sample_num; ++i) {
        // Access input data at offset + i
        dechirped_symbol[i] = input_buffer[buffer_offset_in_buffer + i] * std::conj(chirp_to_use[i]);
    }
    // Copy dechirped signal into FFT input buffer (first d_sample_num points)
    memcpy(fft_input.data(), dechirped_symbol.data(), d_sample_num * sizeof(gr_complex));
    // FFT output buffer
    std::vector<gr_complex> fft_output(d_fft_len);
    // Execute FFT
    // 1. 将输入数据复制到 FFT 的输入缓冲区
    memcpy(d_fft->get_inbuf(), fft_input.data(), d_fft_len * sizeof(gr_complex));
    // 2. 执行 FFT
    d_fft->execute();
    // 3. 从 FFT 的输出缓冲区获取结果
    memcpy(fft_output.data(), d_fft->get_outbuf(), d_fft_len * sizeof(gr_complex));
    // Calculate combined magnitude spectrum (corresponding to matlab abs(ft(1:bin_num)) + abs(ft(fft_len-bin_num+1:fft_len)))
    std::vector<float> combined_mag(d_bin_num);
    for (int i = 0; i < d_bin_num; ++i) {
        // In C++ (0-indexed): ft[0...bin_num-1] and ft[fft_len-bin_num ... fft_len-1]
        combined_mag[i] = std::abs(fft_output[i]) + std::abs(fft_output[d_fft_len - 1 - i]);
    }
    // Find the peak in the combined magnitude spectrum
    // std::max_element returns an iterator to the max element
    auto max_it = std::max_element(combined_mag.begin(), combined_mag.end());
    float peak_mag = *max_it;
    int peak_idx_0_based = std::distance(combined_mag.begin(), max_it); // 0-based index
    // Return (magnitude, 0-based index)
    return {peak_mag, peak_idx_0_based};
}

int css_frame_sync_impl::work(int noutput_items,
                              gr_vector_const_void_star& input_items,
                              gr_vector_void_star& output_items)
{
    const gr_complex *in = (const gr_complex *) input_items[0];
    gr_complex *out = (gr_complex *) output_items[0];
    // Copy input to output immediately, as this is a pass-through block with tags
    memcpy(out, in, noutput_items * sizeof(gr_complex));
    // Get the absolute index of the first item in the current input buffer
    int64_t abs_read_pos = nitems_read(0);
    // Calculate the absolute index of the last item in the current input buffer
    int64_t abs_end_pos = abs_read_pos + noutput_items;
    // Process data based on the current state
    while (d_state != STATE_SYNC_COMPLETE) { // Stay in the loop as long as we are actively syncing
        // Calculate the required data start position in the current buffer
        int64_t required_start_in_buffer = d_current_search_pos - abs_read_pos;
        int required_samples = 0; // Samples needed for the current state's operation
        switch (d_state) {
            case STATE_IDLE:
                // Start searching from d_current_search_pos (0 initially)
                d_state = STATE_SEARCHING_DOWNCHIRP;
                // Fallthrough to searching state logic
            // [[fallthrough]]; // C++17 attribute, or use break and separate logic
            case STATE_SEARCHING_DOWNCHIRP: {
                // Need d_sample_num samples for dechirp
                required_samples = d_sample_num;
                // Check if we have enough data in the buffer to perform a dechirp step
                // The search processes in steps of d_sample_num
                while (d_current_search_pos + d_sample_num <= abs_end_pos) {
                    // Check if the *start* of the symbol is within the current buffer
                    int64_t current_symbol_start_in_buffer = d_current_search_pos - abs_read_pos;
                    if (current_symbol_start_in_buffer < 0) {
                        // This case shouldn't happen if we're careful with d_current_search_pos updates
                        // and buffer boundary checks. But for robustness: advance search pos
                        // to the start of the current buffer and try again.
                        d_current_search_pos = abs_read_pos;
                        current_symbol_start_in_buffer = 0;
                        // Re-check if enough data from the new start pos
                        if (d_current_search_pos + d_sample_num > abs_end_pos) {
                            // Not enough data even from the buffer start, break search loop
                            break;
                        }
                    }
                    // Perform dechirp
                    std::pair<float, int> up_peak = dechirp(in, current_symbol_start_in_buffer, true);
                    std::pair<float, int> down_peak = dechirp(in, current_symbol_start_in_buffer, false);
                    // Check for downchirp detection (matlab: abs(down_peak(1)) > abs(up_peak(1)))
                    if (std::abs(down_peak.first) > std::abs(up_peak.first)) {
                        // Downchirp potentially found
                        // d_current_search_pos already points to the start of this downchirp symbol
                        d_state = STATE_REFINING_POSITION;
                        // fprintf(stderr, "Found potential downchirp at abs_pos: %ld\n", d_current_search_pos);
                        goto next_state_processing; // Exit search loop and process the refinement state
                    }
                    // Move to the next symbol position to search
                    d_current_search_pos += d_sample_num;
                }
                // If the loop finished without finding (i.e., ran out of buffer)
                // We have processed up to the end of the last full symbol within the buffer
                // d_current_search_pos is already updated to the start of the next symbol to check
                // We are done with this buffer chunk for searching.
                return noutput_items; // Consume all input, state and search pos are saved
            next_state_processing:; // Label to jump to if state changes
            } break; // End of STATE_SEARCHING_DOWNCHIRP case
            case STATE_REFINING_POSITION: {
                // Needs d_sample_num samples starting at d_current_search_pos
                required_samples = d_sample_num;
                required_start_in_buffer = d_current_search_pos - abs_read_pos;
                if (required_start_in_buffer < 0 || required_start_in_buffer + required_samples > noutput_items) {
                    // Not enough data in the current buffer for refinement
                    // d_current_search_pos needs to be adjusted to the start of the current buffer
                    // if it was before the buffer start, then wait for next call.
                    // The state implies d_current_search_pos is set correctly from the previous state.
                    // So, just check if enough *remaining* data is in the buffer.
                    return noutput_items; // Consume all input, state and search pos are saved
                }
                // Perform dechirp on the found downchirp symbol
                std::pair<float, int> pkd = dechirp(in, required_start_in_buffer, false);
                // Calculate fine timing offset 'to' (matlab: pkd(2) is 1-based index)
                // C++ pkd.second is 0-based index
                int to;
                if (pkd.second >= d_bin_num / 2) { // Note: matlab used >, C++ uses >= for 0-based
                    to = round((float)(pkd.second - d_bin_num) / d_zero_padding_ratio);
                } else {
                    to = round((float)pkd.second / d_zero_padding_ratio);
                }
                // Apply the offset to the current search position
                d_current_search_pos += to;
                // fprintf(stderr, "Refined position to abs_pos: %ld (offset %d)\n", d_current_search_pos, to);
                d_state = STATE_CFO_CALCULATING;
                // Continue to next state's processing within this work call if possible
            } break; // End of STATE_REFINING_POSITION case
            case STATE_CFO_CALCULATING: {
                // Needs d_sample_num samples starting at d_current_search_pos - 4*d_sample_num
                // This is the start of the last preamble symbol before the sync word
                int64_t preamble_start_abs = d_current_search_pos - 4LL * d_sample_num;
                required_samples = d_sample_num;
                required_start_in_buffer = preamble_start_abs - abs_read_pos;
                if (required_start_in_buffer < 0 || required_start_in_buffer + required_samples > noutput_items) {
                    // Not enough data in the current buffer for CFO calculation
                    // The needed data is *before* the current d_current_search_pos.
                    // If the required data is not in the *current* buffer starting at abs_read_pos,
                    // it means it was in previous buffers and is no longer accessible unless history is large enough.
                    // We set history, but maybe not enough if the initial search pos was very late.
                    // If required_start_in_buffer is negative, the data is before the current buffer start.
                    // This indicates a problem with history or initial search position being too far into the stream.
                    // For simplicity, we assume history guarantees this data is available relative to d_current_search_pos.
                    // So required_start_in_buffer should be >= -history + d_sample_num.
                    // Let's just check if required_start_in_buffer + required_samples is within [0, noutput_items].
                    if (required_start_in_buffer < -((int64_t)this->history() - d_sample_num) || required_start_in_buffer + required_samples > noutput_items - (this->history() - d_sample_num)) {
                        // Not enough history or data in the current buffer.
                        // This state might not be reachable with insufficient history.
                        // If we reach here, it implies data needed is before current buffer AND before history.
                        // This case should be rare with adequate history.
                        // Let's just consume available data and wait.
                        fprintf(stderr, "CFO calculation data not available. Required buffer start: %ld, history: %zu\n", required_start_in_buffer, this->history());
                            return noutput_items; // Consume all input, state and search pos are saved
                    }
                    // Adjust input pointer to the actual start of accessible history data
                    // The dechirp function takes offset from the buffer start (in this case, history start)
                    const gr_complex *in_history = in - (this->history() - d_sample_num);
                    required_start_in_buffer += (this->history() - d_sample_num); // Offset from history start
                    std::pair<float, int> pku = dechirp(in_history, required_start_in_buffer, true);
                    d_preamble_bin = pku.second; // 0-based index
                    // fprintf(stderr, "CFO calculation data available. Required buffer start (adjusted): %ld\n", required_start_in_buffer);
                } else { // Data is fully within the current buffer (from abs_read_pos)
                    std::pair<float, int> pku = dechirp(in, required_start_in_buffer, true);
                    d_preamble_bin = pku.second; // 0-based index
                    // fprintf(stderr, "CFO calculation data fully in buffer. Required buffer start: %ld\n", required_start_in_buffer);
                }
                // Calculate CFO (matlab: preamble_bin is 1-based)
                // C++ d_preamble_bin is 0-based
                if (d_preamble_bin >= d_bin_num / 2) { // Note: matlab used >, C++ uses >= for 0-based index
                    d_cfo = (double)(d_preamble_bin - d_bin_num) * d_bw / d_bin_num;
                } else {
                    d_cfo = (double)d_preamble_bin * d_bw / d_bin_num;
                }
                // fprintf(stderr, "Calculated CFO: %f Hz (Preamble bin: %d)\n", d_cfo, d_preamble_bin);
                d_state = STATE_PAYLOAD_START_CALCULATING;
                // Continue to next state's processing within this work call if possible
            } break; // End of STATE_CFO_CALCULATING case
            case STATE_PAYLOAD_START_CALCULATING: {
                // Needs d_sample_num samples starting at d_current_search_pos - d_sample_num
                // This is the symbol just before the refined sync word start
                int64_t symbol_before_sync_start_abs = d_current_search_pos - (long long)d_sample_num;
                required_samples = d_sample_num;
                required_start_in_buffer = symbol_before_sync_start_abs - abs_read_pos;
                std::pair<float, int> pku, pkd;
                if (required_start_in_buffer < 0 || required_start_in_buffer + required_samples > noutput_items) {
                    // Not enough data in the current buffer for this step
                    // Similar data availability check as CFO calculation
                    // If data needed is before current buffer AND before history:
                    if (required_start_in_buffer < -((int64_t)this->history() - d_sample_num) || required_start_in_buffer + required_samples > noutput_items - (this->history() - d_sample_num)) {
                            fprintf(stderr, "Payload start data not available. Required buffer start: %ld, history: %zu\n", required_start_in_buffer, this->history());
                            return noutput_items; // Consume all input, state and search pos are saved
                    }
                    // Adjust input pointer to history start if needed
                    const gr_complex *in_history = in - (this->history() - d_sample_num);
                    required_start_in_buffer += (this->history() - d_sample_num); // Offset from history start
                    pku = dechirp(in_history, required_start_in_buffer, true);
                    pkd = dechirp(in_history, required_start_in_buffer, false);
                    // fprintf(stderr, "Payload start data available (adjusted). Required buffer start: %ld\n", required_start_in_buffer);
                } else { // Data is fully within the current buffer
                    pku = dechirp(in, required_start_in_buffer, true);
                    pkd = dechirp(in, required_start_in_buffer, false);
                    // fprintf(stderr, "Payload start data fully in buffer. Required buffer start: %ld\n", required_start_in_buffer);
                }
                // Determine payload start index x_sync (matlab logic)
                int64_t x_sync;
                if (std::abs(pku.first) > std::abs(pkd.first)) {
                    // Current symbol (at d_current_search_pos - d_sample_num) is upchirp (preamble)
                    // This means d_current_search_pos is the start of the *first* downchirp
                    x_sync = d_current_search_pos + round(2.25 * d_sample_num);
                } else {
                    // Current symbol is downchirp (first sync word downchirp)
                    // This means d_current_search_pos is the start of the *second* downchirp
                    x_sync = d_current_search_pos + round(1.25 * d_sample_num);
                }
                // Add tag at the payload start position
                int64_t tag_offset_in_buffer = x_sync - abs_read_pos;
                if (tag_offset_in_buffer >= 0 && tag_offset_in_buffer < noutput_items) {
                    // Tag is within the current output buffer chunk
                    this->add_item_tag(0, // Port 0
                                        tag_offset_in_buffer, // Offset in *output* buffer
                                        pmt::string_to_symbol("payload_start"), // Tag key
                                        pmt::make_dict()); // Tag value (can add info here)
                    fprintf(stderr, "!!!!! LoRa Frame Sync Found! Tag added at absolute position %ld (buffer offset %ld) !!!!!\n", x_sync, tag_offset_in_buffer);
                } else {
                    // Tag position is outside the current buffer.
                    // This can happen if x_sync is far beyond the current buffer end,
                    // or before the current buffer start (but history should cover it for processing).
                    // If x_sync is beyond the current buffer, the tag will be added in a future call
                    // when that index falls within the buffer processed by work.
                    // If x_sync is before the current buffer, it means we found sync late,
                    // but the tag position is still valid relative to the total stream.
                    // The tag system should handle this when that index range is produced.
                    // So, we don't add the tag *now*, but the state should track the absolute tag pos if needed.
                    // For simplicity, just print a warning.
                    fprintf(stderr, "Warning: Calculated tag position %ld is outside current buffer range [%ld, %ld). Tag will be added later.\n", x_sync, abs_read_pos, abs_end_pos);
                        // Although add_item_tag documentation suggests it handles offsets relative to nitems_read(0),
                        // it's safest if the offset is within [0, noutput_items).
                        // Let's rely on GNU Radio's tag propagation if it's outside the current chunk.
                        // The tag system usually works with absolute stream positions.
                        // Let's try adding the tag with the absolute position instead of relative buffer offset.
                        this->add_item_tag(0, // Port 0
                                            x_sync, // Absolute offset in *output* stream
                                            pmt::string_to_symbol("payload_start"), // Tag key
                                            pmt::make_dict()); // Tag value
                        fprintf(stderr, "!!!!! LoRa Frame Sync Found! Tag added at absolute position %ld !!!!!\n", x_sync);
                        // This might be the correct way for absolute tagging.
                }
                d_state = STATE_SYNC_COMPLETE; // Synchronization is complete for this frame
                // No break here, let the loop check the state and potentially exit
            } break; // End of STATE_PAYLOAD_START_CALCULATING case
            case STATE_SYNC_COMPLETE:
                // We are done. Just pass data through. The loop will exit.
                break;
        }
        // If we reached here without returning, it means a state transition happened,
        // and we should try processing the next state with potentially the same buffer data.
        // The state machine continues processing until it runs out of data for the current step
        // or reaches STATE_SYNC_COMPLETE.
    }
    // Return the number of items consumed and produced (1:1 block)
    return noutput_items;
}

} /* namespace cssmods */
} /* namespace gr */
