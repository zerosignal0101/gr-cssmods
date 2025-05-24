/* -*- c++ -*- */
/*
 * Copyright 2025 zerosignal.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "css_decode_symbols_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace cssmods {

css_decode_symbols::sptr
css_decode_symbols::make(int sf, double bw, int zero_padding_ratio)
{
    return gnuradio::make_block_sptr<css_decode_symbols_impl>(sf, bw, zero_padding_ratio);
}


/*
 * The private constructor
 */
css_decode_symbols_impl::css_decode_symbols_impl(int sf,
                                                 double bw,
                                                 int zero_padding_ratio)
    : gr::block("css_decode_symbols",
                gr::io_signature::make(1, 1, sizeof(gr_complex)),
                     gr::io_signature::make(1, 1, sizeof(unsigned char))),
      d_sf(sf),
      d_bw(bw),
      d_zero_padding_ratio(zero_padding_ratio),
      d_sample_num(2 * (1 << sf)),
      d_bin_num((1 << sf) * zero_padding_ratio),
      d_fft_len(d_sample_num * zero_padding_ratio),
      d_decoding_active(false),
      d_preamble_bin_ref(-1),
      d_samples_in_buffer(0),
      d_output_tag_pending(false),
      d_debug(true) // Set to true for debug output, or make it a parameter
{
    if (d_debug) {
        fprintf(stderr, "css_decode_symbols_impl: Initializing with SF=%d, BW=%.2f, ZPR=%d\n", d_sf, d_bw, d_zero_padding_ratio);
        fprintf(stderr, "  Calculated: sample_num=%d, bin_num=%d, fft_len=%d\n", d_sample_num, d_bin_num, d_fft_len);
    }

    d_fft = std::make_shared<gr::fft::fft_complex_fwd>(d_fft_len);

    double effective_fs_for_chirp_gen = 2.0 * d_bw; // Consistent with css_frame_sync
    d_upchirp = generate_lora_chirp(true, d_sf, d_bw, effective_fs_for_chirp_gen, 0, 0.0, 0); // CFO=0
    d_downchirp = generate_lora_chirp(false, d_sf, d_bw, effective_fs_for_chirp_gen, 0, 0.0, 0); // CFO=0

    if (d_upchirp.size() != (size_t)d_sample_num || d_downchirp.size() != (size_t)d_sample_num) {
        fprintf(stderr, "css_decode_symbols_impl: Warning: Generated chirp length (%zu) does not match d_sample_num (%d).\n",
                d_upchirp.size(), d_sample_num);
    }
    if (d_debug) {
        print_complex_vector(d_upchirp, "Decoder d_upchirp", 16);
    }

    d_symbol_candidate_buffer.resize(d_sample_num);
    // No set_history needed if we process sample by sample and buffer internally.
    // Or, set_history(d_sample_num) if we want to ensure a full symbol can be grabbed if a tag appears mid-buffer.
    // The current work logic buffers internally, so history isn't strictly for assembly,
    // but for tag alignment, it's good not to have too large history. Default is 0.
    // Let's keep history minimal or 0 and rely on internal buffering.
}

/*
 * Our virtual destructor.
 */
css_decode_symbols_impl::~css_decode_symbols_impl() {}

void css_decode_symbols_impl::forecast(int noutput_items,
                                       gr_vector_int& ninput_items_required)
{
    // To produce noutput_items, we need to consume noutput_items * d_sample_num inputs
    // if decoding is active and buffer is empty.
    // Even if not decoding, we consume inputs.
    // A safe forecast is to request enough inputs to potentially produce the requested outputs.
    ninput_items_required[0] = noutput_items * d_sample_num;
    // If you want it to be called even with less data (e.g., for buffering),
    // you could use a smaller number, but the ratio N_in/N_out is N_sample/1.
    // Requesting N_out * N_sample is standard for such blocks.
}

int css_decode_symbols_impl::general_work(int noutput_items,
                                          gr_vector_int& ninput_items,
                                          gr_vector_const_void_star& input_items,
                                          gr_vector_void_star& output_items)
{
    const gr_complex* in_ptr = (const gr_complex*)input_items[0];
    unsigned char* out_ptr = (unsigned char*)output_items[0];
    int consumed_count = 0;
    int produced_count = 0;
    uint64_t abs_input_offset = nitems_read(0);
    // Iterate through all available input items, or until we decide to stop
    // (e.g., output buffer is full, or no input available).
    for (int i = 0; i < ninput_items[0]; ++i) {
        uint64_t current_abs_pos = abs_input_offset + i;
        std::vector<gr::tag_t> tags;
        // Check tags only for the current item
        get_tags_in_window(tags, 0, current_abs_pos, current_abs_pos + 1);
        for (const auto& tag : tags) {
            if (tag.key == pmt::string_to_symbol("payload_start")) {
                if (d_debug) {
                    fprintf(stderr, "css_decode_symbols_impl: Received 'payload_start' tag at abs_pos %lu (input idx %d).\n", current_abs_pos, i);
                }
                d_decoding_active = true;
                d_samples_in_buffer = 0; // Reset buffer for new frame (start collecting samples *after* this tag)
                d_output_tag_pending = false; // Assume tag is invalid until verified
                d_preamble_bin_ref = -1; // Reset
                if (pmt::is_dict(tag.value)) {
                    d_preamble_bin_ref = pmt::to_long(pmt::dict_ref(tag.value, pmt::string_to_symbol("preamble_bin"), pmt::from_long(-1)));
                    if (d_preamble_bin_ref == -1) {
                        if (d_debug) {
                            fprintf(stderr, "css_decode_symbols_impl: Error: 'payload_start' tag missing valid 'preamble_bin'. Disabling decoding for this frame.\n");
                        }
                        d_decoding_active = false; // Disable decoding if tag is bad
                    } else {
                        if (d_debug) {
                            fprintf(stderr, "  Extracted preamble_bin_ref: %d\n", d_preamble_bin_ref);
                        }
                        d_output_tag_pending = true; // Tag is valid, prepare to propagate
                        d_pending_tag_dict = tag.value; // Copy dictionary
                    }
                } else {
                    if (d_debug) {
                        fprintf(stderr, "css_decode_symbols_impl: Error: 'payload_start' tag value is not a dictionary. Disabling decoding.\n");
                    }
                    d_decoding_active = false;
                }
            }
             // Add handling for other relevant tags (e.g., frame_end if needed)
        }
        // --- Process the current sample (in_ptr[i]) ---
        if (d_decoding_active) {
            // Check if adding this sample would complete a symbol AND the output buffer is full.
            // We check against d_sample_num - 1 because d_samples_in_buffer hasn't been incremented yet.
            if (d_samples_in_buffer == d_sample_num - 1 && produced_count >= noutput_items) {
                // Adding this sample would complete a symbol, but we have no space to output it.
                // Do *not* consume this sample (in_ptr[i]) in this call.
                if (d_debug) {
                     fprintf(stderr, "Decoder: Output buffer full (%d items), cannot complete symbol with sample %d. Stopping processing input.\n", noutput_items, i);
                }
                consumed_count = i; // We have consumed samples 0 through i-1. Sample i remains.
                goto end_work_loop; // Break out of the for loop
            }
            // If we reached here, we can safely add sample i to the buffer.
            d_symbol_candidate_buffer[d_samples_in_buffer++] = in_ptr[i];
            // Increment consumed_count *after* successfully processing/buffering the sample
            consumed_count++;
            // Check if symbol is now complete *after* adding sample i
            if (d_samples_in_buffer == d_sample_num) {
                // A full symbol is collected. Attempt to produce output.
                // We know output space is available because of the check above (produced_count < noutput_items).
                // If we get here, produced_count was < noutput_items *before* processing sample i.
                // It might be == noutput_items *now* after processing sample i and incrementing produced_count,
                // but that's okay, we produced *one* item.
                if (d_output_tag_pending) {
                    add_item_tag(0, // port
                                 nitems_written(0) + produced_count, // abs offset on output stream
                                 pmt::string_to_symbol("payload_start"),
                                 d_pending_tag_dict);
                    d_output_tag_pending = false; // Tag is propagated for this frame's first symbol
                    if (d_debug) {
                        fprintf(stderr, "  Added 'payload_start' tag to output at output_item_idx %lu.\n", nitems_written(0) + produced_count);
                    }
                }
                // Dechirp the collected symbol (payload symbols are typically upchirps)
                std::pair<float, int> peak_info = dechirp(
                    d_symbol_candidate_buffer.data(), 0, true, // Pass true for upchirp dechirp if payload is upchirp based
                    d_sample_num, d_fft, d_fft_len, d_bin_num,
                    d_upchirp, d_downchirp); // Note: you use both chirps in generate_lora_chirp, ensure dechirp uses correct one
                int current_symbol_peak_bin = peak_info.second; // 0-based
                // Symbol calculation (ensure positive result before modulo for robustness)
                // (current_symbol_peak_bin - d_preamble_bin_ref) is the offset from the reference
                // Add d_bin_num before modulo to ensure positive result
                double raw_symbol_val = static_cast<double>(current_symbol_peak_bin - d_preamble_bin_ref);
                 // Adjust for wrap-around if peak is below preamble ref
                 if (raw_symbol_val < 0) {
                     raw_symbol_val += d_bin_num;
                 }
                int symbol_value = static_cast<int>(round(raw_symbol_val / d_zero_padding_ratio)) % (1 << d_sf);
                 // Ensure positive modulo result if round() results in a negative value before adding d_bin_num (less likely with the raw_symbol_val adjustment)
                 // The previous calculation `(static_cast<double>(current_symbol_peak_bin - d_preamble_bin_ref + d_bin_num)) / d_zero_padding_ratio` is also a common way and ensures positive intermediate before division. Let's stick to that one as it seems intended.
                 // Revert symbol calculation to original but ensure modulo is positive:
                 raw_symbol_val = (static_cast<double>(current_symbol_peak_bin - d_preamble_bin_ref + d_bin_num)); // Intermediate might be large but positive
                 symbol_value = static_cast<int>(round(raw_symbol_val / d_zero_padding_ratio));
                 symbol_value = symbol_value % (1 << d_sf); // Apply modulo
                 if (symbol_value < 0) { // Ensure positive result of modulo
                     symbol_value += (1 << d_sf);
                 }
                out_ptr[produced_count++] = static_cast<unsigned char>(symbol_value);
                if (d_debug) {
                    fprintf(stderr, "  Decoded symbol %d: input_idx range [%d, %d], peak_bin=%d, preamble_ref=%d, val_calc_raw=%.2f, final_val=%d (Mag: %.2f)\n",
                            produced_count - 1, i - d_sample_num + 1, i, current_symbol_peak_bin, d_preamble_bin_ref, raw_symbol_val, symbol_value, peak_info.first);
                }
                d_samples_in_buffer = 0; // Reset for next symbol
                // Note: No need to check produced_count >= noutput_items here after producing.
                // The outer loop condition or the 'goto' check before adding the sample handles it.
            } // End if symbol complete
        } else {
            // Not decoding. Just consume the sample without buffering.
            // Increment consumed_count for this sample.
            consumed_count++;
        }
    } // End of for loop (i)
end_work_loop: // Label for goto
    // This debug message was misleading. Replace it with a general status message.
    // if (d_debug) {
    //     fprintf(stderr, "  Decoded symbol: No tag found from %ld to %ld, consumed %d\n",
    //             abs_input_offset, abs_input_offset + ninput_items[0], consumed_count);
    // }
    if (d_debug) {
        fprintf(stderr, "Decoder: Work done. Consumed %d input items (avail: %d), Produced %d items (avail: %d). Buffer size: %d/%d. Decoding: %s.\n",
                consumed_count, ninput_items[0], produced_count, noutput_items, d_samples_in_buffer, d_sample_num, d_decoding_active ? "Yes" : "No");
    }
    // This consumes the samples that were successfully processed within the loop (up to consumed_count).
    consume(0, consumed_count);
    // Return the number of output items produced in this call.
    return produced_count;
}

} /* namespace cssmods */
} /* namespace gr */
