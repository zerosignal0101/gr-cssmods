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
#include <cstdio> // For printf/fprintf

namespace gr {
namespace cssmods {

using input_type = gr_complex;
using output_type = gr_complex;

// Public factory function
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
        d_sample_num(2 * (1 << sf)), // Samples per symbol at 2*BW sampling rate
        d_bin_num((1 << sf) * zero_padding_ratio), // Number of relevant bins after zero-padding
        d_fft_len(d_sample_num * zero_padding_ratio), // FFT length including zero-padding
        d_preamble_len(8),
        d_debug(true), // *** Initialize debug flag to true ***
        d_state(STATE_IDLE),
        d_current_search_pos(0), // Absolute stream index
        d_preamble_bin(-1),
        d_preamble_counter(0),
        d_cfo(0.0)
{
    if (d_debug) {
        fprintf(stderr, "css_frame_sync_impl: Initializing with SF=%d, BW=%f, ZPR=%d\n", d_sf, d_bw, d_zero_padding_ratio);
        fprintf(stderr, "  Calculated: sample_num=%d, bin_num=%d, fft_len=%d\n", d_sample_num, d_bin_num, d_fft_len);
    }

    // Initialize FFT object
    d_fft = std::make_shared<gr::fft::fft_complex_fwd>(d_fft_len); // false for forward FFT

    // Generate nominal chirps (cfo=0) based on the determined parameters
    // LoRa chirps are typically generated with a bandwidth equal to the LoRa BW (d_bw),
    // but sampled at a rate that yields d_sample_num samples over the nominal symbol duration.
    // The nominal symbol duration is (1 << d_sf) / d_bw.
    // So, the effective sampling rate for chirp generation is d_sample_num / ((1 << d_sf) / d_bw)
    // = (2 * (1 << d_sf)) / ((1 << d_sf) / d_bw) = 2 * d_bw. This matches the common 2*BW sampling.
    double effective_fs_for_chirp_gen = 2.0 * d_bw;

    d_upchirp = generate_lora_chirp(
        true, d_sf, d_bw, effective_fs_for_chirp_gen, 0, d_cfo, 0);
    d_downchirp = generate_lora_chirp(
        false, d_sf, d_bw, effective_fs_for_chirp_gen, 0, d_cfo, 0);

    if (d_debug) {
        std::cout << "Generating LoRa chirps with parameters:" << std::endl;
        std::cout << "  Spreading Factor (SF): " << d_sf << std::endl;
        std::cout << "  Bandwidth (BW): " << d_bw << " Hz" << std::endl;
        std::cout << "  Effective sample rate: " << effective_fs_for_chirp_gen << " Hz" << std::endl;
        std::cout << "  Cfo: " << d_cfo << " seconds" << std::endl;
        print_complex_vector(d_upchirp, "Raw d_upchirp", 256);
        print_complex_vector(d_downchirp, "Raw d_downchirp", 256);
    }

    if (d_debug) {
        fprintf(stderr, "css_frame_sync_impl: Generated nominal chirps of length %zu.\n", d_upchirp.size());
    }

    if (d_upchirp.size() != (size_t)d_sample_num || d_downchirp.size() != (size_t)d_sample_num) {
        // This indicates a potential mismatch in how sample_num is derived vs. generate_lora_chirp
        // It might work if generate_lora_chirp internally handles the sample rate differently,
        // but it's safer if they match. Based on the calculation above (2*BW sampling), they should match.
        fprintf(stderr, "css_frame_sync_impl: Warning: Generated chirp length (%zu) does not match d_sample_num (%d). Parameter mismatch?\n", d_upchirp.size(), d_sample_num);
    }

    // Set history to require enough data for the largest look-back step.
    // CFO calculation looks back 4 symbols (4 * d_sample_num) from the sync word start.
    // To dechirp that symbol, we need d_sample_num samples starting at that historical position.
    // So, the history needs to cover 4 * d_sample_num.
    // A history of 5 * d_sample_num ensures that the start of the 4th symbol back
    // (relative to the current symbol start) is always available within the history buffer
    // when we are processing the symbol at d_current_search_pos.
    set_history(5 * d_sample_num);
    if (d_debug) {
        fprintf(stderr, "css_frame_sync_impl: History set to %zu samples.\n", this->history());
    }
}

/*
 * Our virtual destructor.
 */
css_frame_sync_impl::~css_frame_sync_impl() {}


void print_complex_vector(const std::vector<gr_complex>& vec, 
                                              const std::string& name, 
                                              size_t max_print = 10) 
{
    std::cout << "----- " << name << " (size=" << vec.size() << ") -----" << std::endl;
    
    // 输出MATLAB格式的复数数组
    std::cout << "MATLAB format: [";
    size_t print_count = std::min(max_print, vec.size());
    for (size_t i = 0; i < print_count; ++i) {
        // 输出实部和虚部，用"+"或"-"连接，末尾加"i"（MATLAB格式）
        std::cout << vec[i].real() 
                 << ((vec[i].imag() >= 0) ? "+" : "")  // 处理正负号
                 << vec[i].imag() << "i";
        
        // 最后一个元素后不加逗号
        if (i < print_count - 1) {
            std::cout << ", ";
        }
    }
    
    if (vec.size() > max_print) {
        std::cout << ", ...";  // 提示还有更多元素
    }
    
    std::cout << "]" << std::endl;
    
    if (vec.size() > max_print) {
        std::cout << "... (showing first " << max_print << " of " << vec.size() << " elements)" << std::endl;
    }
    
    std::cout << "------------------------" << std::endl;
}



void print_float_vector(const std::vector<float>& vec, 
                                           const std::string& name, 
                                           size_t max_print) 
{
    std::cout << "----- " << name << " (size=" << vec.size() << ") -----" << std::endl;
    
    // 输出 MATLAB 格式的浮点数组
    std::cout << "MATLAB format: [";
    size_t print_count = std::min(max_print, vec.size());
    for (size_t i = 0; i < print_count; ++i) {
        std::cout << vec[i];
        if (i < print_count - 1) {
            std::cout << ", ";  // 元素间用逗号分隔
        }
    }
    
    if (vec.size() > max_print) {
        std::cout << ", ...";  // 提示数据被截断
    }
    
    std::cout << "]" << std::endl;
    
    if (vec.size() > max_print) {
        std::cout << "... (showing first " << max_print << " of " << vec.size() << " elements)" << std::endl;
    }
    
    std::cout << "------------------------" << std::endl;
}



// Dechirp helper function: Apply conjugated nominal chirp, perform FFT, find peak
std::pair<float, int>
dechirp(const gr_complex *input_buffer, int64_t buffer_offset_in_buffer, bool is_up, int sample_num, 
    std::shared_ptr<gr::fft::fft_complex_fwd> fft, int fft_len, int bin_num,
    std::vector<std::complex<float>>& upchirp, std::vector<std::complex<float>>& downchirp)
{
    // Need sample_num samples from the input buffer starting at buffer_offset_in_buffer
    // for the dechirping step.
    // The result is then zero-padded to fft_len for the FFT.

    // Check if the buffer offset and required samples are valid within the input buffer
    // This check relies on the caller (work) ensuring that buffer_offset_in_buffer + sample_num
    // does not exceed the bounds of the provided input_buffer segment.
    // In work, the input_buffer can be the start of the current input chunk (`in`)
    // or the start of the history buffer (`in - this->history() + sample_num`).
    // The offset is calculated relative to whichever pointer is passed.
    // Assuming the caller ensures enough data is available from the given input_buffer start.

    std::vector<gr_complex> dechirped_symbol(sample_num, 0);

    memcpy(dechirped_symbol.data(), 
           input_buffer + buffer_offset_in_buffer, 
           sample_num * sizeof(gr_complex));
    
    // Use a vector for FFT input which will be zero-padded
    std::vector<gr_complex> fft_input(fft_len, 0); // Automatically initialized to zeros

    const std::vector<gr_complex>& chirp_to_use = is_up ? upchirp : downchirp;

    // Apply dechirping (complex multiplication with conjugate chirp)
    // Use VOLK for optimized multiplication if available
    volk_32fc_x2_multiply_conjugate_32fc(dechirped_symbol.data(), 
                                        dechirped_symbol.data(), 
                                        chirp_to_use.data(), 
                                        sample_num);

    // Copy dechirped signal into FFT input buffer (first sample_num points)
    // The remaining points in fft_input (from sample_num to fft_len-1) remain zero (zero-padding)
    std::copy(dechirped_symbol.begin(), dechirped_symbol.end(), fft_input.begin());

    // print_complex_vector(fft_input, "FFT input (after zero-padding)", sample_num);

    // Execute FFT
    // 1. Copy input data to FFT's input buffer
    gr_complex* fft_in = fft->get_inbuf();
    memcpy(fft_in, fft_input.data(), fft_len * sizeof(gr_complex));
    // 2. Execute FFT
    fft->execute();
    // 3. Get result from FFT's output buffer
    // Note: Direct access to outbuf is common, but copying to a vector is also fine
    // std::vector<gr_complex> fft_output(fft_len);
    // memcpy(fft_output.data(), fft->get_outbuf(), fft_len * sizeof(gr_complex));
    const gr_complex* fft_output = fft->get_outbuf(); // Access output directly

    // Calculate combined magnitude spectrum (corresponding to matlab abs(ft(1:bin_num)) + abs(ft(fft_len-bin_num+1:fft_len)))
    // This combines the positive and negative frequency components for a given bin offset due to cyclic property of chirps.
    std::vector<float> combined_mag(bin_num);
    for (int i = 0; i < bin_num; ++i) {
        // In C++ (0-indexed): fft_output[0...bin_num-1] and fft_output[fft_len-bin_num ... fft_len-1]
        // The corresponding bins are i and fft_len - 1 - i for 0-based indexing.
        combined_mag[i] = std::abs(fft_output[i]) + std::abs(fft_output[fft_len - 1 - i]);
    }
    // print_float_vector(combined_mag, "Combine Mag", bin_num);

    // Find the peak in the combined magnitude spectrum
    auto max_it = std::max_element(combined_mag.begin(), combined_mag.end());
    float peak_mag = *max_it;
    int peak_idx_0_based = std::distance(combined_mag.begin(), max_it); // 0-based index in combined_mag [0, bin_num-1]

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

    if (d_debug) {
        fprintf(stderr, "css_frame_sync_impl::work: Processing %d items. Absolute range [%ld, %ld).\n",
                noutput_items, abs_read_pos, abs_end_pos);
        fprintf(stderr, "  Current state: %d, Search position: %ld\n", d_state, d_current_search_pos);
    }


    // Process data based on the current state
    // Stay in the loop as long as we are actively syncing AND have enough data
    // for the current state's *minimum* operation.
    // The data availability checks within each state handle needing more than the minimum.
    while (d_state != STATE_SYNC_COMPLETE) {
        // Calculate the start index within the *current* input buffer for the required data.
        // This is relative to the start of the buffer `in` (abs_read_pos).
        // If required_start_abs < abs_read_pos, the data is in history.
        // If required_start_abs + required_samples > abs_end_pos, not enough data in current buffer.

        switch (d_state) {
            case STATE_IDLE:
                if (d_debug) {
                    fprintf(stderr, "css_frame_sync_impl::work: State IDLE. Transitioning to SEARCHING_DOWNCHIRP.\n");
                }
                d_state = STATE_SEARCHING_PREAMBLE;
                // No break, fallthrough or re-evaluate loop condition to process new state
            
            case STATE_SEARCHING_PREAMBLE: {
                while (d_current_search_pos + d_sample_num <= abs_end_pos) {
                    int64_t current_symbol_start_in_buffer = d_current_search_pos - abs_read_pos;

                    // Re-check if enough data from the new start pos
                    if (d_current_search_pos + d_sample_num > abs_end_pos) {
                        // Not enough data even from the buffer start, break search loop
                        if (d_debug) {
                        fprintf(stderr, "css_frame_sync_impl::work: State SEARCHING_PREAMBLE. Not enough data (%ld < %d) even from buffer start %ld. Waiting for more data.\n",
                                abs_end_pos - d_current_search_pos, d_sample_num, d_current_search_pos);
                        }
                        break; // Exit while loop, return noutput_items
                    }

                    std::pair<float, int> up_peak = dechirp(in, current_symbol_start_in_buffer, true, d_sample_num, d_fft, d_fft_len, d_bin_num, d_upchirp, d_downchirp);

                    if (d_debug) {
                        fprintf(stderr, "css_frame_sync_impl::work: State SEARCHING_PREAMBLE. Checking abs_pos %ld (buffer_offset %ld). Up peak: (mag=%.2f, bin=%d).\n",
                                d_current_search_pos, current_symbol_start_in_buffer, up_peak.first, up_peak.second);
                    }

                    // Check for upchirp detection
                    if (up_peak.first < 100.0) {
                        if (d_debug) {
                            std::cout << "css_frame_sync_impl::work: State SEARCHING_PREAMBLE. "
                                    << "Invalid upchirp detected with peak value " << up_peak.first 
                                    << " (below threshold 100.0)" << std::endl;
                        }
                        // Invalid symbol
                    }
                    else if (d_preamble_bin == -1) {
                        if (d_debug) {
                            std::cout << "css_frame_sync_impl::work: State SEARCHING_PREAMBLE. "
                                    << "First valid upchirp detected at bin " << up_peak.second 
                                    << " with peak value " << up_peak.first << std::endl;
                        }
                        d_preamble_bin = up_peak.second;
                        d_preamble_counter = 1;
                        
                        if (d_debug) {
                            std::cout << "css_frame_sync_impl::work: State SEARCHING_PREAMBLE. "
                                    << "Initialized preamble tracking - bin: " << d_preamble_bin 
                                    << ", counter: " << d_preamble_counter << std::endl;
                        }
                    }
                    else {
                        int bin_diff = (d_preamble_bin - up_peak.second + d_bin_num) % d_bin_num;
                        if (bin_diff > d_bin_num / 2) {
                            bin_diff = d_bin_num - bin_diff;
                        }
                        
                        if (d_debug) {
                            std::cout << "css_frame_sync_impl::work: State SEARCHING_PREAMBLE. "
                                    << "Previous bin: " << d_preamble_bin 
                                    << ", Current bin: " << up_peak.second 
                                    << ", Raw diff: " << (d_preamble_bin - up_peak.second)
                                    << ", Wrapped diff: " << bin_diff 
                                    << ", Threshold: " << d_zero_padding_ratio << std::endl;
                        }

                        // Check the difference between current and last chirp
                        if (bin_diff < d_zero_padding_ratio) {
                            d_preamble_counter++;
                            
                            if (d_debug) {
                                std::cout << "css_frame_sync_impl::work: State SEARCHING_PREAMBLE. "
                                        << "Consistent upchirp detected. Incrementing counter to: " 
                                        << d_preamble_counter << std::endl;
                            }
                            
                            if (d_preamble_counter == d_preamble_len - 1) {
                                if (d_debug) {
                                    std::cout << "css_frame_sync_impl::work: State SEARCHING_PREAMBLE. "
                                            << "Preamble detected! Transitioning to SEARCHING_DOWNCHIRP state. "
                                            << "Detected " << d_preamble_counter 
                                            << " consistent upchirps (threshold was " 
                                            << (d_preamble_len - 1) << ")" << std::endl;
                                }
                                d_state = STATE_SEARCHING_DOWNCHIRP;
                                // Restore to default value
                                d_preamble_counter = 0;
                                d_preamble_bin = -1;
                                
                                float to; // Offset in samples
                                if (up_peak.second >= d_bin_num / 4) { // Note: matlab used >, C++ uses >= for 0-based comparison
                                    if (d_debug) {
                                        fprintf(stderr, "css_frame_sync_impl::work: State REFINING_POSITION. PKD.second larger than d_bin_num / 4: %d > %d \n",
                                                up_peak.second, d_bin_num / 4);
                                    }
                                    // Peak in upper half corresponds to negative time offset
                                    to = float(d_bin_num - up_peak.second) / d_zero_padding_ratio; // Convert bin index to sample offset
                                } else {
                                    if (d_debug) {
                                        fprintf(stderr, "css_frame_sync_impl::work: State REFINING_POSITION. PKD.second smaller than d_bin_num / 4: %d <= %d \n",
                                                up_peak.second, d_bin_num / 4);
                                    }
                                    // Peak in lower half corresponds to positive time offset
                                    to = float(up_peak.second) / d_zero_padding_ratio; // Convert bin index to sample offset
                                }
                                // Round the sample offset for integer sample positioning
                                int sto_move_delta = round(to * 2); 

                                if (d_debug) {
                                    fprintf(stderr, "css_frame_sync_impl::work: State REFINING_POSITION. Move current search position with %d sample\n",
                                            sto_move_delta);
                                }

                                // Apply the calculated sample offset to the current search position
                                d_current_search_pos = d_current_search_pos + sto_move_delta; // Update based on original position + offset
                                
                                current_symbol_start_in_buffer = d_current_search_pos - abs_read_pos; // Offset from 'in'
                                up_peak = dechirp(in, current_symbol_start_in_buffer, true, d_sample_num, d_fft, d_fft_len, d_bin_num, d_upchirp, d_downchirp);

                                if (d_debug) {
                                    fprintf(stderr, "css_frame_sync_impl::work: State REFINING_POSITION. Validate sto. mag %f bin_num %d\n",
                                            up_peak.first, up_peak.second);
                                }    

                                goto end_search_preamble;
                            }
                        }
                        else {
                            if (d_debug) {
                                std::cout << "css_frame_sync_impl::work: State SEARCHING_PREAMBLE. "
                                        << "Inconsistent upchirp detected. Bin difference " << bin_diff 
                                        << " exceeds threshold " << d_zero_padding_ratio 
                                        << ". Resetting counter to 1." << std::endl;
                            }
                            d_preamble_counter = 1;
                        }
                        
                        if (d_debug) {
                            std::cout << "css_frame_sync_impl::work: State SEARCHING_PREAMBLE. "
                                    << "Updating tracking bin from " << d_preamble_bin 
                                    << " to " << up_peak.second << std::endl;
                        }
                        d_preamble_bin = up_peak.second;
                    }


                    // Move to the next symbol position to search
                    d_current_search_pos += d_sample_num;
                }
                // If the loop finished without finding a downchirp, it means we ran out of data
                // in the current buffer chunk. d_current_search_pos is already updated
                // to the start of the next symbol to check in the *next* work call.
                if (d_debug) {
                     fprintf(stderr, "css_frame_sync_impl::work: State SEARCHING_PREAMBLE. Ran out of data in buffer [%ld, %ld) before finding downchirp. Next search starts at %ld. Waiting for more data.\n",
                             abs_read_pos, abs_end_pos, d_current_search_pos);
                }
                return noutput_items; // Consume all input, state and search pos are saved
            end_search_preamble:;
            }
                
            case STATE_SEARCHING_DOWNCHIRP: {
                // Need d_sample_num samples for dechirping a symbol.
                // The search proceeds in steps of d_sample_num.
                // d_current_search_pos holds the absolute index of the symbol start we are currently checking.

                while (d_current_search_pos + d_sample_num <= abs_end_pos) {
                    // We have enough data *from the current search position* within the current buffer.
                    int64_t current_symbol_start_in_buffer = d_current_search_pos - abs_read_pos;

                    // Re-check if enough data from the new start pos
                    if (d_current_search_pos + d_sample_num > abs_end_pos) {
                        // Not enough data even from the buffer start, break search loop
                        if (d_debug) {
                        fprintf(stderr, "css_frame_sync_impl::work: State SEARCHING_DOWNCHIRP. Not enough data (%ld < %d) even from buffer start %ld. Waiting for more data.\n",
                                abs_end_pos - d_current_search_pos, d_sample_num, d_current_search_pos);
                        }
                        break; // Exit while loop, return noutput_items
                    }

                    // Perform dechirp on the symbol at d_current_search_pos
                    std::pair<float, int> up_peak = dechirp(in, current_symbol_start_in_buffer, true, d_sample_num, d_fft, d_fft_len, d_bin_num, d_upchirp, d_downchirp);
                    std::pair<float, int> down_peak = dechirp(in, current_symbol_start_in_buffer, false, d_sample_num, d_fft, d_fft_len, d_bin_num, d_upchirp, d_downchirp);

                    if (d_debug) {
                        fprintf(stderr, "css_frame_sync_impl::work: State SEARCHING_DOWNCHIRP. Checking abs_pos %ld (buffer_offset %ld). Up peak: (mag=%.2f, bin=%d), Down peak: (mag=%.2f, bin=%d).\n",
                                d_current_search_pos, current_symbol_start_in_buffer, up_peak.first, up_peak.second, down_peak.first, down_peak.second);
                    }

                    // Move to the next symbol position to search
                    d_current_search_pos += d_sample_num;

                    // Check for downchirp detection (simple magnitude comparison)
                    if (std::abs(down_peak.first) > std::abs(up_peak.first)) {
                        // Downchirp potentially found. d_current_search_pos is the start of this symbol.
                        if (d_debug) {
                            fprintf(stderr, "css_frame_sync_impl::work: State SEARCHING_DOWNCHIRP. Potential downchirp found at abs_pos: %ld. Transitioning to REFINING_POSITION.\n", d_current_search_pos);
                        }
                        d_state = STATE_REFINING_POSITION;
                        goto next_state_processing; // Exit search loop and process the refinement state
                    }
                }

                // If the loop finished without finding a downchirp, it means we ran out of data
                // in the current buffer chunk. d_current_search_pos is already updated
                // to the start of the next symbol to check in the *next* work call.
                if (d_debug) {
                     fprintf(stderr, "css_frame_sync_impl::work: State SEARCHING_DOWNCHIRP. Ran out of data in buffer [%ld, %ld) before finding downchirp. Next search starts at %ld. Waiting for more data.\n",
                             abs_read_pos, abs_end_pos, d_current_search_pos);
                }
                return noutput_items; // Consume all input, state and search pos are saved
            next_state_processing:; // Label to jump to if state changes
            } break; // End of STATE_SEARCHING_DOWNCHIRP case


            case STATE_REFINING_POSITION: {
                d_state = STATE_CFO_CALCULATING;
                // Continue to next state's processing within this work call if possible
            } break; // End of STATE_REFINING_POSITION case


            case STATE_CFO_CALCULATING: {
                // Needs d_sample_num samples starting at d_current_search_pos - 4*d_sample_num.
                // This is the estimated start of the last preamble symbol before the sync word.
                int64_t preamble_start_abs = d_current_search_pos - 4LL * d_sample_num;
                int required_samples = d_sample_num;
                int64_t required_start_in_buffer = preamble_start_abs - abs_read_pos; // Offset from 'in'

                // Re-check if enough data from the new start pos
                if (required_start_in_buffer < - 5*d_sample_num) {
                    if (d_debug) {
                        fprintf(stderr, "css_frame_sync_impl::work: State CFO_CALCULATING. Data for preamble symbol at abs_pos %ld not available (before history or too far back in history). Required buffer start offset %ld (from buffer start %ld), needed %d samples. Buffer range [%ld, %ld). History size %zu. Waiting for more data.\n",
                            preamble_start_abs, required_start_in_buffer, abs_read_pos, required_samples, abs_read_pos, abs_end_pos, this->history());
                    }
                    // Not enough history or data in the current buffer.
                    return noutput_items; // Consume all input, state and search pos are saved
                }

                if (d_debug) {
                    fprintf(stderr, "css_frame_sync_impl::work: State CFO_CALCULATING. Data for preamble symbol at abs_pos %ld available (adjusted buffer offset %ld). Dechirping upchirp...\n",
                            preamble_start_abs, required_start_in_buffer);
                }
                std::pair<float, int> pku = dechirp(in, required_start_in_buffer, true, d_sample_num, d_fft, d_fft_len, d_bin_num, d_upchirp, d_downchirp);
                d_preamble_bin = pku.second; // 0-based index in [0, d_bin_num-1]

                if (d_debug) {
                     fprintf(stderr, "  CFO dechirp peak: (mag=%.2f, bin=%d)\n", pku.first, d_preamble_bin);
                }

                // Calculate CFO (matlab: preamble_bin is 1-based [1, d_bin_num])
                // C++ d_preamble_bin is 0-based [0, d_bin_num-1]
                // CFO = (bin_offset / bin_num) * BW
                // Bin offset relative to center (d_bin_num/2):
                // If bin >= d_bin_num/2, offset is bin - d_bin_num (wraps around)
                // If bin < d_bin_num/2, offset is bin
                double bin_offset;
                if (d_preamble_bin >= d_bin_num / 2) {
                    bin_offset = (double)(d_preamble_bin - d_bin_num);
                } else {
                    bin_offset = (double)d_preamble_bin;
                }
                d_cfo = bin_offset * d_bw / d_bin_num;

                if (d_debug) {
                    fprintf(stderr, "css_frame_sync_impl::work: State CFO_CALCULATING. Calculated CFO: %f Hz (Preamble bin: %d). Transitioning to PAYLOAD_START_CALCULATING.\n",
                            d_cfo, d_preamble_bin);
                }

                d_state = STATE_PAYLOAD_START_CALCULATING;
                // Continue to next state's processing within this work call if possible
            } break; // End of STATE_CFO_CALCULATING case


            case STATE_PAYLOAD_START_CALCULATING: {
                // Needs d_sample_num samples starting at d_current_search_pos - d_sample_num.
                // This is the symbol just before the refined sync word start (either last preamble up or first sync down).
                int64_t symbol_before_sync_start_abs = d_current_search_pos - (long long)d_sample_num;
                int required_samples = d_sample_num;
                int64_t required_start_in_buffer = symbol_before_sync_start_abs - abs_read_pos; // Offset from 'in'

                // Check if required data is available within buffer + history
                if (required_start_in_buffer < - 5*d_sample_num) {
                    if (d_debug) {
                        fprintf(stderr, "css_frame_sync_impl::work: State PAYLOAD_START_CALCULATING. Data for symbol before sync at abs_pos %ld not available (before history or too far back in history). Required buffer start offset %ld (from buffer start %ld), needed %d samples. Buffer range [%ld, %ld). History size %zu. Waiting for more data.\n",
                                symbol_before_sync_start_abs, required_start_in_buffer, abs_read_pos, required_samples, abs_read_pos, abs_end_pos, this->history());
                    }
                    return noutput_items; // Consume all input, state and search pos are saved
                }

                if (d_debug) {
                    fprintf(stderr, "css_frame_sync_impl::work: State PAYLOAD_START_CALCULATING. Data for symbol before sync word at abs_pos %ld available (adjusted buffer offset %ld). Dechirping up/down...\n",
                             symbol_before_sync_start_abs, required_start_in_buffer);
                }
                std::pair<float, int> pku = dechirp(in, required_start_in_buffer, true, d_sample_num, d_fft, d_fft_len, d_bin_num, d_upchirp, d_downchirp);
                std::pair<float, int> pkd = dechirp(in, required_start_in_buffer, false, d_sample_num, d_fft, d_fft_len, d_bin_num, d_upchirp, d_downchirp);

                if (d_debug) {
                    fprintf(stderr, "  Symbol before sync dechirp peaks: Up (mag=%.2f, bin=%d), Down (mag=%.2f, bin=%d)\n",
                            pku.first, pku.second, pkd.first, pkd.second);
                }

                // Determine payload start index x_sync (matlab logic)
                // d_current_search_pos is the refined start of the detected sync downchirp.
                // The symbol at (d_current_search_pos - d_sample_num) is the one *before* it.
                // If that symbol is UP, then d_current_search_pos must be the start of the *first* sync DOWN.
                // Frame structure: Preamble (8 Up) -> Sync Word (2 Down) -> Header/Payload (starts 2.25 sym after sync word start).
                // If d_current_search_pos is the start of 1st DOWN, payload starts at:
                // d_current_search_pos (start of 1st DOWN) + d_sample_num (1st DOWN) + d_sample_num (2nd DOWN) + 2.25 * d_sample_num (header/payload offset)
                // = d_current_search_pos + 4.25 * d_sample_num? No, matlab is 2.25*d_sample_num.
                // Let's re-read the matlab code comments or documentation.
                // The LoRa standard specifies 8 preamble upchirps + 2 sync downchirps.
                // The header/payload starts 2.25 symbols AFTER the start of the *sync word*.
                // The sync word starts with the *first* downchirp.
                // So, Payload Start = (Start of 1st Sync Down) + 2.25 * d_sample_num.
                // The refinement step `d_current_search_pos += to;` sets `d_current_search_pos` to the refined start of the *detected* downchirp.
                // If the symbol before it is UP, then `d_current_search_pos` *is* the start of the 1st sync DOWN.
                // Payload Start = `d_current_search_pos + round(2.25 * d_sample_num)` -> This matches the matlab logic!
                // If the symbol before it is DOWN, then `d_current_search_pos` must be the start of the *second* sync DOWN.
                // This implies the refinement landed slightly late.
                // The start of the 1st sync DOWN would be `d_current_search_pos - d_sample_num`.
                // Payload Start = (Start of 1st Sync Down) + 2.25 * d_sample_num
                // = (`d_current_search_pos - d_sample_num`) + 2.25 * d_sample_num
                // = `d_current_search_pos + 1.25 * d_sample_num`. -> This also matches the matlab logic!
                // The logic correctly calculates the payload start relative to the refined start of the *detected* downchirp,
                // adjusting based on whether the detected downchirp was the first or second sync symbol.

                int64_t x_sync; // Absolute index for payload start

                if (std::abs(pku.first) > std::abs(pkd.first)) {
                    // The symbol at (d_current_search_pos - d_sample_num) was an UPCHIRP (last preamble)
                    // This means d_current_search_pos is the refined start of the *first* sync DOWNCHIRP.
                    x_sync = d_current_search_pos + round(2.25 * d_sample_num);
                    if (d_debug) {
                        fprintf(stderr, "  Symbol before sync was UP. d_current_search_pos (%ld) is start of 1st sync DOWN. Payload start calc: %ld + round(2.25 * %d) = %ld.\n",
                                d_current_search_pos, d_current_search_pos, d_sample_num, x_sync);
                    }
                } else {
                    // The symbol at (d_current_search_pos - d_sample_num) was a DOWNCHIRP (first sync down)
                    // This means d_current_search_pos is the refined start of the *second* sync DOWNCHIRP.
                    x_sync = d_current_search_pos + round(1.25 * d_sample_num);
                    if (d_debug) {
                         fprintf(stderr, "  Symbol before sync was DOWN. d_current_search_pos (%ld) is start of 2nd sync DOWN. Payload start calc: %ld + round(1.25 * %d) = %ld.\n",
                                d_current_search_pos, d_current_search_pos, d_sample_num, x_sync);
                    }
                }

                // Add tag at the calculated payload start position (absolute stream index)
                // Tagging with absolute position is generally more robust in GNU Radio.
                this->add_item_tag(0, // Port 0
                                   x_sync, // Absolute offset in *output* stream
                                   pmt::string_to_symbol("payload_start"), // Tag key
                                   pmt::make_dict()); // Tag value (can add CFO, SF, BW etc. here)

                if (d_debug) {
                     fprintf(stderr, "css_frame_sync_impl::work: !!!!! LoRa Frame Sync Found! Tag added at absolute position %ld. Calculated CFO: %f Hz. !!!!!\n", x_sync, d_cfo);
                } else {
                     // Always print the final sync message, even if debug is off
                     fprintf(stderr, "!!!!! css_frame_sync: Frame Sync Found! Tag added at absolute position %ld. !!!!!\n", x_sync);
                }

                d_current_search_pos = x_sync;

                d_state = STATE_SYNC_COMPLETE; // Synchronization is complete for this frame
                // No break needed, loop condition will be false and loop will exit

                if (d_debug) {
                    // Net id debug
                    int64_t symbol_start_abs = d_current_search_pos - (17 * d_sample_num / 4);
                    required_start_in_buffer = symbol_start_abs - abs_read_pos;
                    std::pair<float, int> pk_netid_1 = dechirp(in, required_start_in_buffer, true, d_sample_num, d_fft, d_fft_len, d_bin_num, d_upchirp, d_downchirp);
                    symbol_start_abs = d_current_search_pos - (13 * d_sample_num / 4);
                    required_start_in_buffer = symbol_start_abs - abs_read_pos;
                    std::pair<float, int> pk_netid_2 = dechirp(in, required_start_in_buffer, true, d_sample_num, d_fft, d_fft_len, d_bin_num, d_upchirp, d_downchirp);
                    // Net id calc
                    int netid_1 = (int(round((pk_netid_1.second + d_bin_num - d_preamble_bin) / (double)d_zero_padding_ratio))) % (1 << d_sf);
                    int netid_2 = (int(round((pk_netid_2.second + d_bin_num - d_preamble_bin) / (double)d_zero_padding_ratio))) % (1 << d_sf);
                    fprintf(stderr, "css_frame_sync_impl::work: Net id calculated:  %d, %d. !!!!!\n", netid_1, netid_2);
                }
            } break; // End of STATE_PAYLOAD_START_CALCULATING case

            case STATE_SYNC_COMPLETE:
                // We are done. Just pass data through. The loop will exit on the next iteration check.
                if (d_debug) {
                    fprintf(stderr, "css_frame_sync_impl::work: State SYNC_COMPLETE. Passing data through.\n");
                }
            break; // Exit switch, loop condition will be checked
        }
         // If we reached here AND the state is not SYNC_COMPLETE,
         // it means a state transition occurred and there might be enough
         // data in the current buffer to immediately process the next state.
         // The while loop condition re-checks the state and continues if possible.
         // If not enough data was available for the *current* state's operation,
         // the code would have already returned noutput_items inside the state block.
    }

    // Return the number of items consumed and produced (1:1 block)
     if (d_debug) {
        fprintf(stderr, "css_frame_sync_impl::work: Work finished. Returning %d items.\n", noutput_items);
     }
    return noutput_items;
}

} /* namespace cssmods */
} /* namespace gr */
