/* -*- c++ -*- */
/*
 * Copyright 2025 zerosignal.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "css_modulate_impl.h"
#include <gnuradio/io_signature.h>
#include <vector>
#include <complex>
#include <cmath>    // For round, M_PI, cos, sin, atan2
#include <numeric>  // Potentially useful, but maybe not needed here
#include <stdexcept> // For exceptions
#include <algorithm> // For std::min
#include <cstring>   // For memcpy

namespace gr {
namespace cssmods {

css_modulate::sptr
css_modulate::make(int sf, double bw, double fs, int cr, int preamble_len, double cfo)
{
    return gnuradio::make_block_sptr<css_modulate_impl>(sf, bw, fs, cr, preamble_len, cfo);
}

// Helper function to generate a LoRa-like chirp symbol
std::vector<std::complex<float>> generate_lora_chirp(
    bool is_up,
    int sf,
    double bw,
    double fs,
    double h_in, // Use h_in to differentiate from rounded h
    double cfo,
    double tdelta,
    double tscale)
{
    // Validate inputs (simple checks)
    if (sf <= 0 || bw <= 0 || fs <= 0) {
        throw std::runtime_error("Invalid input parameters: sf, bw, and fs must be positive.");
    }
    if (h_in < 0) {
        // h can be rounded to 0, but negative input h doesn't make sense in the context 0..N-1
        throw std::runtime_error("Invalid input parameter: h cannot be negative.");
    }
    double N = std::pow(2.0, sf);
    double T = N / bw;
    int total_samples = static_cast<int>(round(fs * T)); // Target number of samples
    double h_orig = h_in;
    double h = round(h_orig); // Rounded h for segmentation
    // Adjust CFO based on h rounding
    cfo += (h_orig - h) / N * bw;
    double k; // Chirp slope
    double f0; // Baseband frequency at t=0 (including CFO)
    if (is_up) {
        k = bw / T;
        f0 = -bw / 2.0 + cfo;
    } else {
        k = -bw / T;
        f0 = bw / 2.0 + cfo;
    }
    // Determine sample counts for the two output segments based on rounded h
    // Segment 1: Corresponds to time T*h/N to T (in ideal chirp), goes first in output
    // Segment 2: Corresponds to time 0 to T*h/N (in ideal chirp), goes second in output
    // These calculations determine how many samples from the 'c1' and 'c2' style segments are used.
    // Ensure total sample count matches total_samples.
    int num_samps_seg1_output = static_cast<int>(round(static_cast<double>(total_samples) * (N - h) / N));
    int num_samps_seg2_output = total_samples - num_samps_seg1_output;
    // Calculate the number of samples needed for the intermediate c1 vector
    // This vector needs num_samps_seg1_output + 1 samples to get the last sample for phase calculation
    int num_c1_full = num_samps_seg1_output + 1;
    std::vector<std::complex<float>> y(total_samples);
    // --- Generate samples for the intermediate c1 vector (used for segment 1 output and phase phi) ---
    // Matches MATLAB's c1 calculation logic regarding time vector and phase formula
    std::vector<std::complex<float>> c1_vec(num_c1_full);
    for (int i = 0; i < num_c1_full; ++i) {
        double current_t = static_cast<double>(i) / fs * tscale + tdelta;
        // Phase calculation matching MATLAB's c1 argument: t .* (f0 + k*T*h/N + 0.5*k*t)
        double phase_arg = current_t * (f0 + k * T * h / N + 0.5 * k * current_t);
        double phase = 2.0 * M_PI * phase_arg;
        c1_vec[i] = std::complex<float>(cos(phase), sin(phase));
    }
    // --- Calculate phase 'phi' at the end of the c1 segment ---
    // Corresponds to the phase of the last sample in the full c1_vec
    double phi = 0.0;
    if (!c1_vec.empty()) {
         phi = atan2(c1_vec.back().imag(), c1_vec.back().real());
    }
    // --- Generate samples for the c2 vector (used for segment 2 output) ---
    // Matches MATLAB's c2 calculation logic regarding time vector and phase formula
    std::vector<std::complex<float>> c2_vec(num_samps_seg2_output);
     for (int i = 0; i < num_samps_seg2_output; ++i) {
        // Note: MATLAB's c2 time vector did *not* use tscale
        double current_t = static_cast<double>(i) / fs + tdelta; // Matches MATLAB: (0:len-1)/fs + tdelta
        // Phase calculation matching MATLAB's c2 argument: phi + 2*pi*(t.*(f0+0.5*k*t))
        double phase_arg = current_t * (f0 + 0.5 * k * current_t);
        double phase = phi + 2.0 * M_PI * phase_arg;
        c2_vec[i] = std::complex<float>(cos(phase), sin(phase));
    }
    // --- Concatenate samples into the final output vector 'y' ---
    // MATLAB output is [c1(1:end-1), c2]
    // Our y is [c1_vec(0:num_samps_seg1_output-1), c2_vec(0:num_samps_seg2_output-1)]
    // This exactly fills y to total_samples
    for (int i = 0; i < num_samps_seg1_output; ++i) {
        y[i] = c1_vec[i]; // Copy samples from the beginning of c1_vec
    }
     for (int i = 0; i < num_samps_seg2_output; ++i) {
        y[num_samps_seg1_output + i] = c2_vec[i]; // Copy samples from c2_vec
    }
    return y;
}


/*
 * The private constructor
 */
css_modulate_impl::css_modulate_impl(
    int sf, double bw, double fs, int cr, int preamble_len, double cfo)
    : gr::block("css_modulate",
                gr::io_signature::make(0, 0, 0), // No streaming input
                gr::io_signature::make(1, 1, sizeof(std::complex<float>))), // Output: complex samples
      d_debug(false),
      d_sf(sf),
      d_bw(bw),
      d_fs(fs),
      d_cr(cr), // Not used in modulate but stored
      d_preamble_len(preamble_len),
      d_cfo(cfo),
      d_pdu_queue(),
      d_current_pdu(),
      d_pdu_offset(0),
      d_last_time(std::chrono::steady_clock::now()),
      d_items_per_second(fs),
      d_items_per_us(fs / 1e6),
      d_target_items(0),
      d_upchirp(),
      d_downchirp(),
      d_state(IDLE), // Initialize state
      d_preamble_counter(0), // Initialize preamble counter
      d_netid_counter(0),    // Initialize NetID counter
      d_sfd_counter(0)       // Initialize SFD counter
{
    // Validate parameters if necessary
    if (sf < 7 || sf > 12) throw std::runtime_error("LoRa SF must be between 7 and 12");
    if (bw <= 0 || fs <= 0) throw std::runtime_error("Bandwidth and Sampling Frequency must be positive");
    if (preamble_len <= 0) throw std::runtime_error("Preamble length must be positive");
    // Calculate fixed lengths based on parameters
    // Generate a dummy chirp to get the length
    std::vector<std::complex<float>> dummy_uc = generate_lora_chirp(true, d_sf, d_bw, d_fs, 0, d_cfo);
    d_chirp_len = dummy_uc.size();
    if (d_chirp_len == 0) {
         throw std::runtime_error("generate_lora_chirp returned zero length");
    }
    d_sfd_len = static_cast<size_t>(std::round(d_chirp_len / 4.0));
    // Calculate fixed header length: preamble + netid + sfd
    // preamble: preamble_len * chirp_len
    // netid: 2 * chirp_len (for symbols 24 and 32)
    // sfd: 2 * chirp_len + sfd_len (three dc, third one truncated)
    d_header_len = d_preamble_len * d_chirp_len + // Preamble
                   2 * d_chirp_len +             // NetID (2 chirps)
                   2 * d_chirp_len + d_sfd_len;  // SFD (2 full DC + 1 partial DC)
    // PDU input
    message_port_register_in(pmt::mp("pdu"));
    set_msg_handler(pmt::mp("pdu"), [this](pmt::pmt_t msg) {
        this->handle_pdu(msg);
    });
    if (d_debug == true) {
        std::cout << "CSS modulated block initialized with:" << std::endl;
        std::cout << "  Sample rate: " << d_fs << " samples/sec" << std::endl;
    }
    // Pre calculate chirp
    // chirp (uc 和 dc)
    std::vector<std::complex<float>> uc = generate_lora_chirp(true, d_sf, d_bw, d_fs, 0, d_cfo);
    std::vector<std::complex<float>> dc = generate_lora_chirp(false, d_sf, d_bw, d_fs, 0, d_cfo);
    if (uc.size() != d_chirp_len || dc.size() != d_chirp_len) {
        throw std::runtime_error("Can not generate chirp for modulation!");
    }
    d_upchirp = uc;
    d_downchirp = dc;
}

/*
 * PDU input handler.
 */

void css_modulate_impl::handle_pdu(pmt::pmt_t msg) {
    gr::thread::scoped_lock guard(d_mutex);
    
    // Check if this is a valid PDU
    if (!pmt::is_pair(msg)) {
        std::cerr << "Received invalid PDU (not a pair)" << std::endl;
        return;
    }
    
    pmt::pmt_t metadata = pmt::car(msg);
    pmt::pmt_t vector = pmt::cdr(msg);
    
    if (!pmt::is_u32vector(vector)) {
        std::cerr << "Received invalid PDU (data not u32vector)" << std::endl;
        return;
    }
    
    size_t pdu_len = pmt::length(vector);
    const uint32_t* pdu_data = (const uint32_t*)pmt::u32vector_elements(vector, pdu_len);
    
    // Copy the PDU data to our queue
    std::vector<uint32_t> new_pdu(pdu_data, pdu_data + pdu_len);
    d_pdu_queue.push_back(new_pdu);
    
    if (d_debug == true) {
        std::cout << "Received new PDU with length " << pdu_len << " bytes" << std::endl;
        std::cout << "Current PDU queue size: " << d_pdu_queue.size() << std::endl;
    }
}


/*
 * Our virtual destructor.
 */
css_modulate_impl::~css_modulate_impl()
{
    // Clean up any resources if necessary
}

void css_modulate_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required)
{
    // No stream input added
}


int css_modulate_impl::general_work(int noutput_items,
                                    gr_vector_int& ninput_items,
                                    gr_vector_const_void_star& input_items,
                                    gr_vector_void_star& output_items) 
{
    if (d_debug) {
        std::cout << "\n=== WORK CALL START ===" << std::endl;
        std::cout << "[INIT] noutput_items=" << noutput_items 
                  << ", input_items.size()=" << input_items.size() << std::endl;
    }

    gr::thread::scoped_lock guard(d_mutex);
    std::complex<float>* out = reinterpret_cast<std::complex<float>*>(output_items[0]);
    
    // Timing calculation
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - d_last_time);
    double elapsed_seconds = elapsed.count() / 1e6;
    int target_items = static_cast<int>(std::round(elapsed_seconds * d_fs)) + d_target_items;
    // Clear temp target items
    d_target_items = 0;

    if (d_debug) {
        std::cout << "[TIMING] Elapsed=" << elapsed.count() << "us (" 
                  << elapsed_seconds << "s), target_items=" << target_items << std::endl;
    }

    int items_to_produce = std::min(target_items, noutput_items);
    if (items_to_produce <= 0) {
        if (d_debug) {
            std::cout << "[EARLY EXIT] Not enough time elapsed (0 items to produce)" << std::endl;
        }
        return 0;
    }

    if (d_debug) {
        std::cout << "[PRODUCTION] items_to_produce=" << items_to_produce 
                  << ", chirp_len=" << d_chirp_len << std::endl;
    }

    int produced = 0;
    while (produced < items_to_produce) {
        switch (d_state) {
            case IDLE: {
                // Start new PDU if needed
                if (d_current_pdu.empty() && !d_pdu_queue.empty()) {
                    d_current_pdu = d_pdu_queue.front();
                    d_pdu_queue.pop_front();
                    d_pdu_offset = 0;
                    d_state = PREAMBLE; // Transition to PREAMBLE state
                } else {
                    // No PDU, produce padding
                    size_t padding_size = (items_to_produce - produced);
                    if (d_debug && padding_size > 0) {
                        std::cout << "[STATE] IDLE. Padding " << padding_size << " samples." << std::endl;
                    }
                    for (size_t i = 0; i < padding_size; i++) {
                        out[produced + i] = {0.0f, 0.0f}; // Output zero samples
                    }
                    produced += padding_size;
                    // Stay in IDLE, consumed all available output space with padding or had nothing to do
                    goto complete_produce; // Exit switch, will likely exit while loop
                }
            }
            // Fallthrough to PREAMBLE state if new PDU is found
            [[fallthrough]]
            
            case PREAMBLE: {
                if ((items_to_produce - produced) < d_chirp_len) {
                    if (d_debug) {
                        std::cout << "[PREAMBLE] Insufficient space for single chirp (needs " 
                                << d_chirp_len << ", available " << (items_to_produce-produced) 
                                << ")" << std::endl;
                    }
                    goto complete_produce;
                }

                if (d_debug) {
                    std::cout << "[NEW PDU] Starting PDU (size=" << d_current_pdu.size() 
                            << "), queue remaining=" << d_pdu_queue.size() << std::endl;
                    std::cout << "  [PREAMBLE] Adding " << d_preamble_len << " upchirps" << std::endl;
                }

                std::memcpy(out + produced, d_upchirp.data(), d_chirp_len * sizeof(std::complex<float>));
                produced += d_chirp_len;

                d_preamble_counter += 1;

                // Check if all preamble chirps are finished
                if (d_preamble_counter >= d_preamble_len) {
                    d_state = NETID; // Transition to NETID
                    d_preamble_counter = 0;
                    if (d_debug) {
                        std::cout << "[STATE] PREAMBLE (" << d_preamble_len << "/" << d_preamble_len << ") -> NETID" << std::endl;
                    }
                }
            }
            break;

            case NETID: {
                if ((items_to_produce - produced) < d_chirp_len * 2) {
                    if (d_debug) {
                        std::cout << "[NETID] Insufficient space for 2 chirp (needs " 
                                << d_chirp_len * 2 << ", available " << (items_to_produce-produced) 
                                << ")" << std::endl;
                    }
                    goto complete_produce;
                }

                if (d_debug) {
                    std::cout << "  [NETID] Adding NetID chirps (24 & 32)" << std::endl;
                }

                // Add NetID
                std::vector<std::complex<float>> netid1_chirp = generate_lora_chirp(true, d_sf, d_bw, d_fs, 24, d_cfo);
                std::vector<std::complex<float>> netid2_chirp = generate_lora_chirp(true, d_sf, d_bw, d_fs, 32, d_cfo);
                std::memcpy(out + produced, netid1_chirp.data(), d_chirp_len * sizeof(std::complex<float>));
                produced += d_chirp_len;
                std::memcpy(out + produced, netid2_chirp.data(), d_chirp_len * sizeof(std::complex<float>));
                produced += d_chirp_len;

                d_state = SFD; // Transition to SFD
            }
            break;
            case SFD: {
                if ((items_to_produce - produced) < d_chirp_len * 2.25) {
                    if (d_debug) {
                        std::cout << "[NETID] Insufficient space for 2.25 chirp (needs " 
                                << d_chirp_len * 2.25 << ", available " << (items_to_produce-produced) 
                                << ")" << std::endl;
                    }
                    goto complete_produce;
                }
                if (d_debug) {
                    std::cout << "  [SFD] Adding SFD (2 downchirps + partial)" << std::endl;
                }
                // Add SFD
                std::memcpy(out + produced, d_downchirp.data(), d_chirp_len * sizeof(std::complex<float>));
                produced += d_chirp_len;
                std::memcpy(out + produced, d_downchirp.data(), d_chirp_len * sizeof(std::complex<float>));
                produced += d_chirp_len;
                std::memcpy(out + produced, d_downchirp.data(), d_sfd_len * sizeof(std::complex<float>));
                produced += d_sfd_len;

                d_state = PAYLOAD;
            }
            break;
            
            case PAYLOAD: {
                // Data processing
                int remaining_request = (items_to_produce - produced) / d_chirp_len;
                int remaining_pdu = d_current_pdu.empty() ? 0 : (d_current_pdu.size() - d_pdu_offset);
                int chunk_size = std::min(remaining_request, remaining_pdu);

                if (d_debug) {
                    std::cout << "[DATA] remaining_request=" << remaining_request 
                            << ", remaining_pdu=" << remaining_pdu 
                            << ", chunk_size=" << chunk_size << std::endl;
                }

                if (chunk_size > 0) {
                    if (d_debug) {
                        std::cout << "  [PROCESSING] Writing " << chunk_size << " data symbols (offset=" 
                                << d_pdu_offset << ")" << std::endl;
                    }
                    
                    for (int i = 0; i < chunk_size; ++i) {
                        double symbol = float(d_current_pdu[d_pdu_offset + i]);
                        if (d_debug) {
                            std::cout << "    [SYMBOL " << i << "] value=" << symbol;
                            if (i == 0) std::cout << " (first)";
                            if (i == chunk_size-1) std::cout << " (last)";
                            std::cout << std::endl;
                        }
                        
                        std::vector<std::complex<float>> data_chirp = generate_lora_chirp(true, d_sf, d_bw, d_fs, symbol, d_cfo);
                        if (data_chirp.size() != d_chirp_len) {
                            if (d_debug) {
                                std::cerr << "[ERROR] Invalid chirp size: " << data_chirp.size() 
                                        << " (expected " << d_chirp_len << ")" << std::endl;
                            }
                            return 0;
                        }
                        std::memcpy(out + produced, data_chirp.data(), d_chirp_len * sizeof(std::complex<float>));
                        produced += d_chirp_len;
                    }
                    d_pdu_offset += chunk_size;
                } 
                else if (remaining_pdu > 0) {
                    if (d_debug) {
                        std::cout << "[WAIT] PDU data available (remaining=" << remaining_pdu 
                                << ", queue=" << d_pdu_queue.size() 
                                << ") but no space for full chirp" << std::endl;
                    }
                    goto complete_produce;
                } 
                else {
                    d_state = SUFFIX; // Transition to SUFFIX
                }
            }
            break;

            case SUFFIX: {
                if ((items_to_produce - produced) < d_chirp_len) {
                    if (d_debug) {
                        std::cout << "[SUFFIX LEN] Insufficient space for single chirp (needs " 
                                << d_chirp_len << ", available " << (items_to_produce-produced) 
                                << ")" << std::endl;
                    }
                    break;
                }

                // Add an end down chirp
                std::memcpy(out + produced, d_downchirp.data(), d_chirp_len * sizeof(std::complex<float>));
                produced += d_chirp_len;

                // Check PDU completion
                if (!d_current_pdu.empty() && d_pdu_offset >= d_current_pdu.size()) {
                    if (d_debug) {
                        std::cout << "[PDU COMPLETE] Processed PDU (size=" 
                                << d_current_pdu.size() << "), offset=" << d_pdu_offset << std::endl;
                    }
                    d_current_pdu.clear();
                    d_pdu_offset = 0;
                } else {
                    std::cerr << "[SUFFIX] PDU not complete !!!" << std::endl;
                }

                d_state = IDLE;
            }
            break;

            default:
                // Should not happen
                std::cerr << "[ERROR] css_modulate_impl in unknown state: " << d_state << std::endl;
                d_state = IDLE; // Reset to a known state
                break; // Exit switch
        }

        
    } // end while

    complete_produce:;

    // Temporary save unused target_items
    if (produced < target_items) {
        d_target_items = target_items - produced;
        if (d_debug) {
            std::cout << "[SAVE] target_items " << d_target_items << std::endl;
        }
        // 检查处理速度是否过慢
        if (d_target_items > 1e6) {
            std::string error_msg = "Processing too slow! Unprocessed items (" + 
                                std::to_string(d_target_items) + 
                                ") exceed threshold (1,000,000). Cannot generate and send data in real-time.";
            std::cerr << "[WARNING] " << error_msg << std::endl;
            std::cerr << "[INFO] " << "Current state: " << d_state << std::endl;
            std::cerr << "[INFO] " << "PDU queue size: " << d_pdu_queue.size() << std::endl;
            std::cerr << "[INFO] " << "PDU Offset: " << d_pdu_offset << std::endl;
            std::cerr << "[INFO] " << "Current PDU size: " << d_current_pdu.size() << std::endl;
            std::cerr << "[INFO] " << "Current produced: " << produced << std::endl;
            std::cerr << "[INFO] " << "Current noutputitem: " << noutput_items << std::endl;
        }
    }


    // Update timing
    d_last_time = now;
    if (d_debug) {
        std::cout << "[WORK COMPLETE] Produced " << produced << "/" << items_to_produce 
                  << " items (total " << produced << ")" << std::endl;
        std::cout << "=== WORK CALL END ===\n" << std::endl;
    }
    return produced;
}



} /* namespace cssmods */
} /* namespace gr */
