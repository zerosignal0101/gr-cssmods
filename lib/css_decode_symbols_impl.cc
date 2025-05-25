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
      d_current_search_pos(0), // Absolute stream index
      d_preamble_bin(-1),
      d_temp_data(d_sample_num, 0),
      d_temp_pos(0),
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

    set_history(1 * d_sample_num);
    if (d_debug) {
        fprintf(stderr, "css_decode_symbols_impl: History set to %zu samples.\n", this->history());
    }
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
    int produced_count = 0;
    uint64_t abs_read_pos = nitems_read(0);
    size_t current_buffer_length = ninput_items[0];
    uint64_t abs_end_pos = abs_read_pos + current_buffer_length;

    std::vector<tag_t> tags;
    if (d_debug) {
        fprintf(stderr, "css_decode_symbols_impl: Search tags from %lu (input idx %d).\n", 
            abs_read_pos, current_buffer_length);
    }
    get_tags_in_window(tags, 0, 0, current_buffer_length);
    for (const auto &tag : tags) {
        if (d_debug) {
            fprintf(stderr, "css_decode_symbols_impl: Received '%s' tag at abs_pos %lu (input idx %d).\n", pmt::symbol_to_string(tag.key), nitems_read(0), ninput_items[0]);
        }
    }

    for (const auto& tag : tags) {
        // if (d_debug) {
        //     fprintf(stderr, "css_decode_symbols_impl: Received '%s' tag at abs_pos %lu (input idx %d).\n", pmt::symbol_to_string(tag.key), nitems_read(0), ninput_items[0]);
        // }
        if (tag.key == pmt::string_to_symbol("payload_start")) {
            if (d_debug) {
                fprintf(stderr, "css_decode_symbols_impl: Received 'payload_start' tag at abs_pos %lu (input idx %d).\n", nitems_read(0), ninput_items[0]);
            }
            d_decoding_active = true;
            d_preamble_bin = -1; // Reset

            if (d_debug) {
                fprintf(stderr, "css_decode_symbols_impl:Get tag offset %ld (Buffer start idx %ld).\n", tag.offset, abs_read_pos);
            }

            add_item_tag(0, // port
                    nitems_written(0) + produced_count, // abs offset on output stream
                    pmt::string_to_symbol("payload_start"),
                    pmt::make_dict());
        
            if (d_debug) {
                fprintf(stderr, "  css_decode_symbols_impl: Current search pos update from %ld to tag offset: %ld\n",
                        d_current_search_pos, tag.offset);
            }

            d_current_search_pos = tag.offset;

            if (d_debug) {
                // // Data copy
                // int64_t required_start_in_buffer = d_current_search_pos - d_sample_num - abs_read_pos; // Offset from 'in'
                // std::vector<gr_complex> dechirped_symbol(d_sample_num * 4, 0);
                // memcpy(dechirped_symbol.data(), 
                //     in_ptr + required_start_in_buffer, 
                //     d_sample_num * 4 * sizeof(gr_complex));
                // print_complex_vector(dechirped_symbol, "Dechirped symbol around d_current_search_pos +-d_sample_num (ori)", 1024);

                // // 测试 d_current_search_pos 附近的偏移
                // fprintf(stderr, "\nTesting around d_current_search_pos (%ld):\n", d_current_search_pos);
                // for (int offset = -d_sample_num; offset <= d_sample_num; offset += 8) {
                //     int64_t current_symbol_start_in_buffer = d_current_search_pos + offset - abs_read_pos;
                //     std::pair<float, int> up_peak = dechirp(in_ptr, current_symbol_start_in_buffer, true, d_sample_num, d_fft, d_fft_len, 
                //                     d_bin_num, d_upchirp, d_downchirp);
                //     fprintf(stderr, "Offset %+4d: pos %6ld, mag %8.2f, bin %2d\n", 
                //             offset, current_symbol_start_in_buffer, up_peak.first, up_peak.second);
                // }
            }

            if (pmt::is_dict(tag.value)) {
                d_preamble_bin = pmt::to_long(pmt::dict_ref(tag.value, pmt::string_to_symbol("preamble_bin"), pmt::from_long(-1)));
                if (d_preamble_bin == -1) {
                    if (d_debug) {
                        fprintf(stderr, "css_decode_symbols_impl: Error: 'payload_start' tag missing valid 'preamble_bin'. Use zero for demodulate afterwards\n");
                    }
                    // d_decoding_active = false; // Disable decoding if tag is bad
                    d_preamble_bin = 0;
                } else {
                    if (d_debug) {
                        fprintf(stderr, "  Extracted preamble_bin_ref: %d\n", d_preamble_bin);
                    }
                }
                d_cfo = pmt::to_double(pmt::dict_ref(tag.value, pmt::string_to_symbol("cfo"), pmt::from_double(0.0)));
                // d_sf d_bw can also be assessed from tag dict. TODO
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
        while (d_current_search_pos + d_sample_num <= abs_end_pos) {
            int64_t required_start_in_buffer = d_current_search_pos - abs_read_pos; // Offset from 'in'

            // std::vector<gr_complex> dechirped_symbol(d_sample_num, 0);
            // memcpy(dechirped_symbol.data(), 
            //     in_ptr + required_start_in_buffer, 
            //     d_sample_num * sizeof(gr_complex));
            // print_complex_vector(dechirped_symbol, "Dechirped symbol (ori) Decode payload 1", 256);

            // Dechirp the collected symbol (payload symbols are typically upchirps)
            std::pair<float, int> peak_info = dechirp(
                in_ptr, required_start_in_buffer, true, // Pass true for upchirp dechirp if payload is upchirp based
                d_sample_num, d_fft, d_fft_len, d_bin_num,
                d_upchirp, d_downchirp); // Note: you use both chirps in generate_lora_chirp, ensure dechirp uses correct one
            int symbol_value = (int(round((peak_info.second + d_bin_num - d_preamble_bin) / (double)d_zero_padding_ratio))) % (1 << d_sf);
            // // Ensure positive modulo result if round() results in a negative value before adding d_bin_num (less likely with the raw_symbol_val adjustment)
            // // The previous calculation `(static_cast<double>(current_symbol_peak_bin - d_preamble_bin_ref + d_bin_num)) / d_zero_padding_ratio` is also a common way and ensures positive intermediate before division. Let's stick to that one as it seems intended.
            // // Revert symbol calculation to original but ensure modulo is positive:
            // raw_symbol_val = (static_cast<double>(current_symbol_peak_bin - d_preamble_bin + d_bin_num)); // Intermediate might be large but positive
            // symbol_value = static_cast<int>(round(raw_symbol_val / d_zero_padding_ratio));
            // symbol_value = symbol_value % (1 << d_sf); // Apply modulo
            // if (symbol_value < 0) { // Ensure positive result of modulo
            //     symbol_value += (1 << d_sf);
            // }
            out_ptr[produced_count++] = static_cast<unsigned char>(symbol_value);
            if (d_debug) {
                fprintf(stderr, "  Decoded symbol %d: input_idx range [%d, %d), read pos %ld, peak_bin=%d, preamble_ref=%d, val_calc_raw=%d (Mag: %.2f)\n",
                        produced_count - 1, d_current_search_pos, d_current_search_pos + d_sample_num, required_start_in_buffer, peak_info.second, d_preamble_bin, symbol_value, peak_info.first);
            }
            d_current_search_pos += d_sample_num;
        }
        if (d_debug) {
        fprintf(stderr, "Decoder: Symbol extract done. Produced %d items (avail: %d). Current pos %ld < %ld end buffer. Decoding: %s.\n",
                produced_count, noutput_items, d_current_search_pos, abs_end_pos, d_decoding_active ? "Yes" : "No");
        }
    } 
    if (d_debug) {
        fprintf(stderr, "Decoder: Work done. Consumed %d input items (avail: %d), Produced %d items (avail: %d). Decoding: %s.\n",
                current_buffer_length, current_buffer_length, produced_count, noutput_items, d_decoding_active ? "Yes" : "No");
    }
    // This consumes the samples that were successfully processed within the loop (up to consumed_count).
    consume_each(current_buffer_length);
    // Return the number of output items produced in this call.
    return produced_count;
}

} /* namespace cssmods */
} /* namespace gr */
