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
    double tdelta = 0.0,
    double tscale = 1.0)
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
                gr::io_signature::make(1, 1, sizeof(int)), // Input: symbols (integers)
              gr::io_signature::make(1, 1, sizeof(std::complex<float>))), // Output: complex samples
      d_sf(sf),
      d_bw(bw),
      d_fs(fs),
      d_cr(cr), // Not used in modulate but stored
      d_preamble_len(preamble_len),
      d_cfo(cfo)
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
    // Optional: Register ports for messages/control signals if needed
    // message_port_register_in(pmt::mp("msg_in"));
    // message_port_register_out(pmt::mp("msg_out"));
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
    // Calculate how many input items are needed to produce noutput_items
    // Formula derived from: noutput_items = d_header_len + ninput_items * d_chirp_len
    // So: ninput_items = (noutput_items - d_header_len) / d_chirp_len

    // Ensure we have space for at least the header
    if (noutput_items <= d_header_len) {
        // If output is less than header length, no symbols are needed (just header)
        ninput_items_required[0] = 0;
    } else {
        // Calculate required input items, rounding up to ensure we have enough
        ninput_items_required[0] = static_cast<int>(
            std::ceil(static_cast<float>(noutput_items - d_header_len) / d_chirp_len)
        );
    }
}


int css_modulate_impl::general_work(int noutput_items,
                                    gr_vector_int& ninput_items,
                                    gr_vector_const_void_star& input_items,
                                    gr_vector_void_star& output_items)
{
    // Get pointers to input and output buffers (with type casting)
    const int* in = reinterpret_cast<const int*>(input_items[0]);
    std::complex<float>* out = reinterpret_cast<std::complex<float>*>(output_items[0]);

    // --- 逻辑核心调整 ---
    // 旧版：total_out_samples_to_write = d_header_len + ninput_items * d_chirp_len
    // 新版：根据 noutput_items 反推能处理的输入项数
    int max_possible_input_items = 0;
    if (noutput_items > d_header_len) {
        max_possible_input_items = (noutput_items - d_header_len) / d_chirp_len;
    }

    // 实际处理的输入项数不能超过可用的输入项数
    int actual_input_items = std::min(max_possible_input_items, ninput_items[0]);

    // 如果输出空间不足（actual_input_items == 0），返回 0 等待下次调用
    if (actual_input_items <= 0) {
        return 0;
    }

    // 实际输出的总长度（需 <= noutput_items）
    size_t total_out_samples_to_write = d_header_len + actual_input_items * d_chirp_len;

    // --- MATLAB 逻辑翻译（保持不变）---
    // 1. 生成基 chirp (uc 和 dc)
    std::vector<std::complex<float>> uc = generate_lora_chirp(true, d_sf, d_bw, d_fs, 0, d_cfo);
    std::vector<std::complex<float>> dc = generate_lora_chirp(false, d_sf, d_bw, d_fs, 0, d_cfo);
    if (uc.size() != d_chirp_len || dc.size() != d_chirp_len) {
        return 0; // 错误处理
    }

    size_t current_out_offset = 0;
    // 2. 添加前导码 (preamble_len * uc)
    for (int i = 0; i < d_preamble_len; ++i) {
        std::memcpy(out + current_out_offset, uc.data(), d_chirp_len * sizeof(std::complex<float>));
        current_out_offset += d_chirp_len;
    }

    // 3. 添加 NetID (uc_24, uc_32)
    std::vector<std::complex<float>> netid1_chirp = generate_lora_chirp(true, d_sf, d_bw, d_fs, 24, d_cfo);
    std::vector<std::complex<float>> netid2_chirp = generate_lora_chirp(true, d_sf, d_bw, d_fs, 32, d_cfo);
    std::memcpy(out + current_out_offset, netid1_chirp.data(), d_chirp_len * sizeof(std::complex<float>));
    current_out_offset += d_chirp_len;
    std::memcpy(out + current_out_offset, netid2_chirp.data(), d_chirp_len * sizeof(std::complex<float>));
    current_out_offset += d_chirp_len;

    // 4. 添加 SFD (dc, dc, dc[1:chirp_len/4])
    std::memcpy(out + current_out_offset, dc.data(), d_chirp_len * sizeof(std::complex<float>));
    current_out_offset += d_chirp_len;
    std::memcpy(out + current_out_offset, dc.data(), d_chirp_len * sizeof(std::complex<float>));
    current_out_offset += d_chirp_len;
    std::memcpy(out + current_out_offset, dc.data(), d_sfd_len * sizeof(std::complex<float>));
    current_out_offset += d_sfd_len;

    // 5. 添加数据符号（仅处理 actual_input_items 个）
    for (int i = 0; i < actual_input_items; ++i) {
        int symbol = in[i];
        std::vector<std::complex<float>> data_chirp = generate_lora_chirp(true, d_sf, d_bw, d_fs, symbol, d_cfo);
        if (data_chirp.size() != d_chirp_len) {
            return 0;
        }
        std::memcpy(out + current_out_offset, data_chirp.data(), d_chirp_len * sizeof(std::complex<float>));
        current_out_offset += d_chirp_len;
    }

    // --- 结束逻辑翻译 ---
    // 消耗输入项并返回生成的输出项数
    consume_each(actual_input_items);
    return total_out_samples_to_write;
}


} /* namespace cssmods */
} /* namespace gr */
