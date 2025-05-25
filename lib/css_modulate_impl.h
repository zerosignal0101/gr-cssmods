/* -*- c++ -*- */
/*
 * Copyright 2025 zerosignal.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_CSSMODS_CSS_MODULATE_IMPL_H
#define INCLUDED_CSSMODS_CSS_MODULATE_IMPL_H

#include <gnuradio/cssmods/css_modulate.h>
#include <vector>
#include <complex>
#include <algorithm> // For std::min


namespace gr {
namespace cssmods {

// Helper function to generate a LoRa-like chirp symbol
std::vector<std::complex<float>> generate_lora_chirp(
    bool is_up,
    int sf,
    double bw,
    double fs,
    double h_in, // Use h_in to differentiate from rounded h
    double cfo,
    double tdelta = 0.0,
    double tscale = 1.0);

class css_modulate_impl : public css_modulate
{
private:
    bool d_debug;         // Debug flag

    int d_sf;             // spreading factor
    double d_bw;          // bandwidth
    double d_fs;          // sampling frequency
    int d_cr;             // code rate (not used in modulate func but good to store)
    int d_preamble_len;   // preamble length
    double d_cfo;         // carrier frequency offset
    // Calculated values based on parameters
    size_t d_chirp_len; // Number of samples per chirp
    size_t d_sfd_len;   // Number of samples in the short SFD part (chirp_len/4)
    size_t d_header_len; // Total length of preamble, netid, sfd

    // PDU
    std::deque<std::vector<uint32_t>> d_pdu_queue;
    std::vector<uint32_t> d_current_pdu;
    size_t d_pdu_offset;
    gr::thread::mutex d_mutex;
    void handle_pdu(pmt::pmt_t msg);

    // Constant time control
    std::chrono::time_point<std::chrono::steady_clock> d_last_time;
    double d_items_per_second;
    double d_items_per_us;
    int d_target_items;

    // Pre calculated chirp
    std::vector<std::complex<float>> d_upchirp;
    std::vector<std::complex<float>> d_downchirp;

public:
    css_modulate_impl(int sf, double bw, double fs, int cr, int preamble_len, double cfo);
    ~css_modulate_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);
};

} // namespace cssmods
} // namespace gr

#endif /* INCLUDED_CSSMODS_CSS_MODULATE_IMPL_H */
