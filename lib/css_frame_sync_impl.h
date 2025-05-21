/* -*- c++ -*- */
/*
 * Copyright 2025 zerosignal.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_CSSMODS_CSS_FRAME_SYNC_IMPL_H
#define INCLUDED_CSSMODS_CSS_FRAME_SYNC_IMPL_H

#include <gnuradio/cssmods/css_frame_sync.h>
#include "css_modulate_impl.h" // For generate_lora_chirp
#include <gnuradio/fft/fft.h>
#include <gnuradio/fft/window.h>
#include <volk/volk.h>
#include <complex>
#include <vector>

namespace gr {
namespace cssmods {

// State machine for synchronization process
enum sync_state_t {
    STATE_IDLE,                      // Initial state, searching hasn't started effectively
    STATE_SEARCHING_DOWNCHIRP,       // Searching for the first sync downchirp
    STATE_REFINING_POSITION,         // Refining the start index using the downchirp peak
    STATE_CFO_CALCULATING,           // Calculating CFO using preamble upchirp
    STATE_PAYLOAD_START_CALCULATING, // Determining the exact payload start index
    STATE_SYNC_COMPLETE              // Frame synchronized, just pass data (or stop)
};

class css_frame_sync_impl : public css_frame_sync
{
private:
    int d_sf;
    double d_bw;
    int d_zero_padding_ratio;
    int d_sample_num;       // Number of samples per non-padded symbol (2 * 2^sf)
    int d_bin_num;          // Number of useful frequency bins (2^sf * zero_padding_ratio)
    int d_fft_len;          // FFT length (sample_num * zero_padding_ratio)
    std::vector<std::complex<float>> d_upchirp;   // Nominal upchirp (cfo=0)
    std::vector<std::complex<float>> d_downchirp; // Nominal downchirp (cfo=0)
    std::shared_ptr<gr::fft::fft_complex_fwd> d_fft; // FFT object
    // State variables for synchronization
    sync_state_t d_state;
    int64_t d_current_search_pos; // Absolute index in the stream
    int d_preamble_bin;           // Peak bin index of the preamble upchirp (for CFO calc)
    double d_cfo;                 // Calculated Carrier Frequency Offset (Hz)
    // Helper function for dechirping and finding peak
    // Returns (peak_magnitude, peak_bin_index_0_based)
    std::pair<float, int> dechirp(const gr_complex *input_buffer, size_t buffer_offset_in_buffer, bool is_up);

public:
    css_frame_sync_impl(int sf, double bw, int zero_padding_ratio);
    ~css_frame_sync_impl();

    // Where all the action really happens
    int work(int noutput_items,
             gr_vector_const_void_star& input_items,
             gr_vector_void_star& output_items);
};

} // namespace cssmods
} // namespace gr

#endif /* INCLUDED_CSSMODS_CSS_FRAME_SYNC_IMPL_H */
