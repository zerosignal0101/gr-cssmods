/* -*- c++ -*- */
/*
 * Copyright 2025 zerosignal.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_CSSMODS_CSS_DEMODULATE_SYMBOLS_IMPL_H
#define INCLUDED_CSSMODS_CSS_DEMODULATE_SYMBOLS_IMPL_H

#include <gnuradio/cssmods/css_demodulate_symbols.h>
#include "css_modulate_impl.h" // For generate_lora_chirp definition (assuming it's in a header accessible via path)
#include <gnuradio/fft/fft.h>
#include <gnuradio/fft/window.h>
#include <volk/volk.h>
#include <complex>
#include <vector>
#include <cmath>
#include <numeric> // For std::accumulate
#include <algorithm> // For std::max_element
#include <cstdio> // For printf/fprintf

namespace gr {
namespace cssmods {

// Helper function for dechirping and finding peak
// Applies conjugated nominal chirp, performs FFT, finds peak in combined magnitude spectrum.
// input_buffer: Pointer to the start of the complex input buffer.
// buffer_offset_in_buffer: The 0-based index within input_buffer where the symbol starts.
// is_up: True for upchirp dechirp, false for downchirp dechirp.
// Returns {peak_magnitude, peak_bin_index_0_based} in the range [0, d_bin_num - 1].
std::pair<float, int> dechirp(const gr_complex *input_buffer, int64_t buffer_offset_in_buffer, bool is_up, int sample_num, 
    std::shared_ptr<gr::fft::fft_complex_fwd> fft, int fft_len, int bin_num,
    std::vector<std::complex<float>>& upchirp, std::vector<std::complex<float>>& downchirp);

// Debug helper
void print_complex_vector(const std::vector<gr_complex>& vec, const std::string& name, size_t max_print);
void print_float_vector(const std::vector<float>& vec, const std::string& name, size_t max_print = 10);

// State machine for synchronization process
enum sync_state_t {
    STATE_IDLE,                      // Initial state, searching hasn't started effectively
    STATE_SEARCHING_PREAMBLE,       // Searching for 6 preamble upchirp
    STATE_SEARCHING_DOWNCHIRP,       // Searching for the first sync downchirp symbol
    STATE_REFINING_POSITION,         // Refining the start index of the downchirp using its peak
    STATE_CFO_CALCULATING,           // Calculating CFO using the last preamble upchirp symbol
    STATE_PAYLOAD_START_CALCULATING, // Determining the exact payload start index based on sync word position
    STATE_SYNC_COMPLETE,             // Frame synchronization complete, tag added, pass data
    STATE_PDU_OUTPUT
};

class css_demodulate_symbols_impl : public css_demodulate_symbols
{
private:
    int d_sf;
    double d_bw;
    int d_zero_padding_ratio;
    int d_sample_num;       // Number of samples per non-padded symbol (2 * 2^sf for 2*BW sampling)
    int d_bin_num;          // Number of useful frequency bins (2^sf * zero_padding_ratio)
    int d_fft_len;          // FFT length (sample_num * zero_padding_ratio)
    int d_preamble_len;     // Premable length in modulation process
    std::vector<std::complex<float>> d_upchirp;   // Nominal upchirp (cfo=0)
    std::vector<std::complex<float>> d_downchirp; // Nominal downchirp (cfo=0)
    std::shared_ptr<gr::fft::fft_complex_fwd> d_fft; // FFT object for frequency-domain correlation
    bool d_debug;           // Flag to enable/disable debug output

    // State variables for synchronization
    sync_state_t d_state;
    int64_t d_current_search_pos; // Absolute index in the stream where the current state's operation is centered or searching from
    int d_preamble_bin;           // Peak bin index of the last preamble upchirp (used for CFO calc)
    double d_cfo;                 // Calculated Carrier Frequency Offset (Hz)
    int d_preamble_counter;
    std::vector<uint32_t> d_final_symbols;

public:
    css_demodulate_symbols_impl(int sf, double bw, int zero_padding_ratio);
    ~css_demodulate_symbols_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);
};

} // namespace cssmods
} // namespace gr

#endif /* INCLUDED_CSSMODS_CSS_DEMODULATE_SYMBOLS_IMPL_H */
