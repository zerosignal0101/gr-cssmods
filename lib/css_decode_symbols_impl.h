/* -*- c++ -*- */
/*
 * Copyright 2025 zerosignal.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_CSSMODS_CSS_DECODE_SYMBOLS_IMPL_H
#define INCLUDED_CSSMODS_CSS_DECODE_SYMBOLS_IMPL_H

#include <gnuradio/cssmods/css_decode_symbols.h>
#include "css_modulate_impl.h"
#include "css_frame_sync_impl.h"
#include <gnuradio/fft/fft.h>
#include <pmt/pmt.h>
#include <vector>
#include <complex>

namespace gr {
namespace cssmods {

class css_decode_symbols_impl : public css_decode_symbols
{
private:
    // Parameters
    int d_sf;
    double d_bw;
    int d_zero_padding_ratio;

    // Derived parameters
    int d_sample_num;       // Samples per symbol
    int d_bin_num;          // Number of useful frequency bins after padding
    int d_fft_len;          // FFT length

    // Chirps & FFT
    std::vector<std::complex<float>> d_upchirp;
    std::vector<std::complex<float>> d_downchirp; // Though primarily upchirp is used for payload
    std::shared_ptr<gr::fft::fft_complex_fwd> d_fft;

    // State variables
    bool d_decoding_active;
    int64_t d_current_search_pos; // Absolute index in the stream where the current state's operation is centered or searching from
    int d_preamble_bin;           // Peak bin index of the last preamble upchirp (used for CFO calc)
    double d_cfo;                 // Calculated Carrier Frequency Offset (Hz)

    bool d_debug; // For debug prints

public:
    css_decode_symbols_impl(int sf, double bw, int zero_padding_ratio);
    ~css_decode_symbols_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);
};

} // namespace cssmods
} // namespace gr

#endif /* INCLUDED_CSSMODS_CSS_DECODE_SYMBOLS_IMPL_H */
