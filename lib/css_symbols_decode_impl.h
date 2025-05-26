/* -*- c++ -*- */
/*
 * Copyright 2025 zerosignal.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_CSSMODS_CSS_SYMBOLS_DECODE_IMPL_H
#define INCLUDED_CSSMODS_CSS_SYMBOLS_DECODE_IMPL_H

#include <gnuradio/cssmods/css_symbols_decode.h>

namespace gr {
namespace cssmods {

class css_symbols_decode_impl : public css_symbols_decode
{
private:
    int d_sf;
    int d_cr;
    bool d_ldr;

    std::vector<uint8_t> d_whitening_seq;

    // --- Auxiliary functions (transplanted from MATLAB) ---

    void gray_coding_impl(const std::vector<uint32_t>& din_const,
                          std::vector<uint16_t>& symbols_out);

    void diag_deinterleave_impl(const std::vector<uint16_t>& symbols_in,
                                int ppm,
                                std::vector<uint16_t>& codewords_out);
    
    uint8_t bit_reduce(uint16_t codeword, const std::vector<int>& zero_indexed_lsb_positions);
    uint16_t parity_fix(uint8_t p_syndrome);

    void hamming_decode_impl(const std::vector<uint16_t>& codewords_in,
                             int rdd,
                             std::vector<uint8_t>& nibbles_out);

    void dewhiten_impl(const std::vector<uint8_t>& bytes_in,
                       std::vector<uint8_t>& bytes_w_out);

public:
    css_symbols_decode_impl(int sf, int cr, bool ldr);
    ~css_symbols_decode_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);

    void work_on_pdu(pmt::pmt_t msg); // Renamed from 'work'
};

} // namespace cssmods
} // namespace gr

#endif /* INCLUDED_CSSMODS_CSS_SYMBOLS_DECODE_IMPL_H */
