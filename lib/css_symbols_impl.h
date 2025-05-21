/* -*- c++ -*- */
/*
 * Copyright 2025 zerosignal.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_CSSMODS_CSS_SYMBOLS_IMPL_H
#define INCLUDED_CSSMODS_CSS_SYMBOLS_IMPL_H

#include <gnuradio/cssmods/css_symbols.h>

namespace gr {
namespace cssmods {

/*!
 * \brief LoRa Physical Layer Symbol Encoding
 * \ingroup lora
 *
 * Converts a payload byte vector into a sequence of integer symbols
 * suitable for CSS modulation, implementing padding, whitening,
 * nibble extraction, diagonal interleaving, and gray decoding
 * as described by the provided Matlab code snippet.
 *
 * Input: Message containing pmt::pmt_vector_uint8 (the payload)
 * Output: Message containing pmt::pmt_vector_uint32 (the symbols)
 */
class css_symbols_impl : public css_symbols
{
private:
    int d_sf; // Spreading Factor (7-12)
    int d_cr; // Code Rate (1-4, corresponds to 4/5 to 4/8)
    bool d_ldr; // Low Data Rate Optimization flag
    std::vector<uint8_t> d_whitening_seq; // Pre-computed whitening sequence
    // Helper function to generate whitening sequence (using x^7 + x^5 + 1, seed 1)
    std::vector<uint8_t> generate_whitening_seq(size_t max_len);
    // Helper function to calculate total symbols based on payload length and params
    int calc_sym_num(size_t plen);
    // Helper function for diagonal interleaving
    // Input: vector of nibbles, redundancy (rdd)
    // Output: vector of interleaved symbols
    std::vector<uint32_t> diag_interleave_helper(const std::vector<uint8_t>& nibbles_block, int rdd);
    // Helper function for gray decoding (inverse gray + offset/modulo)
    // Input: vector of interleaved symbols (before final transform)
    // Output: vector of final symbols
    std::vector<uint32_t> gray_decoding_helper(const std::vector<uint32_t>& interleaved_symbols);

public:
    /*!
     * \brief Return a shared_ptr to a new instance of lora::css_symbols.
     * \param sf Spreading Factor (7-12)
     * \param cr Code Rate (1-4, corresponds to 4/5 to 4/8)
     * \param ldr Low Data Rate Optimization flag
     */
    css_symbols_impl(int sf, int cr, bool ldr);
    ~css_symbols_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);

    // Message handler for input payload
    void handle_payload_message(pmt::pmt_t msg);
};

} // namespace cssmods
} // namespace gr

#endif /* INCLUDED_CSSMODS_CSS_SYMBOLS_IMPL_H */
