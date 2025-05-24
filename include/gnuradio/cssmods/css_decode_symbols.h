/* -*- c++ -*- */
/*
 * Copyright 2025 zerosignal.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_CSSMODS_CSS_DECODE_SYMBOLS_H
#define INCLUDED_CSSMODS_CSS_DECODE_SYMBOLS_H

#include <gnuradio/block.h>
#include <gnuradio/cssmods/api.h>

namespace gr {
namespace cssmods {

/**
 * @brief LoRa CSS Symbol Decoder.
 *
 * This block takes a complex stream with 'payload_start' tags from
 * css_frame_sync and demodulates LoRa symbols into a byte stream.
 * Each byte in the output stream represents one demodulated symbol.
 *
 * The 'payload_start' tag on the input stream MUST have a dictionary
 * value containing an integer item with key "preamble_bin", which is
 * the reference preamble bin index determined by the frame synchronizer.
 *
 * Input: gr_complex stream with 'payload_start' tags.
 * Output: unsigned char stream (symbol values).
 */
class CSSMODS_API css_decode_symbols : virtual public gr::block
{
public:
    typedef std::shared_ptr<css_decode_symbols> sptr;

    /**
     * @brief Make a css_decode_symbols block.
     *
     * @param sf Spreading Factor.
     * @param bw Bandwidth in Hz.
     * @param zero_padding_ratio Zero padding ratio used in FFT.
     * @return Pointer to the new css_decode_symbols block.
     */
    static sptr make(int sf, double bw, int zero_padding_ratio);
};

} // namespace cssmods
} // namespace gr

#endif /* INCLUDED_CSSMODS_CSS_DECODE_SYMBOLS_H */
