/* -*- c++ -*- */
/*
 * Copyright 2025 zerosignal.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_CSSMODS_CSS_DEMODULATE_SYMBOLS_H
#define INCLUDED_CSSMODS_CSS_DEMODULATE_SYMBOLS_H

#include <gnuradio/block.h>
#include <gnuradio/cssmods/api.h>

namespace gr {
namespace cssmods {

/*!
* \brief LoRa frame synchronization block.
* \ingroup css
*
* Uses LoRa preamble and sync word structure to find frame start.
* Output the demodulated symbols as pdu.
*/
class CSSMODS_API css_demodulate_symbols : virtual public gr::block
{
public:
    typedef std::shared_ptr<css_demodulate_symbols> sptr;

    /*!
    * \brief Return a shared_ptr to a new instance of css_demodulate_symbols.
    *
    * \param sf Spreading factor (7-12).
    * \param bw Bandwidth (Hz).
    * \param zero_padding_ratio Zero padding ratio for FFT (e.g., 1, 2).
    */
    static sptr make(int sf, double bw, int zero_padding_ratio);
};

} // namespace cssmods
} // namespace gr

#endif /* INCLUDED_CSSMODS_CSS_DEMODULATE_SYMBOLS_H */
