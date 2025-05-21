/* -*- c++ -*- */
/*
 * Copyright 2025 zerosignal.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_CSSMODS_CSS_FRAME_SYNC_H
#define INCLUDED_CSSMODS_CSS_FRAME_SYNC_H

#include <gnuradio/cssmods/api.h>
#include <gnuradio/sync_block.h>

namespace gr {
namespace cssmods {

/*!
* \brief LoRa frame synchronization block.
* \ingroup css
*
* Uses LoRa preamble and sync word structure to find frame start.
* Adds a tag at the beginning of the payload.
*/
class CSSMODS_API css_frame_sync : virtual public gr::sync_block
{
public:
    typedef std::shared_ptr<css_frame_sync> sptr;

    /*!
    * \brief Return a shared_ptr to a new instance of css::frame_sync.
    *
    * \param sf Spreading factor (7-12).
    * \param bw Bandwidth (Hz).
    * \param zero_padding_ratio Zero padding ratio for FFT (e.g., 1, 2).
    */
    static sptr make(int sf, double bw, int zero_padding_ratio);
};

} // namespace cssmods
} // namespace gr

#endif /* INCLUDED_CSSMODS_CSS_FRAME_SYNC_H */
