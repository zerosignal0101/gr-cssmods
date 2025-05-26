/* -*- c++ -*- */
/*
 * Copyright 2025 zerosignal.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_CSSMODS_CSS_SYMBOLS_DECODE_H
#define INCLUDED_CSSMODS_CSS_SYMBOLS_DECODE_H

#include <gnuradio/block.h>
#include <gnuradio/cssmods/api.h>

namespace gr {
namespace cssmods {

/*!
 * \brief <+description of block+>
 * \ingroup cssmods
 *
 */
class CSSMODS_API css_symbols_decode : virtual public gr::block
{
public:
    typedef std::shared_ptr<css_symbols_decode> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of cssmods::css_symbols_decode.
     *
     * To avoid accidental use of raw pointers, cssmods::css_symbols_decode's
     * constructor is in a private implementation
     * class. cssmods::css_symbols_decode::make is the public interface for
     * creating new instances.
     */
    static sptr make(int sf, int cr, bool ldr);
};

} // namespace cssmods
} // namespace gr

#endif /* INCLUDED_CSSMODS_CSS_SYMBOLS_DECODE_H */
