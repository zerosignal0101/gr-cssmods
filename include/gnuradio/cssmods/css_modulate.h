/* -*- c++ -*- */
/*
 * Copyright 2025 zerosignal.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_CSSMODS_CSS_MODULATE_H
#define INCLUDED_CSSMODS_CSS_MODULATE_H

#include <gnuradio/block.h>
#include <gnuradio/cssmods/api.h>

namespace gr {
namespace cssmods {

/*!
 * \brief <+description of block+>
 * \ingroup cssmods
 *
 */
class CSSMODS_API css_modulate : virtual public gr::block
{
public:
    typedef std::shared_ptr<css_modulate> sptr;

    /*!
     * \brief Return a shared_ptr to a new instance of cssmods::css_modulate.
     *
     * To avoid accidental use of raw pointers, cssmods::css_modulate's
     * constructor is in a private implementation
     * class. cssmods::css_modulate::make is the public interface for
     * creating new instances.
     */
    static sptr make(int sf, double bw, double fs, double h, double tdelta = 0.0);
};

} // namespace cssmods
} // namespace gr

#endif /* INCLUDED_CSSMODS_CSS_MODULATE_H */
