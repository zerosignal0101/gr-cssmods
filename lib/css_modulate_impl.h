/* -*- c++ -*- */
/*
 * Copyright 2025 zerosignal.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_CSSMODS_CSS_MODULATE_IMPL_H
#define INCLUDED_CSSMODS_CSS_MODULATE_IMPL_H

#include <gnuradio/cssmods/css_modulate.h>

namespace gr {
namespace cssmods {

class css_modulate_impl : public css_modulate
{
private:
    // Nothing to declare in this block.

public:
    css_modulate_impl(int sf, double bw, double fs, double h, double tdelta);
    ~css_modulate_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);
};

} // namespace cssmods
} // namespace gr

#endif /* INCLUDED_CSSMODS_CSS_MODULATE_IMPL_H */
