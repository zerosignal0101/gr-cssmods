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

class css_symbols_impl : public css_symbols
{
private:
    // Nothing to declare in this block.

public:
    css_symbols_impl(int sf, int cr, bool ldr);
    ~css_symbols_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);
};

} // namespace cssmods
} // namespace gr

#endif /* INCLUDED_CSSMODS_CSS_SYMBOLS_IMPL_H */
