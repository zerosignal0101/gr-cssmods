/* -*- c++ -*- */
/*
 * Copyright 2025 zerosignal.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "css_modulate_impl.h"
#include <gnuradio/io_signature.h>
#include <vector>
#include <complex>
#include <cmath>    // For round, M_PI, cos, sin, atan2
#include <numeric>  // Potentially useful, but maybe not needed here
#include <stdexcept> // For exceptions

namespace gr {
namespace cssmods {

#pragma message("set the following appropriately and remove this warning")
using input_type = float;
#pragma message("set the following appropriately and remove this warning")
using output_type = float;
css_modulate::sptr
css_modulate::make(int sf, double bw, double fs, double h, double tdelta)
{
    return gnuradio::make_block_sptr<css_modulate_impl>(sf, bw, fs, h, tdelta);
}

// Helper function to generate a LoRa-like chirp symbol
std::vector<std::complex<float>> generate_lora_chirp(
    bool is_up,
    int sf,
    double bw,
    double fs,
    double h_in, // Use h_in to differentiate from rounded h
    double cfo,
    double tdelta = 0.0,
    double tscale = 1.0)
{
    // Validate inputs (simple checks)
    if (sf <= 0 || bw <= 0 || fs <= 0) {
        throw std::runtime_error("Invalid input parameters: sf, bw, and fs must be positive.");
    }
    if (h_in < 0) {
        // h can be rounded to 0, but negative input h doesn't make sense in the context 0..N-1
        throw std::runtime_error("Invalid input parameter: h cannot be negative.");
    }
    double N = std::pow(2.0, sf);
    double T = N / bw;
    int total_samples = static_cast<int>(round(fs * T)); // Target number of samples
    double h_orig = h_in;
    double h = round(h_orig); // Rounded h for segmentation
    // Adjust CFO based on h rounding
    cfo += (h_orig - h) / N * bw;
    double k; // Chirp slope
    double f0; // Baseband frequency at t=0 (including CFO)
    if (is_up) {
        k = bw / T;
        f0 = -bw / 2.0 + cfo;
    } else {
        k = -bw / T;
        f0 = bw / 2.0 + cfo;
    }
    // Determine sample counts for the two output segments based on rounded h
    // Segment 1: Corresponds to time T*h/N to T (in ideal chirp), goes first in output
    // Segment 2: Corresponds to time 0 to T*h/N (in ideal chirp), goes second in output
    // These calculations determine how many samples from the 'c1' and 'c2' style segments are used.
    // Ensure total sample count matches total_samples.
    int num_samps_seg1_output = static_cast<int>(round(static_cast<double>(total_samples) * (N - h) / N));
    int num_samps_seg2_output = total_samples - num_samps_seg1_output;
    // Calculate the number of samples needed for the intermediate c1 vector
    // This vector needs num_samps_seg1_output + 1 samples to get the last sample for phase calculation
    int num_c1_full = num_samps_seg1_output + 1;
    std::vector<std::complex<float>> y(total_samples);
    // --- Generate samples for the intermediate c1 vector (used for segment 1 output and phase phi) ---
    // Matches MATLAB's c1 calculation logic regarding time vector and phase formula
    std::vector<std::complex<float>> c1_vec(num_c1_full);
    for (int i = 0; i < num_c1_full; ++i) {
        double current_t = static_cast<double>(i) / fs * tscale + tdelta;
        // Phase calculation matching MATLAB's c1 argument: t .* (f0 + k*T*h/N + 0.5*k*t)
        double phase_arg = current_t * (f0 + k * T * h / N + 0.5 * k * current_t);
        double phase = 2.0 * M_PI * phase_arg;
        c1_vec[i] = std::complex<float>(cos(phase), sin(phase));
    }
    // --- Calculate phase 'phi' at the end of the c1 segment ---
    // Corresponds to the phase of the last sample in the full c1_vec
    double phi = 0.0;
    if (!c1_vec.empty()) {
         phi = atan2(c1_vec.back().imag(), c1_vec.back().real());
    }
    // --- Generate samples for the c2 vector (used for segment 2 output) ---
    // Matches MATLAB's c2 calculation logic regarding time vector and phase formula
    std::vector<std::complex<float>> c2_vec(num_samps_seg2_output);
     for (int i = 0; i < num_samps_seg2_output; ++i) {
        // Note: MATLAB's c2 time vector did *not* use tscale
        double current_t = static_cast<double>(i) / fs + tdelta; // Matches MATLAB: (0:len-1)/fs + tdelta
        // Phase calculation matching MATLAB's c2 argument: phi + 2*pi*(t.*(f0+0.5*k*t))
        double phase_arg = current_t * (f0 + 0.5 * k * current_t);
        double phase = phi + 2.0 * M_PI * phase_arg;
        c2_vec[i] = std::complex<float>(cos(phase), sin(phase));
    }
    // --- Concatenate samples into the final output vector 'y' ---
    // MATLAB output is [c1(1:end-1), c2]
    // Our y is [c1_vec(0:num_samps_seg1_output-1), c2_vec(0:num_samps_seg2_output-1)]
    // This exactly fills y to total_samples
    for (int i = 0; i < num_samps_seg1_output; ++i) {
        y[i] = c1_vec[i]; // Copy samples from the beginning of c1_vec
    }
     for (int i = 0; i < num_samps_seg2_output; ++i) {
        y[num_samps_seg1_output + i] = c2_vec[i]; // Copy samples from c2_vec
    }
    return y;
}


/*
 * The private constructor
 */
css_modulate_impl::css_modulate_impl(
    int sf, double bw, double fs, double h, double tdelta)
    : gr::block("css_modulate",
                gr::io_signature::make(
                    1 /* min inputs */, 1 /* max inputs */, sizeof(input_type)),
                gr::io_signature::make(
                    1 /* min outputs */, 1 /*max outputs */, sizeof(output_type)))
{
}

/*
 * Our virtual destructor.
 */
css_modulate_impl::~css_modulate_impl() {}

void css_modulate_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required)
{
#pragma message( \
    "implement a forecast that fills in how many items on each input you need to produce noutput_items and remove this warning")
    /* <+forecast+> e.g. ninput_items_required[0] = noutput_items */
}

int css_modulate_impl::general_work(int noutput_items,
                                    gr_vector_int& ninput_items,
                                    gr_vector_const_void_star& input_items,
                                    gr_vector_void_star& output_items)
{
    auto in = static_cast<const input_type*>(input_items[0]);
    auto out = static_cast<output_type*>(output_items[0]);

#pragma message("Implement the signal processing in your block and remove this warning")
    // Do <+signal processing+>
    // Tell runtime system how many input items we consumed on
    // each input stream.
    consume_each(noutput_items);

    // Tell runtime system how many output items we produced.
    return noutput_items;
}

} /* namespace cssmods */
} /* namespace gr */
