/* -*- c++ -*- */
/*
 * Copyright 2025 zerosignal.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "css_symbols_decode_impl.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace cssmods {

template<typename T>
void print_int_vector(const std::vector<T>& vec, 
                    const std::string& name, 
                    size_t max_print) 
{
    std::cout << "----- " << name << " (size=" << vec.size() << ") -----" << std::endl;
    
    std::cout << "MATLAB format: [";
    size_t print_count = std::min(max_print, vec.size());
    for (size_t i = 0; i < print_count; ++i) {
        std::cout << static_cast<int>(vec[i]);
        if (i < print_count - 1) {
            std::cout << ", ";
        }
    }
    
    if (vec.size() > max_print) {
        std::cout << ", ...";
    }
    
    std::cout << "]" << std::endl;
    
    if (vec.size() > max_print) {
        std::cout << "... (showing first " << max_print << " of " << vec.size() << " elements)" << std::endl;
    }
    
    std::cout << "------------------------" << std::endl;
}


// Optional helper to print binary representations (like MATLAB's print_bin)
void print_binary_vector(const std::string& label, const std::vector<uint16_t>& data, int bits_per_symbol) {
    std::cout << label << ":" << std::endl;
    if (data.empty()) {
        std::cout << "  (empty)" << std::endl;
        return;
    }
    for (uint16_t val : data) {
        std::cout << "  ";
        for (int i = bits_per_symbol - 1; i >= 0; --i) {
            std::cout << ((val >> i) & 1);
        }
        std::cout << " (" << val << ")" << std::endl;
    }
}


css_symbols_decode::sptr css_symbols_decode::make(int sf, int cr, bool ldr)
{
    return gnuradio::make_block_sptr<css_symbols_decode_impl>(sf, cr, ldr);
}


/*
 * The private constructor
 */
css_symbols_decode_impl::css_symbols_decode_impl(int sf, int cr, bool ldr)
    : gr::block("css_symbols_decode",
                gr::io_signature::make(0, 0, 0), // No stream ports
                gr::io_signature::make(0, 0, 0)),
    d_sf(sf),
    d_cr(cr),
    d_ldr(ldr),
    d_debug(true)
{
    // Validate parameters
    if (d_sf < 7 || d_sf > 12) {
        throw std::invalid_argument("SF must be between 7 and 12");
    }
    if (d_cr < 1 || d_cr > 4) {
        // CR is 1-4, corresponds to 4/5 - 4/8
        throw std::invalid_argument("CR must be between 1 and 4");
    }
    // Register message ports
    message_port_register_in(pmt::mp("in"));
    message_port_register_out(pmt::mp("out"));
    set_msg_handler(pmt::mp("in"),
                    [this](pmt::pmt_t msg) { this->work_on_pdu(msg); });

    // Pre calculated whitening sequence
    d_whitening_seq = {
        0xff, 0xfe, 0xfc, 0xf8, 0xf0, 0xe1, 0xc2, 0x85, 0xb,  0x17, 0x2f, 0x5e, 0xbc,
        0x78, 0xf1, 0xe3, 0xc6, 0x8d, 0x1a, 0x34, 0x68, 0xd0, 0xa0, 0x40, 0x80, 0x1,
        0x2,  0x4,  0x8,  0x11, 0x23, 0x47, 0x8e, 0x1c, 0x38, 0x71, 0xe2, 0xc4, 0x89,
        0x12, 0x25, 0x4b, 0x97, 0x2e, 0x5c, 0xb8, 0x70, 0xe0, 0xc0, 0x81, 0x3,  0x6,
        0xc,  0x19, 0x32, 0x64, 0xc9, 0x92, 0x24, 0x49, 0x93, 0x26, 0x4d, 0x9b, 0x37,
        0x6e, 0xdc, 0xb9, 0x72, 0xe4, 0xc8, 0x90, 0x20, 0x41, 0x82, 0x5,  0xa,  0x15,
        0x2b, 0x56, 0xad, 0x5b, 0xb6, 0x6d, 0xda, 0xb5, 0x6b, 0xd6, 0xac, 0x59, 0xb2,
        0x65, 0xcb, 0x96, 0x2c, 0x58, 0xb0, 0x61, 0xc3, 0x87, 0xf,  0x1f, 0x3e, 0x7d,
        0xfb, 0xf6, 0xed, 0xdb, 0xb7, 0x6f, 0xde, 0xbd, 0x7a, 0xf5, 0xeb, 0xd7, 0xae,
        0x5d, 0xba, 0x74, 0xe8, 0xd1, 0xa2, 0x44, 0x88, 0x10, 0x21, 0x43, 0x86, 0xd,
        0x1b, 0x36, 0x6c, 0xd8, 0xb1, 0x63, 0xc7, 0x8f, 0x1e, 0x3c, 0x79, 0xf3, 0xe7,
        0xce, 0x9c, 0x39, 0x73, 0xe6, 0xcc, 0x98, 0x31, 0x62, 0xc5, 0x8b, 0x16, 0x2d,
        0x5a, 0xb4, 0x69, 0xd2, 0xa4, 0x48, 0x91, 0x22, 0x45, 0x8a, 0x14, 0x29, 0x52,
        0xa5, 0x4a, 0x95, 0x2a, 0x54, 0xa9, 0x53, 0xa7, 0x4e, 0x9d, 0x3b, 0x77, 0xee,
        0xdd, 0xbb, 0x76, 0xec, 0xd9, 0xb3, 0x67, 0xcf, 0x9e, 0x3d, 0x7b, 0xf7, 0xef,
        0xdf, 0xbf, 0x7e, 0xfd, 0xfa, 0xf4, 0xe9, 0xd3, 0xa6, 0x4c, 0x99, 0x33, 0x66,
        0xcd, 0x9a, 0x35, 0x6a, 0xd4, 0xa8, 0x51, 0xa3, 0x46, 0x8c, 0x18, 0x30, 0x60,
        0xc1, 0x83, 0x7,  0xe,  0x1d, 0x3a, 0x75, 0xea, 0xd5, 0xaa, 0x55, 0xab, 0x57,
        0xaf, 0x5f, 0xbe, 0x7c, 0xf9, 0xf2, 0xe5, 0xca, 0x94, 0x28, 0x50, 0xa1, 0x42,
        0x84, 0x9,  0x13, 0x27, 0x4f, 0x9f, 0x3f, 0x7f
    };
}

/*
 * Our virtual destructor.
 */
css_symbols_decode_impl::~css_symbols_decode_impl() {}

// --- Auxiliary function implementations ---

void css_symbols_decode_impl::gray_coding_impl(
    const std::vector<uint32_t>& din_const, std::vector<uint16_t>& symbols_out)
{
    // Make a mutable copy for scaling operations
    std::vector<uint32_t> din = din_const; 
    symbols_out.resize(din.size());

    for (size_t i = 0; i < din.size(); ++i) {
        if (d_ldr) {
            din[i] = static_cast<uint32_t>(std::floor(static_cast<double>(din[i]) / 4.0));
        } else {
            // Ensure positive result for modulo of potentially negative (din[i]-1)
            long val = static_cast<long>(din[i]) - 1;
            long mod_val = 1L << d_sf; // 2^sf
            din[i] = static_cast<uint32_t>((val % mod_val + mod_val) % mod_val);
        }
    }

    for (size_t i = 0; i < din.size(); ++i) {
        uint16_t s_val = static_cast<uint16_t>(din[i]);
        symbols_out[i] = s_val ^ (s_val >> 1);
    }
}

std::vector<uint16_t> css_symbols_decode_impl::diag_deinterleave_impl(
    const std::vector<uint16_t>& symbols, int ppm)
{
    if (symbols.empty() || ppm <= 0) {
        return {}; // Return empty if invalid input
    }

    size_t num_input_symbols = symbols.size(); // N in MATLAB context

    // 1. de2bi equivalent: Convert symbols to binary matrix `b_matrix` (N x ppm)
    // Each inner vector is a row representing a symbol's bits (MSB first)
    std::vector<std::vector<int>> b_matrix(num_input_symbols, std::vector<int>(ppm));
    for (size_t i = 0; i < num_input_symbols; ++i) {
        uint16_t current_symbol = symbols[i];
        for (int j = ppm - 1; j >= 0; --j) {
            b_matrix[i][ppm - 1 - j] = (current_symbol >> j) & 1;
        }
    }

    // For debugging: Print b_matrix
    // std::cout << "b_matrix (N x ppm):" << std::endl;
    // for (size_t i = 0; i < num_input_symbols; ++i) {
    //     std::cout << "  ";
    //     for (int j = 0; j < ppm; ++j) {
    //         std::cout << b_matrix[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // 2. circshift equivalent and cell2mat: Create `shifted_b_matrix` (N x ppm)
    std::vector<std::vector<int>> shifted_b_matrix(num_input_symbols, std::vector<int>(ppm));
    for (size_t i = 0; i < num_input_symbols; ++i) {
        // MATLAB x is 1-indexed, so x = i + 1
        // Shift amount is 1 - x = 1 - (i + 1) = -i
        // Negative shift means left circular shift.
        // Positive shift means right circular shift.
        // circshift(row, k): if k > 0, shifts right; if k < 0, shifts left.
        // A shift of -i (left by i) is equivalent to a right shift of (ppm - (i % ppm)) % ppm if i > 0
        // Or more simply: new_idx = (old_idx - shift_amount + ppm) % ppm for right shift
        // new_idx = (old_idx + shift_amount + ppm) % ppm for left shift
        // Here, shift is `k_shift = - (int)i;`
        // `shifted_row[new_j] = original_row[j]`
        // `original_row[j]` moves to `(j + k_shift) mod ppm`
        // So, `shifted_row[ (j + k_shift % ppm + ppm) % ppm ] = original_row[j]`
        // Or, `shifted_row[j_target] = original_row[ (j_target - k_shift % ppm + ppm) % ppm ]`

        // Easier: element at original_row[j_orig] goes to shifted_row[ (j_orig + shift_val) % ppm ]
        // So, shifted_row[j_new] = original_row[ (j_new - shift_val % ppm + ppm) % ppm ]
        // shift_val = - (int)i; (left shift by i)
        // shifted_b_matrix[i][j_col] = b_matrix[i][ (j_col - (-(int)i) % ppm + ppm) % ppm ]
        //                            = b_matrix[i][ (j_col + (int)i) % ppm ]

        for (int j_col = 0; j_col < ppm; ++j_col) {
             shifted_b_matrix[i][j_col] = b_matrix[i][(j_col + (int)i) % ppm];
        }
    }

    // For debugging: Print shifted_b_matrix
    // std::cout << "shifted_b_matrix (N x ppm):" << std::endl;
    // for (size_t i = 0; i < num_input_symbols; ++i) {
    //     std::cout << "  ";
    //     for (int j = 0; j < ppm; ++j) {
    //         std::cout << shifted_b_matrix[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // 3. Transpose: Create `transposed_matrix` (ppm x N)
    std::vector<std::vector<int>> transposed_matrix(ppm, std::vector<int>(num_input_symbols));
    for (int i = 0; i < ppm; ++i) { // iterating rows of transposed matrix (0 to ppm-1)
        for (size_t j = 0; j < num_input_symbols; ++j) { // iterating columns of transposed matrix (0 to N-1)
            transposed_matrix[i][j] = shifted_b_matrix[j][i];
        }
    }

    // For debugging: Print transposed_matrix
    // std::cout << "transposed_matrix (ppm x N):" << std::endl;
    // for (int i = 0; i < ppm; ++i) {
    //     std::cout << "  ";
    //     for (size_t j = 0; j < num_input_symbols; ++j) {
    //         std::cout << transposed_matrix[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }

    // 4. bi2de equivalent: Convert rows of `transposed_matrix` to decimal
    // The output will have `ppm` elements.
    std::vector<uint16_t> codewords_intermediate(ppm);
    for (int i = 0; i < ppm; ++i) { // For each row in transposed_matrix
        uint16_t decimal_value = 0;
        for (size_t j = 0; j < num_input_symbols; ++j) { // For each bit in that row
            // MSB is at index 0 (leftmost)
            decimal_value = (decimal_value << 1) | transposed_matrix[i][j];
        }
        codewords_intermediate[i] = decimal_value;
    }

    // 5. flipud equivalent: Reverse the order of elements
    std::vector<uint16_t> codewords = codewords_intermediate;
    std::reverse(codewords.begin(), codewords.end());

    return codewords;
}

// Helper to XOR specific bits of a codeword.
// `zero_indexed_lsb_positions` contains 0-indexed bit positions from LSB.
uint8_t css_symbols_decode_impl::bit_reduce(uint16_t codeword, const std::vector<int>& zero_indexed_lsb_positions) {
    uint8_t result = 0;
    for (int pos : zero_indexed_lsb_positions) {
        result ^= (codeword >> pos) & 1;
    }
    return result;
}

// Helper for Hamming parity fix
uint16_t css_symbols_decode_impl::parity_fix(uint8_t p_syndrome) {
    switch (p_syndrome) {
        case 3: return 4;  // 011 wrong b3 (0-indexed data bit 2, mask 0...0100)
        case 5: return 8;  // 101 wrong b4 (0-indexed data bit 3, mask 0...1000)
        case 6: return 1;  // 110 wrong b1 (0-indexed data bit 0, mask 0...0001)
        case 7: return 2;  // 111 wrong b2 (0-indexed data bit 1, mask 0...0010)
        default: return 0; // No error or uncorrectable
    }
    // Note: The MATLAB code implies these are masks for data bits D0-D3 if it's H(8,4).
    // If codeword is d3 d2 d1 d0 p3 p2 p1 p0 (MSB...LSB for data, then parity)
    // And nibble is d3 d2 d1 d0.
    // Mask 1 (0001) corrects d0. Mask 2 (0010) corrects d1. Mask 4 (0100) corrects d2. Mask 8 (1000) corrects d3.
    // This mapping seems consistent.
}

void css_symbols_decode_impl::hamming_decode_impl(
    const std::vector<uint16_t>& codewords_in, int rdd, std::vector<uint8_t>& nibbles_out)
{
    nibbles_out.clear();
    nibbles_out.reserve(codewords_in.size());

    // Define 0-indexed LSB positions for parity checks based on MATLAB's 1-based MSB indices
    // For an rdd-bit codeword, MATLAB index `m_idx` (1 to rdd) maps to LSB 0-indexed `rdd - m_idx`.
    std::vector<int> p1_indices, p2_indices, p3_indices, p5_indices; 

    // These indices are for H(8,4)-like structures. Need to be careful if rdd is not 8.
    // The MATLAB code uses fixed indices regardless of rdd for p1-p5 calculation,
    // which is typical if the codeword structure is assumed fixed for these checks,
    // and rdd just tells how many bits are valid or how correction is applied.
    // Let's assume the indices refer to fixed positions if an 8-bit structure is implied.
    // If rdd < 8, some of these bits might not exist. This needs clarification or robust handling.
    // For LoRa, the Hamming codes are H(8,4), H(7,4), H(6,4), H(5,4).
    // The parity equations change based on rdd.
    // The MATLAB code calculates p1-p5 always using the same bit indices, then uses a subset for syndrome.
    // This is fine if the indices are within the 'rdd' bits.
    
    // Example: For p1 = bitxor(codewords, [8 4 3 1]) (1-based MSB for 8-bit assumed structure)
    // LSB 0-indexed: [0, 4, 5, 7]
    // For p2 = bitxor(codewords, [7 4 2 1]) -> [1, 4, 6, 7]
    // For p3 = bitxor(codewords, [5 3 2 1]) -> [3, 5, 6, 7]
    // For p5 = bitxor(codewords, [6 4 3 2]) -> [2, 4, 5, 6] (Note: MATLAB uses p5, not p4 for syndrome)

    // For H(CR+4, 4) code (data d3 d2 d1 d0, parity bits p_cr-1 ... p0)
    // The parity check equations depend on CR (and thus rdd = CR+4)
    // CR=4 (4/8, rdd=8): d3 d2 d1 d0 p3 p2 p1 p0. Syndrome for d0-d3 uses p0,p1,p2. (p5,p3,p2 in MATLAB based on values)
    // CR=3 (4/7, rdd=7): d3 d2 d1 d0 p2 p1 p0.
    // CR=2 (4/6, rdd=6): d3 d2 d1 d0 p1 p0.
    // CR=1 (4/5, rdd=5): d3 d2 d1 d0 p0.
    // The MATLAB code's fixed parity calculation and then switch on rdd for syndrome implies
    // it's mostly geared towards H(8,4) structure for parity calculation, and then adapts.

    for (uint16_t codeword_val : codewords_in) {
        uint16_t current_codeword = codeword_val;
        uint8_t nibble;

        // Parity calculations as in MATLAB, assuming indices are for an 8-bit structure context
        // and an rdd-bit codeword. We must ensure indices are valid for 'rdd'.
        // Example: for [8 4 3 1] (1-based MSB), LSB 0-indexed are [0,4,5,7] for rdd=8.
        // These are bit positions relative to the LSB of the `current_codeword`.
        
        // For rdd=8 (CR 4/8)
        // p_syn_0 (like p5 in matlab) = d0^d1^d2^p0  -- bits {0,1,2,4} if d0-d3,p0-p3 (p0 is bit4)
        // p_syn_1 (like p3 in matlab) = d0^d1^d3^p1  -- bits {0,1,3,5}
        // p_syn_2 (like p2 in matlab) = d0^d2^d3^p2  -- bits {0,2,3,6}
        // The MATLAB indices [8 4 3 1] etc. are a bit opaque without knowing the exact bit ordering convention.
        // Let's directly use the syndrome calculation logic for LoRa standard Hamming codes.
        // Data bits are typically the 4 LSBs of the nibble after correction.
        // Codeword: D3 D2 D1 D0 Px Py Pz ... (actual bit order varies)

        // Given the MATLAB code:
        // p2_m = bit_reduce(current_codeword, {rdd-7, rdd-4, rdd-2, rdd-1}); // [7 4 2 1]
        // p3_m = bit_reduce(current_codeword, {rdd-5, rdd-3, rdd-2, rdd-1}); // [5 3 2 1]
        // p5_m = bit_reduce(current_codeword, {rdd-6, rdd-4, rdd-3, rdd-2}); // [6 4 3 2]
        // This assumes indices are relative to MSB of rdd-bit word.
        // Let's use a simpler approach based on standard LoRa Hamming syndrome calculation.
        // Assuming codeword bits are c(rdd-1) ... c(0) where c3,c2,c1,c0 are data.
        
        uint8_t s0=0, s1=0, s2=0; // syndrome bits

        if (rdd >= 5) { // Common for all CRs
            // Check H(rdd,4) properties. d0,d1,d2,d3 are data bits (lowest 4 bits of nibble)
            // Parity bits are higher up in the 'rdd' bit codeword.
            // Example for H(8,4): Codeword c7 c6 c5 c4 c3 c2 c1 c0
            // Data: c3(D3) c2(D2) c1(D1) c0(D0)
            // Parity: c7(P3) c6(P2) c5(P1) c4(P0)
            // s0 = c0^c1^c2   ^ c4 (P0)
            // s1 = c0^c1^  c3 ^ c5 (P1)
            // s2 = c0^  c2^c3 ^ c6 (P2)
            // s3 = c1^c2^c3   ^ c7 (P3) (only for H(8,4))

            // The MATLAB code uses p2, p3, p5 for syndrome, mapped to parity_fix.
            // This implies a specific syndrome bit ordering.
            // p2_matlab: XOR bits at 1-idx MSB [7,4,2,1] of codeword
            // p3_matlab: XOR bits at 1-idx MSB [5,3,2,1] of codeword
            // p5_matlab: XOR bits at 1-idx MSB [6,4,3,2] of codeword
            // Let's map these to 0-indexed LSB for an rdd-bit word.
            // Example: for rdd=8:
            // p2_idx = {1,4,6,7}; p3_idx = {3,5,6,7}; p5_idx = {2,4,5,6};
            
            // To be safe, use the bit indices as directly as possible from MATLAB:
            // These are 1-based from MSB. Convert to 0-based from LSB: (rdd-1) - (matlab_idx-1) = rdd - matlab_idx
            auto get_bit = [&](uint16_t val, int matlab_idx_from_msb) {
                if (matlab_idx_from_msb < 1 || matlab_idx_from_msb > rdd) return (uint8_t)0; // out of bound
                return uint8_t((val >> (rdd - matlab_idx_from_msb)) & 1);
            };
            
            uint8_t p2_val = get_bit(current_codeword, 7) ^ get_bit(current_codeword, 4) ^ get_bit(current_codeword, 2) ^ get_bit(current_codeword, 1);
            uint8_t p3_val = get_bit(current_codeword, 5) ^ get_bit(current_codeword, 3) ^ get_bit(current_codeword, 2) ^ get_bit(current_codeword, 1);
            uint8_t p5_val = get_bit(current_codeword, 6) ^ get_bit(current_codeword, 4) ^ get_bit(current_codeword, 3) ^ get_bit(current_codeword, 2);


            if (rdd == 7 || rdd == 8) {
                uint8_t syndrome = (p2_val << 2) | (p3_val << 1) | p5_val;
                uint16_t fix_mask = parity_fix(syndrome);
                // The fix_mask from MATLAB (1,2,4,8) is for the 4 data bits.
                // If data bits are D3 D2 D1 D0, this mask directly applies.
                // LoRa typically puts data bits in specific positions, not always LSBs of codeword pre-correction.
                // However, `mod(codewords,16)` implies data is extracted from LSBs post-correction.
                current_codeword ^= fix_mask; 
            }
            // For rdd 5, 6, MATLAB code does no correction, just mod 16.
        }
        nibble = static_cast<uint8_t>(current_codeword & 0x0F); // mod 16
        
        nibbles_out.push_back(nibble);
    }
}


void css_symbols_decode_impl::dewhiten_impl(
    const std::vector<uint8_t>& bytes_in, std::vector<uint8_t>& bytes_w_out)
{
    bytes_w_out.resize(bytes_in.size());
    if (d_whitening_seq.empty()) {
        std::cerr << "Dewhitening called with empty whitening sequence. Output will be same as input." << std::endl;
        bytes_w_out = bytes_in;
        return;
    }
    for (size_t i = 0; i < bytes_in.size(); ++i) {
        // MATLAB: bytes_w = bitxor(uint8(bytes(1:len)), self.whitening_seq(1:len));
        // This implies using whitening_seq[i], not whitening_seq[i % len_whitening_seq]
        // if len <= len_whitening_seq.
        // The precomputed sequence is usually 256 bytes.
        bytes_w_out[i] = bytes_in[i] ^ d_whitening_seq[i % d_whitening_seq.size()];
    }
}

// Main PDU processing function
void css_symbols_decode_impl::work_on_pdu(pmt::pmt_t msg)
{
    if (d_debug) {
        std::cout << "===== ENTERING work_on_pdu =====" << std::endl;
    }

    // Check if this is a valid PDU
    if (!pmt::is_pair(msg)) {
        std::cerr << "Received invalid PDU (not a pair)" << std::endl;
        if (d_debug) {
            std::cout << "PDU validation failed - not a pair" << std::endl;
        }
        return;
    }

    const pmt::pmt_t payload_blob = pmt::cdr(msg);

    if (!pmt::is_u32vector(payload_blob)) {
        std::cerr << "PDU payload is not a u32vector. Ignoring." << std::endl;
        if (d_debug) {
            std::cout << "Invalid payload type - expected u32vector" << std::endl;
        }
        return;
    }

    const std::vector<uint32_t> symbols_m_pkt = pmt::u32vector_elements(payload_blob);
    if (d_debug) {
        std::cout << "Received PDU with " << symbols_m_pkt.size() << " symbols" << std::endl;
    }

    if (symbols_m_pkt.empty()) {
        if (d_debug) {
            std::cout << "Empty PDU received - sending empty output" << std::endl;
        }
        pmt::pmt_t out_payload = pmt::init_u8vector(0, nullptr);
        message_port_pub(pmt::mp("out"), pmt::cons(pmt::PMT_NIL, out_payload));
        return;
    }
    
    // --- Start decoding process based on MATLAB's `decode` function ---
    std::vector<uint16_t> symbols_g;
    gray_coding_impl(symbols_m_pkt, symbols_g);

    if (d_debug) {
        std::cout << "After gray coding: " << symbols_g.size() << " symbols" << std::endl;
        print_int_vector(symbols_g, "Symbols g: ", 40);
    }

    if (symbols_g.empty()) { // Should not happen if symbols_m_pkt was not empty
        if (d_debug) {
            std::cout << "Unexpected empty output from gray coding" << std::endl;
        }
        pmt::pmt_t out_payload = pmt::init_u8vector(0, nullptr);
        message_port_pub(pmt::mp("out"), pmt::cons(pmt::PMT_NIL, out_payload));
        return;
    }

    std::vector<uint8_t> all_nibbles;

    // Part 2: Loop processing (as in MATLAB, starts from beginning of symbols_g)
    int rdd_loop = d_cr + 4;
    int ppm_loop = d_sf - 2 * (d_ldr ? 1 : 0); // ppm for payload part

    if (d_debug) {
        std::cout << "Decoding parameters - rdd_loop: " << rdd_loop 
                  << ", ppm_loop: " << ppm_loop << std::endl;
    }

    if (rdd_loop > 0) { // Ensure rdd_loop is positive to avoid infinite loop or other issues
        if (d_debug) {
            std::cout << "Starting chunk processing with chunk size " << rdd_loop 
                      << " and " << symbols_g.size() << " total symbols" << std::endl;
        }
        
        for (size_t ii = 0; (ii + rdd_loop) <= symbols_g.size(); ii += rdd_loop) {
            if (d_debug) {
                std::cout << "Processing chunk starting at index " << ii << std::endl;
            }
            
            std::vector<uint16_t> chunk_symbols(symbols_g.begin() + ii, 
                                            symbols_g.begin() + ii + rdd_loop);

            std::vector<uint16_t> codewords_loop = diag_deinterleave_impl(chunk_symbols, ppm_loop);

            if (d_debug) {
                print_int_vector(chunk_symbols, "Chunk symbols: ", 20);
                std::cout << "  After deinterleaving: " << codewords_loop.size() 
                          << " codewords" << std::endl;
                print_int_vector(codewords_loop, "Codewords: ", 40);
            }

            std::vector<uint8_t> nibbles_loop;
            hamming_decode_impl(codewords_loop, rdd_loop, nibbles_loop);
            
            if (d_debug) {
                std::cout << "  After Hamming decode: " << nibbles_loop.size() 
                          << " nibbles" << std::endl;
            }
            
            all_nibbles.insert(all_nibbles.end(), nibbles_loop.begin(), nibbles_loop.end());
        }
    }

    if (d_debug) {
        std::cout << "Total nibbles collected: " << all_nibbles.size() << std::endl;
        print_int_vector(all_nibbles, "All nibbles: ", 40);
    }

    // Combine nibbles to bytes
    std::vector<uint8_t> combined_bytes;
    if (all_nibbles.size() >= 2) {
        combined_bytes.reserve(all_nibbles.size() / 2);
        for (size_t i = 0; i < all_nibbles.size() / 2; ++i) {
            uint8_t lsn = all_nibbles[2 * i];
            uint8_t msn = all_nibbles[2 * i + 1];
            combined_bytes.push_back((msn << 4) | lsn);
        }
    }
    
    if (d_debug) {
        std::cout << "Combined bytes: " << combined_bytes.size() << std::endl;
        if (!combined_bytes.empty()) {
            std::cout << "First few bytes (hex):";
            for (size_t i = 0; i < std::min(static_cast<size_t>(5), combined_bytes.size()); ++i) {
                std::cout << " 0x" << std::hex << static_cast<int>(combined_bytes[i]);
            }
            std::cout << std::dec << std::endl;
        }
    }

    // Dewhitening
    std::vector<uint8_t> dewhitened_data;
    size_t len_to_dewhiten = combined_bytes.size();
    
    if (len_to_dewhiten > 0) {
        std::vector<uint8_t> bytes_to_dewhiten(combined_bytes.begin(), 
                                             combined_bytes.begin() + len_to_dewhiten);
        dewhiten_impl(bytes_to_dewhiten, dewhitened_data);
    }

    if (d_debug) {
        std::cout << "After dewhitening: " << dewhitened_data.size() << " bytes" << std::endl;
        if (!dewhitened_data.empty()) {
            std::cout << "First few dewhitened bytes (hex):";
            for (size_t i = 0; i < std::min(static_cast<size_t>(5), dewhitened_data.size()); ++i) {
                std::cout << " 0x" << std::hex << static_cast<int>(dewhitened_data[i]);
            }
            std::cout << std::dec << std::endl;
        }
    }

    // Send the PDU
    pmt::pmt_t out_payload = pmt::init_u8vector(dewhitened_data.size(), dewhitened_data.data());
    message_port_pub(pmt::mp("out"), pmt::cons(pmt::make_dict(), out_payload));

    if (d_debug) {
        std::cout << "Sent output PDU with " << dewhitened_data.size() << " bytes" << std::endl;
        std::cout << "===== EXITING work_on_pdu =====" << std::endl;
    }
}


void css_symbols_decode_impl::forecast(int noutput_items,
                                       gr_vector_int& ninput_items_required)
{
    // No stream input
}

int css_symbols_decode_impl::general_work(int noutput_items,
                                          gr_vector_int& ninput_items,
                                          gr_vector_const_void_star& input_items,
                                          gr_vector_void_star& output_items)
{
    // Do <+signal processing+>
    // Tell runtime system how many input items we consumed on
    // each input stream.
    consume_each(noutput_items);

    // Tell runtime system how many output items we produced.
    return noutput_items;
}

} /* namespace cssmods */
} /* namespace gr */
