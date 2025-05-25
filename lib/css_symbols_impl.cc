/* -*- c++ -*- */
/*
 * Copyright 2025 zerosignal.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "css_symbols_impl.h"
#include <gnuradio/io_signature.h>
#include <iostream> // For potential debugging output like print_bin
#include <stdexcept>

namespace gr {
namespace cssmods {

css_symbols::sptr css_symbols::make(int sf, int cr, bool ldr)
{
    return gnuradio::make_block_sptr<css_symbols_impl>(sf, cr, ldr);
}


/*
 * The private constructor
 */
css_symbols_impl::css_symbols_impl(int sf, int cr, bool ldr)
    : gr::block("css_symbols",
                gr::io_signature::make(0, 0, 0),  // No stream inputs
                gr::io_signature::make(0, 0, 0)), // No stream outputs
      d_sf(sf),
      d_cr(cr),
      d_ldr(ldr)
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
                    [this](pmt::pmt_t msg) { this->handle_payload_message(msg); });

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


// LFSR Whitening sequence generation (x^7 + x^5 + 1, seed 1)
std::vector<uint8_t> css_symbols_impl::generate_whitening_seq(size_t max_len_bytes)
{
    std::vector<uint8_t> seq(max_len_bytes, 0);
    uint8_t state = 0x01; // Initial state (seed) 0000001
    for (size_t i = 0; i < max_len_bytes; ++i) {
        uint8_t byte = 0;
        for (int j = 0; j < 8; ++j) {
            // Get the next bit from the LFSR state (LSB is state & 1)
            // Polynomial x^7 + x^5 + 1 (bits 6, 4, 0)
            uint8_t next_bit = ((state >> 6) ^ (state >> 4) ^ (state >> 0)) & 1;
            // Shift state right and insert the new bit into MSB
            state = (state >> 1) | (next_bit << 7);
            // Store the generated bit into the current byte (MSB first to match Matlab
            // bit order assumption in de2bi/bi2de?) The Matlab code's de2bi(...,
            // 'right-msb') means LSB is bit 0. bi2de(..., 'right-msb') means bit 0 is
            // LSB. Let's generate bits and put into the byte LSB first to match standard
            // byte representation and how de2bi/bi2de work.
            byte |= ((state & 1)
                     << j); // Take LSB of new state, put it into bit j of the byte
        }
        seq[i] = byte;
    }
    return seq;
}

int css_symbols_impl::calc_sym_num(size_t plen)
{
    // Number of payload symbol groups (blocks for interleaving)
    int ppm = d_sf - (d_ldr ? 2 : 0);
    if (ppm <= 0) {
        throw std::runtime_error("Invalid LoRa parameters");
    }
    
    // Equivalent to Matlab's (2*plen - sf + 7 + 4*crc - 5*(1-has_header))
    // Assuming no header (has_header=false) and no crc (crc=0) for this version
    double numerator = 2.0 * plen - d_sf + 7 + 0 - 5*(1-0); 
    
    int num_payload_symbol_groups = std::ceil(numerator / ppm);
    int total_symbols = 8 + std::max(num_payload_symbol_groups * (d_cr + 4), 0);
    
    return total_symbols;
}

// Diagonal interleaving helper
std::vector<uint32_t>
css_symbols_impl::diag_interleave_helper(const std::vector<uint8_t>& nibbles_block,
                                         int rdd)
{
    if (nibbles_block.empty()) {
        return {};
    }
    size_t num_nibbles = nibbles_block.size();
    // The output is rdd symbols
    std::vector<uint32_t> interleaved_symbols(rdd);
    // Convert nibbles to a bit matrix (num_nibbles x rdd)
    std::vector<std::vector<bool>> bits(num_nibbles, std::vector<bool>(rdd));
    for (size_t i = 0; i < num_nibbles; ++i) {
        uint8_t nibble = nibbles_block[i];
        for (int j = 0; j < rdd; ++j) {
            // de2bi(nibble, rdd, 'right-msb') puts LSB in column 0
            bits[i][j] = (nibble >> j) & 1;
        }
    }
    // Perform diagonal shifting and convert back to symbols
    // Shift for column j (0-indexed) is j positions up
    // Bit at bits[row][j] moves to shifted_bits[(row - j + num_nibbles) % num_nibbles][j]
    // Read out rows of the *shifted* conceptual matrix
    for (int r = 0; r < rdd; ++r) { // Output has rdd symbols (rows of shifted matrix)
        uint32_t symbol = 0;
        for (size_t c = 0; c < num_nibbles;
             ++c) { // Each output symbol has num_nibbles bits
            // The bit coming from original column c and original row (r + c) %
            // num_nibbles is the bit at position c in the output symbol r. The shifting
            // moves bit at original[row][col] to shifted[(row - col) % N_rows][col] We
            // are reading shifted[r][c]. This comes from original[(r + c) %
            // num_nibbles][c].
            bool bit_val = bits[(r + c) % num_nibbles][c];
            // bi2de(..., 'right-msb') means bit at index 0 is LSB.
            // The c-th bit (0-indexed) in the output symbol corresponds to column c of
            // the shifted matrix. Let's assume the columns of the shifted matrix form the
            // bits of the symbol, LSB first. This requires re-reading the matlab bi2de
            // usage. bi2de(cell2mat(...))' cell2mat(arrayfun(@(x) circshift(tmp(:,x),
            // 1-x), 1:rdd, 'un', 0))') tmp is (num_nibbles x rdd). circshift(tmp(:,x),
            // 1-x) takes col x (1-indexed) and shifts up by x-1. Result is rdd columns.
            // cell2mat makes it (num_nibbles x rdd). Transpose makes it (rdd x
            // num_nibbles). bi2de acts on rows of the (rdd x num_nibbles) matrix. Each
            // row is a symbol of num_nibbles bits. Bit at row r, column c in the
            // *transposed* shifted matrix corresponds to the bit at original row
            // (r_orig), original column c_orig, where original bit at (r_orig, c_orig)
            // moves to shifted bit at ((r_orig - c_orig) % num_nibbles, c_orig).
            // Transposed shifted bit is at (c_orig, (r_orig - c_orig) % num_nibbles).
            // We are looking for the bit at transposed_shifted[r][c]. This means c_orig =
            // r, (r_orig - r) % num_nibbles = c. r_orig = (c + r + num_nibbles) %
            // num_nibbles. So the bit at transposed_shifted[r][c] is the bit originally
            // at bits[(c + r + num_nibbles) % num_nibbles][r]. This bit is the c-th bit
            // of the r-th output symbol.
            bool bit_val_corrected = bits[(c + r + num_nibbles) % num_nibbles][r];
            // bi2de with 'right-msb' means bit at index c corresponds to 2^c (LSB at 0)
            if (bit_val_corrected) {
                symbol |= (1U << c); // Use c as the bit position (0 to num_nibbles-1)
            }
        }
        interleaved_symbols[r] = symbol;
    }
    return interleaved_symbols;
}

// Gray decoding helper
std::vector<uint32_t>
css_symbols_impl::gray_decoding_helper(const std::vector<uint32_t>& interleaved_symbols)
{
    std::vector<uint32_t> symbols(interleaved_symbols.size());
    uint32_t symbol_mod = (1U << d_sf); // 2^sf
    for (size_t i = 0; i < interleaved_symbols.size(); ++i) {
        uint32_t num = interleaved_symbols[i];
        uint32_t non_gray_num = num;
        uint32_t mask = num >> 1;
        // Inverse Gray mapping
        while (mask != 0) {
            non_gray_num = non_gray_num ^ mask;
            mask = mask >> 1;
        }
        // Apply final transformation
        if (i < 8 ||
            d_ldr) { // Matlab uses 1-based index i, so i <= 8 -> 0-based index i < 8
            symbols[i] = (non_gray_num * 4 + 1) % symbol_mod;
        } else {
            symbols[i] = (non_gray_num + 1) % symbol_mod;
        }
    }
    return symbols;
}

// Hamming encode
template<typename Func>
uint8_t css_symbols_impl::bit_reduce(Func fn, uint8_t w, const std::vector<int>& pos) {
    if (pos.empty()) return 0;
    
    // MATLAB's bitget uses 1-based indexing for the LSB (bit 1 is LSB)
    uint8_t b = (w >> (pos[0] - 1)) & 0x01;
    
    for (size_t i = 1; i < pos.size(); ++i) {
        uint8_t current_bit = (w >> (pos[i] - 1)) & 0x01;
        b = fn(b, current_bit);
    }
    
    return b;
}
template<typename Func>
uint8_t css_symbols_impl::word_reduce(Func fn, const std::vector<uint8_t>& ws) {
    if (ws.empty()) return 0;
    
    uint8_t w = ws[0];
    for (size_t i = 1; i < ws.size(); ++i) {
        w = fn(w, ws[i]);
    }
    
    return w;
}
// Updated hamming_encode_nibble function
uint8_t css_symbols_impl::hamming_encode_nibble(uint8_t nibble, int cr, int sf, int nibble_index) {
    // Calculate parity bits (note: MATLAB uses 1-based indexing)
    uint8_t p1 = bit_reduce(std::bit_xor<uint8_t>(), nibble, {1, 3, 4});
    uint8_t p2 = bit_reduce(std::bit_xor<uint8_t>(), nibble, {1, 2, 4});
    uint8_t p3 = bit_reduce(std::bit_xor<uint8_t>(), nibble, {1, 2, 3});
    uint8_t p4 = bit_reduce(std::bit_xor<uint8_t>(), nibble, {1, 2, 3, 4});
    uint8_t p5 = bit_reduce(std::bit_xor<uint8_t>(), nibble, {2, 3, 4});
    
    // Determine current code rate
    int cr_now = cr; // int cr_now = (nibble_index <= sf - 2) ? 4 : cr;
    
    // Apply Hamming encoding based on code rate
    switch (cr_now) {
        case 1:
            return word_reduce(std::bit_or<uint8_t>(), {static_cast<uint8_t>(p4 << 4), nibble});
        case 2:
            return word_reduce(std::bit_or<uint8_t>(), {static_cast<uint8_t>(p5 << 5), 
                                                       static_cast<uint8_t>(p3 << 4), 
                                                       nibble});
        case 3:
            return word_reduce(std::bit_or<uint8_t>(), {static_cast<uint8_t>(p2 << 6), 
                                                       static_cast<uint8_t>(p5 << 5), 
                                                       static_cast<uint8_t>(p3 << 4), 
                                                       nibble});
        case 4:
            return word_reduce(std::bit_or<uint8_t>(), {static_cast<uint8_t>(p1 << 7), 
                                                       static_cast<uint8_t>(p2 << 6), 
                                                       static_cast<uint8_t>(p5 << 5), 
                                                       static_cast<uint8_t>(p3 << 4), 
                                                       nibble});
        default:
            throw std::runtime_error("Invalid Code Rate!");
    }
}

// Message handler
void css_symbols_impl::handle_payload_message(pmt::pmt_t msg)
{
    // 首先检查是否为PDU（即pair类型）
    if (!pmt::is_pair(msg)) {
        std::cerr << "css_symbols: Received non-PDU message." << std::endl;
        return;
    }
    // 分离元数据和数据部分
    pmt::pmt_t metadata = pmt::car(msg);  // 元数据（可忽略或按需处理）
    pmt::pmt_t data = pmt::cdr(msg);      // 实际数据部分
    // 检查数据是否为u8vector类型
    if (!pmt::is_u8vector(data)) {
        std::cerr << "css_symbols: PDU data is not a uint8 vector." << std::endl;
        return;
    }
    // 提取数据到payload
    std::vector<uint8_t> payload = pmt::u8vector_elements(data);

    size_t plen = payload.size();
    // --- Start of Encoding Logic (ported from Matlab encode function) ---
    // 1. Calculate total symbols and needed nibbles
    int sym_num = calc_sym_num(plen);
    // Based on the Matlab formula: nibble_num = sf - 2 + (sym_num-8)/(cr+4)*(sf-2*ldr)
    // Derived number of payload symbol groups: (sym_num - 8) / (d_cr + 4)
    int num_payload_symbol_groups = (sym_num > 8) ? (sym_num - 8) / (d_cr + 4) : 0;
    int ppm = d_sf - (d_ldr ? 2 : 0); // sf - 2*ldr
    int nibble_num = (d_sf - 2) + num_payload_symbol_groups * ppm;
    // 2. Padding
    size_t bytes_needed = (nibble_num + 1) / 2; // ceil(nibble_num / 2.0)
    std::vector<uint8_t> data_w = payload;
    if (bytes_needed > plen) {
        data_w.resize(bytes_needed, 0xff); // Pad with 0xff
    } else if (bytes_needed < plen) {
        // This case shouldn't happen if calc_sym_num and nibble_num logic is correct
        // based on minimum symbols needed for payload. Handle defensively.
        data_w.resize(bytes_needed);
    }
    // 3. Whitening (only the original payload bytes within the padded buffer)
    // Matlab: data_w(1:plen) = self.whiten(data_w(1:plen));
    // We need to apply whitening to the first 'plen' bytes of data_w.
    if (plen > d_whitening_seq.size()) {
        // Whitening sequence is too short for this payload length
        std::cerr << "css_symbols: Whitening sequence too short for payload length "
                  << plen << std::endl;
        // Handle error - maybe regenerate a longer sequence or throw
        // For simplicity, let's resize the sequence (inefficient if done often)
        d_whitening_seq = generate_whitening_seq(plen);
    }
    for (size_t i = 0; i < plen; ++i) {
        data_w[i] = data_w[i] ^ d_whitening_seq[i];
    }
    // Padded bytes (from index plen onwards) are NOT whitened according to the Matlab
    // code.
    // 4. Nibble Extraction
    std::vector<uint8_t> data_nibbles;
    data_nibbles.reserve(nibble_num); // Reserve space
    for (size_t i = 0; i < bytes_needed; ++i) {
        data_nibbles.push_back(data_w[i] & 0x0f); // Lower nibble
        if (data_nibbles.size() <
            nibble_num) { // Avoid pushing upper nibble if already have enough nibbles
            data_nibbles.push_back(data_w[i] >> 4); // Upper nibble
        }
    }
    // Check if we got the expected number of nibbles
    if (data_nibbles.size() != (size_t)nibble_num) {
        // This indicates an issue in size calculations or padding
        std::cerr << "css_symbols: Nibble extraction mismatch. Expected " << nibble_num
                  << ", got " << data_nibbles.size() << std::endl;
        // Handle error
        return;
    }

    // Calc codewords with hamming encode
    std::vector<uint8_t> codewords;
    codewords.reserve(data_nibbles.size());
    for (size_t i = 0; i < data_nibbles.size(); i++) {
        uint8_t nibble = data_nibbles[i] & 0x0F; // Ensure it's just a nibble
        codewords.push_back(hamming_encode_nibble(nibble, d_cr, d_sf, i));
    }

    // 5. Interleaving
    // Calculate ppm and rdd based on member variables
    int rdd = d_cr + 4;
    // Initialize the vector to store the interleaved symbols
    std::vector<uint32_t> symbols_i;
    // Basic validation for ppm to avoid issues (Matlab might handle implicitly)
    if (ppm <= 0) {
        // Handle error: ppm must be a positive block size.
        throw std::runtime_error("ppm must be a positive block size.");
    } else {
        // Loop through codewords vector in blocks of size ppm
        // Matlab loop: for i = 1:ppm:length(codewords)-ppm+1
        // C++ loop (0-based indexing): for (int i = 0; i <= (int)codewords.size() - ppm; i += ppm)
        // Or, safer with size_t and checking available size:
        for (size_t i = 0; i + ppm <= codewords.size(); i += ppm) {
            // Extract the current block of 'ppm' nibbles (uint8_t)
            // In C++, we create a sub-vector for the slice
            std::vector<uint8_t> nibbles_block(codewords.begin() + i, codewords.begin() + i + ppm);
            // Call the helper function to interleave the block
            std::vector<uint32_t> result_block = diag_interleave_helper(nibbles_block, rdd);
            // Append the resulting interleaved symbols to the main vector
            // Matlab concatenation [symbols_i; result_block] corresponds to appending
            symbols_i.insert(symbols_i.end(), result_block.begin(), result_block.end());
        }
    }
    
    // 6. Gray Decoding
    std::vector<uint32_t> final_symbols = gray_decoding_helper(symbols_i);
    // --- End of Encoding Logic ---
    pmt::pmt_t value = pmt::init_u32vector(final_symbols.size(), final_symbols.data());
    pmt::pmt_t output_msg = pmt::cons(pmt::make_dict(), value);
    // 发布消息
    message_port_pub(pmt::mp("out"), output_msg);
}

// Destructor
css_symbols_impl::~css_symbols_impl()
{
    // Nothing specific to clean up
}

void css_symbols_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required)
{
    // No stream input
}

int css_symbols_impl::general_work(int noutput_items,
                                   gr_vector_int& ninput_items,
                                   gr_vector_const_void_star& input_items,
                                   gr_vector_void_star& output_items)
{
    // This block is message-driven. The work is done in handle_payload_message.
    // If you wanted to support stream input, you would consume bytes here,
    // buffer them until a full packet is received (e.g., signaled by a tag),
    // then process and produce output symbols.
    // For now, just return 0 indicating no stream items processed.
    consume_each(0); // Consume 0 items from input streams
    return 0; // Produce 0 items on output streams
}

} /* namespace cssmods */
} /* namespace gr */
