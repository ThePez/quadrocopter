/*
 ******************************************************************************
 * File: hamming.c
 * Author: Jack Cairns
 * Date: 04-07-2025
 * Brief: Various function definitions used for hamming algorithm
 * REFERENCE: None
 ******************************************************************************
 */

#include "hamming.h"

/* hamming_byte_encode()
 * ----------------------------------
 * Fully encodes a byte of data using the following hamming matrix. Uses an
 * internal helper function to impliment the encoding process for each nibble.
 *
 * G = ; 1 0 0 0 1 1 0 ;
 *     ; 0 1 0 0 1 0 1 ;
 *     ; 0 0 1 0 0 1 1 ;
 *     ; 0 0 0 1 1 1 1 ;
 */
uint16_t hamming_byte_encode(uint8_t value) {

    uint16_t encodedResult = hamming_nibble_encode(value & 0x0F);
    encodedResult |= hamming_nibble_encode(value >> 4) << 8;
    return encodedResult;
}

/* hamming_nibble_encode()
 * ----------------------
 * Implement Hamming Encode algorithm, based on the following generator
 * matrix. An additional overall parity bit has been added into bit0.
 *
 * G = ; 1 0 0 0 1 1 0 ;
 *     ; 0 1 0 0 1 0 1 ;
 *     ; 0 0 1 0 0 1 1 ;
 *     ; 0 0 0 1 1 1 1 ;
 */
uint8_t hamming_nibble_encode(uint8_t value) {

    uint8_t data0, data1, data2, data3, parity, ham0, ham1, ham2;
    uint8_t halfByteEncoded = 0;

    // Extract bits
    data0 = !!(value & DATA_0_POS);
    data1 = !!(value & DATA_1_POS);
    data2 = !!(value & DATA_2_POS);
    data3 = !!(value & DATA_3_POS);

    // Calculate hamming & parity bits
    ham0 = data0 ^ data1 ^ data3;
    ham1 = data0 ^ data2 ^ data3;
    ham2 = data1 ^ data2 ^ data3;
    parity = data0 ^ data1 ^ data2;

    // Generate out byte
    uint8_t bits[] = {parity, ham0, ham1, data0, ham2, data1, data2, data3};
    for (uint8_t i = 0; i < 8; i++) {
        halfByteEncoded |= (bits[i] << i);
    }

    return halfByteEncoded;
}

/* hamming_byte_decode()
 * ----------------------------------
 * Function to decode a half byte of data from the input. The code uses the
 * below parity check matrix, however an addition bit has been added into bit0
 * for overall parity check. This has been taken into account when calculating
 * the single bit error position given by the syndrome.
 *
 * H = ; 1 0 1 0 1 0 1 ; S0
 *     ; 0 1 1 0 0 1 1 ; S1
 *     ; 0 0 0 1 1 1 1 ; S2
 */
uint8_t hamming_byte_decode(uint8_t value) {

    // Extract bits
    uint8_t h0 = (value >> 1) & 1;
    uint8_t h1 = (value >> 2) & 1;
    uint8_t d0 = (value >> 3) & 1;
    uint8_t h2 = (value >> 4) & 1;
    uint8_t d1 = (value >> 5) & 1;
    uint8_t d2 = (value >> 6) & 1;
    uint8_t d3 = (value >> 7) & 1;

    // Calculate the Syndrome
    uint8_t s0 = h0 ^ d0 ^ d1 ^ d3;
    uint8_t s1 = h1 ^ d0 ^ d2 ^ d3;
    uint8_t s2 = h2 ^ d1 ^ d2 ^ d3;
    uint8_t syndrome = (s2 << 2) | (s1 << 1) | s0;
    uint8_t parity = hamming_calculate_parity(value);
    
    if (!syndrome) {
        if (parity) {
            // Parity error occured -> rest of the data is fine
        }

        // No errors in recieved data
    } else {
        if (!parity) {
            // Detected a double error -> return error code
            return DOUBLE_BIT_ERROR;
        }

        // Single error detected -> correct and return value
        value ^= (1 << syndrome);
    }

    // Re-extract bits after correction
    d0 = (value >> 3) & 1;
    d1 = (value >> 5) & 1;
    d2 = (value >> 6) & 1;
    d3 = (value >> 7) & 1;
    return d0 | (d1 << 1) | (d2 << 2) | (d3 << 3);
}

uint8_t hamming_word_decode(uint16_t value) {
    uint8_t upper = hamming_byte_decode((uint8_t) value >> 8);
    uint8_t lower = hamming_byte_decode((uint8_t) value);
    if (upper == DOUBLE_BIT_ERROR || lower == DOUBLE_BIT_ERROR) {
        return DOUBLE_BIT_ERROR;
    }

    return (upper << 4) | lower;
}

/* hamming_calculate_parity()
 * -----------------------------------
 * Function to check the parity of a given value. Returns 1 if a parity error
 * occurs, otherwise 0.
 */
uint8_t hamming_calculate_parity(uint8_t value) {

    int parity = 0;
    for (uint8_t i = 0; i < 8; i++) {
        parity ^= !!(value & (1 << i));
    }

    return parity;
}
