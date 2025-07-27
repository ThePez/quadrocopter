/*
 ******************************************************************************
 * File: hamming.h
 * Author: Jack Cairns
 * Date: 04-07-2025
 * Brief: Header files for the library functions for hamming algorithm.
 * REFERENCE: None
 ******************************************************************************
 */

#ifndef HAMMING_H
#define HAMMING_H

#include <stdint.h>

#define DOUBLE_BIT_ERROR 0x80 // Bits 7 set
#define SINGLE_BIT_ERROR 0x40 // Bit 6 set
#define ERROR_MASK 0xC0       // Bits 7 & 6 set

#define DATA_0_POS 1
#define DATA_1_POS 2
#define DATA_2_POS 4
#define DATA_3_POS 8

uint16_t hamming_byte_encode(uint8_t value);
uint8_t hamming_nibble_encode(uint8_t value);
uint8_t hamming_byte_decode(uint8_t value);
uint8_t hamming_word_decode(uint16_t value);
uint8_t hamming_calculate_parity(uint8_t value);

#endif