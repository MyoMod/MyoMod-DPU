/**
 * @file base64.h
 * @author Leon Farchau (leon2225)
 * @brief base64 encoding and decoding
 * @version 0.1
 * @date 18.07.2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once
#include <string>

std::string base64_encode(const unsigned char *src, size_t len);
unsigned char * base64_decode(const unsigned char *src, size_t len,
			      size_t *out_len);