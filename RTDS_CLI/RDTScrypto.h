#pragma once
#include <stdint.h>

/*
    RDTS Crypto Abstraction
    Provides AES-128-CMAC for beacon authentication.
*/

// Set or clear the shared secret key (len = 16 bytes for AES-128)
void rdtscrypto_set_key(const uint8_t* key, uint8_t len);

// Compute CMAC over buffer
void rdtscrypto_cmac(const uint8_t* data, uint16_t len, uint8_t* out16);

// Return true if a key is provisioned
bool rdtscrypto_has_key(void);