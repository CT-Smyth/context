#include "RDTScrypto.h"
#include <string.h>
#include <Arduino.h>
#include <nrf.h>

static uint8_t cmac_key[16];
static bool key_valid = false;

/*
    nRF52 ECB uses a RAM block:
      key[16], cleartext[16], ciphertext[16]
    ECBDATAPTR points to that block.
*/
typedef struct __attribute__((packed)) {
    uint8_t key[16];
    uint8_t cleartext[16];
    uint8_t ciphertext[16];
} nrf_ecb_datablock_t;

// Ensure 4-byte alignment for ECBDATAPTR requirements
static nrf_ecb_datablock_t ecb_block __attribute__((aligned(4)));

static void aes_ecb_encrypt(const uint8_t* key, const uint8_t* in, uint8_t* out)
{
    // Load key and plaintext into the RAM block
    memcpy(ecb_block.key, key, 16);
    memcpy(ecb_block.cleartext, in, 16);

    // Point ECB to the RAM block
    NRF_ECB->ECBDATAPTR = (uint32_t)&ecb_block;

    // Clear events
    NRF_ECB->EVENTS_ENDECB = 0;
    NRF_ECB->EVENTS_ERRORECB = 0;

    // Start
    NRF_ECB->TASKS_STARTECB = 1;

    // Wait for completion
    while (NRF_ECB->EVENTS_ENDECB == 0 && NRF_ECB->EVENTS_ERRORECB == 0) {
        // busy wait
    }

    // Stop (recommended pattern)
    NRF_ECB->TASKS_STOPECB = 1;

    // If error, output zeros (keeps CMAC deterministic on failure)
    if (NRF_ECB->EVENTS_ERRORECB) {
        memset(out, 0, 16);
        NRF_ECB->EVENTS_ERRORECB = 0;
        return;
    }

    NRF_ECB->EVENTS_ENDECB = 0;

    // Copy ciphertext out
    memcpy(out, ecb_block.ciphertext, 16);
}

static void leftshift(uint8_t* dst, const uint8_t* src)
{
    uint8_t carry = 0;
    for (int i = 15; i >= 0; i--) {
        uint8_t v = src[i];
        dst[i] = (uint8_t)((v << 1) | carry);
        carry = (v & 0x80) ? 1 : 0;
    }
}

static void xor128(uint8_t* out, const uint8_t* a, const uint8_t* b)
{
    for (int i = 0; i < 16; i++) out[i] = a[i] ^ b[i];
}

void rdtscrypto_set_key(const uint8_t* key, uint8_t len)
{
    if (key && len == 16) {
        memcpy(cmac_key, key, 16);
        key_valid = true;
    } else {
        memset(cmac_key, 0, 16);
        key_valid = false;
    }
}

bool rdtscrypto_has_key(void)
{
    return key_valid;
}

void rdtscrypto_cmac(const uint8_t* data, uint16_t len, uint8_t* out16)
{
    // If no key provisioned, output zeros (caller can treat as invalid)
    if (!key_valid) {
        memset(out16, 0, 16);
        return;
    }

    // NIST SP 800-38B: generate subkeys K1, K2
    uint8_t zero[16] = {0};
    uint8_t L[16], K1[16], K2[16];

    aes_ecb_encrypt(cmac_key, zero, L);

    leftshift(K1, L);
    if (L[0] & 0x80) K1[15] ^= 0x87;

    leftshift(K2, K1);
    if (K1[0] & 0x80) K2[15] ^= 0x87;

    uint8_t X[16] = {0};
    uint8_t M[16];

    uint16_t n = (len + 15) / 16;
    bool last_complete = ((len % 16) == 0) && (n != 0);

    // Special case: len == 0 => n == 0 per our formula; CMAC uses one padded block
    if (n == 0) {
        n = 1;
        last_complete = false;
    }

    for (uint16_t i = 0; i < n; i++) {
        memset(M, 0, 16);

        uint16_t block_len;
        if (i == n - 1) {
            // last block may be partial (or len==0)
            uint16_t remaining = (len >= i * 16) ? (len - i * 16) : 0;
            block_len = (remaining > 16) ? 16 : remaining;
        } else {
            block_len = 16;
        }

        if (block_len > 0) {
            memcpy(M, data + i * 16, block_len);
        }

        if (i == n - 1) {
            if (last_complete) {
                xor128(M, M, K1);
            } else {
                // padding: 0x80 then zeros
                if (block_len < 16) M[block_len] = 0x80;
                xor128(M, M, K2);
            }
        }

        xor128(X, X, M);
        aes_ecb_encrypt(cmac_key, X, X);
    }

    memcpy(out16, X, 16);
}