#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "RDTS_packet.h"

/*
 * RDTS decode result codes.
 * All protocol-level validity decisions live here.
 */
typedef enum {
    RDTS_DECODE_OK = 0,
    RDTS_DECODE_ERR_LEN,
    RDTS_DECODE_ERR_VERSION,
    RDTS_DECODE_ERR_ADDR_MODE,
    RDTS_DECODE_ERR_ADDR_COUNT,
    RDTS_DECODE_ERR_MAC,
    RDTS_DECODE_ERR_RESERVED,
    RDTS_DECODE_ERR_TIME_RANGE,  // invalid / implausible unix time
    RDTS_DECODE_ERR_INTERNAL,
} rdts_decode_result_t;

/*
 * Convert RDTS decode result to human-readable string.
 * Returns a constant string; never NULL.
 */
const char *rdts_decode_result_str(rdts_decode_result_t res);

/*
 * Decode RDTS payload (after manufacturer ID).
 * payload points to first RDTS byte (version).
 * len is payload length.
 *
 * Returns RDTS_DECODE_OK on success.
 * Any non-OK value indicates a protocol-level rejection.
 */
rdts_decode_result_t rdts_decode_packet(
    const uint8_t *payload,
    uint8_t len,
    rdts_packet_t *out
);