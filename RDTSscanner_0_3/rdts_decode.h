#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "rdts_packet.h"

// Decode RDTS payload (after company ID)
// payload points to first RDTS byte (version)
// len is remaining length
bool rdts_decode_packet(const uint8_t* payload,
                        uint8_t len,
                        rdts_packet_t* out);