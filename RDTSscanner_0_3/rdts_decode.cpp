#include "rdts_decode.h"
#include <string.h>

bool rdts_decode_packet(const uint8_t* p,
                        uint8_t len,
                        rdts_packet_t* out)
{
    if (!p || !out)
        return false;

    memset(out, 0, sizeof(*out));

    // Fixed header size up to unix_ms
    const uint8_t FIXED_LEN =
        1 + 1 + 1 + 1 + 1 + 1 + 2 + 8;

    if (len < FIXED_LEN)
        return false;

    uint8_t idx = 0;

    out->version     = p[idx++];
    out->addr_mode   = p[idx++];
    out->addr_count  = p[idx++];
    out->window_len  = p[idx++];
    out->mode        = p[idx++];
    out->flags       = p[idx++];

    memcpy(&out->reserved, &p[idx], 2);
    idx += 2;

    memcpy(&out->master_unix_ms, &p[idx], 8);
    idx += 8;

    // Policy checks (unchanged)
    if (out->flags != 0)
        return false;

    if (out->reserved != 0)
        return false;

    if (out->addr_count > RDTS_MAX_ADDRS)
        return false;

    // Address list
    uint8_t addr_bytes = out->addr_count * sizeof(uint32_t);
    if (idx + addr_bytes > len)
        return false;

    for (uint8_t i = 0; i < out->addr_count; i++) {
        memcpy(&out->addr_list[i], &p[idx], 4);
        idx += 4;
    }

    // Optional MAC (authenticated packets only)
    bool has_mac = (len == (idx + RDTS_MAC_LEN));

    if (has_mac) {
        memcpy(out->mac, &p[idx], RDTS_MAC_LEN);
        idx += RDTS_MAC_LEN;
    }

    // Final length sanity
    if (idx != len)
        return false;

    return true;
}