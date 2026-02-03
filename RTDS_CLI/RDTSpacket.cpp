
#include "RDTSpacket.h"
#include "RDTScrypto.h"
#include <string.h>

static inline void wr_u16_le(uint8_t* p, uint16_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
}
static inline void wr_u32_le(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}
static inline void wr_u64_le(uint8_t* p, uint64_t v) {
    for (int i = 0; i < 8; i++) p[i] = (uint8_t)((v >> (8 * i)) & 0xFF);
}

static inline uint16_t rd_u16_le(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline uint32_t rd_u32_le(const uint8_t* p) {
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}
static inline uint64_t rd_u64_le(const uint8_t* p) {
    uint64_t v = 0;
    for (int i = 0; i < 8; i++) v |= ((uint64_t)p[i] << (8 * i));
    return v;
}

size_t rdtspkt_base_len(void)
{
    // version(1) + addr_mode(1) + addr_count(1) + window_len(1) +
    // mode(1) + flags(1) + reserved(2) + t_master(8)
    return 1 + 1 + 1 + 1 + 1 + 1 + 2 + 8;
}

size_t rdtspkt_total_len(uint8_t addr_mode, uint8_t addr_count)
{
    size_t base = rdtspkt_base_len();
    size_t ids = 0;

    if (addr_mode == RDTS_ADDR_LIST) {
        if (addr_count > RDTS_MAX_ADDR_IDS) return 0;
        ids = (size_t)addr_count * 4;
    } else {
        // NONE/ALL: addr_count must be 0 in canonical encoding
        addr_count = 0;
        ids = 0;
    }

    return base + ids + RDTS_MAC_LEN;
}

static bool sane_addr(const rdts_beacon_t* b)
{
    if (!b) return false;
    if (b->version == 0) return false;

    if (b->addr_mode == RDTS_ADDR_NONE || b->addr_mode == RDTS_ADDR_ALL) {
        if (b->addr_count != 0) return false;
        return true;
    }
    if (b->addr_mode == RDTS_ADDR_LIST) {
        if (b->addr_count == 0) return false;
        if (b->addr_count > RDTS_MAX_ADDR_IDS) return false;
        return true;
    }
    return false;
}

size_t rdtspkt_build(uint8_t* out, size_t out_max, const rdts_beacon_t* b)
{
    if (!out || !b) return 0;
    if (!sane_addr(b)) return 0;

    size_t total = rdtspkt_total_len(b->addr_mode, b->addr_count);
    if (total == 0 || out_max < total) return 0;

    // Canonical requirements
    const uint8_t  version  = (b->version == 0) ? (uint8_t)RDTS_PACKET_VERSION : b->version;
    const uint8_t  flags    = b->flags;     // currently must be 0 by policy, but we don't enforce here
    const uint16_t rsvd16   = b->reserved;  // currently must be 0 by policy

    size_t i = 0;
    out[i++] = version;
    out[i++] = b->addr_mode;
    out[i++] = b->addr_count;
    out[i++] = b->window_len;
    out[i++] = b->mode;
    out[i++] = flags;
    wr_u16_le(&out[i], rsvd16); i += 2;
    wr_u64_le(&out[i], b->t_master_unix_ms); i += 8;

    // addr_list
    if (b->addr_mode == RDTS_ADDR_LIST) {
        for (uint8_t k = 0; k < b->addr_count; k++) {
            wr_u32_le(&out[i], b->addr_list[k]); i += 4;
        }
    }

    // MAC over everything so far
    if (!rdtscrypto_has_key()) return 0;

    uint8_t full_cmac[16];
    rdtscrypto_cmac(out, (uint16_t)i, full_cmac);

    // Append truncated MAC
    memcpy(&out[i], full_cmac, RDTS_MAC_LEN);
    i += RDTS_MAC_LEN;

    return i;
}

bool rdtspkt_verify_mac(const uint8_t* data, size_t len)
{
    if (!data) return false;
    if (!rdtscrypto_has_key()) return false;
    if (len < rdtspkt_base_len() + RDTS_MAC_LEN) return false;

    // Determine expected length based on parsed addr_mode/count (without trusting it fully yet)
    const uint8_t addr_mode  = data[1];
    const uint8_t addr_count = data[2];

    size_t expect = rdtspkt_total_len(addr_mode, addr_count);
    if (expect == 0 || len != expect) return false;

    size_t mac_off = len - RDTS_MAC_LEN;

    uint8_t full_cmac[16];
    rdtscrypto_cmac(data, (uint16_t)mac_off, full_cmac);

    // Constant-time compare truncated MAC (simple loop; sufficient here)
    uint8_t diff = 0;
    for (size_t i = 0; i < RDTS_MAC_LEN; i++) diff |= (uint8_t)(data[mac_off + i] ^ full_cmac[i]);
    return diff == 0;
}

bool rdtspkt_parse(const uint8_t* data, size_t len, rdts_beacon_t* out, bool verify_mac)
{
    if (!data || !out) return false;
    memset(out, 0, sizeof(*out));

    if (len < rdtspkt_base_len() + RDTS_MAC_LEN) return false;

    const uint8_t version    = data[0];
    const uint8_t addr_mode  = data[1];
    const uint8_t addr_count = data[2];

    size_t expect = rdtspkt_total_len(addr_mode, addr_count);
    if (expect == 0 || len != expect) return false;

    if (verify_mac) {
        if (!rdtspkt_verify_mac(data, len)) return false;
    }

    size_t i = 0;
    out->version     = data[i++];
    out->addr_mode   = data[i++];
    out->addr_count  = data[i++];
    out->window_len  = data[i++];
    out->mode        = data[i++];
    out->flags       = data[i++];
    out->reserved    = rd_u16_le(&data[i]); i += 2;
    out->t_master_unix_ms = rd_u64_le(&data[i]); i += 8;

    // Canonical constraints (enforced at parse time)
    if (version == 0) return false;
    if (out->addr_mode == RDTS_ADDR_NONE || out->addr_mode == RDTS_ADDR_ALL) {
        if (out->addr_count != 0) return false;
    } else if (out->addr_mode == RDTS_ADDR_LIST) {
        if (out->addr_count == 0 || out->addr_count > RDTS_MAX_ADDR_IDS) return false;
        for (uint8_t k = 0; k < out->addr_count; k++) {
            out->addr_list[k] = rd_u32_le(&data[i]); i += 4;
        }
    } else {
        return false;
    }

    // MAC already verified (or skipped) — caller decides policy for flags/reserved
    return true;
}

size_t rdtspkt_build_noauth(uint8_t* out, size_t out_max, const rdts_beacon_t* b)
{
    if (!out || !b) return 0;
    if (!sane_addr(b)) return 0;

    size_t base = rdtspkt_base_len();
    size_t ids = (b->addr_mode == RDTS_ADDR_LIST) ? (b->addr_count * 4) : 0;
    size_t total = base + ids;

    if (out_max < total) return 0;

    size_t i = 0;
    out[i++] = b->version;
    out[i++] = b->addr_mode;
    out[i++] = b->addr_count;
    out[i++] = b->window_len;
    out[i++] = b->mode;
    out[i++] = b->flags;
    wr_u16_le(&out[i], b->reserved); i += 2;
    wr_u64_le(&out[i], b->t_master_unix_ms); i += 8;

    if (b->addr_mode == RDTS_ADDR_LIST) {
        for (uint8_t k = 0; k < b->addr_count; k++) {
            wr_u32_le(&out[i], b->addr_list[k]); i += 4;
        }
    }
    return i;
}