#pragma once
#include <stdint.h>
#include <stddef.h>

/*
    RDTS Beacon Packet — Canonical Binary Format

    Fields (little-endian on wire):

      u8   version
      u8   addr_mode        (0=NONE, 1=ALL, 2=LIST)
      u8   addr_count
      u8   window_len
      u8   mode
      u8   flags            (reserved, must be 0)
      u16  reserved         (must be 0)
      u64  T_master_unix_ms
      u32  addr_list[addr_count]
      u8   mac[RDTS_MAC_LEN]   (truncated AES-128-CMAC)
*/

#ifndef RDTS_PACKET_VERSION
#define RDTS_PACKET_VERSION 1
#endif

#ifndef RDTS_MAX_ADDR_IDS
#define RDTS_MAX_ADDR_IDS 8
#endif

#ifndef RDTS_MAC_LEN
#define RDTS_MAC_LEN 8
#endif

typedef enum {
    RDTS_ADDR_NONE = 0,
    RDTS_ADDR_ALL  = 1,
    RDTS_ADDR_LIST = 2
} rdts_addr_mode_t;

typedef enum {
    RDTS_MODE_IDLE        = 0,
    RDTS_MODE_RECORD      = 1,
    RDTS_MODE_SLEEP       = 2,
    RDTS_MODE_SHUTDOWN    = 3,
    RDTS_MODE_INTERACTIVE = 4
} rdts_mode_t;

typedef struct {
    uint8_t  version;
    uint8_t  addr_mode;
    uint8_t  addr_count;
    uint8_t  window_len;
    uint8_t  mode;
    uint8_t  flags;
    uint16_t reserved;
    uint64_t t_master_unix_ms;
    uint32_t addr_list[RDTS_MAX_ADDR_IDS];
} rdts_beacon_t;

size_t rdtspkt_base_len(void);
size_t rdtspkt_total_len(uint8_t addr_mode, uint8_t addr_count);

size_t rdtspkt_build(uint8_t* out, size_t out_max, const rdts_beacon_t* b);
size_t rdtspkt_build_noauth(uint8_t* out, size_t out_max, const rdts_beacon_t* b);
bool   rdtspkt_parse(const uint8_t* data, size_t len, rdts_beacon_t* out, bool verify_mac);
bool   rdtspkt_verify_mac(const uint8_t* data, size_t len);