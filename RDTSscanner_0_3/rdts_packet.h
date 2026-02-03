#pragma once
#include <stdint.h>
#include <stdbool.h>

#define RDTS_MAX_ADDRS   8      // policy cap
#define RDTS_MAC_LEN     8      // truncated CMAC length

typedef enum {
    RDTS_ADDR_NONE = 0,
    RDTS_ADDR_ALL  = 1,
    RDTS_ADDR_LIST = 2,
} rdts_addr_mode_t;

typedef struct {
    uint8_t  version;
    uint8_t  addr_mode;
    uint8_t  addr_count;
    uint8_t  window_len;
    uint8_t  mode;
    uint8_t  flags;
    uint16_t reserved;

    uint64_t master_unix_ms;

    uint32_t addr_list[RDTS_MAX_ADDRS];
    uint8_t  mac[RDTS_MAC_LEN];
} rdts_packet_t;