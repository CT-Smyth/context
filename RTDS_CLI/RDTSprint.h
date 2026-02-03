#pragma once
#include <Arduino.h>

inline void rdts_print_u64(uint64_t v)
{
    uint32_t hi = (uint32_t)(v >> 32);
    uint32_t lo = (uint32_t)(v & 0xFFFFFFFF);

    if (hi)
    {
        Serial.print(hi);
        Serial.print('_');
        if (lo < 1000000000UL) Serial.print('0');
    }
    Serial.print(lo);
}

inline void rdts_println_u64(uint64_t v)
{
    rdts_print_u64(v);
    Serial.println();
}