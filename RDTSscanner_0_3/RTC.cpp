#include "RTC.h"
#include <Arduino.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// ---- policy knobs ----
#define RTC_MAX_STEP_MS     5000   // reject absurd jumps after lock

static bool     g_locked = false;
static uint64_t g_anchor_unix_ms = 0;
static uint32_t g_anchor_rx_ms   = 0;
static int32_t  g_last_offset_ms = 0;

void rtc_init(void)
{
    g_locked = false;
    g_anchor_unix_ms = 0;
    g_anchor_rx_ms   = millis();
    g_last_offset_ms = 0;
}

void rtc_update_from_beacon(uint64_t unix_ms, uint32_t rx_ms)
{
    if (!g_locked)
    {
        // First valid beacon: hard lock
        g_anchor_unix_ms = unix_ms;
        g_anchor_rx_ms   = rx_ms;
        g_last_offset_ms = 0;
        g_locked = true;
        return;
    }

    // Predict local time from anchor
    uint64_t predicted =
        g_anchor_unix_ms + (uint64_t)(rx_ms - g_anchor_rx_ms);

    int64_t offset = (int64_t)unix_ms - (int64_t)predicted;
    g_last_offset_ms = (int32_t)offset;

    // Reject insane jumps
    if (offset > RTC_MAX_STEP_MS || offset < -RTC_MAX_STEP_MS)
        return;

    // For now: hard-step update
    g_anchor_unix_ms = unix_ms;
    g_anchor_rx_ms   = rx_ms;
}

uint64_t rtc_now_unix_ms(void)
{
    if (!g_locked)
        return 0;

    return g_anchor_unix_ms +
           (uint64_t)(millis() - g_anchor_rx_ms);
}

bool rtc_is_locked(void)
{
    return g_locked;
}

int32_t rtc_get_last_offset_ms(void)
{
    return g_last_offset_ms;
}