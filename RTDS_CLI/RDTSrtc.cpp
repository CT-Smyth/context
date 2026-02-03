#include "RDTSrtc.h"
#include <Arduino.h>

static uint32_t tick_hz = 1000;
static int32_t slew_ppm = 0;

void rdtsrtc_init(void)
{
    // millis() is monotonic on nRF52840; wrap protection added below
}

uint64_t rdtsrtc_raw_ticks(void)
{
    static uint32_t last = 0;
    static uint64_t high = 0;

    uint32_t now = millis();
    if (now < last) high += (1ULL << 32);
    last = now;

    return high | now;
}

uint64_t rdtsrtc_now_ms(void)
{
    uint64_t t = rdtsrtc_raw_ticks();

    if (slew_ppm != 0)
    {
        // apply ppm slew: t' = t * (1 + ppm / 1e6)
        int64_t adj = (t * slew_ppm) / 1000000;
        return t + adj;
    }

    return t;
}

void rdtsrtc_set_slew_ppm(int32_t ppm)
{
    slew_ppm = ppm;
}

void rdtsrtc_clear_slew_ppm(void)
{
    slew_ppm = 0;
}

uint32_t rdtsrtc_tick_hz(void)
{
    return tick_hz;
}