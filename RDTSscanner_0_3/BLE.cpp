#include <Arduino.h>
#include <bluefruit.h>
#include "BLE.h"

// forward-declared elsewhere
extern void scan_callback(ble_gap_evt_adv_report_t* report);

static bool g_ble_initialized = false;
static bool g_rx_active = false;

void ble_init(void)
{
    if (g_ble_initialized)
        return;

    Bluefruit.begin(0, 1);           // central only
    Bluefruit.setName("RDTS_SCANNER");

    Bluefruit.Scanner.setRxCallback(scan_callback);
    Bluefruit.Scanner.restartOnDisconnect(false);
    Bluefruit.Scanner.useActiveScan(false);

    // interval/window units = 0.625 ms
    Bluefruit.Scanner.setInterval(160, 80);  // 100 ms / 50 ms

    g_ble_initialized = true;
}

void ble_rx_start(void)
{
    if (!g_ble_initialized || g_rx_active)
        return;

    Bluefruit.Scanner.start(0);   // continuous scan
    g_rx_active = true;
}

void ble_rx_stop(void)
{
    if (!g_rx_active)
        return;

    Bluefruit.Scanner.stop();
    g_rx_active = false;
}

bool ble_rx_active(void)
{
    return g_rx_active;
}