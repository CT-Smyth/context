#include <Arduino.h>
#include "RDTSserver.h"
#include "RDTSble.h"
#include "RDTSrtc.h"
#include "RDTSpacket.h"
#include "RDTSprint.h"

static char linebuf[128];
static uint8_t linepos = 0;

static void print_prompt() {
  Serial.print("\nRDTS> ");
}

void setup() {
  Serial.begin(115200);
  delay(2000); // works when on dc only (250 is probably plenty)

  Serial.println("RDTS Server CLI");
  rdtsrtc_init();
  rdtsm_init();
  rdtsble_init();
  rdtsm_beacon_start();
  print_prompt();
}

void print_help() {
  Serial.println();
  Serial.println("RDTS Server CLI — Command Reference");
  Serial.println("-----------------------------------");

  Serial.println("time <unix_ms>        Set session time anchor");
  Serial.println("now                   Show current master time");
  Serial.println("uptime                Show monotonic uptime");
  Serial.println();

  Serial.println("status                Show system status");
  Serial.println();

  Serial.println("beacon start          Start periodic beacon broadcast");
  Serial.println("beacon stop           Stop beacon broadcast");
  Serial.println("period <ms>           Set beacon period");
  Serial.println("burstlen <n>          Set packets per burst");
  Serial.println("burstspan <ms>        Set burst duration");
  Serial.println("txpower <dbm>         Set transmit power");
  Serial.println();

  Serial.println("mode idle|record|sleep|shutdown|setup");
  Serial.println("                     Set global operating mode");
  Serial.println();

  Serial.println("control all <n>       Open control window for all nodes");
  Serial.println("control stop          Stop asserting control windows");
  Serial.println();

  Serial.println("key <hex>             Provision shared authentication key");
  Serial.println("key clear             Clear authentication key");
  Serial.println("auth dev              Disable MAC enforcement");
  Serial.println("auth prod             Enable MAC enforcement");
  Serial.println();

  Serial.println("drift <ppm>           Inject oscillator drift (0 clears)");
  Serial.println("test oneshot          Send one beacon immediately");
  Serial.println();
  Serial.println("defaults              Load factory defaults (RAM only)");
  Serial.println("save                  Save current configuration to flash");
  Serial.println();

  Serial.println("help                  Show this message");
}

void handle_status() {
  Serial.println("\n--- RDTS Server STATUS ---");

  Serial.print("time:     ");
  rdts_println_u64(rdtsm_now_unix_ms());

  Serial.print("uptime:   ");
  rdts_println_u64(rdtsm_uptime_ms());

  Serial.print("beacon:   ");
  Serial.println(rdtsm_is_beacon_running() ? "RUNNING" : "STOPPED");

  Serial.print("period:   ");
  Serial.print(rdtsm_get_beacon_period());
  Serial.println(" ms");

  Serial.print("burstlen: ");
  Serial.println(rdtsm_get_beacon_burst_len());

  Serial.print("burstspan:");
  Serial.print(rdtsm_get_beacon_burst_span());
  Serial.println(" ms");

  Serial.print("txpower:  ");
  Serial.print(rdtsm_get_beacon_tx_power());
  Serial.println(" dBm");

  Serial.print("mode:     ");
  Serial.println(rdtsm_get_mode());

  Serial.print("auth:     ");
  Serial.println(rdtsm_get_auth_mode() == RDTSM_AUTH_PROD ? "PROD" : "DEV");

  Serial.print("control:  ");
  rdts_addr_mode_t am = rdtsm_get_addr_mode();
  if (am == RDTS_ADDR_NONE) Serial.println("NONE");
  else if (am == RDTS_ADDR_ALL) Serial.println("ALL");
  else Serial.println("LIST");

  Serial.print("window:   ");
  Serial.println(rdtsm_get_window_len());

  Serial.print("Burst interval (ms): ");
Serial.println(rdtsm_get_burst_interval_ms());

  Serial.println("---------------------------");
}

void process_cmd(char* cmd) {
  if (!strcmp(cmd, "help")) {
    print_help();
  } else if (!strcmp(cmd, "status")) {
    handle_status();
  } else if (!strncmp(cmd, "time ", 5)) {
    uint64_t t = strtoull(cmd + 5, NULL, 10);
    rdtsm_set_time_anchor(t);
    Serial.println("Time anchor set.");
  } else if (!strcmp(cmd, "beacon start")) {
    rdtsm_beacon_start();
    Serial.println("Beacon scheduler started.");
  } else if (!strcmp(cmd, "beacon stop")) {
    rdtsm_beacon_stop();
    Serial.println("Beacon scheduler stopped.");
  } else if (!strcmp(cmd, "test oneshot")) {
    rdtsm_test_send_oneshot();
    Serial.println("Oneshot beacon sent.");
  } else if (!strcmp(cmd, "now")) {
    Serial.print("Now: ");
    rdts_println_u64(rdtsm_now_unix_ms());
  } else if (!strcmp(cmd, "uptime")) {
    Serial.print("Uptime: ");
    rdts_println_u64(rdtsm_uptime_ms());
  } else if (!strncmp(cmd, "mode ", 5)) {
    if (!strcmp(cmd + 5, "idle")) rdtsm_mode_set(RDTS_MODE_IDLE);
    else if (!strcmp(cmd + 5, "record")) rdtsm_mode_set(RDTS_MODE_RECORD);
    else if (!strcmp(cmd + 5, "sleep")) rdtsm_mode_set(RDTS_MODE_SLEEP);
    else if (!strcmp(cmd + 5, "shutdown")) rdtsm_mode_set(RDTS_MODE_SHUTDOWN);
    else if (!strcmp(cmd + 5, "setup")) rdtsm_mode_set(RDTS_MODE_INTERACTIVE);
    else {
      Serial.println("Unknown mode");
      return;
    }
    Serial.println("Mode updated.");
  } else if (!strncmp(cmd, "period ", 7)) rdtsm_set_beacon_period(atoi(cmd + 7));
  else if (!strncmp(cmd, "burstlen ", 9)) rdtsm_set_beacon_burst_len(atoi(cmd + 9));
  else if (!strncmp(cmd, "burstspan ", 10)) rdtsm_set_beacon_burst_span(atoi(cmd + 10));
  else if (!strncmp(cmd, "txpower ", 8)) rdtsm_set_beacon_tx_power(atoi(cmd + 8));

  else if (!strncmp(cmd, "control all ", 12))
    rdtsm_control_open_all(atoi(cmd + 12));
  else if (!strcmp(cmd, "control stop"))
    rdtsm_control_stop_asserting();

  else if (!strncmp(cmd, "key ", 4)) {
    if (!strcmp(cmd + 4, "clear")) {
      rdtsm_set_key(nullptr, 0);
      Serial.println("Key cleared.");
    } else {
      uint8_t key[32];
      int len = 0;
      const char* p = cmd + 4;

      while (*p && *(p + 1) && len < 32) {
        char hex[3] = { p[0], p[1], 0 };
        key[len++] = strtoul(hex, nullptr, 16);
        p += 2;
      }

      if (len == 16 || len == 32) {
        rdtsm_set_key(key, len);
        Serial.print("Key set (");
        Serial.print(len * 8);
        Serial.println("-bit).");
      } else {
        Serial.println("Key must be 32 or 64 hex chars.");
      }
    }
  }

  else if (!strcmp(cmd, "auth dev")) {
    rdtsm_set_auth_mode(RDTSM_AUTH_DEV);
    Serial.println("Authentication mode: dev (MAC bypass).");
  } else if (!strcmp(cmd, "auth prod")) {
    rdtsm_set_auth_mode(RDTSM_AUTH_PROD);
    Serial.println("Authentication mode: prod (MAC enforced).");
  }

  else if (!strncmp(cmd, "drift ", 6)) {
    int ppm = atoi(cmd + 6);
    rdtsm_test_set_slew_ppm(ppm);

    if (ppm == 0)
      Serial.println("Injected drift cleared.");
    else {
      Serial.print("Injected drift: ");
      Serial.print(ppm);
      Serial.println(" ppm");
    }
  }

  else if (!strcmp(cmd, "save")) {
    rdtsm_save_config();
    Serial.println("Configuration saved.");
  }

  else if (!strcmp(cmd, "defaults")) {
    rdtsm_load_defaults();
    Serial.println("Defaults loaded (not saved).");
  }

  else {
    Serial.println("Unknown command.");
  }
}

void loop() {
  rdtsm_service();
  rdtsble_service();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      linebuf[linepos] = 0;
      if (linepos) process_cmd(linebuf);
      linepos = 0;
      print_prompt();
    } else if (linepos < sizeof(linebuf) - 1) {
      linebuf[linepos++] = c;
    }
  }
}
