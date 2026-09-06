/*
 * zxh_config.h — the only file you need to edit for your setup.
 *
 * One Zigbee endpoint (pack_1, pack_2, ...) is created per entry below.
 * Find battery MACs with `bms-cli scan` (see the zxh-bms CLI project).
 */
#pragma once

#include <stdint.h>

/* Maximum packs this build supports (keep <= 8: one Zigbee endpoint each). */
#define ZXH_MAX_PACKS 8

typedef struct {
    const char *name;      /* informational, appears in the device log        */
    const char *mac;       /* "AA:BB:CC:DD:EE:FF" as advertised by the BMS    */
    uint8_t     modbus_addr; /* 0 = auto-discover after connect              */
} zxh_pack_cfg_t;

/* ------------------------- EDIT ME ---------------------------------------- */
static const zxh_pack_cfg_t ZXH_PACKS[] = {
    { "pack_1", "F1:17:01:0E:39:31", 0 },
    { "pack_2", "F0:5F:00:0E:39:31", 0 },
    { "pack_3", "9C:2C:00:19:37:38", 0 },
    { "pack_4", "F0:E6:00:0E:39:31", 0 },
    { "pack_5", "72:FA:00:19:37:38", 0 },
};
/* --------------------------------------------------------------------------- */

#define ZXH_PACK_COUNT (int)(sizeof(ZXH_PACKS) / sizeof(ZXH_PACKS[0]))

/* Polling: how often each pack is refreshed. */
#define ZXH_POLL_INTERVAL_S 300      /* seconds between polls of one pack */
#define ZXH_CONNECT_TIMEOUT_MS 6000 /* give up connecting after this       */
#define ZXH_CYCLE_TIMEOUT_MS 90000   /* give up a stalled read cycle      */

/* Minimum gap between consecutive Zigbee attribute-report frames. Keeps the
 * single shared radio from flooding the coordinator and lets downlinks
 * (interview/config/reads) interleave. */
#define ZXH_REPORT_GAP_MS 20

/* 1 = after every join, push sentinel reports (voltage=439.81V, soc=126) to
 * all endpoints to prove the report path. Noisy during interviews; enable
 * only for diagnostics. */
#define ZXH_SELFTEST_REPORTS 0

/* BLE: unreachable packs re-attempt after this backoff (shorter than the
 * poll interval, but each attempt now blocks the shared radio for ~6 s). */

/* Hold a BOOT button for 5 seconds WHILE RUNNING (do NOT hold it during
 * boot - that enters the ROM download mode) to factory-reset the Zigbee
 * stack: leave network, wipe saved state, join fresh on the next
 * permit-join window. The LED blinks orange while holding. On the
 * nanoESP32-C6 BOOT (SW3) is GPIO0; GPIO9 also checked for other devkits. */
#define ZXH_ZB_RESET_GPIO 0
#define ZXH_ZB_RESET_GPIO2 9

/* WS2812 status LED (nanoESP32-C6 v1.0: addressable RGB on GPIO8).
 * Blinks red while not joined to a Zigbee network, solid green once joined. */
#define ZXH_RGB_LED_GPIO 8
