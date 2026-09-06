#pragma once

/* Bring up Bluedroid + the multi-pack sequential BLE poller task. */
void zxh_ble_start(void);

/* Keep the BLE poller off the shared radio for `ms` (the C6 has one radio:
 * BLE connect attempts make us deaf to Zigbee downlinks, which breaks the
 * z2m interview and configureReporting). */
void zxh_ble_pause(uint32_t ms);
