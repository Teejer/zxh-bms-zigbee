#pragma once

#include "zxh_proto.h"

/* Start the Zigbee router task: one endpoint per configured pack, each
 * carrying the ZXH-BMS custom cluster (see zigbee2mqtt/zxh-bms.js). */
void zxh_zigbee_start(void);

/* Push one pack's values into its Zigbee endpoint and report them to the
 * coordinator. Safe to call from any task (takes the Zigbee stack lock).
 * online=false marks the pack unreachable but keeps last-known values. */
void zxh_zigbee_publish_pack(int idx, const zxh_pack_t *pack, bool online);
