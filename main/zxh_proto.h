/*
 * zxh_proto.h — wire protocol + decode for the zxh-family BMS boards.
 * Port of lifepo4-cli/bms_cli/{protocol,crc}.py (and the ESPHome port).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "zxh_config.h"

#define ZXH_MAX_CELLS 32
#define ZXH_FUNC_READ_PARAM 0x03
#define ZXH_FUNC_READ_STATUS 0x04
#define ZXH_RX_BUF_SIZE 512

/* Per-connection read cycle kinds. */
enum {
    ZXH_CMD_DISCOVER_ADDRESS = 1,
    ZXH_CMD_LABEL,
    ZXH_CMD_INSTRUMENT,
    ZXH_CMD_CELLS,
    ZXH_CMD_BASIC,
    ZXH_CMD_TEMPS,
};

typedef struct {
    uint8_t kind;
    uint8_t function;
    uint16_t reg;
    uint8_t count;
    uint16_t cell_offset; /* CELLS only */
    uint8_t retries_left;
} zxh_cmd_t;

/* Values exposed to Zigbee. Invalid values keep their last-known state. */
typedef struct {
    bool ever_read;
    uint16_t voltage_cV;   /* centivolts            */
    int16_t current_cA;    /* centiamperes          */
    int16_t power_W;       /* whole watts           */
    uint8_t soc;           /* percent               */
    uint16_t cell_min_mv, cell_max_mv, cell_delta_mv;
    int16_t mos_temp_c10;  /* deci-degrees C        */
    int16_t temp1_c10, temp2_c10;
    bool temp1_valid, temp2_valid;
    uint16_t cycles;
    uint8_t health;
    uint16_t capacity_cAh; /* remaining, 0.1 Ah     */
    uint16_t full_cAh;
    uint8_t cell_count;
    uint16_t protection;   /* bitmask               */
    uint32_t equilibrium;  /* balancing bitmask     */
    /* ZCL octet string: [len][mv_hi,mv_lo x cell_count], len = 2*count <= 64 */
    uint8_t cell_mv[66];
} zxh_values_t;

typedef struct {
    const zxh_pack_cfg_t *cfg;
    uint8_t mac[6];
    bool mac_valid; /* parsed from cfg->mac at init */
    bool use_public; /* peer address type; flipped on connect failure */

    /* link-time state */
    uint8_t bus_addr;
    bool bus_addr_known;
    uint8_t cell_count;
    uint16_t cells_mv[ZXH_MAX_CELLS];
    bool cells_valid;

    /* cycle state */
    zxh_cmd_t queue[16];
    uint8_t queue_len;
    uint8_t cmd_pos;
    bool cycle_active;

    /* scheduler bookkeeping */
    uint32_t next_due_ms;
    bool last_read_ok;

    zxh_values_t values;
} zxh_pack_t;

void zxh_crc16(const uint8_t *data, int len, uint8_t *out_hi, uint8_t *out_lo);
void zxh_build_request(const zxh_pack_t *pack, const zxh_cmd_t *cmd, uint8_t frame[7]);

/* Match + CRC-validate a response in rx; on success sets out/out_len to the
 * frame's data section and consumes it (plus any leading garbage) from rx. */
bool zxh_extract_response(uint8_t *rx, int *rx_len, uint8_t addr, uint8_t function,
                          const uint8_t **out, int *out_len);

void zxh_pack_init(zxh_pack_t *pack, const zxh_pack_cfg_t *cfg);
/* Queue a read cycle for one connection: [addr?] label instrument cells...
 * basic temps. Cell chunk commands are appended when the label response
 * reveals the cell count. */
void zxh_pack_begin_cycle(zxh_pack_t *pack);
zxh_cmd_t *zxh_pack_current_cmd(zxh_pack_t *pack); /* NULL when done */
/* Mark the current command complete and move to the next one. */
void zxh_pack_cmd_advance(zxh_pack_t *pack);
/* Decode one response. Returns false on malformed data (caller retries). */
bool zxh_pack_handle_response(zxh_pack_t *pack, const zxh_cmd_t *cmd,
                              const uint8_t *data, int len);
/* Recompute derived values + the cell_mv blob after a cycle. */
void zxh_pack_finalize_values(zxh_pack_t *pack);
