#include "zxh_proto.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

static const char *TAG = "zxh_proto";

/* CRC16/XMODEM, big-endian output — matches crc16_bytes() in crc.py. */
void zxh_crc16(const uint8_t *data, int len, uint8_t *out_hi, uint8_t *out_lo)
{
    uint16_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    *out_hi = (uint8_t)(crc >> 8);
    *out_lo = (uint8_t)(crc & 0xFF);
}

static uint16_t be16(const uint8_t *d) { return (uint16_t)((d[0] << 8) | d[1]); }

static float decode_temp(uint16_t raw)
{
    float c = (raw - 2732) / 10.0f; /* raw units are (degC * 10) + 2732 */
    return (c < -40.0f) ? NAN : c;  /* vendor "probe absent" threshold  */
}

void zxh_build_request(const zxh_pack_t *pack, const zxh_cmd_t *cmd, uint8_t frame[7])
{
    uint8_t addr = (cmd->kind == ZXH_CMD_DISCOVER_ADDRESS) ? 0 : pack->bus_addr;
    frame[0] = addr;
    frame[1] = cmd->function;
    frame[2] = (uint8_t)(cmd->reg >> 8);
    frame[3] = (uint8_t)(cmd->reg & 0xFF);
    frame[4] = cmd->count;
    zxh_crc16(frame, 5, &frame[5], &frame[6]);
}

/* Response: [addr, func, len_hi, len_lo, data x len, crc_hi, crc_lo]. */
bool zxh_extract_response(uint8_t *rx, int *rx_len, uint8_t addr, uint8_t function,
                          const uint8_t **out, int *out_len)
{
    int n = *rx_len;
    for (int i = 0; i + 6 <= n; i++) {
        if (rx[i] != addr || rx[i + 1] != function) continue;
        uint16_t length = be16(&rx[i + 2]);
        if (length > 240) continue;
        int total = length + 6;
        if (n - i < total) continue;
        uint8_t hi, lo;
        zxh_crc16(&rx[i], total - 2, &hi, &lo);
        if (rx[i + total - 2] != hi || rx[i + total - 1] != lo) continue;
        *out = &rx[i + 4];
        *out_len = length;
        /* consume the frame (and any garbage before it) from rx */
        int consumed = i + total;
        memmove(rx, &rx[consumed], (size_t)(n - consumed));
        *rx_len = n - consumed;
        return true;
    }
    return false;
}

void zxh_pack_init(zxh_pack_t *pack, const zxh_pack_cfg_t *cfg)
{
    memset(pack, 0, sizeof(*pack));
    pack->cfg = cfg;
    pack->bus_addr = cfg->modbus_addr;
    pack->bus_addr_known = (cfg->modbus_addr != 0);
    unsigned m[6];
    if (sscanf(cfg->mac, "%x:%x:%x:%x:%x:%x", &m[0], &m[1], &m[2], &m[3], &m[4], &m[5]) == 6) {
        for (int i = 0; i < 6; i++) pack->mac[i] = (uint8_t)m[i];
        pack->mac_valid = true;
    }
}

static void queue_cmd(zxh_pack_t *p, uint8_t kind, uint8_t fn, uint16_t reg, uint8_t count,
                      uint16_t cell_offset)
{
    if (p->queue_len >= sizeof(p->queue) / sizeof(p->queue[0])) return;
    zxh_cmd_t *c = &p->queue[p->queue_len++];
    c->kind = kind;
    c->function = fn;
    c->reg = reg;
    c->count = count;
    c->cell_offset = cell_offset;
    c->retries_left = 3;
}

/* Queue cell voltage chunk commands (7 cells per read, register stride 14). */
static void queue_cell_chunks(zxh_pack_t *p, uint8_t n)
{
    uint16_t done = 0, page = 0;
    while (done < n) {
        uint8_t cnt = (uint8_t)((n - done < 7) ? (n - done) : 7);
        queue_cmd(p, ZXH_CMD_CELLS, ZXH_FUNC_READ_STATUS, (uint16_t)(3012 + 14 * page), cnt, done);
        done += cnt;
        page++;
    }
}

void zxh_pack_begin_cycle(zxh_pack_t *p)
{
    p->queue_len = 0;
    p->cmd_pos = 0;
    p->cells_valid = false;
    if (!p->bus_addr_known)
        queue_cmd(p, ZXH_CMD_DISCOVER_ADDRESS, ZXH_FUNC_READ_PARAM, 4008, 1, 0);
    queue_cmd(p, ZXH_CMD_LABEL, ZXH_FUNC_READ_PARAM, 4000, 5, 0);
    /* Instrument / basic / temps are appended after LABEL reveals cell_count,
     * but queue the fixed reads now too (order: cells come after instrument). */
    p->cycle_active = true;
}

zxh_cmd_t *zxh_pack_current_cmd(zxh_pack_t *p)
{
    if (!p->cycle_active || p->cmd_pos >= p->queue_len) return NULL;
    return &p->queue[p->cmd_pos];
}

void zxh_pack_cmd_advance(zxh_pack_t *p)
{
    if (p->cmd_pos < p->queue_len) p->cmd_pos++;
}

bool zxh_pack_handle_response(zxh_pack_t *p, const zxh_cmd_t *cmd, const uint8_t *d, int len)
{
    bool ok = true;
    switch (cmd->kind) {
    case ZXH_CMD_DISCOVER_ADDRESS:
        if (len < 1) { ok = false; break; }
        p->bus_addr = d[0];
        if (p->bus_addr == 0) {
            ESP_LOGW(TAG, "%s: address discovery returned 0", p->cfg->name);
            ok = false;
        } else {
            p->bus_addr_known = true;
            ESP_LOGI(TAG, "%s: bus address %u", p->cfg->name, p->bus_addr);
        }
        break;
    case ZXH_CMD_LABEL: {
        if (len < 8) { ok = false; break; }
        uint8_t cells = d[0];
        p->cell_count = (cells == 0 || cells > ZXH_MAX_CELLS) ? 0 : cells;
        p->values.full_cAh = be16(d + 6);
        /* Now we know the cell count: build the rest of the read sequence.
         * Order matters: instrument, then cell chunks, then basic, temps. */
        queue_cmd(p, ZXH_CMD_INSTRUMENT, ZXH_FUNC_READ_STATUS, 3000, 5, 0);
        queue_cell_chunks(p, p->cell_count);
        queue_cmd(p, ZXH_CMD_BASIC, ZXH_FUNC_READ_STATUS, 3076, 7, 0);
        queue_cmd(p, ZXH_CMD_TEMPS, ZXH_FUNC_READ_STATUS, 3087, 4, 0);
        ESP_LOGI(TAG, "%s: %u cells", p->cfg->name, p->cell_count);
        break;
    }
    case ZXH_CMD_INSTRUMENT: {
        if (len < 12) { ok = false; break; }
        /* 3-byte signed big-endian current in mA; sign threshold mirrors the
         * vendor app arithmetic (data[0] > 124). */
        int32_t raw = (int32_t)((d[0] << 16) | (d[1] << 8) | d[2]);
        if (d[0] > 124) raw -= 0x1000000;
        p->values.current_cA = (int16_t)(raw / 10); /* mA -> centiA */
        p->values.soc = d[3];
        float mos = decode_temp(be16(d + 4));
        if (!isnan(mos)) p->values.mos_temp_c10 = (int16_t)(mos * 10);
        p->values.equilibrium =
            (uint32_t)((d[6] << 24) | (d[7] << 16) | (d[8] << 8) | d[9]);
        p->values.protection = be16(d + 10);
        break;
    }
    case ZXH_CMD_CELLS: {
        if (len < 2 * cmd->count) { ok = false; break; }
        for (uint8_t i = 0; i < cmd->count; i++)
            if (cmd->cell_offset + i < ZXH_MAX_CELLS)
                p->cells_mv[cmd->cell_offset + i] = be16(d + 2 * i);
        p->cells_valid = true;
        break;
    }
    case ZXH_CMD_BASIC:
        if (len < 11) { ok = false; break; }
        p->values.capacity_cAh = be16(d);
        p->values.cycles = be16(d + 2);
        p->values.health = d[4];
        break;
    case ZXH_CMD_TEMPS: {
        if (len < 8) { ok = false; break; }
        float t1 = decode_temp(be16(d));
        float t2 = decode_temp(be16(d + 2));
        if (!isnan(t1)) { p->values.temp1_c10 = (int16_t)(t1 * 10); p->values.temp1_valid = true; }
        if (!isnan(t2)) { p->values.temp2_c10 = (int16_t)(t2 * 10); p->values.temp2_valid = true; }
        break;
    }
    default:
        ok = false;
        break;
    }
    return ok;
}

void zxh_pack_finalize_values(zxh_pack_t *p)
{
    if (!p->cells_valid || p->cell_count == 0) return;
    uint32_t sum = 0;
    uint16_t lo = 0xFFFF, hi = 0;
    int pos = 1;
    p->values.cell_mv[0] = (uint8_t)(2 * p->cell_count);
    for (uint8_t i = 0; i < p->cell_count; i++) {
        uint16_t mv = p->cells_mv[i];
        sum += mv;
        if (mv < lo) lo = mv;
        if (mv > hi) hi = mv;
        p->values.cell_mv[pos++] = (uint8_t)(mv >> 8);
        p->values.cell_mv[pos++] = (uint8_t)(mv & 0xFF);
    }
    p->values.voltage_cV = (uint16_t)(sum / 10u); /* mV -> centivolts */
    p->values.cell_min_mv = lo;
    p->values.cell_max_mv = hi;
    p->values.cell_delta_mv = (uint16_t)(hi - lo);
    p->values.cell_count = p->cell_count;
    float watts = (sum / 1000.0f) * (p->values.current_cA / 100.0f);
    p->values.power_W = (int16_t)(watts + (watts >= 0 ? 0.5f : -0.5f));
    p->values.ever_read = true;
}
