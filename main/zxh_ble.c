/*
 * zxh_ble.c — Bluedroid GATT client + sequential multi-pack scheduler.
 *
 * One BLE link at a time: when a pack's poll interval elapses we connect to
 * its configured MAC, auto-detect the GATT profile, run the read cycle,
 * disconnect and publish the results to Zigbee. The protocol state machine
 * mirrors the ESPHome component (zxh-bms-esphome repo); this port runs on raw
 * Bluedroid because the target boots plain ESP-IDF.
 */
#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gattc_api.h"

#include "zxh_ble.h"
#include "zxh_config.h"
#include "zxh_proto.h"
#include "zxh_zigbee.h"

static const char *TAG = "zxh_ble";

#define APP_GATTC_ID 0x2C6A
#define ATTEMPT_TIMEOUT_MS 1500
#define WRITE_ACK_TIMEOUT_MS 500
#define SLOT_GAP_MS 1000

typedef enum {
    ST_IDLE,
    ST_ACTIVE, /* connecting, handshaking or cycling */
} ble_state_t;

static zxh_pack_t packs[ZXH_MAX_PACKS];
static int pack_count;

static QueueHandle_t evt_q;
static SemaphoreHandle_t rx_mtx;
static uint8_t rx[ZXH_RX_BUF_SIZE];
static int rx_len;

static ble_state_t state = ST_IDLE;
static int active = -1;
static esp_gatt_if_t reg_gattc_if = ESP_GATT_IF_NONE;
static esp_gatt_if_t gattc_if = ESP_GATT_IF_NONE;
static bool gattc_ready;

static uint16_t conn_id = 0xFFFF;
static uint16_t notify_handle, write_handle;
static bool write_with_response, write_pending, profile_ready;
static bool open_done, search_started, search_done, notify_req_sent, notify_ok, notify_failed, link_lost;
static bool mtu_done;
static uint32_t search_at;
static uint8_t remote_bda[6];

static uint32_t attempt_deadline, connect_deadline, cycle_deadline, slot_free_ms;
static uint32_t now_ms(void) { return xTaskGetTickCount() * portTICK_PERIOD_MS; }

/* --- GATT profiles, same order/UUIDs as the CLI + ESPHome component ------- */
static const uint8_t PROFILE_PREFIX[3][4] = {
    {0x00, 0x00, 0x27, 0x60}, {0x6E, 0x40, 0x00, 0x01}, {0x00, 0x03, 0xCD, 0xD0}};
static const char *PROFILE_NOTIFY[3] = {
    "00002760-08C2-11E1-9073-0E8AC72E0002",
    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
    "0003CDD1-0000-1000-8000-00805F9B0131",
};
static const char *PROFILE_WRITE[3] = {
    "00002760-08C2-11E1-9073-0E8AC72E0001",
    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
    "0003CDD2-0000-1000-8000-00805F9B0131",
};

static void hex16_be(const char *s, uint8_t out[16])
{
    for (int i = 0; i < 16; i++) {
        while (!isxdigit((unsigned char)*s)) s++;
        unsigned v = 0;
        for (int n = 0; n < 2; n++, s++) {
            char c = *s;
            v <<= 4;
            v += (c <= '9') ? (unsigned)(c - '0') : (unsigned)(tolower(c) - 'a' + 10);
        }
        out[i] = (uint8_t)v;
    }
}

/* Bluedroid stores 128-bit UUIDs little-endian; normalise to display order. */
static void uuid_be(const esp_bt_uuid_t *u, uint8_t out[16])
{
    memset(out, 0, 16);
    if (u->len == ESP_UUID_LEN_128) {
        for (int i = 0; i < 16; i++) out[i] = u->uuid.uuid128[15 - i];
    } else if (u->len == ESP_UUID_LEN_16) {
        out[2] = (uint8_t)(u->uuid.uuid16 >> 8);
        out[3] = (uint8_t)(u->uuid.uuid16 & 0xFF);
    } else if (u->len == ESP_UUID_LEN_32) {
        out[0] = (uint8_t)(u->uuid.uuid32 >> 24);
        out[1] = (uint8_t)(u->uuid.uuid32 >> 16);
        out[2] = (uint8_t)(u->uuid.uuid32 >> 8);
        out[3] = (uint8_t)(u->uuid.uuid32);
    }
}

static void find_profile(void)
{
    notify_handle = write_handle = 0;
    write_with_response = true;

    for (int p = 0; p < 3 && !write_handle; p++) {
        esp_gattc_service_elem_t svc;
        uint16_t n = 1;
        for (uint16_t si = 0; si < 64; si++) {
            n = 1;
            if (esp_ble_gattc_get_service(gattc_if, conn_id, NULL, &svc, &n, si) != ESP_GATT_OK || n == 0)
                break;
            uint8_t be[16];
            uuid_be(&svc.uuid, be);
            if (memcmp(be, PROFILE_PREFIX[p], 4) != 0) continue;

            uint16_t nh = 0, wh = 0, fb_nh = 0, fb_wh = 0;
            bool nr = false, fb_nr = false;
            uint8_t expect_n[16], expect_w[16];
            hex16_be(PROFILE_NOTIFY[p], expect_n);
            hex16_be(PROFILE_WRITE[p], expect_w);

            for (uint16_t ci = 0; ci < 64; ci++) {
                esp_gattc_char_elem_t chr;
                uint16_t cn = 1;
                if (esp_ble_gattc_get_all_char(gattc_if, conn_id, svc.start_handle, svc.end_handle, &chr, &cn, ci) !=
                    ESP_GATT_OK || cn == 0)
                    break;
                uint8_t cbe[16];
                uuid_be(&chr.uuid, cbe);
                if (memcmp(cbe, expect_n, 16) == 0) {
                    nh = chr.char_handle;
                } else if (memcmp(cbe, expect_w, 16) == 0) {
                    wh = chr.char_handle;
                    nr = (chr.properties & ESP_GATT_CHAR_PROP_BIT_WRITE_NR) != 0;
                } else {
                    uint32_t props = chr.properties;
                    if (!fb_nh && (props & (ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_INDICATE)))
                        fb_nh = chr.char_handle;
                    if (!fb_wh && (props & (ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR))) {
                        fb_wh = chr.char_handle;
                        fb_nr = (props & ESP_GATT_CHAR_PROP_BIT_WRITE_NR) != 0;
                    }
                }
                if (nh && wh) break;
            }
            if (!nh) nh = fb_nh;
            if (!wh) { wh = fb_wh; nr = fb_nr; }
            if (nh && wh) {
                notify_handle = nh;
                write_handle = wh;
                write_with_response = !nr;
                ESP_LOGI(TAG, "%s: GATT profile %d matched: notify=0x%02x write=0x%02x wwr=%d",
                         packs[active].cfg->name, p, nh, wh, write_with_response);
                return;
            }
        }
    }
    ESP_LOGE(TAG, "%s: no known BMS profile found on this device", packs[active].cfg->name);
}

/* --- BLE stack callbacks (BTC task context: copy bytes + post event only) - */

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    (void)event;
    (void)param;
}

static void gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t ifx, esp_ble_gattc_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTC_REG_EVT:
        if (param->reg.app_id == APP_GATTC_ID) {
            reg_gattc_if = ifx;
            xQueueSend(evt_q, &(int){1}, 0);
        }
        break;
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status == ESP_GATT_OK) {
            conn_id = param->open.conn_id;
            open_done = true;
        } else {
            link_lost = true;
        }
        xQueueSend(evt_q, &(int){1}, 0);
        break;
    case ESP_GATTC_SEARCH_CMPL_EVT:
        search_done = true;
        /* BLE-only bluedroid on the C6 does not auto-exchange MTU like BlueZ
         * does; some bridges hold notifications until it happens. */
        esp_ble_gattc_send_mtu_req(gattc_if, conn_id);
        xQueueSend(evt_q, &(int){1}, 0);
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT:
        /* bluedroid bookkeeping only; real gate is the CCCD write event */
        ESP_LOGI(TAG, "reg-for-notify handle=0x%02x status=%d", param->reg_for_notify.handle,
                 param->reg_for_notify.status);
        break;
    case ESP_GATTC_NOTIFY_EVT: {
        static char hexbuf[64];
        int n = param->notify.value_len > 15 ? 15 : param->notify.value_len;
        for (int i = 0; i < n; i++) snprintf(&hexbuf[2 * i], 3, "%02x", param->notify.value[i]);
        hexbuf[2 * n] = 0;
        ESP_LOGI(TAG, "NOTIFY handle=0x%02x len=%d data=%s", param->notify.handle,
                 param->notify.value_len, hexbuf);
        if (param->notify.handle != notify_handle) break;
        xSemaphoreTake(rx_mtx, portMAX_DELAY);
        int room = (int)sizeof(rx) - rx_len;
        if (param->notify.value_len > room)
            rx_len = 0; /* overflow: drop and resync */
        else {
            memcpy(&rx[rx_len], param->notify.value, param->notify.value_len);
            rx_len += param->notify.value_len;
        }
        xSemaphoreGive(rx_mtx);
        xQueueSend(evt_q, &(int){1}, 0);
        break;
    }
    case ESP_GATTC_WRITE_DESCR_EVT:
        /* our explicit CCCD write result */
        ESP_LOGI(TAG, "cccd write status=%d handle=0x%02x", param->write.status, param->write.handle);
        if (param->write.status == ESP_GATT_OK)
            notify_ok = true;
        else
            notify_failed = true;
        xQueueSend(evt_q, &(int){1}, 0);
        break;
    case ESP_GATTC_WRITE_CHAR_EVT:
        ESP_LOGI(TAG, "write-char status=%d handle=0x%02x", param->write.status, param->write.handle);
        write_pending = false;
        xQueueSend(evt_q, &(int){1}, 0);
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        link_lost = true;
        xQueueSend(evt_q, &(int){1}, 0);
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGI(TAG, "mtu negotiated: status=%d mtu=%d", param->cfg_mtu.status, param->cfg_mtu.mtu);
        mtu_done = true;
        xQueueSend(evt_q, &(int){1}, 0);
        break;
    default:
        break;
    }
}

/* --- scheduler (our task context) ------------------------------------------ */

static bool rx_extract(uint8_t addr, uint8_t func, uint8_t *data, int *len)
{
    bool found = false;
    xSemaphoreTake(rx_mtx, portMAX_DELAY);
    const uint8_t *out;
    int out_len;
    if (zxh_extract_response(rx, &rx_len, addr, func, &out, &out_len)) {
        memcpy(data, out, (size_t)out_len);
        *len = out_len;
        found = true;
    }
    xSemaphoreGive(rx_mtx);
    return found;
}

static void send_current_cmd(zxh_pack_t *p, const zxh_cmd_t *cmd)
{
    uint8_t frame[7];
    zxh_build_request(p, cmd, frame);
    ESP_LOGI(TAG, "tx kind=%d: %02x %02x %02x %02x %02x %02x %02x", cmd->kind, frame[0], frame[1], frame[2],
             frame[3], frame[4], frame[5], frame[6]);
    xSemaphoreTake(rx_mtx, portMAX_DELAY);
    rx_len = 0;
    xSemaphoreGive(rx_mtx);
    write_pending = true;
    esp_err_t err = esp_ble_gattc_write_char(gattc_if, conn_id, write_handle, sizeof(frame), frame,
                                             write_with_response ? ESP_GATT_WRITE_TYPE_RSP
                                                                 : ESP_GATT_WRITE_TYPE_NO_RSP,
                                             ESP_GATT_AUTH_REQ_NONE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "write failed: %d", err);
        write_pending = false;
    }
    attempt_deadline = now_ms();
}

static void reset_conn_flags(void)
{
    open_done = search_started = search_done = notify_req_sent = notify_ok = notify_failed = link_lost = false;
    mtu_done = false;
    search_at = 0;
    profile_ready = write_pending = false;
    conn_id = 0xFFFF;
    notify_handle = write_handle = 0;
}

static void end_cycle(bool ok)
{
    if (active >= 0) {
        zxh_pack_t *p = &packs[active];
        p->last_read_ok = ok;
        if (ok) {
            zxh_pack_finalize_values(p);
            p->next_due_ms = now_ms() + ZXH_POLL_INTERVAL_S * 1000u;
        } else {
            /* Retry failed packs sooner, and flip the peer address type:
               boards advertise either random-static or public. */
            p->use_public = !p->use_public;
            p->next_due_ms = now_ms() + 60000u;
        }
        zxh_zigbee_publish_pack(active, p, ok);
        ESP_LOGI(TAG, "%s: cycle %s", p->cfg->name, ok ? "complete" : "FAILED");
        if (gattc_ready && conn_id != 0xFFFF) esp_ble_gattc_close(gattc_if, conn_id);
    }
    state = ST_IDLE;
    active = -1;
    reset_conn_flags();
    slot_free_ms = now_ms() + SLOT_GAP_MS;
}

static void start_next_cycle(void)
{
    uint32_t now = now_ms();
    if (now < slot_free_ms) return;

    int best = -1;
    uint32_t best_due = UINT32_MAX;
    for (int i = 0; i < pack_count; i++) {
        if ((int32_t)(now - packs[i].next_due_ms) >= 0 && packs[i].next_due_ms < best_due) {
            best = i;
            best_due = packs[i].next_due_ms;
        }
    }
    if (best < 0) return;
    if (!packs[best].mac_valid) {
        ESP_LOGE(TAG, "%s: bad MAC in zxh_config.h", packs[best].cfg->name);
        packs[best].next_due_ms = now + ZXH_POLL_INTERVAL_S * 1000u;
        return;
    }

    active = best;
    zxh_pack_t *p = &packs[active];
    memcpy(remote_bda, p->mac, 6);
    p->bus_addr_known = false; /* re-discover the bus address every connection */
    p->cells_valid = false;
    zxh_pack_begin_cycle(p);
    ESP_LOGI(TAG, "%s: polling now (%s addr)", p->cfg->name, p->use_public ? "public" : "random");

    reset_conn_flags();
    state = ST_ACTIVE;
    connect_deadline = now + ZXH_CONNECT_TIMEOUT_MS;
    cycle_deadline = now + ZXH_CYCLE_TIMEOUT_MS;

    /* BLE-only Bluedroid on the C6 drops the legacy open(); use enh_open.
     * is_aux MUST be true on the C6 (BLE 5.0-only controller) even for
     * legacy (4.x) advertisers like these BMS boards — otherwise the
     * connection is never established (failure reason 0x3e). */
    esp_ble_gatt_creat_conn_params_t conn = {0};
    memcpy(conn.remote_bda, p->mac, 6);
    conn.remote_addr_type = p->use_public ? BLE_ADDR_TYPE_PUBLIC : BLE_ADDR_TYPE_RANDOM;
    conn.is_direct = true;
    conn.is_aux = true;
    if (esp_ble_gattc_enh_open(gattc_if, &conn) != ESP_OK)
        ESP_LOGW(TAG, "%s: open rejected", p->cfg->name); /* connect_deadline handles it */
}

static void process_active(void)
{
    zxh_pack_t *p = &packs[active];
    uint32_t now = now_ms();

    if (link_lost) {
        end_cycle(false);
        return;
    }

    if (!profile_ready) {
        if (open_done && !search_started) {
            search_started = true;
            if (esp_ble_gattc_search_service(gattc_if, conn_id, NULL) != ESP_OK) {
                ESP_LOGW(TAG, "%s: search_service rejected", p->cfg->name);
                end_cycle(false);
                return;
            }
        }
        if (search_done && !notify_req_sent) {
            if (!search_at) search_at = now;
            if (!mtu_done && now - search_at < 800) return; /* let MTU exchange land first */
            find_profile();
            if (notify_handle && write_handle) {
                /* Two-step enable: bluedroid filters incoming notifications by
                 * its register table (needs register_for_notify), but on the
                 * C6 that call does not reliably write the actual CCCD, so
                 * also do BlueZ's exact CCCD write explicitly. */
                esp_ble_gattc_register_for_notify(gattc_if, remote_bda, notify_handle);
                esp_bt_uuid_t cc = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = { .uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG },
                };
                esp_gattc_descr_elem_t de;
                uint16_t dn = 1;
                uint16_t cccd = notify_handle + 1;
                if (esp_ble_gattc_get_descr_by_char_handle(gattc_if, conn_id, notify_handle, cc, &de,
                                                           &dn) == ESP_GATT_OK && dn)
                    cccd = de.handle;
                uint8_t val[2] = { 0x01, 0x00 };
                esp_ble_gattc_write_char_descr(gattc_if, conn_id, cccd, 2, val, ESP_GATT_WRITE_TYPE_RSP,
                                               ESP_GATT_AUTH_REQ_NONE);
            } else {
                notify_failed = true;
            }
            notify_req_sent = true;
        }
        if (notify_ok) {
            profile_ready = true;
            zxh_cmd_t *cmd = zxh_pack_current_cmd(p);
            if (cmd) send_current_cmd(p, cmd);
        } else if (notify_failed || now > connect_deadline) {
            end_cycle(false);
        }
        return;
    }

    if (write_pending && now - attempt_deadline >= WRITE_ACK_TIMEOUT_MS) write_pending = false;

    zxh_cmd_t *cmd = zxh_pack_current_cmd(p);
    if (!cmd) {
        end_cycle(true);
        return;
    }
    if (!write_pending) {
        uint8_t data[ZXH_RX_BUF_SIZE];
        int len;
        uint8_t want_addr = (cmd->kind == ZXH_CMD_DISCOVER_ADDRESS) ? 0 : p->bus_addr;
        if (rx_extract(want_addr, cmd->function, data, &len)) {
            if (zxh_pack_handle_response(p, cmd, data, len)) {
                zxh_pack_cmd_advance(p);
                zxh_cmd_t *next = zxh_pack_current_cmd(p);
                if (!next)
                    end_cycle(true);
                else
                    send_current_cmd(p, next);
            } else if (--cmd->retries_left <= 0) {
                end_cycle(false);
            } else {
                send_current_cmd(p, cmd);
            }
        } else if (now - attempt_deadline >= ATTEMPT_TIMEOUT_MS) {
            if (--cmd->retries_left <= 0) {
                if (cmd->kind == ZXH_CMD_DISCOVER_ADDRESS && !write_with_response) {
                    /* Some boards ignore write-without-response commands;
                     * retry forcing write-with-response. */
                    ESP_LOGW(TAG, "%s: no response; forcing write-with-response", p->cfg->name);
                    write_with_response = true;
                    cmd->retries_left = 3;
                    send_current_cmd(p, cmd);
                } else {
                    end_cycle(false);
                }
            } else {
                send_current_cmd(p, cmd);
            }
        }
    }
    if (now > cycle_deadline) end_cycle(false);
}

static void ble_task(void *arg)
{
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_cb));
    ESP_ERROR_CHECK(esp_ble_gattc_register_callback(gattc_cb));
    ESP_ERROR_CHECK(esp_ble_gattc_app_register(APP_GATTC_ID));

    int evt;
    while (1) {
        xQueueReceive(evt_q, &evt, pdMS_TO_TICKS(50));

        if (!gattc_ready && reg_gattc_if != ESP_GATT_IF_NONE) {
            gattc_if = reg_gattc_if;
            gattc_ready = true;
            ESP_LOGI(TAG, "BLE GATTC ready");
        }

        if (state == ST_IDLE) {
            if (gattc_ready) start_next_cycle();
        } else {
            process_active();
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void zxh_ble_start(void)
{
    evt_q = xQueueCreate(16, sizeof(int));
    rx_mtx = xSemaphoreCreateMutex();
    pack_count = ZXH_PACK_COUNT > ZXH_MAX_PACKS ? ZXH_MAX_PACKS : ZXH_PACK_COUNT;
    for (int i = 0; i < pack_count; i++) {
        zxh_pack_init(&packs[i], &ZXH_PACKS[i]);
        packs[i].next_due_ms = 10000u + (uint32_t)i * 3000u; /* stagger first round */
    }
    xTaskCreate(ble_task, "zxh_ble", 6144, NULL, 6, NULL);
}
