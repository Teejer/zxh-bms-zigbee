/*
 * zxh_zigbee.c — Zigbee router exposing one custom-cluster endpoint per
 * battery pack, publishing to zigbee2mqtt.
 *
 * Device:  model "ZXH-BMS", endpoints 1..N (pack_1..pack_N in the converter)
 * Cluster: 0xFF10 custom, attributes listed below.
 * Reporting: explicit attribute-report commands to the coordinator after each
 * BLE poll cycle, so no configureReporting is required to get data flowing.
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_zigbee.h"

#include "zxh_config.h"
#include "zxh_proto.h"
#include "zxh_ble.h"
#include "zxh_zigbee.h"

static const char *TAG = "zxh_zb";

#define ZXH_CLUSTER_ID 0xFF10
#define ZXH_DEVICE_ID 0xFF01
#define ZXH_MODEL_ID "\x07" "ZXH-BMS"
#define ZXH_MFG_ID "\x03" "ZXH"

/* Custom cluster attribute IDs — keep in sync with zigbee2mqtt/zxh-bms.js */
#define ATTR_VOLTAGE 0x0001      /* u16 centivolts   */
#define ATTR_CURRENT 0x0002      /* s16 centiamps    */
#define ATTR_POWER 0x0003        /* s16 watts        */
#define ATTR_SOC 0x0004          /* u8 percent       */
#define ATTR_CELL_MIN 0x0005     /* u16 mV           */
#define ATTR_CELL_MAX 0x0006     /* u16 mV           */
#define ATTR_CELL_DELTA 0x0007   /* u16 mV           */
#define ATTR_MOS_TEMP 0x0008     /* s16 0.1 degC     */
#define ATTR_TEMP1 0x0009        /* s16 0.1 degC     */
#define ATTR_TEMP2 0x000A        /* s16 0.1 degC     */
#define ATTR_CYCLES 0x000B       /* u16              */
#define ATTR_HEALTH 0x000C       /* u8 percent       */
#define ATTR_CAPACITY 0x000D     /* u16 0.1 Ah left  */
#define ATTR_FULL_CAP 0x000E     /* u16 0.1 Ah full  */
#define ATTR_CELL_COUNT 0x000F   /* u8               */
#define ATTR_PROTECTION 0x0010   /* u16 bitmap       */
#define ATTR_EQUILIBRIUM 0x0011  /* u32 bitmap       */
#define ATTR_CELL_MV 0x0012      /* octr: len + BE u16 x N */
#define ATTR_ONLINE 0x0013       /* u8               */

typedef struct {
    uint16_t voltage;
    int16_t current;
    int16_t power;
    uint8_t soc;
    uint16_t cell_min, cell_max, cell_delta;
    int16_t mos_temp, temp1, temp2;
    uint16_t cycles;
    uint8_t health;
    uint16_t capacity, full;
    uint8_t cell_count;
    uint16_t protection;
    uint32_t equilibrium;
    uint8_t cell_mv[66];
    uint8_t online;
} attr_store_t;

static attr_store_t store[ZXH_MAX_PACKS];
static bool joined;
static esp_timer_handle_t steer_timer;
static esp_timer_handle_t annce_timer;
static volatile bool steer_armed;
static uint32_t steer_backoff_ms = 5000; /* grows after kicks, resets on join */
static uint32_t full_pending_mask; /* packs that must send all attrs once */
static int ep_count;

#define ZXH_ALL_CHANNELS_MASK 0x07FFF800U /* Zigbee channels 11-26 */

static void custom_cluster_init(uint8_t ep_id) { (void)ep_id; }
static void custom_cluster_deinit(uint8_t ep_id) { (void)ep_id; }

static ezb_zcl_status_t custom_cmd_handler(const ezb_zcl_cmd_hdr_t *header, const uint8_t *payload,
                                           uint16_t payload_length)
{
    (void)header;
    (void)payload;
    (void)payload_length;
    return EZB_ZCL_STATUS_UNSUP_CMD; /* read-only cluster: no custom commands */
}

static void register_custom_handlers_once(void)
{
    static bool done;
    if (done) return;
    ezb_zcl_custom_cluster_handlers_t h = {
        .cluster_id = ZXH_CLUSTER_ID,
        .cluster_role = EZB_ZCL_CLUSTER_SERVER,
        .process_cmd_cb = custom_cmd_handler,
    };
    ESP_ERROR_CHECK(ezb_zcl_custom_cluster_handlers_register(&h));
    done = true;
}

static void add_attr(ezb_zcl_cluster_desc_t d, uint16_t id, uint8_t type, const void *val)
{
    ESP_ERROR_CHECK(ezb_zcl_custom_cluster_desc_add_attr(d, id, type,
                                                         EZB_ZCL_ATTR_ACCESS_READ | EZB_ZCL_ATTR_ACCESS_REPORTING,
                                                         val));
}

esp_err_t zxh_zigbee_create_device(void)
{
    ezb_af_device_desc_t dev_desc = ezb_af_create_device_desc();

    for (int i = 0; i < ep_count; i++) {
        /* Attribute storage is static; values are pushed via publish_pack(). */
        memset(&store[i], 0, sizeof(store[i]));
        store[i].cell_mv[0] = 64; /* reserve full octet-string length */

        ezb_zcl_basic_cluster_server_config_t basic_cfg = {
            .zcl_version = EZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
            .power_source = EZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE,
        };
        ezb_zcl_cluster_desc_t basic = ezb_zcl_basic_create_cluster_desc(&basic_cfg, EZB_ZCL_CLUSTER_SERVER);
        ezb_zcl_basic_cluster_desc_add_attr(basic, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)ZXH_MFG_ID);
        ezb_zcl_basic_cluster_desc_add_attr(basic, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)ZXH_MODEL_ID);

        ezb_zcl_custom_cluster_config_t custom_cfg = {
            .cluster_id = ZXH_CLUSTER_ID,
            .init_func = custom_cluster_init,
            .deinit_func = custom_cluster_deinit,
        };
        ezb_zcl_cluster_desc_t custom = ezb_zcl_custom_create_cluster_desc(&custom_cfg, EZB_ZCL_CLUSTER_SERVER);
        attr_store_t *s = &store[i];
        add_attr(custom, ATTR_VOLTAGE, EZB_ZCL_ATTR_TYPE_UINT16, &s->voltage);
        add_attr(custom, ATTR_CURRENT, EZB_ZCL_ATTR_TYPE_INT16, &s->current);
        add_attr(custom, ATTR_POWER, EZB_ZCL_ATTR_TYPE_INT16, &s->power);
        add_attr(custom, ATTR_SOC, EZB_ZCL_ATTR_TYPE_UINT8, &s->soc);
        add_attr(custom, ATTR_CELL_MIN, EZB_ZCL_ATTR_TYPE_UINT16, &s->cell_min);
        add_attr(custom, ATTR_CELL_MAX, EZB_ZCL_ATTR_TYPE_UINT16, &s->cell_max);
        add_attr(custom, ATTR_CELL_DELTA, EZB_ZCL_ATTR_TYPE_UINT16, &s->cell_delta);
        add_attr(custom, ATTR_MOS_TEMP, EZB_ZCL_ATTR_TYPE_INT16, &s->mos_temp);
        add_attr(custom, ATTR_TEMP1, EZB_ZCL_ATTR_TYPE_INT16, &s->temp1);
        add_attr(custom, ATTR_TEMP2, EZB_ZCL_ATTR_TYPE_INT16, &s->temp2);
        add_attr(custom, ATTR_CYCLES, EZB_ZCL_ATTR_TYPE_UINT16, &s->cycles);
        add_attr(custom, ATTR_HEALTH, EZB_ZCL_ATTR_TYPE_UINT8, &s->health);
        add_attr(custom, ATTR_CAPACITY, EZB_ZCL_ATTR_TYPE_UINT16, &s->capacity);
        add_attr(custom, ATTR_FULL_CAP, EZB_ZCL_ATTR_TYPE_UINT16, &s->full);
        add_attr(custom, ATTR_CELL_COUNT, EZB_ZCL_ATTR_TYPE_UINT8, &s->cell_count);
        add_attr(custom, ATTR_PROTECTION, EZB_ZCL_ATTR_TYPE_MAP16, &s->protection);
        add_attr(custom, ATTR_EQUILIBRIUM, EZB_ZCL_ATTR_TYPE_UINT32, &s->equilibrium);
        add_attr(custom, ATTR_CELL_MV, EZB_ZCL_ATTR_TYPE_OCTSTR, &s->cell_mv);
        add_attr(custom, ATTR_ONLINE, EZB_ZCL_ATTR_TYPE_UINT8, &s->online);

        ezb_af_ep_config_t ep_cfg = {
            .ep_id = (uint8_t)(i + 1),
            .app_profile_id = EZB_AF_HA_PROFILE_ID,
            .app_device_id = ZXH_DEVICE_ID,
            .app_device_version = 0,
        };
        ezb_af_ep_desc_t ep = ezb_af_create_endpoint_desc(&ep_cfg);
        ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep, basic));
        ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep, custom));
        ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev_desc, ep));
    }

    ESP_ERROR_CHECK(ezb_af_device_desc_register(dev_desc));
    return ESP_OK;
}

/* --- commissioning ---------------------------------------------------------- */

static void try_steering(void)
{
    if (joined) return;
    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
    esp_zigbee_lock_release();
}

static void steer_timer_cb(void *arg)
{
    (void)arg;
    steer_armed = false;
    try_steering();
}

static void schedule_steering_retry(void)
{
    if (steer_armed || joined) return;
    steer_armed = true;
    esp_timer_start_once(steer_timer, (uint64_t)steer_backoff_ms * 1000);
}

static void zb_factory_reset_now(void)
{
    ESP_LOGW(TAG, "BOOT held 5s: factory resetting Zigbee stack "
                  "(remove device in z2m, open pairing, I will rejoin)");
    joined = false;
    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_bdb_reset_via_local_action();
    esp_zigbee_lock_release();
    schedule_steering_retry();
}

static void annce_timer_cb(void *arg)
{
    (void)arg;
    /* ZBOSS does not device_announce after a passive rejoin ("Rejoined
     * stored network"), so the coordinator keeps stale addressing and every
     * downlink (interview/config/bind) times out. Announce explicitly. */
    const ezb_zdo_device_annce_req_t req = {0};
    esp_zigbee_lock_acquire(portMAX_DELAY);
    const ezb_err_t ret = ezb_zdo_device_annce_req(&req);
    esp_zigbee_lock_release();
    ESP_LOGI(TAG, "Device announce sent (refresh coordinator routes): 0x%02x", ret);
}

static bool app_signal_handler(const ezb_app_signal_t *app_signal);
static void schedule_selftest(void);

static bool app_signal_handler(const ezb_app_signal_t *app_signal)
{
    ezb_app_signal_type_t type = ezb_app_signal_get_type(app_signal);
    switch (type) {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack init");
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
        break;
    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case EZB_BDB_SIGNAL_DEVICE_REBOOT: {
        ezb_bdb_comm_status_t status = *(ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal);
        if (status != EZB_BDB_STATUS_SUCCESS) {
            ESP_LOGW(TAG, "boot commissioning failed 0x%02x", status);
            schedule_steering_retry();
            break;
        }
        if (ezb_bdb_is_factory_new()) {
            ESP_LOGI(TAG, "Factory new: scanning for a network to join "
                          "(open pairing on the coordinator)");
            try_steering();
        } else {
            ESP_LOGI(TAG, "Rejoined stored network");
            joined = true;
            steer_backoff_ms = 5000;
            zxh_ble_pause(120000); /* let configure + first reports through */
            esp_timer_start_once(annce_timer, 2000 * 1000);
            schedule_selftest();
        }
        break;
    }
    case EZB_BDB_SIGNAL_STEERING: {
        ezb_bdb_comm_status_t status = *(ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal);
        if (status == EZB_BDB_STATUS_SUCCESS) {
            joined = true;
            steer_backoff_ms = 5000;
            ESP_LOGI(TAG, "Joined network, short address 0x%04hx, channel %d",
                     ezb_nwk_get_short_address(), ezb_nwk_get_current_channel());
            /* Keep BLE off the shared radio so the z2m interview and
             * configureReporting downlinks can actually reach us — the full
             * interview + HA discovery + configure chain can take ~3 min. */
            zxh_ble_pause(180000);
            /* Second announce well after the interview window: if z2m's
             * first interview round failed (stale TC entry after a
             * remove/rejoin), this refreshes routing again so the automatic
             * re-interview lands. */
            esp_timer_start_once(annce_timer, 20 * 1000);
            schedule_selftest();
        } else {
            ESP_LOGI(TAG, "Steering failed, status 0x%02x, retrying in 5s", status);
            schedule_steering_retry();
        }
        break;
    }
    case EZB_ZDO_SIGNAL_DEVICE_ANNCE:
        joined = true;
        break;
    case EZB_ZDO_SIGNAL_LEAVE: {
        ESP_LOGW(TAG, "Zigbee LEAVE signal received (kicked or self-left)");
        joined = false;
        /* Usually z2m kicking us after a failed interview. Stay away long
         * enough for it to settle, backing off harder on repeated kicks —
         * rejoining within 2 s just restarts the doomed interview loop. */
        if (steer_backoff_ms <= 5000) steer_backoff_ms = 30000;
        else if (steer_backoff_ms < 300000) steer_backoff_ms *= 2;
        schedule_steering_retry();
        break;
    }
    default:
        break;
    }
    return true;
}

/* --- publish ---------------------------------------------------------------- */

static void set_attr(uint8_t ep, uint16_t attr, void *val)
{
    ezb_zcl_set_attr_value(ep, ZXH_CLUSTER_ID, EZB_ZCL_CLUSTER_SERVER, attr, EZB_ZCL_STD_MANUF_CODE, val, false);
}

/* Queue one unsolicited AttributeReport for (ep, attr) to the coordinator. */
static ezb_err_t report_one(uint8_t ep, uint16_t attr)
{
    ezb_zcl_report_attr_cmd_t cmd = {
        .cmd_ctrl =
            {
                .dst_addr = EZB_ADDRESS_SHORT(0x0000),
                .dst_ep = 1,
                .src_ep = ep,
                .cluster_id = ZXH_CLUSTER_ID,
                .manuf_code = EZB_ZCL_STD_MANUF_CODE,
                .fc =
                    {
                        .direction = EZB_ZCL_CMD_DIRECTION_TO_CLI,
                        .dis_default_rsp = true,
                    },
            },
        .payload = {.attr_id = attr},
    };
    return ezb_zcl_report_attr_cmd_req(&cmd);
}

/* Diagnostic: after joining, push a distinctive sentinel (voltage=0xABCD,
 * soc=0x7E, online=1) to every endpoint so z2m visibly updates all packs at
 * once. Confirms the report path end to end; real polls overwrite it. */
void zxh_zigbee_selftest_reports(void)
{
    if (!joined) {
        ESP_LOGW(TAG, "self-test skipped: not joined");
        return;
    }
    const uint16_t volt = 0xABCD;
    const uint8_t soc = 0x7E, online = 1;
    int sent = 0;
    for (int i = 0; i < ep_count; i++) {
        uint8_t ep = (uint8_t)(i + 1);
        esp_zigbee_lock_acquire(portMAX_DELAY);
        set_attr(ep, ATTR_VOLTAGE, (void *)&volt);
        set_attr(ep, ATTR_SOC, (void *)&soc);
        set_attr(ep, ATTR_ONLINE, (void *)&online);
        ezb_err_t r1 = report_one(ep, ATTR_VOLTAGE);
        esp_zigbee_lock_release();
        vTaskDelay(pdMS_TO_TICKS(60));
        esp_zigbee_lock_acquire(portMAX_DELAY);
        ezb_err_t r2 = report_one(ep, ATTR_SOC);
        esp_zigbee_lock_release();
        vTaskDelay(pdMS_TO_TICKS(60));
        esp_zigbee_lock_acquire(portMAX_DELAY);
        ezb_err_t r3 = report_one(ep, ATTR_ONLINE);
        esp_zigbee_lock_release();
        if (r1 == EZB_ERR_NONE && r3 == EZB_ERR_NONE) sent++;
        else ESP_LOGW(TAG, "selftest ep%d errs: volt 0x%x soc 0x%x online 0x%x",
                      ep, (unsigned) r1, (unsigned) r2, (unsigned) r3);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    ESP_LOGW(TAG, "SELF-TEST: sent sentinel reports to %d/%d endpoints "
                  "(expect voltage=439.81V soc=126 online=true in z2m)",
             sent, ep_count);
}

void zxh_zigbee_publish_pack(int idx, const zxh_pack_t *pack, bool online)
{
    if (idx < 0 || idx >= ep_count || !pack) return;
    uint8_t ep = (uint8_t)(idx + 1);
    const zxh_values_t *v = &pack->values;

    esp_zigbee_lock_acquire(portMAX_DELAY);
    attr_store_t *s = &store[idx];
    if (online && v->ever_read) {
        s->voltage = v->voltage_cV;
        s->current = v->current_cA;
        s->power = v->power_W;
        s->soc = v->soc;
        s->cell_min = v->cell_min_mv;
        s->cell_max = v->cell_max_mv;
        s->cell_delta = v->cell_delta_mv;
        s->mos_temp = v->mos_temp_c10;
        if (v->temp1_valid) s->temp1 = v->temp1_c10;
        if (v->temp2_valid) s->temp2 = v->temp2_c10;
        s->cycles = v->cycles;
        s->health = v->health;
        s->capacity = v->capacity_cAh;
        s->full = v->full_cAh;
        s->cell_count = v->cell_count;
        s->protection = v->protection;
        s->equilibrium = v->equilibrium;
        memcpy(s->cell_mv, v->cell_mv, (size_t)v->cell_mv[0] + 1);
    }
    s->online = online ? 1 : 0;

    set_attr(ep, ATTR_VOLTAGE, &s->voltage);
    set_attr(ep, ATTR_CURRENT, &s->current);
    set_attr(ep, ATTR_POWER, &s->power);
    set_attr(ep, ATTR_SOC, &s->soc);
    set_attr(ep, ATTR_CELL_MIN, &s->cell_min);
    set_attr(ep, ATTR_CELL_MAX, &s->cell_max);
    set_attr(ep, ATTR_CELL_DELTA, &s->cell_delta);
    set_attr(ep, ATTR_MOS_TEMP, &s->mos_temp);
    set_attr(ep, ATTR_TEMP1, &s->temp1);
    set_attr(ep, ATTR_TEMP2, &s->temp2);
    set_attr(ep, ATTR_CYCLES, &s->cycles);
    set_attr(ep, ATTR_HEALTH, &s->health);
    set_attr(ep, ATTR_CAPACITY, &s->capacity);
    set_attr(ep, ATTR_FULL_CAP, &s->full);
    set_attr(ep, ATTR_CELL_COUNT, &s->cell_count);
    set_attr(ep, ATTR_PROTECTION, &s->protection);
    set_attr(ep, ATTR_EQUILIBRIUM, &s->equilibrium);
    set_attr(ep, ATTR_CELL_MV, &s->cell_mv);
    set_attr(ep, ATTR_ONLINE, &s->online);

    /* Report only attributes whose value actually changed. After a device
     * reboot last_sent is zero, so the first poll of each pack sends
     * everything — spread out per pack, never as one join-time burst. */
    static attr_store_t last_sent[ZXH_MAX_PACKS];
    bool full = (full_pending_mask >> idx) & 1u;
    attr_store_t *last = &last_sent[idx];

    struct {
        uint16_t attr;
        const void *cur;
        const void *prev;
        size_t size;
    } fields[] = {
        {ATTR_VOLTAGE, &s->voltage, &last->voltage, sizeof(s->voltage)},
        {ATTR_CURRENT, &s->current, &last->current, sizeof(s->current)},
        {ATTR_POWER, &s->power, &last->power, sizeof(s->power)},
        {ATTR_SOC, &s->soc, &last->soc, sizeof(s->soc)},
        {ATTR_CELL_MIN, &s->cell_min, &last->cell_min, sizeof(s->cell_min)},
        {ATTR_CELL_MAX, &s->cell_max, &last->cell_max, sizeof(s->cell_max)},
        {ATTR_CELL_DELTA, &s->cell_delta, &last->cell_delta, sizeof(s->cell_delta)},
        {ATTR_MOS_TEMP, &s->mos_temp, &last->mos_temp, sizeof(s->mos_temp)},
        {ATTR_TEMP1, &s->temp1, &last->temp1, sizeof(s->temp1)},
        {ATTR_TEMP2, &s->temp2, &last->temp2, sizeof(s->temp2)},
        {ATTR_CYCLES, &s->cycles, &last->cycles, sizeof(s->cycles)},
        {ATTR_HEALTH, &s->health, &last->health, sizeof(s->health)},
        {ATTR_CAPACITY, &s->capacity, &last->capacity, sizeof(s->capacity)},
        {ATTR_FULL_CAP, &s->full, &last->full, sizeof(s->full)},
        {ATTR_CELL_COUNT, &s->cell_count, &last->cell_count, sizeof(s->cell_count)},
        {ATTR_PROTECTION, &s->protection, &last->protection, sizeof(s->protection)},
        {ATTR_EQUILIBRIUM, &s->equilibrium, &last->equilibrium, sizeof(s->equilibrium)},
        {ATTR_CELL_MV, s->cell_mv, last->cell_mv, sizeof(s->cell_mv)},
        {ATTR_ONLINE, &s->online, &last->online, sizeof(s->online)},
    };

    if (joined) {
        int sent = 0, errs = 0;
        ezb_err_t first_err = EZB_ERR_NONE;
        for (size_t i = 0; i < sizeof(fields) / sizeof(fields[0]); i++) {
            if (!full && memcmp(fields[i].cur, fields[i].prev, fields[i].size) == 0) continue;
            /* Pace reports so we don't flood the radio / coordinator, and
             * release the lock around the pause so pending downlinks
             * (interview, configReport, reads) can interleave. */
            if (sent) {
                esp_zigbee_lock_release();
                vTaskDelay(pdMS_TO_TICKS(ZXH_REPORT_GAP_MS));
                esp_zigbee_lock_acquire(portMAX_DELAY);
            }
            ezb_zcl_report_attr_cmd_t cmd = {
                .cmd_ctrl =
                    {
                        .dst_addr = EZB_ADDRESS_SHORT(0x0000),
                        .dst_ep = 1,
                        .src_ep = ep,
                        .cluster_id = ZXH_CLUSTER_ID,
                        .manuf_code = EZB_ZCL_STD_MANUF_CODE,
                        .fc =
                            {
                                .direction = EZB_ZCL_CMD_DIRECTION_TO_CLI,
                                .dis_default_rsp = true,
                            },
                    },
                .payload = {.attr_id = (uint16_t) fields[i].attr},
            };
            ezb_err_t ret = ezb_zcl_report_attr_cmd_req(&cmd);
            sent++;
            if (ret != EZB_ERR_NONE) {
                /* Queue congested — drop the rest for this pack instead of
                 * piling more onto a jammed APS queue; next poll retries. */
                errs++;
                if (first_err == EZB_ERR_NONE) first_err = ret;
                esp_zigbee_lock_release();
                if (joined) full_pending_mask &= ~(1u << (unsigned)idx);
                ESP_LOGW(TAG, "ep%d: report queue busy (%d sent, first err 0x%x)", ep, sent,
                         (unsigned) ret);
                return;
            }
        }
        ESP_LOGI(TAG, "ep%d: reported %d attrs, %d failed (first err 0x%x)", ep, sent, errs,
                 (unsigned) first_err);
        if (errs) ESP_LOGW(TAG, "ep%d: report failures", ep);
        full_pending_mask &= ~(1u << (unsigned)idx);
    } else {
        ESP_LOGI(TAG, "ep%d published locally (not joined yet, no reports sent)", ep);
    }
    memcpy(last, s, sizeof(*s));
    esp_zigbee_lock_release();
}

/* --- task ------------------------------------------------------------------- */

/* Runs the sentinel sweep a few seconds after joining, off the ZBOSS task. */
#if ZXH_SELFTEST_REPORTS
static void selftest_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(8000));
    zxh_zigbee_selftest_reports();
    vTaskDelete(NULL);
}
#endif

static void schedule_selftest(void)
{
    /* NB: do NOT arm a full-attribute pass on join anymore — 19 attrs x 5
     * endpoints collides with the z2m interview and jams the ZBOSS APS
     * retry queue, whose 7.5 s retransmit storm then starves the BLE
     * scheduler on the shared radio. Reports are strictly on-change now;
     * after a reboot last_sent is zero so the first polls push everything
     * naturally, spread across endpoints. */
    (void)full_pending_mask;
#if ZXH_SELFTEST_REPORTS
    full_pending_mask = (1u << (unsigned)ep_count) - 1u;
    xTaskCreate(selftest_task, "zxh_self", 3072, NULL, 3, NULL);
#endif
}

/* --- status LED (WS2812 on ZXH_RGB_LED_GPIO) --------------------------------- */

#if ZXH_RGB_LED_GPIO >= 0
static rmt_channel_handle_t rgb_chan;
static rmt_encoder_handle_t rgb_enc;

static void rgb_set(uint8_t r, uint8_t g, uint8_t b)
{
    const uint8_t grb[3] = {g, r, b};
    const rmt_transmit_config_t tx = {.loop_count = 0};
    if (rmt_transmit(rgb_chan, rgb_enc, grb, sizeof(grb), &tx) == ESP_OK) {
        rmt_tx_wait_all_done(rgb_chan, portMAX_DELAY);
    }
}

static void led_task(void *arg)
{
    (void)arg;
    const rmt_tx_channel_config_t ch_cfg = {
        .gpio_num = ZXH_RGB_LED_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000, /* 100 ns per tick -> WS2812 bit shapes */
        .mem_block_symbols = 48,
        .trans_queue_depth = 2,
    };
    if (rmt_new_tx_channel(&ch_cfg, &rgb_chan) != ESP_OK) {
        ESP_LOGW(TAG, "RGB LED init failed");
        vTaskDelete(NULL);
        return;
    }
    const rmt_bytes_encoder_config_t enc_cfg = {
        .bit0 = {.duration0 = 4, .duration1 = 8, .level0 = 1, .level1 = 0},
        .bit1 = {.duration0 = 8, .duration1 = 4, .level0 = 1, .level1 = 0},
        .flags.msb_first = 1,
    };
    if (rmt_new_bytes_encoder(&enc_cfg, &rgb_enc) != ESP_OK || rmt_enable(rgb_chan) != ESP_OK) {
        ESP_LOGW(TAG, "RGB LED encoder init failed");
        vTaskDelete(NULL);
        return;
    }
    bool on = false;
    int tick = 0;
    int hold = 0;
    bool fired = false;
    const gpio_config_t io = {
        .pin_bit_mask = (1ULL << ZXH_ZB_RESET_GPIO) | (1ULL << ZXH_ZB_RESET_GPIO2),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    for (;;) {
        const bool pressed = !gpio_get_level(ZXH_ZB_RESET_GPIO) ||
                             !gpio_get_level(ZXH_ZB_RESET_GPIO2);
        if (pressed) {
            hold++;
            if (hold == 50 && !fired) {
                zb_factory_reset_now(); /* BOOT held 5 s: fresh rejoin */
                fired = true;
            }
        } else {
            hold = 0;
            fired = false;
        }
        if (hold > 0 && hold < 52) {
            rgb_set(((tick / 2) % 2) ? 40 : 20, ((tick / 2) % 2) ? 20 : 40, 0); /* orange fast blink */
        } else if (joined) {
            rgb_set(0, 40, 0); /* joined: solid dim green */
        } else {
            if (++tick >= 5) { /* not joined: red blink at 1 Hz */
                tick = 0;
                on = !on;
            }
            rgb_set(on ? 40 : 0, 0, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
#endif /* ZXH_RGB_LED_GPIO >= 0 */

static void zigbee_task(void *arg)
{
    esp_zigbee_config_t config = {
        .device_config =
            {
                .device_type = EZB_NWK_DEVICE_TYPE_ROUTER,
                .install_code_policy = false,
                .zczr_config = {.max_children = 0},
            },
        .platform_config =
            {
                .storage_partition_name = "nvs",
                .radio_config = {.radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE},
            },
    };

    ESP_ERROR_CHECK(esp_zigbee_init(&config));

    ezb_aps_secur_enable_distributed_security(false);
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(ZXH_ALL_CHANNELS_MASK));
    ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(ZXH_ALL_CHANNELS_MASK));
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(app_signal_handler));

    register_custom_handlers_once();
    ESP_ERROR_CHECK(zxh_zigbee_create_device());

    const esp_timer_create_args_t timer_cfg = {.callback = steer_timer_cb, .name = "zxh_steer"};
    ESP_ERROR_CHECK(esp_timer_create(&timer_cfg, &steer_timer));
    const esp_timer_create_args_t annce_cfg = {.callback = annce_timer_cb, .name = "zxh_annce"};
    ESP_ERROR_CHECK(esp_timer_create(&annce_cfg, &annce_timer));

    ESP_ERROR_CHECK(esp_zigbee_start(false));
    ESP_LOGI(TAG, "Zigbee router started (%d endpoints)", ep_count);
    esp_zigbee_launch_mainloop();
}

void zxh_zigbee_start(void)
{
    ep_count = ZXH_PACK_COUNT > ZXH_MAX_PACKS ? ZXH_MAX_PACKS : ZXH_PACK_COUNT;
#if ZXH_RGB_LED_GPIO >= 0
    xTaskCreate(led_task, "zxh_led", 3072, NULL, 2, NULL);
#endif
    xTaskCreate(zigbee_task, "zxh_zb", 8192, NULL, 5, NULL);
}
