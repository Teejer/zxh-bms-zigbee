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

#include "esp_zigbee.h"

#include "zxh_config.h"
#include "zxh_proto.h"
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
static volatile bool steer_armed;
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
            .power_source = EZB_ZCL_BASIC_POWER_SOURCE_BATTERY,
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
    esp_timer_start_once(steer_timer, 5000 * 1000);
}

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
        }
        break;
    }
    case EZB_BDB_SIGNAL_STEERING: {
        ezb_bdb_comm_status_t status = *(ezb_bdb_comm_status_t *)ezb_app_signal_get_params(app_signal);
        if (status == EZB_BDB_STATUS_SUCCESS) {
            joined = true;
            ESP_LOGI(TAG, "Joined network, short address 0x%04hx, channel %d",
                     ezb_nwk_get_short_address(), ezb_nwk_get_current_channel());
        } else {
            ESP_LOGI(TAG, "No network joinable yet, retrying in 5s");
            schedule_steering_retry();
        }
        break;
    }
    case EZB_ZDO_SIGNAL_DEVICE_ANNCE:
        joined = true;
        break;
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

    if (joined) {
        static const uint16_t report_attrs[] = {
            ATTR_VOLTAGE, ATTR_CURRENT, ATTR_POWER,  ATTR_SOC,      ATTR_CELL_MIN,
            ATTR_CELL_MAX, ATTR_CELL_DELTA, ATTR_MOS_TEMP, ATTR_TEMP1, ATTR_TEMP2,
            ATTR_CYCLES, ATTR_HEALTH, ATTR_CAPACITY, ATTR_FULL_CAP, ATTR_CELL_COUNT,
            ATTR_PROTECTION, ATTR_EQUILIBRIUM, ATTR_CELL_MV, ATTR_ONLINE,
        };
        int errs = 0;
        ezb_err_t first_err = EZB_ERR_NONE;
        for (size_t i = 0; i < sizeof(report_attrs) / sizeof(report_attrs[0]); i++) {
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
                .payload = {.attr_id = report_attrs[i]},
            };
            ezb_err_t ret = ezb_zcl_report_attr_cmd_req(&cmd);
            if (ret != EZB_ERR_NONE) {
                errs++;
                if (first_err == EZB_ERR_NONE) first_err = ret;
            }
        }
        if (errs)
            ESP_LOGW(TAG, "ep%d: %d/%zu report cmds failed, first err=0x%x", ep, errs,
                     sizeof(report_attrs) / sizeof(report_attrs[0]), (unsigned) first_err);
    } else {
        ESP_LOGI(TAG, "ep%d published locally (not joined to a network yet, no reports sent)", ep);
    }
    esp_zigbee_lock_release();
}

/* --- task ------------------------------------------------------------------- */

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

    ESP_ERROR_CHECK(esp_zigbee_start(false));
    ESP_LOGI(TAG, "Zigbee router started (%d endpoints)", ep_count);
    esp_zigbee_launch_mainloop();
}

void zxh_zigbee_start(void)
{
    ep_count = ZXH_PACK_COUNT > ZXH_MAX_PACKS ? ZXH_MAX_PACKS : ZXH_PACK_COUNT;
    xTaskCreate(zigbee_task, "zxh_zb", 8192, NULL, 5, NULL);
}
