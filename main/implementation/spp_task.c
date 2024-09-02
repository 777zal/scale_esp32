#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "spp_task.h"

#include "esp_vfs.h"
#include "sys/unistd.h"

#define SPP_SERVER_NAME "SPP_SERVER"
static uint16_t  spp_event_callback;
static char *spp_device_name;
static char *spp_tag;
static uint8_t *spp_data = NULL;
static int  spp_size = 0;
static uint8_t data[40];

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
// static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static char *bda2str(uint8_t * bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

static void spp_message_handler(void * param)
{
    int size = 0;
    int fd = (int)param;
    uint8_t *spp_data = NULL;

    spp_data = malloc(SPP_DATA_LEN);
    if (!spp_data) {
        ESP_LOGE(spp_tag, "malloc spp_data failed, fd:%d", fd);
        goto done;
    }

    do {
        /* The frequency of calling this function also limits the speed at which the peer device can send data. */
        size = read(fd, spp_data, SPP_DATA_LEN);
        // spp_size = size;
        if (size < 0) {
            break;
        } else if (size == 0) {
            /* There is no data, retry after 500 ms */
            vTaskDelay(500 / portTICK_PERIOD_MS);
        } else {
            ESP_LOGI(spp_tag, "fd = %d data_len = %d", fd, size);
            esp_log_buffer_hex(spp_tag, spp_data, size);
            for (int i=0; i< size; i++)
            {
                data[i] = spp_data[i];
            }
            // for (int i = 0; i < 20; ++i) {
            //     spp_data[i] = i;
            // }
            spp_size = size;
            // size = write(fd, spp_data, 20);
            /* To avoid task watchdog */
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    } while (1);
done:
    if (spp_data) {
        free(spp_data);
    }
    spp_wr_task_shut_down();
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(spp_tag, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(spp_tag, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(spp_tag, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(spp_tag, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(spp_tag, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(spp_tag, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_EXAMPLE_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(spp_tag, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %"PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(spp_tag, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%"PRIu32, param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(spp_tag, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(spp_tag, "ESP_BT_GAP_MODE_CHG_EVT mode:%d", param->mode_chg.mode);
        break;

    default: {
        ESP_LOGI(spp_tag, "event: %d", event);
        break;
    }
    }
    return;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    char bda_str[18] = {0};

    spp_event_callback = (uint16_t) event;
    ESP_LOGI("BINDING", "%d", spp_event_callback);
    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(spp_tag, "ESP_SPP_INIT_EVT");
            /* Enable SPP VFS mode */
            esp_spp_vfs_register();
        } else {
            ESP_LOGE(spp_tag, "ESP_SPP_INIT_EVT status:%d", param->init.status);
        }
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(spp_tag, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(spp_tag, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(spp_tag, "ESP_SPP_CLOSE_EVT status:%d handle:%"PRIu32" close_by_remote:%d", param->close.status,
                 param->close.handle, param->close.async);
        break;
    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(spp_tag, "ESP_SPP_START_EVT handle:%"PRIu32" sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
                     param->start.scn);
            esp_bt_gap_set_device_name(spp_device_name);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        } else {
            ESP_LOGE(spp_tag, "ESP_SPP_START_EVT status:%d", param->start.status);
        }
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(spp_tag, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(spp_tag, "ESP_SPP_SRV_OPEN_EVT status:%d handle:%"PRIu32", rem_bda:[%s]", param->srv_open.status,
                 param->srv_open.handle, bda2str(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));
        if (param->srv_open.status == ESP_SPP_SUCCESS) {
            spp_wr_task_start_up(spp_message_handler, param->srv_open.fd);
        }
        break;
    case ESP_SPP_VFS_REGISTER_EVT:
        if (param->vfs_register.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(spp_tag, "ESP_SPP_VFS_REGISTER_EVT");
            esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
        } else {
            ESP_LOGE(spp_tag, "ESP_SPP_VFS_REGISTER_EVT status:%d", param->vfs_register.status);
        }
        break;
    default:
        break;
    }
}

void spp_wr_task_start_up(spp_wr_task_cb_t p_cback, int fd)
{
    xTaskCreate(p_cback, "write_read", 4096, (void *)fd, 5, NULL);
}

void spp_wr_task_shut_down(void)
{
    vTaskDelete(NULL);
}

uint16_t spp_get_event(void)
{
    return spp_event_callback;
}

uint8_t *spp_get_data(void)
{
    return &data[0];
}

int spp_is_message_available(void)
{
    // if(spp_data != NULL){
    //     data = spp_data;
    //     ESP_LOGI("check", "%d", *data);
    //     // ESP_LOGI("size", "%d", spp_size);
    // }
    return spp_size;
}

void spp_task_init(char *device_name, uint8_t mode,  char *log_tag)
{
    spp_device_name     = device_name;
    spp_tag             = log_tag;
    char bda_str[18] = {0};
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
        ESP_LOGE(spp_tag, "%s initialize controller failed", __func__);
        return;
    } else {
        ESP_LOGI(spp_tag, "initialize controller OK");
    }

    if (esp_bt_controller_enable(mode) != ESP_OK) {
        ESP_LOGE(spp_tag, "%s enable controller failed", __func__);
        return;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();

    if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
        ESP_LOGE(spp_tag, "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if (esp_bluedroid_enable() != ESP_OK) {
        ESP_LOGE(spp_tag, "%s enable bluedroid failed", __func__);
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(spp_tag, "%s gap register failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if (esp_spp_register_callback(esp_spp_cb) != ESP_OK) {
        ESP_LOGE(spp_tag, "%s spp register failed", __func__);
        return;
    }

    esp_spp_cfg_t bt_spp_cfg = BT_SPP_DEFAULT_CONFIG();
    if (esp_spp_enhanced_init(&bt_spp_cfg) != ESP_OK) {
        ESP_LOGE(spp_tag, "%s spp init failed", __func__);
        return;
    }

#if (CONFIG_EXAMPLE_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    ESP_LOGI(spp_tag, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
}