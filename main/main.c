#include "config.h"
#include "bt_interface.h"

bt_parameter_t param = {
    .device_name = DEVICE_NAME,
    .log_tag = SPP_TAG,
    .mode = ESP_BT_MODE_CLASSIC_BT
};

void app_main(void)
{
    bluetooth_init(&param);
}
