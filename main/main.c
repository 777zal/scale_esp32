#include "config.h"
#include "bt_interface.h"

bt_parameter_t param = {
    .device_name = DEVICE_NAME,
    .log_tag = SPP_TAG,
    .mode = ESP_BT_MODE_CLASSIC_BT
};
uint8_t *data;

static void main_task(void * par)
{
    int size;
    while(1){
        size = bluetooth_get_message();
        ESP_LOGI("test", "%d, %d", bluetooth_get_event(), size);
        
        if(size > 0){
            ESP_LOGI("debug", "%d", *bluetooth_get_data());
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }      
}

void app_main(void)
{
    xTaskCreate(main_task, "task_read", 4096, NULL, 3, NULL);
    bluetooth_init(&param);
}
