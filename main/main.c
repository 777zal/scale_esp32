#include "config.h"
#include "bt_interface.h"

bt_parameter_t param = {
    .device_name = DEVICE_NAME,
    .log_tag = SPP_TAG,
    .mode = ESP_BT_MODE_CLASSIC_BT
};

static void main_task(void * par)
{

    while(1){
        // printf("test \r\n");
        ESP_LOGI("test", "%d", param.event);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }   

    
}

void app_main(void)
{
    xTaskCreate(main_task, "task_read", 4096, NULL, 3, NULL);
    bluetooth_init(&param);
}
