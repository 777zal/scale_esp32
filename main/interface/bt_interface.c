#include "bt_interface.h"


void bluetooth_init(bt_parameter_t *param)
{
    spp_task_init(param->device_name, param->mode, param->log_tag);
}

bt_spp_event_t bluetooth_get_event(void)
{
    return spp_get_event();
}

int bluetooth_get_message(void)
{
    return spp_is_message_available(); 
}

uint8_t *bluetooth_get_data(void)
{
    return spp_get_data();
}