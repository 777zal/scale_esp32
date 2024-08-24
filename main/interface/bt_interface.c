#include "bt_interface.h"


static char *device_name;


void bluetooth_init(bt_parameter_t *param)
{
    spp_task_init((uint16_t)param->event, param->mode, param->device_name, param->log_tag);
}

bt_spp_event_t bluetooth_get_event(void)
{
    return BT_SPP_INIT_EVT;
}

char* bluetooth_get_message(void)
{
    return NULL;
}

