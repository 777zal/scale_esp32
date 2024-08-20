#include "bt_interface.h"


static char *device_name;

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

