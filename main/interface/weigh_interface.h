#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "ad1115_task.h"

typedef enum {
    WG_IDLE                           = 0,                /*!< When SPP is initialized, the event comes */
    WG_CALIBRATE                      = 0,
} wg_event_t;

typedef struct wg_parameter {
    wg_event_t  event;
    uint16_t    weight;
    uint16_t    offset;
} wg_parameter_t;

void weigh_init(wg_parameter_t *param);