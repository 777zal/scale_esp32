#include "weigh_interface.h"

void weigh_init(wg_parameter_t *param)
{
    ad1115_init((uint8_t*) &param->event, param->offset, param->weight);
}