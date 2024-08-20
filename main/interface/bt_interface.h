#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "spp_task.h"

typedef enum {
    BT_SPP_INIT_EVT                    = 0,                /*!< When SPP is initialized, the event comes */
    BT_SPP_UNINIT_EVT                  = 1,                /*!< When SPP is deinitialized, the event comes */
    BT_SPP_DISCOVERY_COMP_EVT          = 8,                /*!< When SDP discovery complete, the event comes */
    BT_SPP_OPEN_EVT                    = 26,               /*!< When SPP Client connection open, the event comes */
    BT_SPP_CLOSE_EVT                   = 27,               /*!< When SPP connection closed, the event comes */
    BT_SPP_START_EVT                   = 28,               /*!< When SPP server started, the event comes */
    BT_SPP_CL_INIT_EVT                 = 29,               /*!< When SPP client initiated a connection, the event comes */
    BT_SPP_DATA_IND_EVT                = 30,               /*!< When SPP connection received data, the event comes, only for ESP_SPP_MODE_CB */
    BT_SPP_CONG_EVT                    = 31,               /*!< When SPP connection congestion status changed, the event comes, only for ESP_SPP_MODE_CB */
    BT_SPP_WRITE_EVT                   = 33,               /*!< When SPP write operation completes, the event comes, only for ESP_SPP_MODE_CB */
    BT_SPP_SRV_OPEN_EVT                = 34,               /*!< When SPP Server connection open, the event comes */
    BT_SPP_SRV_STOP_EVT                = 35,               /*!< When SPP server stopped, the event comes */
    BT_SPP_VFS_REGISTER_EVT            = 36,               /*!< When SPP VFS register, the event comes */
    BT_SPP_VFS_UNREGISTER_EVT          = 37,               /*!< When SPP VFS unregister, the event comes */
} bt_spp_event_t;


typedef struct bt_parameter {
    bt_spp_event_t      event;
    uint8_t             mode;
    char                *device_name;
    char                *log_tag;
} bt_parameter_t;

void bluetooth_init(bt_parameter_t *param);
bt_spp_event_t bluetooth_get_event(void);
char* bluetooth_get_message(void);
