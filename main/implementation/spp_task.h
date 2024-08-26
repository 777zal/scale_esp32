
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "nvs.h"
#include "nvs_flash.h"

#define SPP_TASK_TAG                       "SPP_TASK"

#define SPP_TASK_SIG_WORK_DISPATCH          (0x01)
#define SPP_DATA_LEN                        100
/**
 * @brief     handler for the dispatched work
 */
typedef void (* spp_task_cb_t) (uint16_t event, void *param);


/* message to be sent */
typedef struct {
    uint16_t             sig;      /*!< signal to spp_task_task */
    uint16_t             event;    /*!< message event id */
    spp_task_cb_t        cb;       /*!< context switch callback */
    void                 *param;   /*!< parameter area needs to be last */
} spp_task_msg_t;

/**
 * @brief     parameter deep-copy function to be customized
 */
typedef void (* spp_task_copy_cb_t) (spp_task_msg_t *msg, void *p_dest, void *p_src);

/**
 * @brief     work dispatcher for the application task
 */
bool spp_task_work_dispatch(spp_task_cb_t p_cback, uint16_t event, void *p_params, int param_len, spp_task_copy_cb_t p_copy_cback);

void spp_task_task_start_up(void);

void spp_task_task_shut_down(void);


/**
 * @brief     handler for write and read
 */
typedef void (* spp_wr_task_cb_t) (void *fd);

void spp_wr_task_start_up(spp_wr_task_cb_t p_cback, int fd);

void spp_wr_task_shut_down(void);

void spp_task_init(char *device_name, uint8_t mode,  char *log_tag);

uint16_t spp_get_event(void);
uint8_t  *spp_get_data(void);
int spp_is_message_available(void);
