#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO           32                         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           33                         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

typedef struct ad1115_parameter {
    uint16_t    weight;
    uint16_t    offset;
} ad1115_parameter_t;

void ad1115_init(void);
void ad1115_wr_task_start_up(ad1115_parameter_t p_cback, int fd);
uint16_t ad1115_get_data(void);