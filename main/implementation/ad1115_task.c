#include "ad1115_task.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"

esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


static void ad1115_message_handler(void * param){
    
}

void ad1115_init(void)
{
    esp_err_t ret = i2c_master_init();
    ESP_ERROR_CHECK( ret );
    printf("YAYYYYYYYYYY");
}

static esp_err_t ads1115_write_register(i2c_port_t i2c_port, uint8_t device_address, uint8_t *data, uint16_t length){
    i2c_cmd_handle_t cmd;
    esp_err_t ret = ESP_OK;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                        // generate a start command
    i2c_master_write_byte(cmd,(device_address<<1) | I2C_MASTER_WRITE, ACK_VAL);   // specify address and write command
    for(uint16_t i=0; i<length; i++)
    {
      i2c_master_write_byte(cmd,*(data+i),ACK_VAL); 
    }
    i2c_master_stop(cmd); // generate a stop command
    ret = i2c_master_cmd_begin(i2c_port, cmd, 100 / portTICK_PERIOD_MS); // send the i2c command
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
      return ret;
    }

    
}

static esp_err_t ads1115_read_register(i2c_port_t i2c_port, uint8_t device_address, uint8_t command_address, uint8_t *data, uint16_t length) {
    i2c_cmd_handle_t cmd;
    esp_err_t ret = ESP_OK;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                        // generate a start command
    i2c_master_write_byte(cmd,(device_address<<1) | I2C_MASTER_WRITE, ACK_VAL);   // specify address and write command
    i2c_master_write_byte(cmd,command_address,ACK_VAL);   // specify address and write command
    i2c_master_stop(cmd); // generate a stop command
    ret = i2c_master_cmd_begin(i2c_port, cmd, 100 / portTICK_PERIOD_MS); // send the i2c command
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
      return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                            // generate a start command
    i2c_master_write_byte(cmd,(device_address<<1) | I2C_MASTER_READ,1); // specify address and read command
    for(uint16_t i=0; i< length; i++)
    {
      i2c_master_read(cmd, data, length, 0); // read all wanted data
    }

    i2c_master_stop(cmd); // generate stop command
    ret = i2c_master_cmd_begin(i2c_port, cmd, portTICK_PERIOD_MS); // send the i2c command
    i2c_cmd_link_delete(cmd);
    return ret;
}


static void ad1115_set_config(void){
    uint16_t config =   ADS1X15_REG_CONFIG_CQUE_NONE    |     // Disable the comparator (default val)
                        ADS1X15_REG_CONFIG_CLAT_NONLAT  |   // Non-latching (default val)
                        ADS1X15_REG_CONFIG_CPOL_ACTVLOW |  // Alert/Rdy active low   (default val)
                        ADS1X15_REG_CONFIG_CMODE_TRAD   |    // Traditional comparator (default val)
                        ADS1X15_REG_CONFIG_MODE_SINGLE;    // Single-shot mode (default)

    // Set PGA/voltage range
    config |= ADS1X15_REG_CONFIG_PGA_6_144V;

    // Set Samples per Second
    config |= 0x0080;  // 128

    // Set channels
    config |= ADS1X15_REG_CONFIG_MUX_SINGLE_0;  // set P and N inputs for differential

    // Set 'start single-conversion' bit
    config |= ADS1X15_REG_CONFIG_OS_SINGLE;

    int ret;
    
    ret = ads1115_write_register(I2C_MASTER_NUM, ADS1X15_ADDRESS, (uint8_t*) &config, 2);
    ESP_LOGI(ADS1115_TAG, "Return value from pointing to the config register: %d", ret);

}

// static inline esp_err_t ads1115_write_register(ads1115_t* ads, ads1115_register_addresses_t reg, uint16_t data) {
//     i2c_cmd_handle_t cmd;
//     esp_err_t ret;
//     uint8_t out[2];

//     out[0] = data >> 8; // get 8 greater bits
//     out[1] = data & 0xFF; // get 8 lower bits
//     cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd); // generate a start command
//     i2c_master_write_byte(cmd,(ads->address<<1) | I2C_MASTER_WRITE,1); // specify address and write command
//     i2c_master_write_byte(cmd,reg,1); // specify register
//     i2c_master_write(cmd,out,2,1); // write it
//     i2c_master_stop(cmd); // generate a stop command
//     ret = i2c_master_cmd_begin(ads->i2c_port, cmd, ads->max_ticks); // send the i2c command
//     i2c_cmd_link_delete(cmd);
//     ads->last_reg = reg; // change the internally saved register
//     return ret;
// }

// static esp_err_t ads1115_read_register(ads1115_t* ads, ads1115_register_addresses_t reg, uint8_t* data, uint8_t len) {
//     i2c_cmd_handle_t cmd;
//     esp_err_t ret;

//     if(ads->last_reg != reg) { // if we're not on the correct register, change it
//       cmd = i2c_cmd_link_create();
//       i2c_master_start(cmd);
//       i2c_master_write_byte(cmd,(ads->address<<1) | I2C_MASTER_WRITE,1);
//       i2c_master_write_byte(cmd,reg,1);
//       i2c_master_stop(cmd);
//       i2c_master_cmd_begin(ads->i2c_port, cmd, ads->max_ticks);
//       i2c_cmd_link_delete(cmd);
//       ads->last_reg = reg;
//     }
//     cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd); // generate start command
//     i2c_master_write_byte(cmd,(ads->address<<1) | I2C_MASTER_READ,1); // specify address and read command
//     i2c_master_read(cmd, data, len, 0); // read all wanted data
//     i2c_master_stop(cmd); // generate stop command
//     ret = i2c_master_cmd_begin(ads->i2c_port, cmd, ads->max_ticks); // send the i2c command
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }