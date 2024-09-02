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

static uint16_t ad1115_set_config(void){
    uint16_t config =   ADS1X15_REG_CONFIG_CQUE_NONE    |     // Disable the comparator (default val)
                        ADS1X15_REG_CONFIG_CLAT_NONLAT  |   // Non-latching (default val)
                        ADS1X15_REG_CONFIG_CPOL_ACTVLOW |  // Alert/Rdy active low   (default val)
                        ADS1X15_REG_CONFIG_CMODE_TRAD   |    // Traditional comparator (default val)
                        ADS1X15_REG_CONFIG_MODE_SINGLE;    // Single-shot mode (default)

    // Set PGA/voltage range
    config |= ADS1X15_REG_CONFIG_PGA_4_096V;

    // Set Samples per Second
    config |= 0x0080;  // 128

    // Set channels
    config |= ADS1X15_REG_CONFIG_MUX_SINGLE_0;  // set P and N inputs for differential

    // Set 'start single-conversion' bit
    config |= ADS1X15_REG_CONFIG_OS_SINGLE;

    return config;
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

    return ret;
}

static esp_err_t ads1115_read_register(i2c_port_t i2c_port, uint8_t device_address, uint8_t command_address, uint8_t *data, uint16_t length) {
    i2c_cmd_handle_t cmd;
    esp_err_t ret = ESP_OK;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                        // generate a start command
    i2c_master_write_byte(cmd,(device_address<<1) | I2C_MASTER_WRITE, ACK_VAL);   // specify address and write command
    i2c_master_write_byte(cmd,command_address,ACK_VAL);   // specify address and write command
    i2c_master_stop(cmd); // generate a stop command
    ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_PERIOD_MS); // send the i2c command
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
      return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                            // generate a start command
    i2c_master_write_byte(cmd,(device_address<<1) | I2C_MASTER_READ, ACK_VAL); // specify address and read command
    for(uint16_t i=0; i< length; i++)
    {
      if(i== length - 1)
      {
        i2c_master_read(cmd, data+i, 1, NACK_VAL); // read all wanted data
      } else
      {
        i2c_master_read(cmd, data+i, 1, ACK_VAL); // read all wanted data        
      }
    }
    i2c_master_stop(cmd); // generate stop command
    ret = i2c_master_cmd_begin(i2c_port, cmd, portTICK_PERIOD_MS); // send the i2c command
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK){
      return ret;
    }

    return ret;
}

static void ad1115_message_handler(void * param){
  uint16_t data = (uint16_t) param;
  uint16_t data_ad;
  uint16_t config;
  uint8_t  buff[3];
  esp_err_t ret;
  do {
    
    /* 1. Write to Config register */
    config = ad1115_set_config();
    buff[0] = ADS1X15_REG_POINTER_CONFIG;
    buff[1] = 0xFF & (config >> 8);
    buff[2] = 0xFF & (config);
    ret = ads1115_write_register(I2C_MASTER_NUM, ADS1X15_ADDRESS, (uint8_t*) &buff[0], 3);
    if(ret != ESP_OK)
    {
      printf("ERROR 1");
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
    /* 2. Write to Convert register and read the data */
    ret = ads1115_read_register(I2C_MASTER_NUM, ADS1X15_ADDRESS, ADS1X15_REG_POINTER_CONVERT, &buff[0], 2);
    if(ret != ESP_OK)
    {
      printf("ERROR 2");
    }
    data_ad = ((uint16_t)(buff[0]) << 8) | ((uint16_t)(buff[1]));
    ESP_LOGI("ad1115-data", "%d", data_ad);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  } while (1);
}

void ad1115_wr_task_start_up(ad115_wr_task_cb_t p_cback, uint16_t weight){
  // xTaskCreate(p_cback, "ad1115 task", 6, (void*)(weight), 4, NULL);
  xTaskCreate(p_cback, "ad1115 task", 6, NULL, 4, NULL);
}

void ad1115_init(void)
{
    esp_err_t ret = i2c_master_init();
    ESP_ERROR_CHECK( ret );
    printf("YAYYYYYYYYYY");
    xTaskCreate(ad1115_message_handler, "ad1115 task", 4096, NULL, 3, NULL);
    // ad1115_wr_task_start_up(ad1115_message_handler, 0);
}