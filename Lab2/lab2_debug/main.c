#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO 8
#define I2C_MASTER_SDA_IO 10
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define SHTC3_ADDR 0x70

void readSHTC3() {
    uint8_t data[6];
    uint8_t command[2] = {0x7C, 0xA2};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHTC3_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, command, 2, true);  // Start Measurement Command
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    vTaskDelay(pdMS_TO_TICKS(15));  // Wait for the measurement to complete

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHTC3_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 6, I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    // Parse temperature and humidity data
    uint16_t temperature = (data[0] << 8) | data[1];
    uint16_t humidity = (data[3] << 8) | data[4];

    float temperature_C = -45 + (175 * (float)temperature / 65535);
    float temperature_F = temperature_C * (9/5) + 32;
    float humidity_RH = 100 * (float)humidity / 65535;

    printf("Temperature is %.0fC (or %.0fF) with a %.0f%% humidity\n", temperature_C, temperature_F, humidity_RH);
 // printf("Humidity: %.2f%%\n", humidity_RH);
}

void app_main() {
    i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    while (1) {
        readSHTC3();
        vTaskDelay(pdMS_TO_TICKS(2000)); // Read data every 2 seconds
    }
}

