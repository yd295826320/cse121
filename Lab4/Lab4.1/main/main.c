#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO 8          // SCL pin
#define I2C_MASTER_SDA_IO 10         // SDA pin
#define I2C_MASTER_NUM I2C_NUM_0     // I2C port number
#define I2C_MASTER_FREQ_HZ 400000    // I2C master clock frequency

#define ICM42670P_I2C_ADDR 0x68      // ICM-42670-P I2C address (default address)
#define X1 0x0B
#define X0 0x0C
#define Y1 0x0D
#define Y0 0x0E




static const char *TAG = "ICM42670P";

esp_err_t i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
	.scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    return i2c_param_config(I2C_MASTER_NUM, &conf);
}

void initialize_ICM42670P() {
    esp_err_t ret;
    i2c_cmd_handle_t cmd;
    // Initialize I2C master
    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C master initialization failed");
        return;
    }

    // Install I2C driver
    ret = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation failed");
        return;
    }
    

    // Configure power management settings to power up the accelerometer in "Low Power (LP) Mode"
    //uint8_t pwr_mgmt0_cmd[] = {0x1F, 0b00011111};  // Command to set ACCEL_MODE to "Low Power (LP) Mode"
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ICM42670P_I2C_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x1F, true);
    i2c_master_write_byte(cmd, 0b00011111, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure power management settings");
        return;
    }
    /*
    // Initialize ICM-42670-P sensor                                                                                                                                                                               
    uint8_t init_cmd[] = {0x20, 0b01100010};  // Command to initialize sensor                                                                                                                                            
    cmd = i2c_cmd_link_create();                                                                                                                                                                  
    i2c_master_start(cmd);                                                                                                                                                                                         
    i2c_master_write_byte(cmd, ICM42670P_I2C_ADDR << 1 | I2C_MASTER_WRITE, true);                                                                                                                                  
    i2c_master_write(cmd, init_cmd, sizeof(init_cmd), true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ICM-42670-P initialization failed");
        return;
    }
    */
}

uint8_t read(uint8_t addr){
	uint8_t accel_data = 0;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ICM42670P_I2C_ADDR << 1 | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, addr, true);  // Register address for ACCEL_DATA_X1
        i2c_master_start(cmd);  // Send a repeated start condition
        i2c_master_write_byte(cmd, ICM42670P_I2C_ADDR << 1 | I2C_MASTER_READ, true);
        i2c_master_read_byte(cmd, &accel_data, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
	return accel_data;
}






void app_main() {
	
    initialize_ICM42670P();
    while (1) {
        // Read accelerometer data
            int16_t accel_x = (int16_t)((read(X1) << 8) | read(X0));
            int16_t accel_y = (int16_t)((read(Y1) << 8) | read(Y0));

	    ESP_LOGI(TAG, "Acceleration X: %i, Y: %i", accel_x, accel_y);

            // Determine inclination direction
            char inclination_str[20];
            if (accel_x > 1000) {
                strcpy(inclination_str, "LEFT");
            } else if (accel_x < -1000) {
                strcpy(inclination_str, "RIGHT");
            } else {
                strcpy(inclination_str, "");
            }

            if (accel_y > 1000) {
                if (strlen(inclination_str) > 0) {
                    strcat(inclination_str, " ");
                }
                strcat(inclination_str, "UP");
            } else if (accel_y < -1000) {
                if (strlen(inclination_str) > 0) {
                    strcat(inclination_str, " ");
                }
                strcat(inclination_str, "DOWN");
            }

            // Print the inclination
            if (strlen(inclination_str) > 0) {
                ESP_LOGI(TAG, "Board inclination: %s", inclination_str);
            } else {
                ESP_LOGI(TAG, "Board inclination: LEVEL");
            }

        // Add a delay to control the sampling rate
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Adjust the delay as needed
    }
}

