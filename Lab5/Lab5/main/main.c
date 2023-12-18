#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <freertos/semphr.h>
#include <esp_timer.h>
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO 8
#define I2C_MASTER_SDA_IO 10
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define SHTC3_ADDR 0x70

#define TRIG_PIN GPIO_NUM_3
#define ECHO_PIN GPIO_NUM_2

float readSHTC3() {
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
    //uint16_t humidity = (data[3] << 8) | data[4];                                                      
                                                                                                       
    float temperature_C = -45 + (175 * (float)temperature / 65535);                                    
    //float temperature_F = temperature_C * (9/5) + 32;                                                  
    //float humidity_RH = 100 * (float)humidity / 65535;                                                 
                                                                                                       
    //printf("Temperature is %.0fC (or %.0fF) with a %.0f%% humidity\n", temperature_C, temperature_F, humidity_RH);                                                                                            
 // printf("Humidity: %.2f%%\n", humidity_RH);
    return temperature_C;
}

void init_hcsr04() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << TRIG_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    io_conf.pin_bit_mask = (1ULL << ECHO_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);
}

float get_distance() {
    // Trigger the sensor
    gpio_set_level(TRIG_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level(TRIG_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(TRIG_PIN, 0);

    // Wait for the pulse on the Echo pin
    while (gpio_get_level(ECHO_PIN) == 0) {}
    int64_t start = esp_timer_get_time();

    while (gpio_get_level(ECHO_PIN) == 1) {}
    int64_t end = esp_timer_get_time();
	
    // float temp = readSHTC3();
    // Calculate distance based on the speed of sound (340 m/s)
    float distance = (float)(end - start);

    return distance;
}

void app_main() {
    init_hcsr04();
	
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
	float temp = readSHTC3();	
        float distance = get_distance();
	float d = (distance * (float)(0.0331 + 0.00006 * temp)) / 2.0;
        printf("Distance: %.2f cm at %.0fC\n", d, temp);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

