#include <stdio.h>
#include "DFRobot_LCD.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO 8
#define I2C_MASTER_SDA_IO 10
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 400000
#define SHTC3_ADDR 0x70

                                                                                                                                                                                                                   

extern "C" void app_main(void)
{

        DFRobot_LCD lcd(16,2);
	
	lcd.init();
	while(true) {
		uint8_t data[6];
   		uint8_t command[2] = {0x7C, 0xA2};
    		i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    		i2c_master_start(cmd);
    		i2c_master_write_byte(cmd, (SHTC3_ADDR << 1) | I2C_MASTER_WRITE, true);
    		i2c_master_write(cmd, command, 2, true);  // Start Measurement Command
    		i2c_master_stop(cmd);
   		i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(500));
    		i2c_cmd_link_delete(cmd);

    		vTaskDelay(pdMS_TO_TICKS(10));  // Wait for the measurement to complete                                                                                                                                        
                                                                                                                                                                                                                   
    		cmd = i2c_cmd_link_create();                                                                                                                                                                                   
    		i2c_master_start(cmd);                                                                                                                                                                                         
    		i2c_master_write_byte(cmd, (SHTC3_ADDR << 1) | I2C_MASTER_READ, true);                                                                                                                                         
    		i2c_master_read(cmd, data, 6, I2C_MASTER_ACK);                                                                                                                                                                 
    		i2c_master_stop(cmd);                                                                                                                                                                                          	   
		i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(500));                                                                                                                                                    
		i2c_cmd_link_delete(cmd);                                                                                                                                                                                    

		// Parse temperature and humidity data                                                                                                      
    		uint16_t temperature = (data[0] << 8) | data[1];                                                                                                                                                               
                uint16_t humidity = (data[3] << 8) | data[4];                                                                                                                                                                                                                                                                                                                          
    		float temperature_C = -45 + (175 * (float)temperature / 65535);                                                                                                                                                
    		//float temperature_F = temperature_C * (9/5) + 32;                                                                                                                                                              
    		float humidity_RH = 100 * (float)humidity / 65535; 
	        vTaskDelay(pdMS_TO_TICKS(500)); // Read data every 0.5 seconds
		
		char tem[10];
		char hum[10];
		sprintf(tem, "Temp: %.0fC", temperature_C);
		sprintf(hum, "Hum : %.0f%%", humidity_RH);



		lcd.setRGB(255,0,0);
		lcd.setCursor(0,0);
		lcd.printstr(tem);
		lcd.setCursor(0,1);
		lcd.printstr(hum);
	}


}
