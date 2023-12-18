/* HTTP GET Example using plain POSIX sockets

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "driver/i2c.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "sdkconfig.h"

/* Constants that aren't configurable in menuconfig */
#define WEB_SERVER "192.168.30.193"
#define WEB_PORT "1234"
#define WEB_PATH "/weather"
#define buffsize 2048                                                                                  
                                                                                                       
#define I2C_MASTER_SCL_IO 8                                                                            
#define I2C_MASTER_SDA_IO 10                                                                           
#define I2C_MASTER_NUM I2C_NUM_0                                                                       
#define I2C_MASTER_FREQ_HZ 400000                                                                      
#define SHTC3_ADDR 0x70

static const char *TAG = "example";

static const char *REQUEST = "POST " WEB_PATH " HTTP/1.0\r\n"
    "Host: "WEB_SERVER":"WEB_PORT"\r\n"
    "Content-Type: text/plain\r\n"
    "Content-Length:3\r\n"
    "\r\n"
    "Duo";

float temp;                                                                                            
float hum;                                                                                             
                                                                                                       
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
                                                                                                       
    temp = -45 + (175 * (float)temperature / 65535);                                    
    //float temperature_F = temperature_C * (9/5) + 32;                                                                                                                                                       
    hum = 100 * (float)humidity / 65535;                                                                                                                                                      
                                                                                                       
    //printf("Temperature is %.0fC (or %.0fF) with a %.0f%% humidity\n", temperature_C, temperature_F, humidity_RH);                                                                                                                                                                                                 
 // printf("Humidity: %.2f%%\n", humidity_RH);
    
}


static void http_get_task(void *pvParameters)
{
    const struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[64];
    i2c_config_t conf = {
                .mode = I2C_MODE_MASTER,
                .sda_io_num = I2C_MASTER_SDA_IO,
                .scl_io_num = I2C_MASTER_SCL_IO,
                .sda_pullup_en = GPIO_PULLUP_ENABLE,
                .scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);


    while(1) {
        int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

        if(err != 0 || res == NULL) {
            ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }

        /* Code to print the resolved IP.

           Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
        addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
        ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

        s = socket(res->ai_family, res->ai_socktype, 0);
        if(s < 0) {
            ESP_LOGE(TAG, "... Failed to allocate socket.");
            freeaddrinfo(res);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... allocated socket");

        if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
            ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
            close(s);
            freeaddrinfo(res);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "... connected");
        freeaddrinfo(res);

	readSHTC3();
        char cont[200];
        char request[500];
        snprintf(cont, sizeof(cont), "Tem:%.0fC AND Hum:%.0f%%", temp, hum);
        snprintf(request, sizeof(request), "POST %s HTTP/1.0\r\n"
                "Host: %s:%s\r\n"
                "Content-Type: text/plain\r\n"
                "Content-Length: %d\r\n"
                "\r\n"
                "%s", WEB_PATH, WEB_SERVER, WEB_PORT, strlen(cont), cont);

	
        if (write(s, request, strlen(request)) < 0) {
            ESP_LOGE(TAG, "... socket send failed");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... socket send success");

        struct timeval receiving_timeout;
        receiving_timeout.tv_sec = 5;
        receiving_timeout.tv_usec = 0;
        if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
                sizeof(receiving_timeout)) < 0) {
            ESP_LOGE(TAG, "... failed to set socket receiving timeout");
            close(s);
            vTaskDelay(4000 / portTICK_PERIOD_MS);
            continue;
        }
        ESP_LOGI(TAG, "... set socket receiving timeout success");

        /* Read HTTP response */
        do {
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            for(int i = 0; i < r; i++) {
                putchar(recv_buf[i]);
            }
        } while(r > 0);

        ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.", r, errno);
        close(s);
        for(int countdown = 3; countdown >= 0; countdown--) {
            ESP_LOGI(TAG, "%d... ", countdown);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        ESP_LOGI(TAG, "Starting again!");
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    xTaskCreate(&http_get_task, "http_get_task", 4096, NULL, 5, NULL);
}
