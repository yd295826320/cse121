/*
 * HTTPS GET Example using plain Mbed TLS sockets
 *
 * Contacts the howsmyssl.com API via TLS v1.2 and reads a JSON
 * response.
 *
 * Adapted from the ssl_client1 example in Mbed TLS.
 *
 * SPDX-FileCopyrightText: The Mbed TLS Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPDX-FileContributor: 2015-2023 Espressif Systems (Shanghai) CO LTD
 */

#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "protocol_examples_common.h"
#include "esp_sntp.h"
#include "esp_netif.h"
#include "driver/i2c.h"


#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "esp_tls.h"
#include "sdkconfig.h"
#if CONFIG_MBEDTLS_CERTIFICATE_BUNDLE
#include "esp_crt_bundle.h"
#endif
#include "time_sync.h"

/* Constants that aren't configurable in menuconfig */
#define WEB_SERVER "www.wttr.in"
#define WEB_PORT "443"
#define WEB_URL "https://www.wttr.in"
#define MY_SERVER "192.168.30.193"
#define MY_PORT "1234"
#define MY_PATH "/weather"
//#define path "/Santa+Cruz?format=%l:+%t+%h"
#define I2C_MASTER_SCL_IO 8                                                                            
#define I2C_MASTER_SDA_IO 10                                                                           
#define I2C_MASTER_NUM I2C_NUM_0                                                                       
#define I2C_MASTER_FREQ_HZ 400000                                                                      
#define SHTC3_ADDR 0x70   

#define SERVER_URL_MAX_SZ 256

static const char *TAG = "example";

/* Timer interval once every day (24 Hours) */
#define TIME_PERIOD (86400000000ULL)
float temp;                                                                                            
float hum;
float temp_f;
//char buff[300];
                                                                                                       
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
                                                                                                       
    vTaskDelay(pdMS_TO_TICKS(20));  // Wait for the measurement to complete                            
                                                                                                       
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
    temp_f = temp * (9/5) + 32;                                                                                                                                                       
    hum = 100 * (float)humidity / 65535;                                                                                                                                                      
                                                                                                       
    //printf("Temperature is %.0fC (or %.0fF) with a %.0f%% humidity\n", temperature_C, temperature_F, humidity_RH);                                                                                                                                                                                                 
 // printf("Humidity: %.2f%%\n", humidity_RH);
    
}


static const char HOWSMYSSL_REQUEST[] = "GET /Santa+Cruz?format=%l:+%t+%h HTTP/1.1\r\n"
                             "Host: "WEB_SERVER"\r\n"
                             "User-Agent: esp-idf/1.0 esp32\r\n"
                             "\r\n";

#ifdef CONFIG_EXAMPLE_CLIENT_SESSION_TICKETS
static const char LOCAL_SRV_REQUEST[] = "GET " CONFIG_EXAMPLE_LOCAL_SERVER_URL " HTTP/1.1\r\n"
                             "Host: "WEB_SERVER"\r\n"
                             "User-Agent: esp-idf/1.0 esp32\r\n"
                             "\r\n";
#endif

/* Root cert for howsmyssl.com, taken from server_root_cert.pem

   The PEM file was extracted from the output of this command:
   openssl s_client -showcerts -connect www.howsmyssl.com:443 </dev/null

   The CA root cert is the last cert given in the chain of certs.

   To embed it in the app binary, the PEM file is named
   in the component.mk COMPONENT_EMBED_TXTFILES variable.
*/
extern const uint8_t server_root_cert_pem_start[] asm("_binary_server_root_cert_pem_start");
extern const uint8_t server_root_cert_pem_end[]   asm("_binary_server_root_cert_pem_end");

extern const uint8_t local_server_cert_pem_start[] asm("_binary_local_server_cert_pem_start");
extern const uint8_t local_server_cert_pem_end[]   asm("_binary_local_server_cert_pem_end");
//static const int server_supported_ciphersuites[] = {MBEDTLS_TLS_RSA_WITH_AES_256_GCM_SHA384, MBEDTLS_TLS_ECDHE_RSA_WITH_AES_128_CBC_SHA256, 0};
//static const int server_unsupported_ciphersuites[] = {MBEDTLS_TLS_ECDHE_RSA_WITH_ARIA_128_CBC_SHA256, 0};
#ifdef CONFIG_EXAMPLE_CLIENT_SESSION_TICKETS
static esp_tls_client_session_t *tls_client_session = NULL;
static bool save_client_session = false;
#endif

static void http_post_task(void *pvParameters)
{
    struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM,
    };
    struct addrinfo *res;
    struct in_addr *addr;
    int s, r;
    char recv_buf[64];
        int err = getaddrinfo(MY_SERVER, MY_PORT, &hints, &res);
    while(1){
	//xTaskCreate(&https_request_task, "https_get_task", 4096, NULL, 5, NULL);
	esp_tls_cfg_t cfg = {
        	.crt_bundle_attach = esp_crt_bundle_attach,
	};
	//char* buff = ""; 
	//buff = https_get_request(cfg, WEB_URL, HOWSMYSSL_REQUEST);
	//printf("This is buf:\n%s\n", buf);
	
        //fprintf(stdout,"at top");
        char buf[300];
    	int ret, len;
    	esp_tls_t *tls = esp_tls_init();
    	if (!tls) {
        	ESP_LOGE(TAG, "Failed to allocate esp_tls handle!");
        	goto exit;
    	}
    	//fprintf(stdout,"at middle");
    	if (esp_tls_conn_http_new_sync(WEB_URL, &cfg, tls) == 1) {
        	ESP_LOGI(TAG, "Connection established...");
    	} else {
        	ESP_LOGE(TAG, "Connection failed...");
        	int esp_tls_code = 0, esp_tls_flags = 0;
        	esp_tls_error_handle_t tls_e = NULL;
        	esp_tls_get_error_handle(tls, &tls_e);
        // Try to get TLS stack level error and certificate failure flags, if any 
        	ret = esp_tls_get_and_clear_last_error(tls_e, &esp_tls_code, &esp_tls_flags);
        if (ret == ESP_OK) {
            ESP_LOGE(TAG, "TLS error = -0x%x, TLS flags = -0x%x", esp_tls_code, esp_tls_flags);
        }
        goto cleanup;
    	}
    //fprintf(stdout,"at middle2");
#ifdef CONFIG_EXAMPLE_CLIENT_SESSION_TICKETS
    // The TLS session is successfully established, now saving the session ctx for reuse 
    	if (save_client_session) {
        	esp_tls_free_client_session(tls_client_session);
        	tls_client_session = esp_tls_get_client_session(tls);
    	}
#endif
    	size_t written_bytes = 0;
    	//fprintf(stdout,"before while");
    	do {
        	ret = esp_tls_conn_write(tls,
                                 HOWSMYSSL_REQUEST + written_bytes,
                                 strlen(HOWSMYSSL_REQUEST) - written_bytes);
        	if (ret >= 0) {
            	ESP_LOGI(TAG, "%d bytes written", ret);
            	written_bytes += ret;
        	} else if (ret != ESP_TLS_ERR_SSL_WANT_READ  && ret != ESP_TLS_ERR_SSL_WANT_WRITE) {
            		ESP_LOGE(TAG, "esp_tls_conn_write  returned: [0x%02X](%s)", ret, esp_err_to_name(ret));
            		goto cleanup;
        	}
    	} while (written_bytes < strlen(HOWSMYSSL_REQUEST));

    	ESP_LOGI(TAG, "Reading HTTP response...");
    	memset(buf, 0x00, sizeof(buf));
    	//char* buf = malloc(sizeof(char));
    	do {
        	len = sizeof(buf) - 1;
        	//memset(buf, 0x00, sizeof(buf));
        	ret = esp_tls_conn_read(tls, buf, len);

        	if (ret == ESP_TLS_ERR_SSL_WANT_WRITE  || ret == ESP_TLS_ERR_SSL_WANT_READ) {
            		continue;
       		} else if (ret < 0) {
            		ESP_LOGE(TAG, "esp_tls_conn_read  returned [-0x%02X](%s)", -ret, esp_err_to_name(ret));
            		break;
        	} else if (ret == 0) {
            		ESP_LOGI(TAG, "connection closed");
            		break;
        	}
    	} while (1);
cleanup:
    	esp_tls_conn_destroy(tls);
exit:
	
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

        vTaskDelay(2000 / portTICK_PERIOD_MS);
	//fprintf(stdout,"%s",buf);
	char *sss = "Santa";
	char *search = strstr(buf,sss);
	//char *search = "test";
        char cont[200];
        char request[300];
        snprintf(cont, sizeof(cont), "ESP:\nTem:%.0fF AND Hum:%.0f%%\nWEB:\n%s", temp_f, hum, search);
        snprintf(request, sizeof(request), "POST %s HTTP/1.0\r\n"
                "Host: %s:%s\r\n"
                "Content-Type: text/plain\r\n"
                "Content-Length: %d\r\n"
                "\r\n"
                "%s", MY_PATH, MY_SERVER, MY_PORT, strlen(cont), cont);

	fprintf(stdout,"%s", request);        
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
	/* Read HTTP response */
        do {
            bzero(recv_buf, sizeof(recv_buf));
            r = read(s, recv_buf, sizeof(recv_buf)-1);
            for(int i = 0; i < r; i++) {
                putchar(recv_buf[i]);
            }
        } while(r > 0);
	//free(buf);
        close(s);
	for(int countdown = 10; countdown >= 0; countdown--) {
            ESP_LOGI(TAG, "%d... ", countdown);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        ESP_LOGI(TAG, "Starting again!");

     }
}

void app_main(void)
{
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


    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    if (esp_reset_reason() == ESP_RST_POWERON) {
        ESP_LOGI(TAG, "Updating time from NVS");
        ESP_ERROR_CHECK(update_time_from_nvs());
    }

    const esp_timer_create_args_t nvs_update_timer_args = {
            .callback = (void *)&fetch_and_store_time_in_nvs,
    };

    esp_timer_handle_t nvs_update_timer;
    ESP_ERROR_CHECK(esp_timer_create(&nvs_update_timer_args, &nvs_update_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(nvs_update_timer, TIME_PERIOD));
    //while(1){
    	//xTaskCreate(&https_request_task, "https_get_task", 4096, NULL, 5, NULL);
	//vTaskDelay(1000 / portTICK_PERIOD_MS);
    xTaskCreate(&http_post_task, "http_post_task", 4096, NULL, 5, NULL);
	//vTaskDelay(30000 / portTICK_PERIOD_MS);
    //}
}
