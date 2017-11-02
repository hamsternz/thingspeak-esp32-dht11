/******************************************************************************
   thingspeak-esp32-dht11.c - A example thingspeak client, logging temperature 
                              and humidity.
							 
   Author: Mike Field <hamster@snap.net.nz>
							 
   Largely adapted from the ESP-IDF OpenSSL client example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*******************************************************************************
*  Configure like the rest of the ESP-IDF examples, with "make menuconfig" 
******************************************************************************/

#include <string.h>

#include <openssl/ssl.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event_loop.h>

#include <driver/rmt.h>
#include <soc/rmt_reg.h>

#include <nvs_flash.h>

#include <lwip/sockets.h>
#include <lwip/netdb.h>

static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const static int CONNECTED_BIT = BIT0;

const static char *TAG = "thingspeak_client_dht11";

int temp_x10 = 123;
int humidity = 60;

#define HTTPS_TASK_NAME        "https_connection"
#define HTTPS_TASK_STACK_WORDS 10240
#define HTTPS_TASK_PRIORITY    8
#define HTTPS_RECV_BUF_LEN       1024
#define HTTPS_LOCAL_TCP_PORT     443

/*********************************************************************************
 * RMT receiver initialization
 *********************************************************************************/
static void dht11_rmt_rx_init(int gpio_pin, int channel)
{
	const int RMT_CLK_DIV            = 80;     /*!< RMT counter clock divider */
	const int RMT_TICK_10_US         = (80000000/RMT_CLK_DIV/100000);   /*!< RMT counter value for 10 us.(Source clock is APB clock) */
	const int  rmt_item32_tIMEOUT_US = 1000;   /*!< RMT receiver timeout value(us) */

    rmt_config_t rmt_rx;
    rmt_rx.gpio_num                      = gpio_pin;
    rmt_rx.channel                       = channel;
    rmt_rx.clk_div                       = RMT_CLK_DIV;
    rmt_rx.mem_block_num                 = 1;
    rmt_rx.rmt_mode                      = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en           = false;
    rmt_rx.rx_config.filter_ticks_thresh = 100;
    rmt_rx.rx_config.idle_threshold      = rmt_item32_tIMEOUT_US / 10 * (RMT_TICK_10_US);
    rmt_config(&rmt_rx);
    rmt_driver_install(rmt_rx.channel, 1000, 0);
}

/*********************************************************************************
 * Processing the pulse data into temp and humidity
 *********************************************************************************/
static int parse_items(rmt_item32_t* item, int item_num, int *humidity, int *temp_x10)
{
	int i=0;
	unsigned rh = 0, temp = 0, checksum = 0;

	///////////////////////////////
	// Check we have enough pulses
	///////////////////////////////
    if(item_num < 42)  return 0;

	///////////////////////////////////////
	// Skip the start of transmission pulse
	///////////////////////////////////////
	item++;  

	///////////////////////////////
    // Extract the humidity data 	
	///////////////////////////////
	for(i = 0; i < 16; i++, item++) 
	    rh = (rh <<1) + (item->duration1 < 35 ? 0 : 1);

	///////////////////////////////
	// Extract the temperature data
	///////////////////////////////
	for(i = 0; i < 16; i++, item++) 
	    temp = (temp <<1) + (item->duration1 < 35 ? 0 : 1);

	///////////////////////////////
	// Extract the checksum
	///////////////////////////////
	for(i = 0; i < 8; i++, item++) 
	    checksum = (checksum <<1) + (item->duration1 < 35 ? 0 : 1);
	
	///////////////////////////////
	// Check the checksum
	///////////////////////////////
	if((((temp>>8) + temp + (rh>>8) + rh)&0xFF) != checksum) {
		printf("Checksum failure %4X %4X %2X\n", temp, rh, checksum);
       return 0;   
    }

	///////////////////////////////
	// Store into return values
	///////////////////////////////
    *humidity = rh>>8;
	*temp_x10 = (temp>>8)*10+(temp&0xFF);
    return 1;
}

/*********************************************************************************
 * Use the RMT receiver to get the DHT11 data
 *********************************************************************************/
static int dht11_rmt_rx(int gpio_pin, int rmt_channel, 
                        int *humidity, int *temp_x10)
{
    RingbufHandle_t rb = NULL;
    size_t rx_size = 0;
	rmt_item32_t* item;
	int rtn = 0;
	
	//get RMT RX ringbuffer
    rmt_get_ringbuf_handle(rmt_channel, &rb);
    if(!rb) 
		return 0;

	//////////////////////////////////////////////////
	// Send the 20ms pulse to kick the DHT11 into life
	//////////////////////////////////////////////////
	gpio_set_level( gpio_pin, 1 );
	gpio_set_direction( gpio_pin, GPIO_MODE_OUTPUT );
	ets_delay_us( 1000 );
	gpio_set_level( gpio_pin, 0 );
	ets_delay_us( 20000 );

	////////////////////////////////////////////////
	// Bring rmt_rx_start & rmt_rx_stop into cache
	////////////////////////////////////////////////
    rmt_rx_start(rmt_channel, 1);
    rmt_rx_stop(rmt_channel);

	//////////////////////////////////////////////////
	// Now get the sensor to send the data
	//////////////////////////////////////////////////
	gpio_set_level( gpio_pin, 1 );
	gpio_set_direction( gpio_pin, GPIO_MODE_INPUT );
	
	////////////////////////////////////////////////
	// Start the RMT receiver for the data this time
	////////////////////////////////////////////////
    rmt_rx_start(rmt_channel, 1);
    
	/////////////////////////////////////////////////
	// Pull the data from the ring buffer
	/////////////////////////////////////////////////
	item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, 2);
	if(item != NULL) {
		int n;
		n = rx_size / 4 - 0;
		//parse data value from ringbuffer.
		rtn = parse_items(item, n, humidity, temp_x10);
		//after parsing the data, return spaces to ringbuffer.
		vRingbufferReturnItem(rb, (void*) item);
	}
	/////////////////////////////////////////////////
	// Stop the RMT Receiver
	/////////////////////////////////////////////////
    rmt_rx_stop(rmt_channel);

	return rtn;
}

static void update_thingspeak_task(void *p)
{
    int ret;
    SSL_CTX *ctx;
    SSL *ssl;
    int socket;
    struct sockaddr_in sock_addr;
    struct hostent *hp;
    struct ip4_addr *ip4_addr;
    int recv_bytes = 0;
	int send_bytes;
    char recv_buf[HTTPS_RECV_BUF_LEN];
	const char template[] = "GET /update?api_key=" CONFIG_THINGSPEAK_API_WRITE_KEY "&field1=%i.%i&field2=%i.0 HTTP/1.1\r\n"
              "Host: hamsterworks.co.nz \r\n"
              "Connection: close\r\n\r\n";
    char send_data[sizeof(template)+20];
	
	/* Clamp the inputs to enure we don't blow the buffer out */
	if(temp_x10 > 999) temp_x10 = 999;
	if(temp_x10 < -999) temp_x10 = -999;
	if(humidity > 1000) humidity = 100;
	if(humidity < 0) humidity = 0;
	
    /* Build the message */
    sprintf(send_data, template, temp_x10/10, temp_x10%10, humidity);
    send_bytes = strlen(send_data);

    ESP_LOGI(TAG, "OpenSSL demo thread start OK");

    ESP_LOGI(TAG, "get target IP address");
    hp = gethostbyname("api.thingspeak.com");
    if (!hp) {
        ESP_LOGI(TAG, "failed");
        goto failed1;
    }
    ESP_LOGI(TAG, "OK");

    ip4_addr = (struct ip4_addr *)hp->h_addr;
    ESP_LOGI(TAG, IPSTR, IP2STR(ip4_addr));

    ESP_LOGI(TAG, "create SSL context ......");
    ctx = SSL_CTX_new(TLSv1_1_client_method());
    if (!ctx) {
        ESP_LOGI(TAG, "failed");
        goto failed1;
    }
    ESP_LOGI(TAG, "OK");

    ESP_LOGI(TAG, "create socket ......");
    socket = socket(AF_INET, SOCK_STREAM, 0);
    if (socket < 0) {
        ESP_LOGI(TAG, "failed");
        goto failed2;
    }
    ESP_LOGI(TAG, "OK");

    ESP_LOGI(TAG, "bind socket ......");
    memset(&sock_addr, 0, sizeof(sock_addr));
    sock_addr.sin_family = AF_INET;
    sock_addr.sin_addr.s_addr = 0;
    sock_addr.sin_port = htons(8192);
    ret = bind(socket, (struct sockaddr*)&sock_addr, sizeof(sock_addr));
    if (ret) {
        ESP_LOGI(TAG, "failed");
        goto failed3;
    }
    ESP_LOGI(TAG, "OK");

    ESP_LOGI(TAG, "socket connect to server ......");
    memset(&sock_addr, 0, sizeof(sock_addr));
    sock_addr.sin_family = AF_INET;
    sock_addr.sin_addr.s_addr = ip4_addr->addr;
    sock_addr.sin_port = htons(443);
    ret = connect(socket, (struct sockaddr*)&sock_addr, sizeof(sock_addr));
    if (ret) {
        ESP_LOGI(TAG, "failed");
        goto failed3;
    }
    ESP_LOGI(TAG, "OK");

    ESP_LOGI(TAG, "create SSL ......");
    ssl = SSL_new(ctx);
    if (!ssl) {
        ESP_LOGI(TAG, "failed");
        goto failed3;
    }
    ESP_LOGI(TAG, "OK");

    SSL_set_fd(ssl, socket);

    ESP_LOGI(TAG, "SSL connected ......");
    ret = SSL_connect(ssl);
    if (!ret) {
        ESP_LOGI(TAG, "failed " );
        goto failed4;
    }
    ESP_LOGI(TAG, "OK");

    ESP_LOGI(TAG, "send https request '%s'...",send_data);
    ret = SSL_write(ssl, send_data, send_bytes);
    if (ret <= 0) {
        ESP_LOGI(TAG, "failed");
        goto failed5;
    }
    ESP_LOGI(TAG, "OK");

    do {
        ret = SSL_read(ssl, recv_buf, HTTPS_RECV_BUF_LEN - 1);
        if (ret <= 0) {
            break;
        }
        recv_buf[ret] = '\0';
        recv_bytes += ret;
        ESP_LOGI(TAG, "%s", recv_buf);
    } while (1);
    
    ESP_LOGI(TAG, "read %d bytes of data ......", recv_bytes);

failed5:
    SSL_shutdown(ssl);
failed4:
    SSL_free(ssl);
    ssl = NULL;
failed3:
    close(socket);
    socket = -1;
failed2:
    SSL_CTX_free(ctx);
    ctx = NULL;
failed1:
    vTaskDelete(NULL);
    return ;
}

static void HTTPS_client_init(void)
{
    int ret;
    xTaskHandle openssl_handle;

    ret = xTaskCreate(update_thingspeak_task,
                      HTTPS_TASK_NAME,
                      HTTPS_TASK_STACK_WORDS,
                      NULL,
                      HTTPS_TASK_PRIORITY,
                      &openssl_handle);

    if (ret != pdPASS)  {
        ESP_LOGI(TAG, "create thread %s failed", HTTPS_TASK_NAME);
    }
}

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        HTTPS_client_init();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect(); 
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void wifi_conn_init(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(wifi_event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid =     CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_LOGI(TAG, "start the WIFI SSID:[%s]\n", CONFIG_WIFI_SSID);
    ESP_ERROR_CHECK( esp_wifi_start() );
}

void app_main(void)
{
	const int gpio_pin    = GPIO_NUM_5;
	const int rmt_channel = 0;
    const int wakeup_time_sec = 300;

    ESP_ERROR_CHECK( nvs_flash_init() );

	// Set up the RMT_RX module
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "Enabling timer wakeup, %ds\n", wakeup_time_sec);
    esp_sleep_enable_timer_wakeup(wakeup_time_sec  * 1000000);  // Wake up every 5 minutes
    dht11_rmt_rx_init(gpio_pin, rmt_channel);
	sleep(2);
    if(dht11_rmt_rx(gpio_pin, rmt_channel, &humidity, &temp_x10)) {
       ESP_LOGI(TAG, "Temperature & humidity read %i.%iC & %i%%\n", temp_x10/10, temp_x10%10, humidity);
	   wifi_conn_init();
    } else {
        ESP_LOGI(TAG, "Sensor failure - retrying\n");
		sleep(5);
		if(dht11_rmt_rx(gpio_pin, rmt_channel, &humidity, &temp_x10)) {
			ESP_LOGI(TAG, "Temperature & humidity read %i.%iC & %i%%\n", temp_x10/10, temp_x10%10, humidity);
			wifi_conn_init();
		} else {
	      ESP_LOGI(TAG, "Sensor failure\n");
		}
	}
	sleep(10);
	ESP_LOGI(TAG, "Entering deep sleep\n");
	esp_deep_sleep_start();
}
