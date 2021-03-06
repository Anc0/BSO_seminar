#include "espressif/esp_common.h"
#include "esp/uart.h"

#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <ssid_config.h>

#include <espressif/esp_sta.h>
#include <espressif/esp_wifi.h>

#include <paho_mqtt_c/MQTTESP8266.h>
#include <paho_mqtt_c/MQTTClient.h>

#include <semphr.h>

#include "i2c/i2c.h"
#include "bmp280/bmp280.h"

//#include "RF24/nRF24L01.h"
//#include "RF24/RF24.h"

#define MQTT_HOST ("15kw3c.messaging.internetofthings.ibmcloud.com")
#define MQTT_PORT 1883
#define MQTT_USER "use-token-auth"
//#define MQTT_PASS "Hiue1T)1X22R6OIrYQ"
//#define CLIENT_ID "d:15kw3c:temp-sensor:temp-sensor-1"
#define MQTT_PASS "Y-rY**+OF8IsRgmYu_"
#define CLIENT_ID "d:15kw3c:temp-sensor:temp-sensor-2"
#define MQTT_TOPIC "iot-2/evt/status/fmt/txt"
//#define MQTT_USER "a-15kw3c-h3c5qm5b01"
//#define MQTT_PASS "*D_A7*-ABa6RJgdd1p"
//#define CLIENT_ID "a:15kw3c:temp-app-1"
//#define MQTT_TOPIC "iot-2/type/temp-sensor/id/temp-sensor-1/evt/status/fmt/txt"

#define BUS_I2C		0
#define SCL 14
#define SDA 12

#define CE_NRF		3
#define CS_NRF		0
#define channel		33

QueueHandle_t publish_queue;
const uint8_t address[] = { 0x01, 0x23, 0x45, 0x67, 0x89 };
//RF24 radio(CE_NRF, CS_NRF);

SemaphoreHandle_t wifi_alive;
#define PUB_MSG_LEN 16

#define TRANSMIT 0

typedef enum {
	BMP280_TEMPERATURE, BMP280_PRESSURE
} bmp280_quantity;

bmp280_t bmp280_dev;

float read_bmp280(bmp280_quantity quantity) {

	float temperature, pressure;

	bmp280_force_measurement(&bmp280_dev);
	// wait for measurement to complete
	while (bmp280_is_measuring(&bmp280_dev)) {
	};
	bmp280_read_float(&bmp280_dev, &temperature, &pressure, NULL);

	if (quantity == BMP280_TEMPERATURE) {
		return temperature;
	} else if (quantity == BMP280_PRESSURE) {
		return pressure;
	}
	return 0;
}


static const char *  get_my_id(void)
{
    // Use MAC address for Station as unique ID
    static char my_id[13];
    static bool my_id_done = false;
    int8_t i;
    uint8_t x;
    if (my_id_done)
        return my_id;
    if (!sdk_wifi_get_macaddr(STATION_IF, (uint8_t *)my_id))
        return NULL;
    for (i = 5; i >= 0; --i)
    {
        x = my_id[i] & 0x0F;
        if (x > 9) x += 7;
        my_id[i * 2 + 1] = x + '0';
        x = my_id[i] >> 4;
        if (x > 9) x += 7;
        my_id[i * 2] = x + '0';
    }
    my_id[12] = '\0';
    my_id_done = true;
    return my_id;
}

static void  mqtt_task(void *pvParameters)
{
    int ret         = 0;
    struct mqtt_network network;
    mqtt_client_t client   = mqtt_client_default;
    char mqtt_client_id[20];
    uint8_t mqtt_buf[100];
    uint8_t mqtt_readbuf[100];
    mqtt_packet_connect_data_t data = mqtt_packet_connect_data_initializer;

    mqtt_network_new( &network );
    memset(mqtt_client_id, 0, sizeof(mqtt_client_id));
    strcpy(mqtt_client_id, "ESP-");
    strcat(mqtt_client_id, get_my_id());

    while(1) {
        xSemaphoreTake(wifi_alive, portMAX_DELAY);
        printf("%s: started\n\r", __func__);
        printf("%s: (Re)connecting to MQTT server %s ... ",__func__,
               MQTT_HOST);
        ret = mqtt_network_connect(&network, MQTT_HOST, MQTT_PORT);
        if( ret ){
            printf("error: %d\n\r", ret);
            taskYIELD();
            continue;
        }
        printf("done\n\r");
        mqtt_client_new(&client, &network, 5000, mqtt_buf, 100,
                      mqtt_readbuf, 100);

        data.willFlag       = 0;
        data.MQTTVersion    = 3;
        data.clientID.cstring   = CLIENT_ID;
        data.username.cstring   = MQTT_USER;
        data.password.cstring   = MQTT_PASS;
        data.keepAliveInterval  = 10;
        data.cleansession   = 0;
	
	printf("Measuring temperature...\n");
	printf("Temperature: %.2f C\n", read_bmp280(BMP280_TEMPERATURE));
	printf("done\n");

	char msg[PUB_MSG_LEN - 1] = "\0";

	int ret = snprintf(msg, sizeof msg, "%f", read_bmp280(BMP280_TEMPERATURE));

	msg[PUB_MSG_LEN - 1] = "\0";

	if (ret < 0) {
	    printf("FAILURE");
	}
	if (ret >= sizeof msg) {
	    printf("INCORRECT SIZE");
	} else {
	    printf("Message: %s\n", msg);
	}

        printf("Send MQTT connect ... ");
        ret = mqtt_connect(&client, &data);
        if(ret){
            printf("error: %d\n\r", ret);
            mqtt_network_disconnect(&network);
            taskYIELD();
            continue;
        }
        printf("done\r\n");

        mqtt_message_t message;
        message.payload = msg;
        message.payloadlen = PUB_MSG_LEN;
        message.dup = 0;
        message.qos = MQTT_QOS1;
        message.retained = 0;
        ret = mqtt_publish(&client, MQTT_TOPIC, &message);
        if (ret != MQTT_SUCCESS ){
            printf("error while publishing message: %d\n", ret );
            break;
        }
        mqtt_network_disconnect(&network);
	vTaskDelay( 60000 / portTICK_PERIOD_MS );
    }
}

static void  wifi_task(void *pvParameters)
{
    uint8_t status  = 0;
    uint8_t retries = 30;
    struct sdk_station_config config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
    };

    printf("WiFi: connecting to WiFi\n\r");
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);

    while(1)
    {
        while ((status != STATION_GOT_IP) && (retries)){
            status = sdk_wifi_station_get_connect_status();
            printf("%s: status = %d\n\r", __func__, status );
            if( status == STATION_WRONG_PASSWORD ){
                printf("WiFi: wrong password\n\r");
                break;
            } else if( status == STATION_NO_AP_FOUND ) {
                printf("WiFi: AP not found\n\r");
                break;
            } else if( status == STATION_CONNECT_FAIL ) {
                printf("WiFi: connection failed\r\n");
                break;
            }
            vTaskDelay( 1000 / portTICK_PERIOD_MS );
            --retries;
        }
        if (status == STATION_GOT_IP) {
            printf("WiFi: Connected\n\r");
            xSemaphoreGive( wifi_alive );
            taskYIELD();
        }

        while ((status = sdk_wifi_station_get_connect_status()) == STATION_GOT_IP) {
            xSemaphoreGive( wifi_alive );
            taskYIELD();
        }
        printf("WiFi: disconnected\n\r");
        sdk_wifi_station_disconnect();
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
    }
}



// Inter device communication
// transmit data
/*void transmit_nrf24(void *pvParameters) {
	char tx_buffer[PUB_MSG_LEN];

	while(1) {
		int ret = snprintf(tx_buffer, sizeof tx_buffer, "%f", read_bmp280(BMP280_TEMPERATURE));

		tx_buffer[PUB_MSG_LEN - 1] = "\0";

		if (ret < 0) {
		    printf("FAILURE");
		}
		if (ret >= sizeof tx_buffer) {
		    printf("INCORRECT SIZE");
		} else {
		    printf("Message: %s\n", tx_buffer);
		}
		radio.powerUp();
		radio.stopListening();
		radio.write(&tx_buffer, sizeof(tx_buffer));
		radio.powerDown();

		
	}

}*/

// receive data
/*void receive_nrf24(void *pvParameters) {
	static char msg[PUB_MSG_LEN];
	static int count = 0;

	while (1) {

		if (radio.available()) {
			radio.read(&rx_data, sizeof(rx_data));
			printf("Received message: %s\n", rx_data);

			// publish to MQTT topic
			snprintf(msg, PUB_MSG_LEN, "%s %d\r\n", rx_data, count++);
			if (xQueueSend(publish_queue, (void *)msg, 0) == pdFALSE) {
				printf("Publish queue overflow.\r\n");
			}
		}

		// sleep for 200 ms
		radio.powerDown();
		vTaskDelay(pdMS_TO_TICKS(200));
		radio.powerUp();
	}
}*/


void user_init(void)
{
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());

    i2c_init(BUS_I2C, SCL, SDA, I2C_FREQ_100K);

    // BMP280 configuration
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    params.mode = BMP280_MODE_FORCED;
    bmp280_dev.i2c_dev.bus = BUS_I2C;
    bmp280_dev.i2c_dev.addr = BMP280_I2C_ADDRESS_0;
    bmp280_init(&bmp280_dev, &params);

    // NRF config
    //gpio_enable(SCL, GPIO_OUTPUT);
    //gpio_enable(CS_NRF, GPIO_OUTPUT);
    // radio configuration
    //radio.begin();
    //radio.setChannel(channel);

    vSemaphoreCreateBinary(wifi_alive);
    publish_queue = xQueueCreate(3, PUB_MSG_LEN);

    xTaskCreate(&wifi_task, "wifi_task",  256, NULL, 2, NULL);
    xTaskCreate(&mqtt_task, "mqtt_task", 1024, NULL, 4, NULL);
    /*if(TRANSMIT==1) {
	xTaskCreate(transmit_nrf24, "transmit_task", 1024, NULL, 4, NULL);
    } else {
    	xTaskCreate(receive_nrf24, "receive_task", 1024, NULL, 4, NULL);
    }*/

}
