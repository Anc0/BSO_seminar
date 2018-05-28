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


#define MQTT_HOST ("15kw3c.messaging.internetofthings.ibmcloud.com")
#define MQTT_PORT 1883
#define MQTT_USER "use-token-auth"
//#define MQTT_PASS "Hiue1T)1X22R6OIrYQ"
//#define CLIENT_ID "d:15kw3c:temp-sensor:temp-sensor-1"
#define MQTT_PASS "Y-rY**+OF8IsRgmYu_"
#define CLIENT_ID "d:15kw3c:temp-sensor:temp-sensor-2"
#define MQTT_TOPIC "iot-2/evt/status/fmt/txt"

#define BUS_I2C		0
#define SCL 14
#define SDA 12

SemaphoreHandle_t wifi_alive;
QueueHandle_t publish_queue;
#define PUB_MSG_LEN 16

typedef enum {
	BMP280_TEMPERATURE, BMP280_PRESSURE
} bmp280_quantity;

bmp280_t bmp280_dev;

// read BMP280 sensor values
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


static void  beat_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    char msg[PUB_MSG_LEN];
    int count = 0;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, 10000 / portTICK_PERIOD_MS);
        printf("beat\r\n");
        snprintf(msg, PUB_MSG_LEN, "Beat %d\r\n", count++);
        if (xQueueSend(publish_queue, (void *)msg, 0) == pdFALSE) {
            printf("Publish queue overflow.\r\n");
        }
    }
}

static void  topic_received(mqtt_message_data_t *md)
{
    int i;
    mqtt_message_t *message = md->message;
    printf("Received: ");
    for( i = 0; i < md->topic->lenstring.len; ++i)
        printf("%c", md->topic->lenstring.data[ i ]);

    printf(" = ");
    for( i = 0; i < (int)message->payloadlen; ++i)
        printf("%c", ((char *)(message->payload))[i]);

    printf("\r\n");
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
	printf("GOVNO\n");
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
        mqtt_subscribe(&client, MQTT_TOPIC, MQTT_QOS1, topic_received);
        xQueueReset(publish_queue);

        while(1){
	    char msg[PUB_MSG_LEN - 1] = "\0";

            int ret = snprintf(msg, sizeof msg, "%f", read_bmp280(BMP280_TEMPERATURE));

	    if (ret < 0) {
	        printf("FAILURE");
	    }
	    if (ret >= sizeof msg) {
	    	printf("INCORRECT SIZE");
            } else {
		printf("GOVNO");
		printf(msg);
	    }

            
            while(xQueueReceive(publish_queue, (void *)msg, 0) ==
                  pdTRUE){
                printf("got message to publish\r\n");
                mqtt_message_t message;
                message.payload = msg;
                message.payloadlen = PUB_MSG_LEN;
                message.dup = 0;
                message.qos = MQTT_QOS1;
                message.retained = 0;
                ret = mqtt_publish(&client, "/beat", &message);
                if (ret != MQTT_SUCCESS ){
                    printf("error while publishing message: %d\n", ret );
                    break;
                }
            }

            ret = mqtt_yield(&client, 1000);
            if (ret == MQTT_DISCONNECTED)
                break;
        }
        printf("Connection dropped, request restart\n\r");
        mqtt_network_disconnect(&network);
        taskYIELD();
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

    vSemaphoreCreateBinary(wifi_alive);
    publish_queue = xQueueCreate(3, PUB_MSG_LEN);
    xTaskCreate(&wifi_task, "wifi_task",  256, NULL, 2, NULL);
    xTaskCreate(&beat_task, "beat_task", 256, NULL, 3, NULL);
    xTaskCreate(&mqtt_task, "mqtt_task", 1024, NULL, 4, NULL);
}
