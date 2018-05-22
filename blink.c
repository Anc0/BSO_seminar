#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "i2c/i2c.h"
#include "bmp280/bmp280.h"
#include "ssid_config.h"
#include "ota-tftp.h"
#include "httpd/httpd.h"

#define PCF_ADDRESS	0x38
#define MPU_ADDRESS	0x68
#define BUS_I2C		0
#define SCL 14
#define SDA 12

//					mask	returned value
#define button1		0x20	// 0b ??0? ????
#define button2		0x10	// 0b ???0 ????
#define button3		0x80	// 0b 0??? ????
#define button4		0x40	// 0b ?0?? ????
#define clr_btn		0xf0

#define led1 		0xfe	// 0b ???? ???0
#define led2 		0xfd	// 0b ???? ??0?
#define led3 		0xfb	// 0b ???? ?0??
#define led4 		0xf7	// 0b ???? 0???
#define leds_off	0xff

#define gpio_wemos_led	2

typedef enum {
	BMP280_TEMPERATURE, BMP280_PRESSURE
} bmp280_quantity;

typedef enum {
	MPU9250_ACCEL_X = 0x3b,
	MPU9250_ACCEL_Y = 0x3d,
	MPU9250_ACCEL_Z = 0x3f,
	MPU9250_TEMP = 0x41,
	MPU9250_GYRO_X = 0x43,
	MPU9250_GYRO_Y = 0x45,
	MPU9250_GYRO_Z = 0x47
} mpu9250_quantity;

enum {
	SSI_UPTIME, SSI_FREE_HEAP, SSI_LED_STATE
};

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

// write byte to PCF on I2C bus
void write_byte_pcf(uint8_t data) {
	i2c_slave_write(BUS_I2C, PCF_ADDRESS, NULL, &data, 1);
}

// read byte from PCF on I2C bus
uint8_t read_byte_pcf() {
	uint8_t data;

	i2c_slave_read(BUS_I2C, PCF_ADDRESS, NULL, &data, 1);
	return data;
}

// check for pressed buttons
void pcf_task(void *pvParameters) {

	uint8_t pcf_byte;

	// turn off all leds
	write_byte_pcf(leds_off);

	while (1) {
		printf("GOVNO");
		pcf_byte = read_byte_pcf();

		// clear buttons states and toggle led 1
		write_byte_pcf((pcf_byte ^ ~led1) | clr_btn);
		// display temperature
		printf("Temperature: %.2f C\n", read_bmp280(BMP280_TEMPERATURE));


		// check again after 200 ms
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void user_init(void) {
	printf("Hello world");

	uart_set_baud(0, 115200);
	i2c_init(BUS_I2C, SCL, SDA, I2C_FREQ_100K);
	// fix i2c driver to work with MPU-9250
	gpio_enable(SCL, GPIO_OUTPUT);

	// WiFi configuration
	//struct sdk_station_config config = { .ssid = WIFI_SSID, .password = WIFI_PASS, };
	//sdk_wifi_station_set_auto_connect(1);
	//sdk_wifi_set_opmode(STATION_MODE);
	//sdk_wifi_station_set_config(&config);
	//sdk_wifi_station_connect();

	// OTA configuration
	//ota_tftp_init_server(TFTP_PORT);

	// BMP280 configuration
	bmp280_params_t params;
	bmp280_init_default_params(&params);
	params.mode = BMP280_MODE_FORCED;
	bmp280_dev.i2c_dev.bus = BUS_I2C;
	bmp280_dev.i2c_dev.addr = BMP280_I2C_ADDRESS_0;
	bmp280_init(&bmp280_dev, &params);

	// turn off Wemos led
	gpio_enable(gpio_wemos_led, GPIO_OUTPUT);
	gpio_write(gpio_wemos_led, 1);

	// create pcf task
	xTaskCreate(pcf_task, "PCF task", 1000, NULL, 2, NULL);

}

