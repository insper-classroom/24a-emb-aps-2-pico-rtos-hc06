/*
 * LED blink with FreeRTOS
 */
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include "mpu6050.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"

#include <string.h>
#include <Fusion.h>


#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>

#include "hc06.h"

#define deadzone 120

volatile int ADC_X = 26;
volatile int ADC_Y = 27;
volatile int I2C_PORT = 0;


typedef struct adc {
    int axis;
    int val;
} adc_t;

typedef struct movement {
    int x;
    int y;
} movement_t;

QueueHandle_t xQueueAdc;
movement_t *movement;



void x_adc_task(void *p) {
    adc_t data;
    adc_init();
    adc_gpio_init(ADC_X);

    while (1) {
        adc_select_input(0); // Select ADC input 0 (GPIO26)
        float result = adc_read();

        result = result - 2048;
        result = result / 8;

        if (abs(result) < deadzone) {
            result = 0;
            movement->x = 0;
        }else{
            movement->x = result/abs(result);
            }

        data.val = result;
        data.axis = 1;
        xQueueSend(xQueueAdc, &data, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void y_adc_task(void *p) {
    adc_t data;
    adc_init();
    adc_gpio_init(ADC_Y);

    while (1) {
        adc_select_input(1); // Select ADC input 1 (GPIO27)
        float result = adc_read();

        result = result - 2048;
        result = result / 8;

        if (abs(result) < deadzone) {
            result = 0;
            movement->y = 0;
        }else{
            movement->y = result/abs(result);
        }
        data.val = result;
        data.axis = 0;
        xQueueSend(xQueueAdc, &data, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}



void hc06_task(void *p) {
    int connected;
    uart_init(HC06_UART_ID, HC06_BAUD_RATE);
    gpio_set_function(HC06_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(HC06_RX_PIN, GPIO_FUNC_UART);
    
    hc06_init("HC06", "6");

    while (true) {
        //uart_puts(HC06_UART_ID, "OLAAA ");
        if (connected == 0){
            if (hc06_check_connection()){
            printf("Connected\n");
            connected = 1;
            } else {
                printf("Not connected\n");

            };
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else{
            adc_t data;
            xQueueAdc = xQueueCreate(10, sizeof(adc_t));

            xQueueReceive(xQueueAdc, &data, portMAX_DELAY);
            printf("Axis: %d, Value: %d\n", data.axis, data.val);
            printf("X: %d, Y: %d\n", movement->x, movement->y);
            // É AQUI QUE A GENTE MANDA O MOVIMENTO PRO PYTHON;
            // SE X FOR 1, MANDA PRA DIREITA, SE FOR -1, MANDA PRA ESQUERDA
            // SE Y FOR 1, MANDA PRA FRENTE, SE FOR -1, MANDA PRA TRAS

        }

    }
}

static void mpu6050_read_raw(int16_t *accel, int16_t *gyro) {

    uint8_t buffer[14];
    uint8_t val = 0x38;


    i2c_write_blocking(i2c_default, MPU6050_I2C_DEFAULT, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU6050_I2C_DEFAULT, buffer, 14, false);

    accel[0] = (buffer[0] << 8) | buffer[1];
    accel[1] = (buffer[2] << 8) | buffer[3];
    accel[2] = (buffer[4] << 8) | buffer[5];

    gyro[0] = (buffer[8] << 8) | buffer[9];
    gyro[1] = (buffer[10] << 8) | buffer[11];
    gyro[2] = (buffer[12] << 8) | buffer[13];
}


void mpu6050_task(void *p) {
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    // Inicializa o vibrador
    gpio_init(VIBRATOR_PIN);
    gpio_set_dir(VIBRATOR_PIN, GPIO_OUT);

    mpu6050_reset();
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    int16_t acceleration[3], gyro[3];

    while(1) {
        mpu6050_read_raw(acceleration, gyro);

        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f, // Conversão para graus/s
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f, // Conversão para g
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // printf("Accel x: %0.2f g, y: %0.2f g, z: %0.2f g\n", accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z);
        // printf("Gyro x: %0.2f deg/s, y: %0.2f deg/s, z: %0.2f deg/s\n", gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z);
    }

int main() {
    stdio_init_all();

    printf("Start bluetooth task\n");

    xTaskCreate(hc06_task, "UART_Task 1", 4096, NULL, 1, NULL);
    xTaskCreate(x_adc_task, "ADC_Task 1", 4096, NULL, 1, NULL);
    xTaskCreate(y_adc_task, "ADC_Task 2", 4096, NULL, 1, NULL);
    


    vTaskStartScheduler();

    while (true)
        ;
}
