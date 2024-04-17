/*
 * LED blink with FreeRTOS
 */
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include <string.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hc06.h"

#define deadzone 120

volatile int ADC_X = 26;
volatile int ADC_Y = 27;



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
            movement = malloc(sizeof(movement_t));
            xQueueAdc = xQueueCreate(10, sizeof(adc_t));

            xQueueReceive(xQueueAdc, &data, portMAX_DELAY);
            printf("Axis: %d, Value: %d\n", data.axis, data.val);
            printf("X: %d, Y: %d\n", movement->x, movement->y);
            // É AQUI QUE A GENTE MANDA O MOVIMENTO PRO PYTHON;
            // SE X FOR 1, MANDA PRA DIREITA, SE FOR -1, MANDA PRA ESQUERDA
            // SE Y FOR 1, MANDA PRA FRENTE, SE FOR -1, MANDA PRA TRAS

            free(movement);
        }

    }
}

void analog_task(void *p) {
    while (true) {
        printf("Analog value: %d\n", adc_read());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
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
