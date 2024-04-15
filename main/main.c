#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/adc.h"
#include "hc06.h"
#include <math.h>
#include <stdlib.h>

uint const XPIN = 26;
uint const YPIN = 27;
const int BLUE_BTN = 20;
const int RED_BTN = 21;

QueueHandle_t xQueueAdc;
QueueHandle_t xQueueBtn;

void btn_callback(uint gpio, uint32_t events){
    int red = 0;
    int blue = 1;
    if (events == 0x4){
        if(gpio == RED_BTN){
            xQueueSend(xQueueBtn, &red, portMAX_DELAY);  // Envia a leitura para a fila

        }
        else if(gpio == BLUE_BTN){
            xQueueSend(xQueueBtn, &blue, portMAX_DELAY);  // Envia a leitura para a fila

        } 
    }
}


typedef struct adc {
    int axis;
    int val;
} adc_t;

#define MOVING_AVERAGE_SIZE 5



void x_task(void *p) {
    
    int x_buffer[MOVING_AVERAGE_SIZE] = {0};
    int x_index = 0;
    adc_t x_read;
    x_read.axis = 0; // define o canal 0 pro eixo x

    while (1) {
        adc_select_input(0);  // Seleciona o canal ADC para o eixo X (GP26 = ADC0)
        int current_read = adc_read(); // Realiza a leitura do ADC
        if ((current_read - 2047) / 8 > -30 && (current_read - 2047) / 8 < 30) { //zona morta
            x_buffer[x_index] = 0;
        } else {
            x_buffer[x_index] = (current_read - 2047) / 128;  // Normaliza o valor lido
        }

        // Atualiza a soma para calcular a média móvel
        int sum = 0;
        for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
            sum += x_buffer[i];
        }
        x_read.val = sum / MOVING_AVERAGE_SIZE;

        xQueueSend(xQueueAdc, &x_read, portMAX_DELAY);  // Envia a leitura para a fila
        x_index = (x_index + 1) % MOVING_AVERAGE_SIZE;  // Atualiza o índice circularmente

        vTaskDelay(pdMS_TO_TICKS(100));  // Atraso para desacoplamento das tarefas
    }
}

void y_task(void *p) {

    int y_buffer[MOVING_AVERAGE_SIZE] = {0};
    int y_index = 0;
    adc_t y_read;
    y_read.axis = 1; // define o canal 1 pro eixo y

    while (1) {
        adc_select_input(1);  // Seleciona o canal ADC para o eixo Y (GP27 = ADC1)
        int current_read = adc_read(); // Realiza a leitura do ADC
        if ((current_read - 2047) / 8 > -30 && (current_read - 2047) / 8 < 30) { //zona morta
            y_buffer[y_index] = 0;
        } else {
            y_buffer[y_index] = (current_read - 2047) / 128;  // Normaliza o valor lido
        }

        // Atualiza a soma para calcular a média móvel
        int sum = 0;
        for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
            sum += y_buffer[i];
        }
        y_read.val = sum / MOVING_AVERAGE_SIZE;

        xQueueSend(xQueueAdc, &y_read, portMAX_DELAY);  // Envia a leitura para a fila
        y_index = (y_index + 1) % MOVING_AVERAGE_SIZE;  // Atualiza o índice circularmente

        vTaskDelay(pdMS_TO_TICKS(100));  // Atraso para desacoplamento das tarefas
    }
}

void write(adc_t data){
    int val = data.val;
    int msb = val >> 8;
    int lsb = val & 0xFF ;

    uart_putc_raw(uart0, data.axis);
    uart_putc_raw(uart0, lsb);
    uart_putc_raw(uart0, msb);
    uart_putc_raw(uart0, -1); 
}

void uart_task(void *p) {
    adc_t data;

    while (1) {
        if (xQueueReceive(xQueueAdc, &data, portMAX_DELAY)) {
            write(data);
        }
    }
}

void btn_task(void *p) {
    int btn;
    adc_t data;

    while (1) {
        if (xQueueReceive(xQueueBtn, &btn, portMAX_DELAY)) {
            data.axis = 2;
            data.val = btn;
            write(data);
        }
    }
}



void hc06_task(void *p) {
    uart_init(HC06_UART_ID, HC06_BAUD_RATE);
    gpio_set_function(HC06_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(HC06_RX_PIN, GPIO_FUNC_UART);
    hc06_init("aps2_legal", "1234");

    while (true) {
        uart_puts(HC06_UART_ID, "OLAAA ");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


int main() {
    stdio_init_all();
    adc_init();

    adc_gpio_init(XPIN);
    adc_gpio_init(YPIN);

    gpio_init(BLUE_BTN);
    gpio_set_dir(BLUE_BTN, GPIO_IN);
    gpio_pull_up(BLUE_BTN);

    gpio_init(RED_BTN);
    gpio_set_dir(RED_BTN, GPIO_IN);
    gpio_pull_up(RED_BTN);

    gpio_set_irq_enabled_with_callback(RED_BTN, GPIO_IRQ_EDGE_FALL, true, &btn_callback);
    gpio_set_irq_enabled_with_callback(BLUE_BTN, GPIO_IRQ_EDGE_FALL, true, &btn_callback);


    xQueueAdc = xQueueCreate(32, sizeof(adc_t));
    xQueueBtn = xQueueCreate(32, sizeof(adc_t));

    printf("Start bluetooth task\n");

    xTaskCreate(hc06_task, "UART_Task 1", 4096, NULL, 1, NULL);

    xTaskCreate(uart_task, "uart_task", 4096, NULL, 1, NULL);
    xTaskCreate(x_task, "x_task", 4096, NULL, 1, NULL);
    xTaskCreate(y_task, "y_task", 4096, NULL, 1, NULL);
    xTaskCreate(btn_task, "btn_task", 4096, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true);
}