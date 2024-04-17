#include "hc06.h"



bool hc06_check_connection() {
    char str[32];
    int i = 0;
    uart_puts(HC06_UART_ID, "AT");
    while (uart_is_readable_within_us(HC06_UART_ID, 1000)) {
        str[i++] = uart_getc(HC06_UART_ID);
    }
    str[i] = '\0';

    if (strstr(str, "OK") > 0)
        return true;
    else
        return false;
}

bool hc06_set_name(char name[]) {
    char str[32];
    int i = 0;

    sprintf(str, "AT+NAME%s", name);
    uart_puts(HC06_UART_ID, str);
    while (uart_is_readable_within_us(HC06_UART_ID, 1000)) {
        str[i++] = uart_getc(HC06_UART_ID);
    }
    str[i] = '\0';

    if (strstr(str, "OK") > 0)
        return true;
    else
        return false;
}

bool hc06_set_pin(char pin[]) {
    char str[32];
    int i = 0;

    sprintf(str, "AT+PIN%s", pin);
    uart_puts(HC06_UART_ID, str);
    while (uart_is_readable_within_us(HC06_UART_ID, 1000)) {
        str[i++] = uart_getc(HC06_UART_ID);
    }
    str[i] = '\0';

    if (strstr(str, "OK") > 0)
        return true;
    else
        return false;
}

bool hc06_set_at_mode(int on){
    gpio_put(HC06_PIN, on);
}

bool hc06_init(char name[], char pin[]) {
    hc06_set_at_mode(1);
    printf("check connection\n");
    while (hc06_check_connection() == false) {
        printf("not connected\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("Connected \n");

    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("set name\n");
    while (hc06_set_name(name) == false) {
        printf("set name failed\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("name ok\n");

    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("set pin\n");
    while (hc06_set_pin(pin) == false) {
        printf("set pin failed\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("pin ok\n");
    hc06_set_at_mode(0);
}
