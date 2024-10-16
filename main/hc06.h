#ifndef HC06_H_
#define HC06_H_

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include <string.h>

#include "pico/stdlib.h"
#include <stdio.h>

#define HC06_UART_ID uart1
#define HC06_BAUD_RATE 9600
#define HC06_RX_PIN 4
#define HC06_TX_PIN 5
#define HC06_EN_PIN 6

bool hc06_check_connection();
bool hc06_set_name(char name[]);
bool hc06_set_pin(char pin[]);
bool hc06_set_at_mode(int on);
bool hc06_init(char name[], char pin[]);


#endif // HC06_H_
