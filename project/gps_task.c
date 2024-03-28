#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/rtc.h"
#include "hardware/uart.h"
#include "pico/util/datetime.h"

#include "pinmap.h"
#include <memory.h>
#include <stdio.h>
#include <string.h>

#include "gps.h"
#include "leds.h"

#define GPS_UART uart0
#define GPS_BAUD 9600
#define GPS_DATA_BITS 8
#define GPS_STOP_BITS 1
#define GPS_PARITY UART_PARITY_NONE

static QueueHandle_t gpsQueue;
typedef char nmea_buffer_t[85];

void on_uart_rx() {
  BaseType_t higherPriorityTaskWoken = pdFALSE;
  static nmea_buffer_t nmea_buffer = {0}; // longest NMEA string is 82 bytes
  static int buffer_position = 0;

  while (uart_is_readable(GPS_UART)) {
    char ch = uart_getc(GPS_UART);
    switch (ch) {
    case '\r': // discard this character
      break;
    case '\n': // post the message on the GPS queue
      xQueueSendFromISR(gpsQueue, &nmea_buffer, &higherPriorityTaskWoken);
      buffer_position = 0;
      memset(nmea_buffer, 0, sizeof(nmea_buffer));
      break;
    default:
      nmea_buffer[buffer_position] = ch;
      buffer_position++;
      if (buffer_position > sizeof(nmea_buffer) - 1) {
        buffer_position = sizeof(nmea_buffer) - 1;
      }
      break;
    }
  }
  portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

static void gps_task(void *parameter) {
  struct gps_tpv tpv;
  gps_init_tpv(&tpv);

  uart_init(GPS_UART, GPS_BAUD);
  gpio_set_function(GPS_TX, GPIO_FUNC_UART);
  gpio_set_function(GPS_RX, GPIO_FUNC_UART);
  uart_set_hw_flow(GPS_UART, false, false);
  uart_set_format(GPS_UART, GPS_DATA_BITS, GPS_STOP_BITS, GPS_PARITY);
  uart_set_fifo_enabled(GPS_UART, true);
  const int UART_IRQ = GPS_UART == uart0 ? UART0_IRQ : UART1_IRQ;
  irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
  irq_set_enabled(UART_IRQ, true);
  uart_set_irq_enables(GPS_UART, true, false);

  gpio_init(GPS_ON);
  gpio_set_dir(GPS_ON, true);
  gpio_put(GPS_ON, true);

  vTaskDelay(pdMS_TO_TICKS(10));

  gpio_init(GPS_RESET);
  gpio_set_dir(GPS_RESET, true);
  gpio_put(GPS_RESET, true);

  for (;;) {
    nmea_buffer_t nmea_message = {0};
    if (xQueueReceive(gpsQueue, &nmea_message, pdMS_TO_TICKS(2000)) == pdTRUE) {
      strncat(nmea_message, "\r\n", 3);
      int gps_error = gps_decode(&tpv, nmea_message);
      if (GPS_OK == gps_error) {
        switch (tpv.mode) {
        case GPS_MODE_3D_FIX:
          break;
        case GPS_MODE_2D_FIX:
          break;
        case GPS_MODE_NO_FIX:
          break;
        case GPS_MODE_UNKNOWN:
          break;
        default:
          break;
        }
      }
    } else {
      puts("No GPS data for 2 seconds.");
    }
  }
}

void init_gps(void) {
  gpsQueue = xQueueCreate(10, sizeof(nmea_buffer_t));
  xTaskCreate(gps_task, "GPS", 1000, NULL, 10, NULL);
}
