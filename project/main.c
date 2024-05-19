#include "pico/stdlib.h"
#include <stdio.h>

#include <stdlib.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "gps_task.h"
#include "sensors.h"

int main() {
  stdio_init_all();

  // i2c_sensorInit();

  init_gps();
  sensors_init();

  vTaskStartScheduler();
}
