#include "pinmap.h"
#include "pio_spi.h"

static pio_spi_inst_t spi;
static uint offset;

void my_spi_init() {
  spi.pio = pio0;
  spi.sm = 0;
  spi.cs_pin = ALTIMETER_CS;

  offset = pio_add_program(spi.pio, &spi_cpha0_program);

  gpio_init(ALTIMETER_CS);
  gpio_set_dir(ALTIMETER_CS, true);
  gpio_put(ALTIMETER_CS, true);

  gpio_init(IMU_CS);
  gpio_set_dir(IMU_CS, true);
  gpio_put(IMU_CS, true);

  pio_spi_init(spi.pio, spi.sm, offset, 8, 31.25f, false, false, SCK1, SDI1,
               SDO1);

  uint8_t test[4] = {0xFF, 0x55, 0xAA, 0xF0};
  for (;;) {
    pio_spi_write8_blocking(&spi, test, sizeof(test));
  }
}
