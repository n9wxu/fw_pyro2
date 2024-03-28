#include "FreeRTOS.h"
#include "task.h"

#include "hardware/gpio.h"
#include "hardware/spi.h"

#include "pinmap.h"

#include "lsm6ds3tr-c_reg.h"

#include "pico/binary_info.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void platform_delay(uint32_t ms);

static void worker(void *parameter) {

  static int16_t data_raw_acceleration[3];
  static int16_t data_raw_angular_rate[3];
  static int16_t data_raw_temperature;
  static float acceleration_mg[3];
  static float angular_rate_mdps[3];
  static float temperature_degC;
  static uint8_t tx_buffer[1000] = {0};

  gpio_init(IMU_CS);
  gpio_set_dir(IMU_CS, true);
  gpio_put(IMU_CS, true);

  gpio_init(ALTIMETER_CS);
  gpio_set_dir(ALTIMETER_CS, true);
  gpio_put(ALTIMETER_CS, true);

  spi_init(spi1, 100000);
  gpio_set_function(SCK1, GPIO_FUNC_SPI);
  gpio_set_function(SDO1, GPIO_FUNC_SPI);
  gpio_set_function(SDI1, GPIO_FUNC_SPI);
  bi_decl(bi_3pins_with_func(SDI1, SDO1, SCK1, GPIO_FUNC_SPI));
  bi_decl(bi_1pin_with_name(IMU_CS, "IMU SPI CS"));
  bi_decl(bi_1pin_with_name(ALTIMETER_CS, "Altimeter SPI CS"));

  spi_set_format(spi1, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
  spi_write_blocking(spi1, tx_buffer,
                     1); // dummy write to force the SDO to idle high

  /* Initialize mems driver interface */
  stmdev_ctx_t dev_ctx;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = spi1;

  vTaskDelay(pdMS_TO_TICKS(20));

  uint8_t whoamI = 0;
  lsm6ds3tr_c_device_id_get(&dev_ctx, &whoamI);
  if (whoamI != LSM6DS3TR_C_ID) {
    printf("No IMU found: %x\n", whoamI);
  }
  assert(whoamI == LSM6DS3TR_C_ID);

  /* Restore default configuration */
  lsm6ds3tr_c_reset_set(&dev_ctx, PROPERTY_ENABLE);

  uint8_t rst = 0;
  do {
    lsm6ds3tr_c_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lsm6ds3tr_c_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /* Set Output Data Rate */
  lsm6ds3tr_c_xl_data_rate_set(&dev_ctx, LSM6DS3TR_C_XL_ODR_12Hz5);
  lsm6ds3tr_c_gy_data_rate_set(&dev_ctx, LSM6DS3TR_C_GY_ODR_12Hz5);
  /* Set full scale */
  lsm6ds3tr_c_xl_full_scale_set(&dev_ctx, LSM6DS3TR_C_2g);
  lsm6ds3tr_c_gy_full_scale_set(&dev_ctx, LSM6DS3TR_C_2000dps);
  /* Configure filtering chain(No aux interface) */
  /* Accelerometer - analog filter */
  lsm6ds3tr_c_xl_filter_analog_set(&dev_ctx, LSM6DS3TR_C_XL_ANA_BW_400Hz);
  /* Accelerometer - LPF1 path ( LPF2 not used )*/
  // lsm6ds3tr_c_xl_lp1_bandwidth_set(&dev_ctx, LSM6DS3TR_C_XL_LP1_ODR_DIV_4);
  /* Accelerometer - LPF1 + LPF2 path */
  lsm6ds3tr_c_xl_lp2_bandwidth_set(&dev_ctx,
                                   LSM6DS3TR_C_XL_LOW_NOISE_LP_ODR_DIV_100);
  /* Accelerometer - High Pass / Slope path */
  // lsm6ds3tr_c_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
  // lsm6ds3tr_c_xl_hp_bandwidth_set(&dev_ctx, LSM6DS3TR_C_XL_HP_ODR_DIV_100);
  /* Gyroscope - filtering chain */
  lsm6ds3tr_c_gy_band_pass_set(&dev_ctx, LSM6DS3TR_C_HP_260mHz_LP1_STRONG);

  for (;;) {
    /* Read output only if new value is available */
    lsm6ds3tr_c_reg_t reg;
    lsm6ds3tr_c_status_reg_get(&dev_ctx, &reg.status_reg);

    if (reg.status_reg.xlda) {
      /* Read magnetic field data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      lsm6ds3tr_c_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
      acceleration_mg[0] =
          lsm6ds3tr_c_from_fs2g_to_mg(data_raw_acceleration[0]);
      acceleration_mg[1] =
          lsm6ds3tr_c_from_fs2g_to_mg(data_raw_acceleration[1]);
      acceleration_mg[2] =
          lsm6ds3tr_c_from_fs2g_to_mg(data_raw_acceleration[2]);
      sprintf((char *)tx_buffer, "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
      puts(tx_buffer);
    }

    if (reg.status_reg.gda) {
      /* Read magnetic field data */
      memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
      lsm6ds3tr_c_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
      angular_rate_mdps[0] =
          lsm6ds3tr_c_from_fs2000dps_to_mdps(data_raw_angular_rate[0]);
      angular_rate_mdps[1] =
          lsm6ds3tr_c_from_fs2000dps_to_mdps(data_raw_angular_rate[1]);
      angular_rate_mdps[2] =
          lsm6ds3tr_c_from_fs2000dps_to_mdps(data_raw_angular_rate[2]);
      sprintf((char *)tx_buffer, "Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
      puts(tx_buffer);
    }

    if (reg.status_reg.tda) {
      /* Read temperature data */
      memset(&data_raw_temperature, 0x00, sizeof(int16_t));
      lsm6ds3tr_c_temperature_raw_get(&dev_ctx, &data_raw_temperature);
      temperature_degC = lsm6ds3tr_c_from_lsb_to_celsius(data_raw_temperature);
      sprintf((char *)tx_buffer, "Temperature [degC]:%6.2f\r\n",
              temperature_degC);
      puts(tx_buffer);
    }
  }
}

void sensors_init(void) { xTaskCreate(worker, "Sensors", 4096, NULL, 5, NULL); }

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len) {
  spi_inst_t *spi = handle;

  asm volatile("nop \n nop \n nop");
  gpio_put(IMU_CS, false);
  asm volatile("nop \n nop \n nop");
  spi_write_blocking(spi, &reg, 1);
  spi_write_blocking(spi, bufp, len);
  asm volatile("nop \n nop \n nop");
  gpio_put(IMU_CS, true);
  asm volatile("nop \n nop \n nop");
}
/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len) {
  spi_inst_t *spi = handle;

  reg |= 0x80;
  asm volatile("nop \n nop \n nop");
  gpio_put(IMU_CS, false);
  asm volatile("nop \n nop \n nop");
  spi_write_blocking(spi, &reg, 1);
  spi_read_blocking(spi, 0, bufp, len);
  asm volatile("nop \n nop \n nop");
  gpio_put(IMU_CS, true);
  asm volatile("nop \n nop \n nop");
}

static void platform_delay(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }
