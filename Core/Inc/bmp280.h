#ifndef BMP280_H
#define BMP280_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define BMP280_I2C_ADDR_DEFAULT   0x76U
#define BMP280_CHIP_ID_VALUE      0x58U

typedef enum
{
  BMP280_OK = 0,
  BMP280_ERR_BUS,
  BMP280_ERR_CHIP_ID,
  BMP280_ERR_CONFIG
} bmp280_status_t;

typedef struct
{
  I2C_HandleTypeDef *hi2c;
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
  uint16_t dev_addr;
  uint8_t use_spi;
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;
} bmp280_t;

bmp280_status_t bmp280_init_i2c(bmp280_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7);
bmp280_status_t bmp280_init_spi(bmp280_t *dev, SPI_HandleTypeDef *hspi,
                                GPIO_TypeDef *cs_port, uint16_t cs_pin);
bmp280_status_t bmp280_read_temperature(bmp280_t *dev, float *deg_c);
bmp280_status_t bmp280_read_pressure(bmp280_t *dev, float *hpa);

#ifdef __cplusplus
}
#endif

#endif /* BMP280_H */
