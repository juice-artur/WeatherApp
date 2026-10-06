#include "bmp280.h"

#define REG_CALIB        0x88U
#define REG_ID           0xD0U
#define REG_RESET        0xE0U
#define REG_CTRL_MEAS    0xF4U
#define REG_CONFIG       0xF5U
#define REG_PRESS_MSB    0xF7U

#define CMD_SOFT_RESET   0xB6U

#define CS_LOW(dev)      HAL_GPIO_WritePin((dev)->cs_port, (dev)->cs_pin, GPIO_PIN_RESET)
#define CS_HIGH(dev)     HAL_GPIO_WritePin((dev)->cs_port, (dev)->cs_pin, GPIO_PIN_SET)

static bmp280_status_t bmp280_read_regs(bmp280_t *dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
  if (dev->use_spi)
  {
    uint8_t addr = reg & 0x7FU;
    CS_LOW(dev);
    if (HAL_SPI_Transmit(dev->hspi, &addr, 1, HAL_MAX_DELAY) != HAL_OK)
    {
      CS_HIGH(dev);
      return BMP280_ERR_BUS;
    }
    if (HAL_SPI_Receive(dev->hspi, buf, len, HAL_MAX_DELAY) != HAL_OK)
    {
      CS_HIGH(dev);
      return BMP280_ERR_BUS;
    }
    CS_HIGH(dev);
    return BMP280_OK;
  }

  if (HAL_I2C_Mem_Read(dev->hi2c, (uint16_t)(dev->dev_addr << 1),
                       reg, I2C_MEMADD_SIZE_8BIT, buf, len, HAL_MAX_DELAY) != HAL_OK)
  {
    return BMP280_ERR_BUS;
  }
  return BMP280_OK;
}

static bmp280_status_t bmp280_write_regs(bmp280_t *dev, uint8_t reg, const uint8_t *buf, uint8_t len)
{
  if (dev->use_spi)
  {
    uint8_t tx[9];
    tx[0] = reg | 0x80U;
    for (uint8_t i = 0; i < len; i++)
    {
      tx[i + 1U] = buf[i];
    }
    CS_LOW(dev);
    if (HAL_SPI_Transmit(dev->hspi, tx, (uint16_t)(len + 1U), HAL_MAX_DELAY) != HAL_OK)
    {
      CS_HIGH(dev);
      return BMP280_ERR_BUS;
    }
    CS_HIGH(dev);
    return BMP280_OK;
  }

  if (HAL_I2C_Mem_Write(dev->hi2c, (uint16_t)(dev->dev_addr << 1),
                        reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len, HAL_MAX_DELAY) != HAL_OK)
  {
    return BMP280_ERR_BUS;
  }
  return BMP280_OK;
}

static bmp280_status_t bmp280_write8(bmp280_t *dev, uint8_t reg, uint8_t value)
{
  return bmp280_write_regs(dev, reg, &value, 1);
}

static bmp280_status_t bmp280_load_calib(bmp280_t *dev)
{
  uint8_t calib[24];
  bmp280_status_t st = bmp280_read_regs(dev, REG_CALIB, calib, sizeof(calib));
  if (st != BMP280_OK)
  {
    return st;
  }

  dev->dig_T1 = (uint16_t)((calib[1] << 8) | calib[0]);
  dev->dig_T2 = (int16_t)((calib[3] << 8) | calib[2]);
  dev->dig_T3 = (int16_t)((calib[5] << 8) | calib[4]);
  dev->dig_P1 = (uint16_t)((calib[7] << 8) | calib[6]);
  dev->dig_P2 = (int16_t)((calib[9] << 8) | calib[8]);
  dev->dig_P3 = (int16_t)((calib[11] << 8) | calib[10]);
  dev->dig_P4 = (int16_t)((calib[13] << 8) | calib[12]);
  dev->dig_P5 = (int16_t)((calib[15] << 8) | calib[14]);
  dev->dig_P6 = (int16_t)((calib[17] << 8) | calib[16]);
  dev->dig_P7 = (int16_t)((calib[19] << 8) | calib[18]);
  dev->dig_P8 = (int16_t)((calib[21] << 8) | calib[20]);
  dev->dig_P9 = (int16_t)((calib[23] << 8) | calib[22]);
  return BMP280_OK;
}

static bmp280_status_t bmp280_configure(bmp280_t *dev)
{
  bmp280_status_t st;

  st = bmp280_write8(dev, REG_RESET, CMD_SOFT_RESET);
  if (st != BMP280_OK)
  {
    return st;
  }
  HAL_Delay(5);

  st = bmp280_load_calib(dev);
  if (st != BMP280_OK)
  {
    return st;
  }

  st = bmp280_write8(dev, REG_CONFIG, 0x80U | (3U << 2));
  if (st != BMP280_OK)
  {
    return st;
  }

  st = bmp280_write8(dev, REG_CTRL_MEAS, ((2U << 5) | (5U << 2) | 3U));
  return st;
}

static bmp280_status_t bmp280_check_id(bmp280_t *dev)
{
  uint8_t id = 0;
  bmp280_status_t st = bmp280_read_regs(dev, REG_ID, &id, 1);
  if (st != BMP280_OK)
  {
    return st;
  }
  if (id != BMP280_CHIP_ID_VALUE)
  {
    return BMP280_ERR_CHIP_ID;
  }
  return BMP280_OK;
}

static bmp280_status_t bmp280_read_raw(bmp280_t *dev, int32_t *adc_t, int32_t *adc_p)
{
  uint8_t data[6];
  bmp280_status_t st = bmp280_read_regs(dev, REG_PRESS_MSB, data, sizeof(data));
  if (st != BMP280_OK)
  {
    return st;
  }

  *adc_p = (int32_t)((((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | ((uint32_t)data[2] >> 4)));
  *adc_t = (int32_t)((((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | ((uint32_t)data[5] >> 4)));
  return BMP280_OK;
}

static float bmp280_compensate_t(bmp280_t *dev, int32_t adc_t, int32_t *t_fine)
{
  int32_t var1 = ((((adc_t >> 3) - ((int32_t)dev->dig_T1 << 1))) * ((int32_t)dev->dig_T2)) >> 11;
  int32_t var2 = (((((adc_t >> 4) - ((int32_t)dev->dig_T1)) *
                    ((adc_t >> 4) - ((int32_t)dev->dig_T1))) >> 12) *
                  ((int32_t)dev->dig_T3)) >> 14;

  *t_fine = var1 + var2;
  return (float)((*t_fine * 5 + 128) >> 8) / 100.0f;
}

static float bmp280_compensate_p(int32_t adc_p, int32_t t_fine, bmp280_t *dev)
{
  int64_t var1;
  int64_t var2;
  int64_t p;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dev->dig_P6;
  var2 = var2 + ((var1 * (int64_t)dev->dig_P5) << 17);
  var2 = var2 + (((int64_t)dev->dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dev->dig_P3) >> 8) + ((var1 * (int64_t)dev->dig_P2) << 12);
  var1 = ((((int64_t)1 << 47) + var1) * ((int64_t)dev->dig_P1)) >> 33;

  if (var1 == 0)
  {
    return 0.0f;
  }

  p = 1048576 - (int64_t)adc_p;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dev->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dev->dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dev->dig_P7) << 4);

  return (float)p / 256.0f / 100.0f;
}

bmp280_status_t bmp280_init_i2c(bmp280_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
  dev->hi2c = hi2c;
  dev->dev_addr = addr7;
  dev->use_spi = 0;
  dev->hspi = NULL;
  dev->cs_port = NULL;
  dev->cs_pin = 0;

  bmp280_status_t st = bmp280_check_id(dev);
  if (st != BMP280_OK)
  {
    return st;
  }
  return bmp280_configure(dev);
}

bmp280_status_t bmp280_init_spi(bmp280_t *dev, SPI_HandleTypeDef *hspi,
                                GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
  dev->hspi = hspi;
  dev->cs_port = cs_port;
  dev->cs_pin = cs_pin;
  dev->use_spi = 1;
  dev->hi2c = NULL;
  dev->dev_addr = 0;

  CS_HIGH(dev);

  bmp280_status_t st = bmp280_check_id(dev);
  if (st != BMP280_OK)
  {
    return st;
  }
  return bmp280_configure(dev);
}

bmp280_status_t bmp280_read_temperature(bmp280_t *dev, float *deg_c)
{
  int32_t adc_t;
  int32_t adc_p;
  int32_t t_fine;

  bmp280_status_t st = bmp280_read_raw(dev, &adc_t, &adc_p);
  if (st != BMP280_OK)
  {
    return st;
  }

  *deg_c = bmp280_compensate_t(dev, adc_t, &t_fine);
  return BMP280_OK;
}

bmp280_status_t bmp280_read_pressure(bmp280_t *dev, float *hpa)
{
  int32_t adc_t;
  int32_t adc_p;
  int32_t t_fine;

  bmp280_status_t st = bmp280_read_raw(dev, &adc_t, &adc_p);
  if (st != BMP280_OK)
  {
    return st;
  }

  (void)bmp280_compensate_t(dev, adc_t, &t_fine);
  *hpa = bmp280_compensate_p(adc_p, t_fine, dev);
  return BMP280_OK;
}
