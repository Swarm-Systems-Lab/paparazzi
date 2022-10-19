/*
 * Copyright (C) 2022 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file peripherals/invensense3.c
 *
 * Driver for the Invensense v3 IMUs
 * ICM40605, ICM40609, ICM42605, IIM42652 and ICM42688
 */

#include "peripherals/invensense3.h"
#include "peripherals/invensense3_regs.h"
#include "math/pprz_isa.h"
#include "math/pprz_algebra_int.h"
#include "modules/imu/imu.h"
#include "modules/core/abi.h"
#include "mcu_periph/gpio_arch.h"


/* Local functions */
static void invensense3_parse_data(struct invensense3_t *inv, volatile uint8_t *data, uint16_t len);
static void invensense3_fix_config(struct invensense3_t *inv);
static bool invensense3_register_write(struct invensense3_t *inv, uint16_t bank_reg, uint8_t value);
static bool invensense3_register_read(struct invensense3_t *inv, uint16_t bank_reg, uint16_t size);
static bool invensense3_select_bank(struct invensense3_t *inv, uint8_t bank);
static bool invensense3_config(struct invensense3_t *inv);

/* Default gyro scalings */
static const struct Int32Rates invensense3_gyro_scale[5][2] = {
  { {30267, 30267, 30267},
    {55463, 55463, 55463} }, // 250DPS
  { {60534, 60534, 60534},
    {55463, 55463, 55463} }, // 500DPS
  { {40147, 40147, 40147},
    {18420, 18420, 18420} }, // 1000DPS
  { {40147, 40147, 40147},
    {9210,  9210,  9210} },  // 2000DPS
  { {40147, 40147, 40147},
    {4605,  4605,  4605} }   // 4000DPS
};

/* Default accel scalings */
static const struct Int32Vect3 invensense3_accel_scale[5][2] = {
  { {3189, 3189, 3189},
    {5203, 5203, 5203} },   // 2G
  { {6378, 6378, 6378},
    {5203, 5203, 5203} },   // 4G
  { {12756, 12756, 12756},
    {5203,  5203,  5203} }, // 8G
  { {25512, 25512, 25512},
    {5203,  5203,  5203} }, // 16G
  { {51024, 51024, 51024},
    {5203,  5203,  5203} }  // 30G
};

/* AAF settings (3dB Bandwidth [Hz], AAF_DELT, AAF_DELTSQR, AAF_BITSHIFT) */
static const uint16_t invensense3_aaf[][4] = {
  {42, 1, 1, 15},
  {84, 2, 4, 13},
  {126, 3, 9, 12},
  {170, 4, 16, 11},
  {213, 5, 25, 10},
  {258, 6, 36, 10},
  {303, 7, 49, 9},
  {348, 8, 64, 9},
  {394, 9, 81, 9},
  {441, 10, 100, 8},
  {488, 11, 122, 8},
  {536, 12, 144, 8},
  {585, 13, 170, 8},
  {634, 14, 196, 7},
  {684, 15, 224, 7},
  {734, 16, 256, 7},
  {785, 17, 288, 7},
  {837, 18, 324, 7},
  {890, 19, 360, 6},
  {943, 20, 400, 6},
  {997, 21, 440, 6},
  {1051, 22, 488, 6},
  {1107, 23, 528, 6},
  {1163, 24, 576, 6},
  {1220, 25, 624, 6},
  {1277, 26, 680, 6},
  {1336, 27, 736, 5},
  {1395, 28, 784, 5},
  {1454, 29, 848, 5},
  {1515, 30, 896, 5},
  {1577, 31, 960, 5},
  {1639, 32, 1024, 5},
  {1702, 33, 1088, 5},
  {1766, 34, 1152, 5},
  {1830, 35, 1232, 5},
  {1896, 36, 1296, 5},
  {1962, 37, 1376, 4},
  {2029, 38, 1440, 4},
  {2097, 39, 1536, 4},
  {2166, 40, 1600, 4},
  {2235, 41, 1696, 4},
  {2306, 42, 1760, 4},
  {2377, 43, 1856, 4},
  {2449, 44, 1952, 4},
  {2522, 45, 2016, 4},
  {2596, 46, 2112, 4},
  {2671, 47, 2208, 4},
  {2746, 48, 2304, 4},
  {2823, 49, 2400, 4},
  {2900, 50, 2496, 4},
  {2978, 51, 2592, 4},
  {3057, 52, 2720, 4},
  {3137, 53, 2816, 3},
  {3217, 54, 2944, 3},
  {3299, 55, 3008, 3},
  {3381, 56, 3136, 3},
  {3464, 57, 3264, 3},
  {3548, 58, 3392, 3},
  {3633, 59, 3456, 3},
  {3718, 60, 3584, 3},
  {3805, 61, 3712, 3},
  {3892, 62, 3840, 3},
  {3979, 63, 3968, 3}
};

static const uint16_t invensense3_aaf4x605[][4] = {
  {10, 1, 1, 15},
  {21, 2, 4, 13},
  {32, 3, 9, 12},
  {42, 4, 16, 11},
  {53, 5, 25, 10},
  {64, 6, 36, 10},
  {76, 7, 49, 9},
  {87, 8, 64, 9},
  {99, 9, 81, 9},
  {110, 10, 100, 8},
  {122, 11, 122, 8},
  {134, 12, 144, 8},
  {146, 13, 170, 8},
  {158, 14, 196, 7},
  {171, 15, 224, 7},
  {184, 16, 256, 7},
  {196, 17, 288, 7},
  {209, 18, 324, 7},
  {222, 19, 360, 6},
  {236, 20, 400, 6},
  {249, 21, 440, 6},
  {263, 22, 488, 6},
  {277, 23, 528, 6},
  {291, 24, 576, 6},
  {305, 25, 624, 6},
  {319, 26, 680, 6},
  {334, 27, 736, 5},
  {349, 28, 784, 5},
  {364, 29, 848, 5},
  {379, 30, 896, 5},
  {394, 31, 960, 5},
  {410, 32, 1024, 5},
  {425, 33, 1088, 5},
  {441, 34, 1152, 5},
  {458, 35, 1232, 5},
  {474, 36, 1296, 5},
  {490, 37, 1376, 4},
  {507, 38, 1440, 4},
  {524, 39, 1536, 4},
  {541, 40, 1600, 4},
  {559, 41, 1696, 4},
  {576, 42, 1760, 4},
  {594, 43, 1856, 4},
  {612, 44, 1952, 4},
  {631, 45, 2016, 4},
  {649, 46, 2112, 4},
  {668, 47, 2208, 4},
  {687, 48, 2304, 4},
  {706, 49, 2400, 4},
  {725, 50, 2496, 4},
  {745, 51, 2592, 4},
  {764, 52, 2720, 4},
  {784, 53, 2816, 3},
  {804, 54, 2944, 3},
  {825, 55, 3008, 3},
  {845, 56, 3136, 3},
  {866, 57, 3264, 3},
  {887, 58, 3392, 3},
  {908, 59, 3456, 3},
  {930, 60, 3584, 3},
  {951, 61, 3712, 3},
  {973, 62, 3840, 3},
  {995, 63, 3968, 3}
};

/**
 * @brief Initialize the invensense v3 sensor instance
 * 
 * @param inv The structure containing the configuration of the invensense v3 instance
 */
void invensense3_init(struct invensense3_t *inv) {
  /* General setup */
  inv->status = INVENSENSE3_IDLE;
  inv->device = INVENSENSE3_UNKOWN;
  inv->register_bank = 0xFF;
  inv->config_idx = 0;

  /* SPI setup */
  if(inv->bus == INVENSENSE3_SPI) {
    inv->spi.trans.cpol = SPICpolIdleHigh;
    inv->spi.trans.cpha = SPICphaEdge2;
    inv->spi.trans.dss = SPIDss8bit;
    inv->spi.trans.bitorder = SPIMSBFirst;
    inv->spi.trans.cdiv = SPIDiv16;

    inv->spi.trans.select = SPISelectUnselect;
    inv->spi.trans.slave_idx = inv->spi.slave_idx;
    inv->spi.trans.output_length = 0;
    inv->spi.trans.input_length = 0;
    inv->spi.trans.before_cb = NULL;
    inv->spi.trans.after_cb = NULL;
    inv->spi.trans.input_buf = inv->spi.rx_buf;
    inv->spi.trans.output_buf = inv->spi.tx_buf;
    inv->spi.trans.status = SPITransDone;
  }
  /* I2C setup */
  else {
    inv->i2c.trans.slave_addr = inv->i2c.slave_addr;
    inv->i2c.trans.status = I2CTransDone;
  }
}

/**
 * @brief Should be called periodically to request sensor readings
 * - First detects the sensor using WHO_AM_I reading
 * - Configures the sensor according the users requested configuration
 * - Requests a sensor reading by reading the FIFO_COUNT register
 * 
 * @param inv The invensense v3 instance
 */
void invensense3_periodic(struct invensense3_t *inv) {
  /* Idle */
  if((inv->bus == INVENSENSE3_SPI && inv->spi.trans.status == SPITransDone) || 
     (inv->bus == INVENSENSE3_I2C && inv->i2c.trans.status == I2CTransDone)) {

    /* Depending on the status choose what to do */
    switch(inv->status) {
      case INVENSENSE3_IDLE:
        /* Request WHO_AM_I */
        invensense3_register_read(inv, INV3REG_WHO_AM_I, 1);
        break;
      case INVENSENSE3_CONFIG:
        /* Start configuring */
        if(invensense3_config(inv)) {
          inv->status = INVENSENSE3_RUNNING;
        }
        break;
      case INVENSENSE3_RUNNING:
        /* Request a sensor reading */
        invensense3_register_read(inv, INV3REG_FIFO_COUNTH, 2);
        break;
    }
  }
}

/**
 * @brief Should be called in the event thread
 * - Checks the response of the WHO_AM_I reading
 * - Configures the sensor and reads the responses
 * - Parse and request the sensor data from the FIFO buffers
 * 
 * @param inv The invensense v3 instance
 */
void invensense3_event(struct invensense3_t *inv) {
  volatile uint8_t *rx_buffer, *tx_buffer;
  uint16_t rx_length = 0;

  /* Set the buffers depending on the interface */
  if(inv->bus == INVENSENSE3_SPI) {
    rx_buffer = inv->spi.rx_buf;
    tx_buffer = inv->spi.tx_buf;
    rx_length = inv->spi.trans.input_length;
  }
  else {
    rx_buffer = inv->i2c.trans.buf;
    tx_buffer = inv->i2c.trans.buf;
    rx_length = inv->i2c.trans.len_r;
  }

  /* Successful transfer */
  if((inv->bus == INVENSENSE3_SPI && inv->spi.trans.status == SPITransSuccess) || 
     (inv->bus == INVENSENSE3_I2C && inv->i2c.trans.status == I2CTransSuccess)) {

    /* Update the register bank */
    if(tx_buffer[0] == INV3REG_BANK_SEL)
      inv->register_bank = inv->spi.tx_buf[1] >> 4;

    /* Set the transaction as done and update register bank if needed */
    if(inv->bus == INVENSENSE3_SPI)
      inv->spi.trans.status = SPITransDone;
    else
      inv->i2c.trans.status = I2CTransDone;
    
    /* Look at the results */
    switch(inv->status) {
      case INVENSENSE3_IDLE:
        /* Check the response of the WHO_AM_I */
        inv->status = INVENSENSE3_CONFIG;
        if(rx_buffer[1] == INV3_WHOAMI_ICM40605) {
          inv->device = INVENSENSE3_ICM40605;
        } else if(rx_buffer[1] == INV3_WHOAMI_ICM40609) {
          inv->device = INVENSENSE3_ICM40609;
        } else if(rx_buffer[1] == INV3_WHOAMI_ICM42605) {
          inv->device = INVENSENSE3_ICM42605;
        } else if(rx_buffer[1] == INV3_WHOAMI_IIM42652) {
          inv->device = INVENSENSE3_IIM42652;
        } else if(rx_buffer[1] == INV3_WHOAMI_ICM42688) {
          inv->device = INVENSENSE3_ICM42688;
        } else {
          inv->status = INVENSENSE3_IDLE;
        }

        /* Fix the configuration and set the scaling */
        if(inv->status == INVENSENSE3_CONFIG)
          invensense3_fix_config(inv);
        break;
      case INVENSENSE3_CONFIG:
        /* Apply the next configuration register */
        if(invensense3_config(inv)) {
          inv->status = INVENSENSE3_RUNNING;
        }
        break;
      case INVENSENSE3_RUNNING: {
        /* Parse the results */
        static const uint16_t max_bytes = sizeof(inv->spi.rx_buf) - 3;
        uint16_t fifo_bytes = (uint16_t)rx_buffer[1] << 8 | rx_buffer[2];

        // We read an incorrect length (try again)
        if(fifo_bytes > 4096) {
          invensense3_register_read(inv, INV3REG_FIFO_COUNTH, 2);
          return;
        }

        // Parse the data
        if((rx_length - 3) > 0) {
          uint16_t valid_bytes = ((rx_length - 3) < fifo_bytes)? (rx_length - 3) : fifo_bytes;
          invensense3_parse_data(inv, &rx_buffer[3], valid_bytes);
          inv->timer -= valid_bytes;
        } else {
          fifo_bytes -= fifo_bytes%INVENSENSE3_SAMPLE_SIZE;
          inv->timer = fifo_bytes;
        }

        // If we have more data request more
        if(inv->timer > 0) {
          uint16_t read_bytes = (inv->timer > max_bytes)? max_bytes : inv->timer;
          invensense3_register_read(inv, INV3REG_FIFO_COUNTH, 2 + read_bytes);
        }
        break;
      }
    }
  }
  /* Failed transaction */
  if((inv->bus == INVENSENSE3_SPI && inv->spi.trans.status == SPITransFailed) || 
     (inv->bus == INVENSENSE3_I2C && inv->i2c.trans.status == I2CTransFailed)) {

    /* Set the transaction as done and update register bank if needed */
    if(inv->bus == INVENSENSE3_SPI) {
      inv->spi.trans.status = SPITransDone;
    }
    else {
      inv->i2c.trans.status = I2CTransDone;
    }

    /* Retry or ignore */
    switch(inv->status) {
      case INVENSENSE3_CONFIG:
        /* If was not a bus switch decrease the index */
        if(inv->config_idx > 0 && ((inv->bus == INVENSENSE3_SPI && inv->spi.tx_buf[0] != INV3REG_BANK_SEL) ||
        (inv->bus == INVENSENSE3_I2C && inv->i2c.trans.buf[0] != INV3REG_BANK_SEL))) {
          inv->config_idx--;
        }
        /* Try again */
        if(invensense3_config(inv)) {
          inv->status = INVENSENSE3_RUNNING;
        }
        break;
      case INVENSENSE3_IDLE:
      case INVENSENSE3_RUNNING:
        /* Ignore while idle/running */
        break;
    }
  }
}

/**
 * @brief Parse the FIFO buffer data
 * 
 * @param inv The invensense v3 instance
 * @param data The FIFO buffer data to parse
 * @param len The length of the FIFO buffer
 */
static void invensense3_parse_data(struct invensense3_t *inv, volatile uint8_t *data, uint16_t len) {
  uint8_t samples = len  / INVENSENSE3_SAMPLE_SIZE;
  static struct Int32Vect3 accel[INVENSENSE3_SAMPLE_CNT] = {0};
  static struct Int32Rates gyro[INVENSENSE3_SAMPLE_CNT] = {0};

  if(samples > INVENSENSE3_SAMPLE_CNT)
    samples = INVENSENSE3_SAMPLE_CNT;

  uint16_t gyro_samplerate = 9000;
  if(inv->gyro_dlpf != INVENSENSE3_GYRO_DLPF_OFF)
    gyro_samplerate = 1125;

  uint16_t accel_samplerate = 4500;
  if(inv->accel_dlpf != INVENSENSE3_ACCEL_DLPF_OFF)
    accel_samplerate = 1125;

  // Go through the different samples
  uint8_t j = 0;
  uint8_t downsample = gyro_samplerate / accel_samplerate;
  int32_t temp = 0;
  for(uint8_t i = 0; i < samples; i++) { 
    if(i % downsample == 0) {
      accel[j].x = (int16_t)((uint16_t)data[2] << 8 | data[3]);
      accel[j].y = (int16_t)((uint16_t)data[0] << 8 | data[1]);
      accel[j].z = -(int16_t)((uint16_t)data[4] << 8 | data[5]);
      j++;
    }

    gyro[i].p = (int16_t)((uint16_t)data[8] << 8 | data[9]);
    gyro[i].q = (int16_t)((uint16_t)data[6] << 8 | data[7]);
    gyro[i].r = -(int16_t)((uint16_t)data[10] << 8 | data[11]);

    temp += (int16_t)((uint16_t)data[12] << 8 | data[13]);
    data += INVENSENSE3_SAMPLE_SIZE;
  }

  float temp_f = ((float)temp / samples) / 333.87f - 21.f;

  // Send the scaled values over ABI
  uint32_t now_ts = get_sys_time_usec();
  AbiSendMsgIMU_GYRO_RAW(inv->abi_id, now_ts, gyro, samples);
  AbiSendMsgIMU_ACCEL_RAW(inv->abi_id, now_ts, accel, j);
  AbiSendMsgTEMPERATURE(inv->abi_id, temp_f);
}

/**
 * @brief This fixes the configuration errors and sets the correct scalings
 * 
 * @param inv The invensense v3 instance
 */
static void invensense3_fix_config(struct invensense3_t *inv) {
  /* Fix wrong configuration settings (prevent user error) */
  if(inv->gyro_odr > INVENSENSE3_GYRO_ODR_12_5HZ && inv->gyro_odr != INVENSENSE3_GYRO_ODR_500HZ)
    inv->gyro_odr = INVENSENSE3_GYRO_ODR_12_5HZ;
  
  if(inv->device == INVENSENSE3_ICM40605 || inv->device == INVENSENSE3_ICM42605) {
    if(inv->gyro_odr > INVENSENSE3_GYRO_ODR_25HZ && inv->gyro_odr != INVENSENSE3_GYRO_ODR_500HZ)
      inv->gyro_odr = INVENSENSE3_GYRO_ODR_25HZ;
    else if(inv->gyro_odr < INVENSENSE3_GYRO_ODR_8KHZ)
      inv->gyro_odr = INVENSENSE3_GYRO_ODR_8KHZ;
    
    if(inv->device == INVENSENSE3_ICM40605 && inv->accel_odr > INVENSENSE3_ACCEL_ODR_25HZ && inv->accel_odr != INVENSENSE3_ACCEL_ODR_500HZ)
      inv->accel_odr = INVENSENSE3_ACCEL_ODR_25HZ;
    else if(inv->accel_odr < INVENSENSE3_ACCEL_ODR_8KHZ)
      inv->accel_odr = INVENSENSE3_ACCEL_ODR_8KHZ;
  }

  if(inv->device != INVENSENSE3_ICM40609 && inv->accel_range < INVENSENSE3_ACCEL_RANGE_16G)
    inv->accel_range = INVENSENSE3_ACCEL_RANGE_16G;
  else if(inv->device == INVENSENSE3_ICM40609 && inv->accel_range > INVENSENSE3_ACCEL_RANGE_4G)
    inv->accel_range = INVENSENSE3_ACCEL_RANGE_4G;

  /* Fix the AAF bandwidth setting */
  const uint16_t (*aaf_table)[4];
  if(inv->device == INVENSENSE3_ICM40605 || inv->device == INVENSENSE3_ICM42605) {
    aaf_len = sizeof(invensense3_aaf4x605) / sizeof(invensense3_aaf4x605[0]);
    aaf_table = invensense3_aaf4x605;
  }
  else {
    aaf_len = sizeof(invensense3_aaf) / sizeof(invensense3_aaf[0]);
    aaf_table = invensense3_aaf;
  }

  uint16_t i = 0;
  for(i = 0; i < aaf_len; i++) {
    if(inv->gyro_aaf <= aaf_table[i][0]) {
      inv->gyro_aaf = aaf_table[i][0];
      inv->gyro_aaf_regs = aaf_table[i];
      break;
    }
  }
  if(i >= (aaf_len-1)) {
    inv->gyro_aaf = aaf_table[aaf_len-1][0];
    inv->gyro_aaf_regs = aaf_table[aaf_len-1];
  }

  for(i = 0; i < aaf_len; i++) {
    if(inv->accel_aaf <= aaf_table[i][0]) {
      inv->accel_aaf = aaf_table[i][0];
      inv->accel_aaf_regs = aaf_table[i];
      break;
    }
  }
  if(i >= (aaf_len-1)) {
    inv->accel_aaf = aaf_table[aaf_len-1][0];
    inv->accel_aaf_regs = aaf_table[aaf_len-1];
  }
  
  /* Set the default values */
  imu_set_defaults_gyro(inv->abi_id, NULL, NULL, invensense3_gyro_scale[inv->gyro_range]);
  imu_set_defaults_accel(inv->abi_id, NULL, NULL, invensense3_accel_scale[inv->accel_range]);
}

/**
 * @brief Write a register based on a combined bank/regsiter value
 * 
 * @param inv The invensense v3 instance
 * @param bank_reg The bank is shifted 8 bits left, adn register is &0xFF
 * @param value The value to write to the register
 * @return true Whenever the register write was started
 * @return false First we are busy switching the register bank
 */
static bool invensense3_register_write(struct invensense3_t *inv, uint16_t bank_reg, uint8_t value) {
  /* Split the register and bank */
  uint8_t bank = bank_reg >> 8;
  uint8_t reg = bank_reg & 0xFF;

  /* Switch the register bank if needed */
  if(invensense3_select_bank(inv, bank)) {
    return false;
  }

  /* SPI transaction */
  if(inv->bus == INVENSENSE3_SPI) {
    inv->spi.trans.output_length = 2;
    inv->spi.trans.input_length = 0;
    inv->spi.tx_buf[0] = reg;
    inv->spi.tx_buf[1] = value;
    spi_submit(inv->spi.p, &(inv->spi.trans));
  }
  /* I2C transaction */
  else {
    inv->i2c.trans.buf[0] = reg;
    inv->i2c.trans.buf[1] = value;
    i2c_transmit(inv->i2c.p, &(inv->i2c.trans), inv->i2c.slave_addr, 2);
  }

  return true;
}

/**
 * @brief Read a register based on a combined bank/regsiter value
 * 
 * @param inv The invensense v3 instance
 * @param bank_reg The bank is shifted 8 bits left, adn register is &0xFF
 * @param size The size to read (already 1 is added for the transmission of the register to read)
 * @return true If we initiated the register read succesfully
 * @return false First we are busy switching the register bank
 */
static bool invensense3_register_read(struct invensense3_t *inv, uint16_t bank_reg, uint16_t size) {
  /* Split the register and bank */
  uint8_t bank = bank_reg >> 8;
  uint8_t reg = bank_reg & 0xFF;

  /* Switch the register bank if needed */
  if(invensense3_select_bank(inv, bank)) {
    return false;
  }

  /* SPI transaction */
  if(inv->bus == INVENSENSE3_SPI) {
    inv->spi.trans.output_length = 2;
    inv->spi.trans.input_length = 1 + size;
    inv->spi.tx_buf[0] = reg | INV3_READ_FLAG;
    inv->spi.tx_buf[1] = 0;
    spi_submit(inv->spi.p, &(inv->spi.trans));
  }
  /* I2C transaction */
  else {
    inv->i2c.trans.buf[0] = reg | INV3_READ_FLAG;
    i2c_transceive(inv->i2c.p, &(inv->i2c.trans), inv->i2c.slave_addr, 1, (1 + size));
  }

  return true;
}

/**
 * @brief Select the correct register bank
 * 
 * @param inv The invensense v3 instance
 * @param bank The bank ID to select
 * @return true The bank change has been requested
 * @return false The register bank is already correct
 */
static bool invensense3_select_bank(struct invensense3_t *inv, uint8_t bank) {
  /* If we already selected the correct bank continue */
  if(inv->register_bank == bank)
    return false;

  /* SPI transaction */
  if(inv->bus == INVENSENSE3_SPI) {
    inv->spi.trans.output_length = 2;
    inv->spi.trans.input_length = 0;
    inv->spi.tx_buf[0] = INV3REG_BANK_SEL;
    inv->spi.tx_buf[1] = bank << 4;
    spi_submit(inv->spi.p, &(inv->spi.trans));
  }
  /* I2C transaction */
  else {
    inv->i2c.trans.buf[0] = INV3REG_BANK_SEL;
    inv->i2c.trans.buf[1] = bank << 4;
    i2c_transmit(inv->i2c.p, &(inv->i2c.trans), inv->i2c.slave_addr, 2);
  }
  
  return true;
}

/**
 * @brief Configure the Invensense 3 device register by register
 * 
 * @param inv The invensense v3 instance
 * @return true When the configuration is completed
 * @return false Still busy configuring
 */
static bool invensense3_config(struct invensense3_t *inv) {
  switch(inv->config_idx) {
    case 0:
      /* Reset the device */
      if(invensense3_register_write(inv, INV3REG_DEVICE_CONFIG, BIT_DEVICE_CONFIG_SOFT_RESET_CONFIG))
        inv->config_idx++;
      inv->timer = get_sys_time_usec();
      break;
    case 1: {
      /* Because reset takes time wait ~5ms */
      if((get_sys_time_usec() - inv->timer) < 5e3)
        break;

      /* Start the accel en gyro in low noise mode */
      if(register_write(INV3REG_PWR_MGMT0, (ACCEL_MODE_LN << ACCEL_MODE_SHIFT) | (GYRO_MODE_LN << GYRO_MODE_SHIFT)))
        inv->config_idx++;
      inv->timer = get_sys_time_usec();
      break;
    }
    case 2: {
      /* Because starting takes time wait ~1ms */
      if((get_sys_time_usec() - inv->timer) < 1e3)
        break;

      /* Configure the gyro ODR/FS */
      if(invensense3_register_write(inv, INV3REG_GYRO_CONFIG0, (inv->gyro_range << GYRO_FS_SEL_SHIFT) | (inv->gyro_odr << GYRO_ODR_SHIFT)))
        inv->config_idx++;
      break;
    }
    case 3: {
      /* Configure the accel ODR/FS */
      uint8_t accel_config = (inv->accel_odr << ACCEL_ODR_SHIFT);
      if(inv->device == INVENSENSE3_ICM20649 && inv->gyro_range > 0)
        accel_config |= inv->accel_range << ACCEL_FS_SEL_SHIFT;
      else
        accel_config |= (inv->accel_range - 1) << ACCEL_FS_SEL_SHIFT;

      if(invensense3_register_write(inv, INV3REG_ACCEL_CONFIG0, accel_config))
        inv->config_idx++;
      break;
    }
    case 4:
      /* Configure gyro AAF enable */
      if(invensense3_register_write(inv, INV3REG_GYRO_CONFIG_STATIC2, 0x00))
        inv->config_idx++;
      break;
    case 5:
      /* Configure gyro AAF DELT */
      if(invensense3_register_write(inv, INV3REG_GYRO_CONFIG_STATIC3, inv->gyro_aaf_regs[1]))
        inv->config_idx++;
      break;
    case 6:
      /* Configure gyro AAF DELTSQR lower */
      if(invensense3_register_write(inv, INV3REG_GYRO_CONFIG_STATIC4, (inv->gyro_aaf_regs[2]&0xFF)))
        inv->config_idx++;
      break;
    case 7:
      /* Configure gyro AAF DELTSQR upper and bitshift */
      if(invensense3_register_write(inv, INV3REG_GYRO_CONFIG_STATIC5, (inv->gyro_aaf_regs[3] << GYRO_AAF_BITSHIFT_SHIFT) | ((inv->gyro_aaf_regs[2]>>8) & 0x0F)))
        inv->config_idx++;
      break;
    case 8:
      /* Configure accel AAF enable and DELT */
      if(invensense3_register_write(inv, INV3REG_ACCEL_CONFIG_STATIC2, (inv->accel_aaf_regs[1] << ACCEL_AAF_DELT_SHIFT)))
        inv->config_idx++;
      break;
    case 9:
      /* Configure accel AAF DELTSQR lower */
      if(invensense3_register_write(inv, INV3REG_ACCEL_CONFIG_STATIC3, (inv->accel_aaf_regs[2]&0xFF)))
        inv->config_idx++;
      break;
    case 10:
      /* Configure accel AAF DELTSQR upper and bitshift */
      if(invensense3_register_write(inv, INV3REG_ACCEL_CONFIG_STATIC5, (inv->accel_aaf_regs[3] << ACCEL_AAF_BITSHIFT_SHIFT) | ((inv->accel_aaf_regs[2]>>8) & 0x0F)))
        inv->config_idx++;
      break;


    case 8:
      /* FIFO reset 2 */
      if(invensense3_register_write(inv, INV3REG_FIFO_RST, 0x00))
        inv->config_idx++;
      break;
    case 9: {
      /* Enable FIFO */
      uint8_t user_ctrl = BIT_USER_CTRL_FIFO_EN;
      if(inv->bus == INVENSENSE3_SPI)
        user_ctrl |= BIT_USER_CTRL_I2C_IF_DIS;
      if(invensense3_register_write(inv, INV3REG_USER_CTRL, user_ctrl))
        inv->config_idx++;
      break;
    }
    case 10:
      /* Cofigure FIFO enable */
      if(invensense3_register_write(inv, INV3REG_FIFO_EN_2, BIT_XG_FIFO_EN | BIT_YG_FIFO_EN |
                    BIT_ZG_FIFO_EN | BIT_ACCEL_FIFO_EN | BIT_TEMP_FIFO_EN))
        inv->config_idx++;
      break;
    case 11:
      /* Enable interrupt pin/status */
      if(invensense3_register_write(inv, INV3REG_INT_ENABLE_1, 0x1))
        inv->config_idx++;
      break;
    default:
      inv->timer = 0;
      return true;
  }
  return false;
}
