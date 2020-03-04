/*
 * LSM9DS1.c
 *
 *  Created on: Feb 24, 2018
 *      Author: steveb
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include "spi.h"
#include "gpio.h"
#include "transact_SPI.h"
#include "LSM9DS1.h"
#include "LSM9DS1_registers.h"

void initialize_accelerometer_and_gyroscope(
    volatile struct spi_register *spi,
    volatile struct gpio_register*gpio,
    int                           chip_select_AG_pin )
{
  union  LSM9DS1_transaction  transaction;

  /*
   * read the who_am_i register... just for kicks
   * in CTRL_REG_G, enable the gyro (low speed, low rate is fine)
   * in CTRL_REG_G, set the output to the data register
   * the status register has flags for whether data is available or not... I probably need to wait for boot complete
   * in CTRL_REG, enable the accel (low speed, low G is fine), BLE set to Little Endian (just in case)
   */
  /* print WHO_AM_I */
  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_WHO_AM_I;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.WHO_AM_I), chip_select_AG_pin, gpio, spi );
  printf( "WHOAMI (0x68) = 0x%2.2X\n", transaction.field.body.WHO_AM_I.SIX_EIGHT );

  /* in CTRL_REG1_G, enable the gyro (low speed, low rate is fine)
   * in CTRL_REG2_G, set the output to the data register
   */
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG1_G;
  transaction.field.body.CTRL_REG1_G.BW_G         = 3;  // see table 47
  transaction.field.body.CTRL_REG1_G.FS_G         = 0;
  transaction.field.body.CTRL_REG1_G.ODR_G        = 6;  // see table 46
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG1_G), chip_select_AG_pin, gpio, spi );
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG2_G;
  transaction.field.body.CTRL_REG2_G.OUT_SEL      = 0;
  transaction.field.body.CTRL_REG2_G.INT_SEL      = 0;
  transaction.field.body.CTRL_REG2_G.zero         = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG2_G), chip_select_AG_pin, gpio, spi );

  /* in CTRL_REG, enable the accel (low speed, low G is fine), set block data update (race conditions are bad), BLE set to Little Endian (just in case), disable I2C
   */
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG4;
  transaction.field.body.CTRL_REG4.zero1          = 0;
  transaction.field.body.CTRL_REG4.Zen_G          = 1;
  transaction.field.body.CTRL_REG4.Yen_G          = 1;
  transaction.field.body.CTRL_REG4.Xen_G          = 1;
  transaction.field.body.CTRL_REG4.zero0          = 0;
  transaction.field.body.CTRL_REG4.LIR_XL1        = 0;
  transaction.field.body.CTRL_REG4.FOURD_XL1      = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG4), chip_select_AG_pin, gpio, spi );
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG5_XL;
  transaction.field.body.CTRL_REG5_XL.DEC         = 0;
  transaction.field.body.CTRL_REG5_XL.Zen_XL      = 1;
  transaction.field.body.CTRL_REG5_XL.Yen_XL      = 1;
  transaction.field.body.CTRL_REG5_XL.Xen_XL      = 1;
  transaction.field.body.CTRL_REG5_XL.zero        = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG5_XL), chip_select_AG_pin, gpio, spi );
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG6_XL;
  transaction.field.body.CTRL_REG6_XL.ODR_XL      = 3;  // see table 68
  transaction.field.body.CTRL_REG6_XL.FS_XL       = 0;
  transaction.field.body.CTRL_REG6_XL.BW_SCAL_ODR = 0;
  transaction.field.body.CTRL_REG6_XL.BW_XL       = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG6_XL), chip_select_AG_pin, gpio, spi );
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG7_XL;
  transaction.field.body.CTRL_REG7_XL.HR          = 0;
  transaction.field.body.CTRL_REG7_XL.DCF         = 0;
  transaction.field.body.CTRL_REG7_XL.zero1       = 0;
  transaction.field.body.CTRL_REG7_XL.FDS         = 0;
  transaction.field.body.CTRL_REG7_XL.zero0       = 0;
  transaction.field.body.CTRL_REG7_XL.HPIS1       = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG7_XL), chip_select_AG_pin, gpio, spi );
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG8;
  transaction.field.body.CTRL_REG8.BOOT           = 0;
  transaction.field.body.CTRL_REG8.BDU            = 0;
  transaction.field.body.CTRL_REG8.H_LACTIVE      = 0;
  transaction.field.body.CTRL_REG8.PP_OD          = 0;
  transaction.field.body.CTRL_REG8.SIM            = 0;
  transaction.field.body.CTRL_REG8.IF_ADD_INC     = 1;  /* you still have to set the M_S bit in the command */
  transaction.field.body.CTRL_REG8.BLE            = 0;  /* ARM processors default to Little Endian */
  transaction.field.body.CTRL_REG8.SW_RESET       = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG8), chip_select_AG_pin, gpio, spi );
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG9;
  transaction.field.body.CTRL_REG9.zero1          = 0;
  transaction.field.body.CTRL_REG9.SLEEP_G        = 0;
  transaction.field.body.CTRL_REG9.zero0          = 0;
  transaction.field.body.CTRL_REG9.FIFO_TEMP_EN   = 0;
  transaction.field.body.CTRL_REG9.DRDY_mask_bit  = 0;
  transaction.field.body.CTRL_REG9.I2C_DISABLE    = 1;
  transaction.field.body.CTRL_REG9.FIFO_EN        = 0;
  transaction.field.body.CTRL_REG9.STOP_ON_FTH    = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG9), chip_select_AG_pin, gpio, spi );
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG10;
  transaction.field.body.CTRL_REG10.zero1         = 0;
  transaction.field.body.CTRL_REG10.ST_G          = 0;
  transaction.field.body.CTRL_REG10.zero0         = 0;
  transaction.field.body.CTRL_REG10.ST_XL         = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG10), chip_select_AG_pin, gpio, spi );

  return;
}

void initialize_magnetometer(
    volatile struct spi_register *spi,
    volatile struct gpio_register*gpio,
    int                           chip_select_M_pin )
{
  union  LSM9DS1_transaction  transaction;

  /* print WHO_AM_I */
  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_WHO_AM_I_M;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.WHO_AM_I_M), chip_select_M_pin, gpio, spi );
  printf( "WHOAMI_M (0x3D) = 0x%2.2X\n", transaction.field.body.WHO_AM_I_M.THREE_D);

  /*
   * in CTRL_REG1_M, no temperature compensation, enable X/Y, run in medium performance at 10Hz
   * in CTRL_REG2_M, use 4 gauss scale
   * in CTRL_REG3_M, run in continuous conversion, disable I2C, disable low-power (leave SIM default)
   * in CTRL_REG4_M, enable Z with Little Endian (just in case)
   * in CTRL_REG5_M, enable block data updates (race conditions are bad)
   */
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG1_M;
  transaction.field.body.CTRL_REG1_M.TEMP_COMP    = 0;
  transaction.field.body.CTRL_REG1_M.OM           = 3;  // see table 110
  transaction.field.body.CTRL_REG1_M.DO           = 7;  // see table 111
  transaction.field.body.CTRL_REG1_M.ST           = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG1_M), chip_select_M_pin, gpio, spi );
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG2_M;
  transaction.field.body.CTRL_REG2_M.zero2        = 0;
  transaction.field.body.CTRL_REG2_M.FS           = 0;
  transaction.field.body.CTRL_REG2_M.zero1        = 0;
  transaction.field.body.CTRL_REG2_M.REBOOT       = 0;
  transaction.field.body.CTRL_REG2_M.SOFT_RST     = 0;
  transaction.field.body.CTRL_REG2_M.zero0        = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG2_M), chip_select_M_pin, gpio, spi );
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG3_M;
  transaction.field.body.CTRL_REG3_M.I2C_DISABLE  = 1;
  transaction.field.body.CTRL_REG3_M.zero1        = 0;
  transaction.field.body.CTRL_REG3_M.LP           = 0;
  transaction.field.body.CTRL_REG3_M.zero0        = 0;
  transaction.field.body.CTRL_REG3_M.SIM          = 0;
  transaction.field.body.CTRL_REG3_M.MD           = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG3_M), chip_select_M_pin, gpio, spi );
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG4_M;
  transaction.field.body.CTRL_REG4_M.zero1        = 0;
  transaction.field.body.CTRL_REG4_M.OMZ          = 3;  // see table 120
  transaction.field.body.CTRL_REG4_M.BLE          = 0;
  transaction.field.body.CTRL_REG4_M.zero0        = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG4_M), chip_select_M_pin, gpio, spi );
  transaction.field.command.READ                  = 0;
  transaction.field.command.M_S                   = 0;
  transaction.field.command.AD                    = LSM9DS1_REGISTER_CTRL_REG5_M;
  transaction.field.body.CTRL_REG5_M.zero1        = 0;
  transaction.field.body.CTRL_REG5_M.BDU          = 1;
  transaction.field.body.CTRL_REG5_M.zero0        = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.CTRL_REG5_M), chip_select_M_pin, gpio, spi );

  return;
}

void read_accelerometer(
    volatile struct spi_register *spi,
    volatile struct gpio_register*gpio,
    int                           chip_select_AG_pin,
    struct LSM9DS1_reading_t *    LSM9DS1_reading )
{
  union  LSM9DS1_transaction  transaction;
  union uint16_to_2uint8      OUT_XL_X;
  union uint16_to_2uint8      OUT_XL_Y;
  union uint16_to_2uint8      OUT_XL_Z;

  /*
   * poll the status register and it tells you when it is done
   * Once it is done, read the data registers and the next conversion starts
   */
  do
  {
    usleep( 100 );  /* sleeping a little too long should not hurt */

    transaction.field.command.READ  = 1;
    transaction.field.command.M_S   = 0;
    transaction.field.command.AD    = LSM9DS1_REGISTER_STATUS_REG;
    transaction.value[1]            = 0;
    transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.STATUS_REG), chip_select_AG_pin, gpio, spi );
  } while (transaction.field.body.STATUS_REG.XLDA == 0);

  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_X_L_XL;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_X_L_XL), chip_select_AG_pin, gpio, spi );
  OUT_XL_X.field.L                = transaction.value[1];
  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_X_H_XL;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_X_H_XL), chip_select_AG_pin, gpio, spi );
  OUT_XL_X.field.H                = transaction.value[1];

  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Y_L_XL;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Y_L_XL), chip_select_AG_pin, gpio, spi );
  OUT_XL_Y.field.L                = transaction.value[1];
  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Y_H_XL;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Y_H_XL), chip_select_AG_pin, gpio, spi );
  OUT_XL_Y.field.H                = transaction.value[1];

  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Z_L_XL;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Z_L_XL), chip_select_AG_pin, gpio, spi );
  OUT_XL_Z.field.L                = transaction.value[1];
  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Z_H_XL;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Z_H_XL), chip_select_AG_pin, gpio, spi );
  OUT_XL_Z.field.H                = transaction.value[1];

  LSM9DS1_reading->X = OUT_XL_X.signed_value*2.0/32768.0*9.80665;  /* 2g range, 16-bit signed fixed-point */
  LSM9DS1_reading->Y = OUT_XL_Y.signed_value*2.0/32768.0*9.80665;
  LSM9DS1_reading->Z = OUT_XL_Z.signed_value*2.0/32768.0*9.80665;

  return;
}

void read_gyroscope(
    volatile struct spi_register *spi,
    volatile struct gpio_register*gpio,
    int                           chip_select_AG_pin,
    struct LSM9DS1_reading_t *    LSM9DS1_reading )
{
  union  LSM9DS1_transaction  transaction;
  union uint16_to_2uint8      OUT_X_G;
  union uint16_to_2uint8      OUT_Y_G;
  union uint16_to_2uint8      OUT_Z_G;

  /*
   * poll the status register and it tells you when it is done
   * Once it is done, read the data registers and the next conversion starts
   */
  do
  {
    usleep( 100 );  /* sleeping a little too long should not hurt */

    transaction.field.command.READ  = 1;
    transaction.field.command.M_S   = 0;
    transaction.field.command.AD    = LSM9DS1_REGISTER_STATUS_REG;
    transaction.value[1]            = 0;
    transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.STATUS_REG), chip_select_AG_pin, gpio, spi );
  } while (transaction.field.body.STATUS_REG.GDA == 0);

  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_X_L_G;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_X_L_XL), chip_select_AG_pin, gpio, spi );
  OUT_X_G.field.L                = transaction.value[1];
  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_X_H_G;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_X_H_XL), chip_select_AG_pin, gpio, spi );
  OUT_X_G.field.H                = transaction.value[1];

  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Y_L_G;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Y_L_XL), chip_select_AG_pin, gpio, spi );
  OUT_Y_G.field.L                = transaction.value[1];
  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Y_H_G;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Y_H_XL), chip_select_AG_pin, gpio, spi );
  OUT_Y_G.field.H                = transaction.value[1];

  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Z_L_G;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Z_L_XL), chip_select_AG_pin, gpio, spi );
  OUT_Z_G.field.L                = transaction.value[1];
  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Z_H_G;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Z_H_XL), chip_select_AG_pin, gpio, spi );
  OUT_Z_G.field.H                = transaction.value[1];

  LSM9DS1_reading->X = OUT_X_G.signed_value*245.0/32768.0;  /* 245dps range, 16-bit signed fixed-point */
  LSM9DS1_reading->Y = OUT_Y_G.signed_value*245.0/32768.0;
  LSM9DS1_reading->Z = OUT_Z_G.signed_value*245.0/32768.0;

  return;
}

void read_magnetometer(
    volatile struct spi_register *spi,
    volatile struct gpio_register*gpio,
    int                           chip_select_M_pin,
    struct LSM9DS1_reading_t *    LSM9DS1_reading )
{
  union  LSM9DS1_transaction  transaction;
  union uint16_to_2uint8      OUT_X_M;
  union uint16_to_2uint8      OUT_Y_M;
  union uint16_to_2uint8      OUT_Z_M;

  /*
   * poll the status register and it tells you when it is done
   * Once it is done, read the data registers and the next conversion starts
   */
  do
  {
    usleep( 100 );  /* sleeping a little too long should not hurt */

    transaction.field.command.READ  = 1;
    transaction.field.command.M_S   = 0;
    transaction.field.command.AD    = LSM9DS1_REGISTER_STATUS_REG_M;
    transaction.value[1]            = 0;
    transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.STATUS_REG_M), chip_select_M_pin, gpio, spi );
  } while (transaction.field.body.STATUS_REG_M.ZYXDA == 0);

  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_X_L_M;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_X_L_M), chip_select_M_pin, gpio, spi );
  OUT_X_M.field.L                = transaction.value[1];
  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_X_H_M;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_X_H_M), chip_select_M_pin, gpio, spi );
  OUT_X_M.field.H                = transaction.value[1];

  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Y_L_M;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Y_L_M), chip_select_M_pin, gpio, spi );
  OUT_Y_M.field.L                = transaction.value[1];
  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Y_H_M;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Y_H_M), chip_select_M_pin, gpio, spi );
  OUT_Y_M.field.H                = transaction.value[1];

  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Z_L_M;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Z_L_M), chip_select_M_pin, gpio, spi );
  OUT_Z_M.field.L                = transaction.value[1];
  transaction.field.command.READ  = 1;
  transaction.field.command.M_S   = 0;
  transaction.field.command.AD    = LSM9DS1_REGISTER_OUT_Z_H_M;
  transaction.value[1]            = 0;
  transact_SPI( transaction.value, transaction.value, sizeof(transaction.field.command)+sizeof(transaction.field.body.OUT_Z_H_M), chip_select_M_pin, gpio, spi );
  OUT_Z_M.field.H                = transaction.value[1];

  LSM9DS1_reading->X = OUT_X_M.signed_value*4.0/32768.0;  /* 4 gauss range, 16-bit signed fixed-point */
  LSM9DS1_reading->Y = OUT_Y_M.signed_value*4.0/32768.0;
  LSM9DS1_reading->Z = OUT_Z_M.signed_value*4.0/32768.0;

  return;
}
