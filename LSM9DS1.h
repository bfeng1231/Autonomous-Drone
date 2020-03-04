/*
 * LSM9DS1.h
 *
 *  Created on: Feb 20, 2017
 *      Author: steveb
 */

#ifndef LSM9DS1_H_
#define LSM9DS1_H_

struct LSM9DS1_reading_t
{
  float X;
  float Y;
  float Z;
};

void initialize_accelerometer_and_gyroscope(
    volatile struct spi_register *spi,
    volatile struct gpio_register*gpio,
    int                           chip_select_AG_pin );
void initialize_magnetometer(
    volatile struct spi_register *spi,
    volatile struct gpio_register*gpio,
    int                           chip_select_M_pin );
void read_accelerometer(
    volatile struct spi_register *spi,
    volatile struct gpio_register*gpio,
    int                           chip_select_AG_pin,
    struct LSM9DS1_reading_t *    LSM9DS1_reading );
void read_gyroscope(
    volatile struct spi_register *spi,
    volatile struct gpio_register*gpio,
    int                           chip_select_AG_pin,
    struct LSM9DS1_reading_t *    LSM9DS1_reading );
void read_magnetometer(
    volatile struct spi_register *spi,
    volatile struct gpio_register*gpio,
    int                           chip_select_M_pin,
    struct LSM9DS1_reading_t *    LSM9DS1_reading );

#endif /* LSM9DS1_H_ */
