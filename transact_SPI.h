/*
 * transact_SPI.h
 *
 *  Created on: Feb 24, 2018
 *      Author: steveb
 */

#ifndef TRANSACT_SPI_H_
#define TRANSACT_SPI_H_

void transact_SPI( uint8_t const *write_data, uint8_t *read_data, size_t data_length, int CS_pin, volatile struct gpio_register *gpio, volatile struct spi_register *spi );

#endif /* TRANSACT_SPI_H_ */
