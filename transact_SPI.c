/*
 * transact_SPI.c
 *
 *  Created on: Feb 24, 2018
 *      Author: steveb
 */

#include <stdint.h>
#include <unistd.h>
#include "gpio.h"
#include "spi.h"
#include "transact_SPI.h"

void transact_SPI(                                /* send/receive SPI data */
    uint8_t const *                 write_data,   /* the data to send to a SPI device */
    uint8_t *                       read_data,    /* the data read from the SPI device */
    size_t                          data_length,  /* the length of data to send/receive */
    int                             CS_pin,       /* the pin to toggle when communicating */
    volatile struct gpio_register * gpio,         /* the GPIO address */
    volatile struct spi_register *  spi )         /* the SPI address */
{
  size_t  write_count;  /* used to index through the bytes to be sent/received */
  size_t  read_count;   /* used to index through the bytes to be sent/received */

  /* clear out anything in the RX FIFO */
  while (spi->CS.field.RXD != 0)
  {
    (void)spi->FIFO;
  }

  /* enable the chip select on the device */
  GPIO_CLR( gpio, CS_pin );
  usleep( 10 );

  /* see section 10.6.1 of the BCM2835 datasheet
   * Note that the loop below is a busy-wait loop and may burn a lot of clock cycles, depending on the amount of data to be transferred
   */
  spi->CS.field.TA = 1;
  write_count = 0;
  read_count  = 0;
  do
  {
    /* transfer bytes to the device */
    while ((write_count != data_length) &&
           (spi->CS.field.TXD != 0))
    {
      spi->FIFO = (uint32_t)*write_data;

      write_data++;
      write_count++;
    }

    /* drain the RX FIFO */
    while ((read_count != data_length) &&
           (spi->CS.field.RXD != 0))
    {
      if (read_data != NULL)
      {
        *read_data = spi->FIFO;

        read_data++;
        read_count++;
      }
      else
      {
        (void)spi->FIFO;

        read_count++;
      }
    }
  } while (spi->CS.field.DONE == 0);
  spi->CS.field.TA = 0;

  /* disable the chip select on the device */
  usleep( 10 );
  GPIO_SET( gpio, CS_pin );

  return;
}
