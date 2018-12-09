/*
 * Digital Potentiometer Library
 * Scott Baker, http://www.smbaker.com
 */

#include <stdio.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <inttypes.h>

#include <util/atomic.h>
#include <util/delay.h>

#include "digipot.h"

#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif

#define SPI_MOSI_HIGH PORTB |= (1<<spi_mosi_pin)
#define SPI_MOSI_LOW PORTB &= ~(1<<spi_mosi_pin)
#define SPI_CLK_HIGH PORTB |= (1<<spi_clk_pin)
#define SPI_CLK_LOW PORTB &= ~(1<<spi_clk_pin)
#define POT0_CS_HIGH PORTB |= (1<<pot0_cs_pin)
#define POT0_CS_LOW PORTB &= ~(1<<pot0_cs_pin)
#define POT1_CS_HIGH PORTB |= (1<<pot1_cs_pin)
#define POT1_CS_LOW PORTB &= ~(1<<pot1_cs_pin)

uint8_t spi_mosi_pin, spi_clk_pin, pot0_cs_pin, pot1_cs_pin;

void setPOT0(uint8_t pot_num, uint8_t pot_val);
void setPOT1(uint8_t pot_num, uint8_t pot_val);

void initPOT0(uint8_t cs_pin, uint8_t mosi_pin, uint8_t clk_pin)
{
    pot0_cs_pin = cs_pin;
    spi_mosi_pin = mosi_pin;
    spi_clk_pin = clk_pin;

    DDRB |= (1<< spi_mosi_pin);
    DDRB |= (1<< spi_clk_pin);
    DDRB |= (1<< pot0_cs_pin);

    POT0_CS_HIGH;
    SPI_CLK_HIGH;
}

void initPOT1(uint8_t cs_pin)
{
    pot1_cs_pin = cs_pin;

    DDRB |= (1<< pot1_cs_pin);

    POT1_CS_HIGH;
}

void writeSPI(uint8_t v)
{
    uint8_t i;
    for (i=0; i<8; i++) {
        if (v & 128) {
            SPI_MOSI_HIGH;
        } else {
            SPI_MOSI_LOW;
        }

        SPI_CLK_LOW;
        _delay_us(1);

        v = v << 1;

        SPI_CLK_HIGH;
    }
}

void setPOT0(uint8_t pot_num, uint8_t pot_val)
{
    POT0_CS_LOW;
    writeSPI(0x11 + pot_num);
    writeSPI(pot_val);
    POT0_CS_HIGH;
}

void setPOT1(uint8_t pot_num, uint8_t pot_val)
{
    POT1_CS_LOW;
    writeSPI(0x11 + pot_num);
    writeSPI(pot_val);
    POT1_CS_HIGH;
}

