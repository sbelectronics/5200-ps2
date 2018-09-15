// 5200controller
//#define F_CPU 14745600

#include <stdio.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <inttypes.h>

#include <util/atomic.h>
#include <util/delay.h>

#define POT_CLK_PIN  PB2
#define POT_CS_PIN PB1
#define POT_DATA_PIN  PB0

#define SPI_MOSI_HIGH PORTB |= (1<<POT_DATA_PIN)
#define SPI_MOSI_LOW PORTB &= ~(1<<POT_DATA_PIN)
#define SPI_CLK_HIGH PORTB |= (1<<POT_CLK_PIN)
#define SPI_CLK_LOW PORTB &= ~(1<<POT_CLK_PIN)
#define SPI_CS_HIGH PORTB |= (1<<POT_CS_PIN)
#define SPI_CS_LOW PORTB &= ~(1<<POT_CS_PIN)

void writeSPI(uint8_t v)
{
    uint8_t i;
    for (i=0; i<8; i++) {
        if (v & 128) {
            SPI_MOSI_HIGH;
        } else {
            SPI_MOSI_LOW;
        }

        SPI_CLK_HIGH;
        SPI_CLK_LOW;

        v = v << 1;
    }
}

void setPOT(uint8_t pot_num, uint8_t pot_val)
{
    SPI_CS_LOW;
    writeSPI(0x11 + pot_num);
    writeSPI(pot_val);
    //shiftOut(POT_DATA_PIN, POT_CLOCK_PIN, MSBFIRST, 0x11 + pot_num);
    //shiftOut(POT_DATA_PIN, POT_CLOCK_PIN, MSBFIRST, pot_val);
    SPI_CS_HIGH;
}

int main() {
    // configure SPI pins as outputs
    DDRB |= (1<<POT_DATA_PIN);
    DDRB |= (1<<POT_CLK_PIN);
    DDRB |= (1<<POT_CS_PIN);

    SPI_CLK_LOW;

    while (1) {
    }
    return 0;
}
