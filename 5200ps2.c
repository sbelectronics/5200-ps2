// 5200controller
//#define F_CPU 14745600

#include <stdio.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <inttypes.h>

#include <util/atomic.h>
#include <util/delay.h>

#define POT0_CS_PIN PB3
#define POT1_CS_PIN PB4
#define PS2_CS_PIN PB5

#define SPI_CLK_PIN  PB2
#define SPI_MISO_PIN  PB1
#define SPI_MOSI_PIN  PB0

#define SPI_MOSI_HIGH PORTB |= (1<<SPI_MOSI_PIN)
#define SPI_MOSI_LOW PORTB &= ~(1<<SPI_MOSI_PIN)
#define SPI_CLK_HIGH PORTB |= (1<<SPI_CLK_PIN)
#define SPI_CLK_LOW PORTB &= ~(1<<SPI_CLK_PIN)
#define POT0_CS_HIGH PORTB |= (1<<POT0_CS_PIN)
#define POT0_CS_LOW PORTB &= ~(1<<POT0_CS_PIN)
#define POT1_CS_HIGH PORTB |= (1<<POT0_CS_PIN)
#define POT1_CS_LOW PORTB &= ~(1<<POT0_CS_PIN)
#define PS2_CS_HIGH PORTB |= (1<<POT0_CS_PIN)
#define PS2_CS_LOW PORTB &= ~(1<<POT0_CS_PIN)

uint8_t mode, rx, ry, lx, ly, buttons0, buttons1;

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

uint8_t ps2Transfer(uint8_t v)
{
    uint8_t i,in_v;
    in_v=0;

    for (i=0; i<8; i++) {
        if (v & 1) {
            SPI_MOSI_HIGH;
        } else {
            SPI_MOSI_LOW;
        }

        SPI_CLK_LOW;

        _delay_us(1);

        if (PORTB & SPI_MISO_PIN) {
            in_v = in_v | 128;
        }

        in_v = in_v >> 1;
        v = v >> 1;

        SPI_CLK_HIGH;
    }

    _delay_us(20);

    return in_v;
}

void ps2_enter_config()
{
    // enter config mode
    PS2_CS_LOW;
    ps2Transfer(0x01);
    ps2Transfer(0x43);
    ps2Transfer(0x00);
    ps2Transfer(0x01);
    ps2Transfer(0x00);
    PS2_CS_HIGH;
}

void ps2_exit_config()
{
    PS2_CS_LOW;
    ps2Transfer(0x01);
    ps2Transfer(0x43);
    ps2Transfer(0x00);
    ps2Transfer(0x00);
    ps2Transfer(0x5A);
    ps2Transfer(0x5A);
    ps2Transfer(0x5A);
    ps2Transfer(0x5A);
    ps2Transfer(0x5A);
    PS2_CS_HIGH;
}

void ps2_config_analog()
{
   PS2_CS_LOW;
   ps2Transfer(0x01);
   ps2Transfer(0x44);
   ps2Transfer(0x00);
   ps2Transfer(0x01);
   ps2Transfer(0x03);
   ps2Transfer(0x00);
   ps2Transfer(0x00);
   ps2Transfer(0x00);
   ps2Transfer(0x00);
   PS2_CS_HIGH;
}

void ps2_poll()
{
    PS2_CS_LOW;
    ps2Transfer(0x01);
    mode = ps2Transfer(0x42);  // 0x73 if analog
    ps2Transfer(0x00);
    buttons0 = ps2Transfer(0x00);
    buttons1 = ps2Transfer(0x00);
    rx = ps2Transfer(0x00);
    ry = ps2Transfer(0x00);
    lx = ps2Transfer(0x00);
    ly = ps2Transfer(0x00);
    PS2_CS_HIGH;
}

void ps2_setup()
{
   mode = 0;

   while (mode != 0x73) {
       ps2_enter_config();
       _delay_ms(1);
       ps2_config_analog();
       _delay_ms(10);
       ps2_exit_config();
       _delay_ms(10);
       ps2_poll(); // this will set mode as a side-effect
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

int main() {
    // configure SPI pins as outputs
    DDRB |= (1<<SPI_MOSI_PIN);
    DDRB |= (1<<SPI_CLK_PIN);
    DDRB |= (1<<POT0_CS_PIN);
    DDRB |= (1<<POT1_CS_PIN);
    DDRB |= (1<<PS2_CS_PIN);

    SPI_CLK_HIGH;

    ps2_setup();
    while (1) {
        ps2_poll();
        setPOT0(0, rx);
        setPOT0(0, ry);
        setPOT1(1, lx);
        setPOT1(1, ly);


    }
    return 0;
}
