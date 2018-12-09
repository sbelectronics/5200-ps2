#include <stdio.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <util/atomic.h>
#include <util/delay.h>

#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif

#define SPI_MOSI_HIGH PORTB |= (1<<spi_mosi_pin)
#define SPI_MOSI_LOW PORTB &= ~(1<<spi_mosi_pin)
#define SPI_CLK_HIGH PORTB |= (1<<spi_clk_pin)
#define SPI_CLK_LOW PORTB &= ~(1<<spi_clk_pin)
#define PS2_CS_HIGH PORTB |= (1<<ps2_cs_pin)
#define PS2_CS_LOW PORTB &= ~(1<<ps2_cs_pin)

uint8_t ps2_cs_pin, spi_clk_pin, spi_mosi_pin, spi_miso_pin;
uint8_t device_mode, rx, ry, lx, ly, buttons0, buttons1, mode, kpd_down, kpd_desired, matrix_mask;

uint8_t ps2Transfer(uint8_t v)
{
    uint8_t i,in_v;
    in_v=0;

    for (i=0; i<8; i++) {
        in_v = in_v >> 1;

        if (v & 1) {
            SPI_MOSI_HIGH;
        } else {
            SPI_MOSI_LOW;
        }

        SPI_CLK_LOW;

        _delay_us(1);

        if (PINB & (1<<spi_miso_pin)) {
            in_v = in_v | 128;
        }

        v = v >> 1;

        SPI_CLK_HIGH;
    }

    _delay_us(20);

    SPI_MOSI_HIGH;

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
    device_mode = ps2Transfer(0x42);  // High nibble is mode (0x70=analog, 0x40=digital). Low nibble is num words.
    ps2Transfer(0x00);
    if ((device_mode & 0xF0) == 0x70) {
        buttons0 = ps2Transfer(0x00);
        buttons1 = ps2Transfer(0x00);
        rx = ps2Transfer(0x00);
        ry = ps2Transfer(0x00);
        lx = ps2Transfer(0x00);
        ly = ps2Transfer(0x00);
    } else {
        // Something unexpected happened. Let's read the bytes from the controller, but don't store
        // them.
        ps2Transfer(0x00);
        ps2Transfer(0x00);
        ps2Transfer(0x00);
        ps2Transfer(0x00);
        ps2Transfer(0x00);
        ps2Transfer(0x00);
    }
    PS2_CS_HIGH;
}

void ps2_setup_once()
{
    ps2_enter_config();
    _delay_ms(1);
    ps2_config_analog();
    _delay_ms(10);
    ps2_exit_config();
    _delay_ms(10);
}

void ps2_setup_controller_old()
{
   device_mode = 0;

   while (device_mode != 0x73) {
       ps2_setup_once();
       ps2_poll(); // this will set device_mode as a side-effect
   }
}

void ps2_init(uint8_t cs_pin, uint8_t mosi_pin, uint8_t miso_pin, uint8_t clk_pin)
{
    ps2_cs_pin = cs_pin;
    spi_mosi_pin = mosi_pin;
    spi_clk_pin = clk_pin;
    spi_miso_pin = miso_pin;

    DDRB |= (1<< spi_mosi_pin);
    DDRB |= (1<< spi_clk_pin);
    DDRB |= (1<< ps2_cs_pin);

    device_mode = 0;
}
