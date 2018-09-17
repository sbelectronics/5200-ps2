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

#define TRIG0_PIN PA0
#define TRIG1_PIN PA1

#define SPI_MOSI_HIGH PORTB |= (1<<SPI_MOSI_PIN)
#define SPI_MOSI_LOW PORTB &= ~(1<<SPI_MOSI_PIN)
#define SPI_CLK_HIGH PORTB |= (1<<SPI_CLK_PIN)
#define SPI_CLK_LOW PORTB &= ~(1<<SPI_CLK_PIN)
#define POT0_CS_HIGH PORTB |= (1<<POT0_CS_PIN)
#define POT0_CS_LOW PORTB &= ~(1<<POT0_CS_PIN)
#define POT1_CS_HIGH PORTB |= (1<<POT1_CS_PIN)
#define POT1_CS_LOW PORTB &= ~(1<<POT1_CS_PIN)
#define PS2_CS_HIGH PORTB |= (1<<PS2_CS_PIN)
#define PS2_CS_LOW PORTB &= ~(1<<PS2_CS_PIN)

#define TRIG0_HIGH PORTA |= (1<<TRIG0_PIN)
#define TRIG0_LOW PORTA &= ~(1<<TRIG0_PIN)
#define TRIG1_HIGH PORTA |= (1<<TRIG1_PIN)
#define TRIG1_LOW PORTA &= ~(1<<TRIG1_PIN)


#define BUT_SELECT 0x01
#define BUT_L3 0x02
#define BUT_R3 0x04
#define BUT_START 0x08
#define BUT_UP 0x10
#define BUT_RIGHT 0x20
#define BUT_DOWN 0x40
#define BUT_LEFT 0x80

#define BUT_L2 0x01
#define BUT_R2 0x02
#define BUT_L1 0x04
#define BUT_R1 0x08
#define BUT_TRIANGLE 0x10
#define BUT_O 0x20
#define BUT_X 0x40
#define BUT_SQUARE 0x80

#define BUT_FIRE0 (BUT_L1 | BUT_R1)
#define BUT_FIRE1 (BUT_L2 | BUT_R2)

#define XFUNC(v)  ((v-128)*17/20)+128-4
#define YFUNC(v)  ((v-128)*18/20)+128

#define MODE_AMBIDEXTROUS 0
#define MODE_LEFT 1
#define MODE_RIGHT 2

uint8_t device_mode, rx, ry, lx, ly, buttons0, buttons1, mode;

void setPOT0(uint8_t pot_num, uint8_t pot_val);
void setPOT1(uint8_t pot_num, uint8_t pot_val);

#define absminus(a,b) (a>=b) ? (a-b) : (b-a)

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
        in_v = in_v >> 1;

        if (v & 1) {
            SPI_MOSI_HIGH;
        } else {
            SPI_MOSI_LOW;
        }

        SPI_CLK_LOW;

        _delay_us(1);

        if (PINB & (1<<SPI_MISO_PIN)) {
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
    device_mode = ps2Transfer(0x42);  // 0x73 if analog
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
   device_mode = 0;

   while (device_mode != 0x73) {
       setPOT0(1, 50);
       ps2_enter_config();
       _delay_ms(1);
       ps2_config_analog();
       _delay_ms(10);
       ps2_exit_config();
       _delay_ms(10);
       ps2_poll(); // this will set device_mode as a side-effect
       setPOT0(1, 80);
   }

   setPOT0(1, 150);
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

uint8_t max(uint8_t x, uint8_t y)
{
    if (x>y) {
        return x;
    } else {
        return y;
    }
}

int main() {
    uint8_t deltar, deltal, ambistick;

    // outputs
    DDRB |= (1<<SPI_MOSI_PIN);
    DDRB |= (1<<SPI_CLK_PIN);
    DDRB |= (1<<POT0_CS_PIN);
    DDRB |= (1<<POT1_CS_PIN);
    DDRB |= (1<<PS2_CS_PIN);
    DDRA |= (1<<TRIG0_PIN);
    DDRA |= (1<<TRIG1_PIN);

    // inputs
    DDRB &= ~(1<<SPI_MISO_PIN);
    PORTB |= (1<<SPI_MISO_PIN); // enable pullup

    // default all CS to high
    POT0_CS_HIGH;
    POT1_CS_HIGH;
    PS2_CS_HIGH;

    SPI_CLK_HIGH;

    mode = MODE_AMBIDEXTROUS;
    ambistick = 0;

    ps2_setup();

    while (1) {
        ps2_poll();

        // weird note: if the setPOTs don't get called, the triggers will go haywire

        switch (mode) {
            case MODE_AMBIDEXTROUS:
                deltar = max(absminus(rx,128), absminus(ry,128));
                deltal = max(absminus(lx,128), absminus(ly,128));
                if (deltar>64) {
                    ambistick = 0;
                } else if (deltal>64) {
                    ambistick = 1;
                }
                if (ambistick == 0) {
                    setPOT0(0, XFUNC(rx));
                    setPOT0(1, YFUNC(ry));
                    setPOT1(0, XFUNC(rx));
                    setPOT1(1, YFUNC(ry));
                } else {
                    setPOT0(0, XFUNC(lx));
                    setPOT0(1, YFUNC(ly));
                    setPOT1(0, XFUNC(lx));
                    setPOT1(1, YFUNC(ly));
                }
                break;

            case MODE_RIGHT:
                setPOT0(0, XFUNC(rx));
                setPOT0(1, YFUNC(ry));
                setPOT1(0, XFUNC(lx));
                setPOT1(1, XFUNC(ly));
                break;

            case MODE_LEFT:
                setPOT0(0, XFUNC(lx));
                setPOT0(1, YFUNC(ly));
                setPOT1(0, XFUNC(rx));
                setPOT1(1, YFUNC(ry));
                break;
        }

        if ((buttons0 & BUT_SELECT) == 0) {
            if ((buttons1 & BUT_TRIANGLE) == 0) {
                mode = MODE_AMBIDEXTROUS;
            } else if ((buttons1 & BUT_SQUARE) == 0) {
                mode = MODE_LEFT;
            } else if ((buttons1 & BUT_O) == 0) {
                mode = MODE_RIGHT;
            }
        }

        if ((buttons1 & BUT_FIRE0) != BUT_FIRE0) {
            TRIG0_LOW;
        } else {
            TRIG0_HIGH;
        }

        if ((buttons1 & BUT_FIRE1) != BUT_FIRE1) {
            TRIG1_LOW;
        } else {
            TRIG1_HIGH;
        }

    }
    return 0;
}
