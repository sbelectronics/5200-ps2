// 5200controller
//#define F_CPU 14745600

#include <stdio.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <inttypes.h>

#include <util/atomic.h>
#include <util/delay.h>

#include "../lib/ps2.h"

#ifndef TRUE
#define TRUE 1
#define FALSE 0
#endif

#define POT0_CS_PIN PB0
#define POT1_CS_PIN PB1
#define PS2_CS_PIN PB2

#define SPI_CLK_PIN  PB5
#define SPI_MISO_PIN  PB4
#define SPI_MOSI_PIN  PB3

#define TRIG0_PIN PC0    // primary controller port
#define TRIG1_PIN PC1
#define TRIG01_PIN PC2   // secondary controller port
#define TRIG11_PIN PC3

#define EAST_PIN PC4
#define WEST_PIN PC5
#define NORTH_PIN PB6
#define SOUTH_PIN PB7

//#define KSPR_MC_PIN PB6
#define KSPR_KPD_PIN PD0
#define K321S_PIN PD5
#define K987R_PIN PD7
#define K654P_PIN PD6

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

#define TRIG0_HIGH PORTC |= (1<<TRIG0_PIN)
#define TRIG0_LOW PORTC &= ~(1<<TRIG0_PIN)
#define TRIG1_HIGH PORTC |= (1<<TRIG1_PIN)
#define TRIG1_LOW PORTC &= ~(1<<TRIG1_PIN)
#define TRIG01_HIGH PORTC |= (1<<TRIG01_PIN)
#define TRIG01_LOW PORTC &= ~(1<<TRIG01_PIN)
#define TRIG11_HIGH PORTC |= (1<<TRIG11_PIN)
#define TRIG11_LOW PORTC &= ~(1<<TRIG11_PIN)

#define K321S_FLOAT DDRD &= ~(1<<K321S_PIN)
#define K321S_LOW DDRD |= (1<<K321S_PIN)
#define K987R_FLOAT DDRD &= ~(1<<K987R_PIN)
#define K987R_LOW DDRD |= (1<<K987R_PIN)
#define K654P_FLOAT DDRD &= ~(1<<K654P_PIN)
#define K654P_LOW DDRD |= (1<<K654P_PIN)

#define EAST_PRESSED ((PINC & (1<<EAST_PIN)) == 0)
#define WEST_PRESSED ((PINC & (1<<WEST_PIN)) == 0)
#define NORTH_PRESSED ((PINB & (1<<NORTH_PIN)) == 0)
#define SOUTH_PRESSED ((PINB & (1<<SOUTH_PIN)) == 0)
#define FIRE0_PRESSED ((PINC & (1<<TRIG01_PIN)) == 0)
#define FIRE1_PRESSED ((PINC & (1<<TRIG11_PIN)) == 0)

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

#define BUT_FIRE0 (BUT_L1 | BUT_R1 | BUT_TRIANGLE)
#define BUT_FIRE1 (BUT_L2 | BUT_R2 | BUT_X)

#define KPD_NONE 0
#define KPD_START 1
#define KPD_PAUSE 2
#define KPD_RESET 3

#define V_CENTER 128
#define H_CENTER 128

#define XFUNC(v)  ((v-128)*18/20)+128+2
#define YFUNC(v)  ((v-128)*18/20)+128

#define MODE_AMBIDEXTROUS 0
#define MODE_LEFT 1
#define MODE_RIGHT 2

uint8_t device_mode, rx, ry, lx, ly, buttons0, buttons1, mode, kpd_down, kpd_desired, matrix_mask;

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

ISR(PCINT2_vect)
{
    if ((PIND & (1<<KSPR_KPD_PIN)) == 0) {
        DDRD |= matrix_mask;
    } else {
        DDRD &= ~((1<<K321S_PIN) | (1<<K987R_PIN) | (1<<K654P_PIN));
    }
}

int main() {
    uint8_t deltar, deltal, ambistick, select_pressed, matrix_mask_new, digital_hpot, digital_vpot;

    // outputs
    DDRB |= (1<<SPI_MOSI_PIN);
    DDRB |= (1<<SPI_CLK_PIN);
    DDRB |= (1<<POT0_CS_PIN);
    DDRB |= (1<<POT1_CS_PIN);
    DDRB |= (1<<PS2_CS_PIN);
    DDRC |= (1<<TRIG0_PIN);
    DDRC |= (1<<TRIG1_PIN);
    //DDRC |= (1<<TRIG01_PIN);
    //DDRC |= (1<<TRIG11_PIN);

    // inputs
    DDRB &= ~(1<<SPI_MISO_PIN);
    PORTB |= (1<<SPI_MISO_PIN); // enable pullup

    // inputs -- 2600 joystick
    DDRB &= ~(1<<NORTH_PIN);
    PORTB |= (1<<NORTH_PIN);
    DDRB &= ~(1<<SOUTH_PIN);
    PORTB |= (1<<SOUTH_PIN);
    DDRC &= ~(1<<EAST_PIN);
    PORTC |= (1<<EAST_PIN);
    DDRC &= ~(1<<WEST_PIN);
    PORTC |= (1<<WEST_PIN);
    DDRC |= (1<<TRIG01_PIN);
    PORTC |= (1<<TRIG01_PIN);
    DDRC |= (1<<TRIG11_PIN);
    PORTC |= (1<<TRIG11_PIN);

    DDRD &= ~(1<<KSPR_KPD_PIN);
    PORTD |= (1<<KSPR_KPD_PIN); // enable pullup

    K321S_FLOAT;
    K987R_FLOAT;
    K654P_FLOAT;

    // set all three keypad pins low, these will function as open collector outputs
    PORTD &= ~(1<<K321S_PIN);
    PORTD &= ~(1<<K987R_PIN);
    PORTD &= ~(1<<K654P_PIN);

    // default all CS to high
    POT0_CS_HIGH;
    POT1_CS_HIGH;
    PS2_CS_HIGH;

    // default matrix keypad to no push signal
    //KSPR_MC_LOW;

    // default all triggers to high
    TRIG0_HIGH;
    TRIG1_HIGH;
    TRIG01_HIGH;
    TRIG11_HIGH;

    SPI_CLK_HIGH;

    mode = MODE_AMBIDEXTROUS;
    ambistick = 0;
    matrix_mask = 0;

    // Set default buttons state from the ps2 controller as all unpushed.
    buttons0 = 0xFF;
    buttons1 = 0xFF;

    kpd_down = KPD_NONE;
    PCMSK2 |= (1 << PCINT16);
    PCICR |= (1 << PCIE2);
    sei();

    ps2_init(PS2_CS_PIN, SPI_MOSI_PIN, SPI_MISO_PIN, SPI_CLK_PIN);

    ps2_setup_controller();

    while (1) {
        ps2_poll();

        select_pressed = ((buttons0 & BUT_SELECT) == 0);

        // Start by assuming the pot is in the center
        digital_vpot = V_CENTER;
        digital_hpot = H_CENTER;

        // Check the 2600 Joystick port
        if (NORTH_PRESSED) {
            digital_vpot = 1;
        } else if (SOUTH_PRESSED) {
            digital_vpot = 254;
        }
        if (WEST_PRESSED) {
            digital_hpot = 1;
        } else if (EAST_PRESSED) {
            digital_hpot = 254;
        }

        if ((digital_hpot != H_CENTER) || (digital_vpot != V_CENTER)) {
            setPOT0(0, digital_hpot);
            setPOT0(1, digital_vpot);
        } else {
            // weird note: if the setPOTs don't get called, the triggers will go haywires
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
        }

        matrix_mask_new = 0;

        // Handle select button logic, to change modes if select + some other button is pushed.
        if (select_pressed) {
            if ((buttons1 & BUT_TRIANGLE) == 0) {
                mode = MODE_AMBIDEXTROUS;
            } else if ((buttons1 & BUT_SQUARE) == 0) {
                mode = MODE_LEFT;
            } else if ((buttons1 & BUT_O) == 0) {
                mode = MODE_RIGHT;
            }
            if ((buttons0 & BUT_START) == 0) {
                matrix_mask_new = matrix_mask_new | (1 << K987R_PIN);
            }
            if ((buttons1 & BUT_X) == 0) {
                matrix_mask_new = matrix_mask_new | (1 << K654P_PIN);
            }
        } else {
            if ((buttons0 & BUT_START) == 0) {
                matrix_mask_new = matrix_mask_new | (1 << K321S_PIN);
            }
        }

        // Let the ISR deal with it
        matrix_mask = matrix_mask_new;

        // Trigger logic. BUT_FIRE0 and BUT_FIRE1 each map to several possible fire buttons.

        if ((!select_pressed && ((buttons1 & BUT_FIRE0) != BUT_FIRE0)) || (FIRE0_PRESSED)) {
            TRIG0_LOW;
        } else {
            TRIG0_HIGH;
        }

        if ((!select_pressed && ((buttons1 & BUT_FIRE1) != BUT_FIRE1)) || (FIRE1_PRESSED)) {
            TRIG1_LOW;
        } else {
            TRIG1_HIGH;
        }

    }
    return 0;
}
