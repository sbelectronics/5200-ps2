#ifndef PS2_H
#define PS2_H

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

#define PS2_GOOD ((device_mode == 0x73) || (device_mode == 0x79))

extern uint8_t device_mode, rx, ry, lx, ly, buttons0, buttons1;

void ps2_init(uint8_t cs_pin, uint8_t mosi_pin, uint8_t miso_pin, uint8_t clk_pin);
void ps2_setup_controller();
void ps2_setup_once();
void ps2_poll();

#endif