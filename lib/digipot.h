#ifndef digipot_h
#define digipot_h

void setPOT0(uint8_t pot_num, uint8_t pot_val);
void setPOT1(uint8_t pot_num, uint8_t pot_val);

void initPOT0(uint8_t cs_pin, uint8_t mosi_pin, uint8_t clk_pin);
void initPOT1(uint8_t cs_pin);

#endif
