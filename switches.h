#ifndef SWITCHES_H_
#define SWITCHES_H_

void switches_init(void);
void switches_tick(void);

extern volatile uint8_t sw_porta;
extern volatile uint8_t sw_portb;
extern volatile uint8_t sw_porta_state;
extern volatile uint8_t sw_portb_state;
extern volatile int8_t sw_enc_delta;

#endif
