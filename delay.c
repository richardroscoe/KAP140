#include <util/delay.h>

void
delay_ms(uint16_t millis)
{
  while ( millis ) {
    _delay_ms(1);
    millis--;
  }
}
