/* =============================================================================
 * US_RIGHT.c  –  Right ultrasonic (HC-SR04), timed off Timer2.
 * TRIG=PB5  ECHO=PD3
 * Not called by the current navigation logic (front IR only) -- kept in
 * case a later version brings the side ultrasonics back in.
 * =============================================================================*/
#include "TWI_UART_GPIO.h"

#define TRIG_PIN       PB5
#define ECHO_PIN       PD3
#define MAX_DISTANCE   400u
#define TIMER2_TICK_US 8u

static uint16_t us_right_read_cm(void) {
    uint32_t timeout;
    uint16_t count;
    uint16_t overflow = 0;

    PORTB &= ~(1 << TRIG_PIN);
    _delay_us(2);
    PORTB |= (1 << TRIG_PIN);
    _delay_us(10);
    PORTB &= ~(1 << TRIG_PIN);

    timeout = 8000;
    while (!(PIND & (1 << ECHO_PIN))) { if (--timeout == 0) return MAX_DISTANCE; }

    TCNT2 = 0;
    TIFR2 |= (1 << TOV2);

    timeout = 8000;
    while (PIND & (1 << ECHO_PIN)) {
        if (TIFR2 & (1 << TOV2)) { TIFR2 |= (1 << TOV2); overflow++; }
        if (--timeout == 0) return MAX_DISTANCE;
    }

    count = TCNT2 + (overflow * 256u);
    return (count * TIMER2_TICK_US) / 58u;
}

static float   rightFilter   = 0.0f;
static uint8_t rightFirstRun = 1;

uint16_t getRightDistance(void) {
    uint16_t rawDistance = us_right_read_cm();
    if (rightFirstRun) { rightFilter = rawDistance; rightFirstRun = 0; }
    if (abs((int)rawDistance - (int)rightFilter) > 100) rawDistance = (uint16_t)rightFilter;
    return (uint16_t)filter_distance(rawDistance, &rightFilter);
}