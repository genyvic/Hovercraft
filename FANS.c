/* =============================================================================
 * FANS.c  –  Fan PWM control, both on Timer0 fast-PWM.
 * PD6 = Lifting fan (OC0A)   PD5 = Thrust fan (OC0B)
 * =============================================================================*/
#include "TWI_UART_GPIO.h"

void init_fan_system(void) {
    DDRD |= (1 << DDD6) | (1 << DDD5);
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS00);
}
void lift_fan_control(uint8_t speed)   { OCR0A = speed; }
void lift_fan_stop(void)               { OCR0A = 0;     }
void thrust_fan_control(uint8_t speed) { OCR0B = speed; }
void thrust_fan_off(void)              { OCR0B = 0;     }