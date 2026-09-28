/* =============================================================================
 * IR_TOP.c  –  GP2Y0A21YK IR sensor: raw ADC -> distance in cm.
 * Mounted at the front for obstacle detection.
 * =============================================================================*/
#include "TWI_UART_GPIO.h"

#define IR_ADC_MAX 440
#define IR_ADC_MIN  40
#define IR_CM_MIN   10
#define IR_CM_MAX   80

uint16_t gp2y0a21yk_read_cm(uint16_t adc_value) {
    if (adc_value > IR_ADC_MAX) adc_value = IR_ADC_MAX;
    if (adc_value < IR_ADC_MIN) adc_value = IR_ADC_MIN;
    float distance = 5500.0f / (adc_value - 20.0f);
    if (distance < IR_CM_MIN) distance = IR_CM_MIN;
    if (distance > IR_CM_MAX) distance = IR_CM_MAX;
    return (uint16_t)distance;
}