/* =============================================================================
 * TWI_UART_GPIO.c  –  Hardware drivers: UART, GPIO, timers, ADC, TWI/I2C.
 * =============================================================================*/
#include "TWI_UART_GPIO.h"

/* ============================== UART ============================== */
void uart_init(uint16_t ubrr) {
    UCSR0A = (1 << U2X0);
    UBRR0H = (ubrr >> 8);
    UBRR0L = ubrr;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}
void uart_tx(char c) { while (!(UCSR0A & (1 << UDRE0))); UDR0 = c; }
void uart_print(const char *s) { for (; *s; s++) uart_tx(*s); }
void uart_print_int(int val) {
    char buf[12]; int i = 0, is_neg = 0;
    if (val == 0) { buf[i++] = '0'; }
    else {
        if (val < 0) { is_neg = 1; val = -val; }
        while (val > 0) { buf[i++] = (val % 10) + '0'; val /= 10; }
        if (is_neg) buf[i++] = '-';
    }
    buf[i] = '\0';
    for (int j = 0; j < i/2; j++) { char t = buf[j]; buf[j] = buf[i-j-1]; buf[i-j-1] = t; }
    uart_print(buf);
}

/* ============================== GPIO ============================== */
void gpio_init(void) {
    DDRB |= (1 << PB1) | (1 << PB3) | (1 << PB5);  /* servo + US triggers */
    DDRD &= ~((1 << PD2) | (1 << PD3));              /* US echo inputs      */
}

/* ============================== Timers ============================== */
volatile uint8_t  loop_flag    = 0;
volatile uint32_t system_ticks = 0;

void timer1_init(void) {
    DDRB |= (1 << PB1);
    ICR1   = 39999;
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13)  | (1 << WGM12) | (1 << CS11);
    TIMSK1 |= (1 << TOIE1);
    sei();
}
ISR(TIMER1_OVF_vect) { loop_flag = 1; system_ticks++; }

void timer2_init(void) {
    TCCR2A = 0;
    TCCR2B = (1 << CS22) | (1 << CS20);
    TCNT2  = 0;
}
void delay_ms(uint16_t ms) { while (ms--) _delay_ms(1); }

/* ============================== ADC ============================== */
volatile uint8_t  adc_ready = 0;
volatile uint16_t adc_value = 0;

void adc_init(void) {
    ADMUX  = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    sei();
}
void adc_start(uint8_t channel) {
    adc_ready = 0;
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC);
}
ISR(ADC_vect) { adc_value = ADC; adc_ready = 1; }
uint16_t adc_get(void) { return adc_value; }

/* ============================== TWI ============================== */
volatile struct { uint8_t TWI_ACK:1; } flags;
volatile uint8_t TWI_status, TWI_byte;

void twi_init(void) {
    TWSR &= ~((1<<TWPS0)|(1<<TWPS1)); TWBR = 72; TWCR = (1<<TWEN);
}
uint8_t TWI_start(uint8_t addr, uint8_t rw) {
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
    if (((TWSR&0xF8)!=TW_START) && ((TWSR&0xF8)!=TW_REP_START)) return 1;
    TWDR = (addr<<1)|(rw&1); TWCR = (1<<TWINT)|(1<<TWEN);
    while (!(TWCR & (1<<TWINT)));
    if (((TWSR&0xF8)!=TW_MT_SLA_ACK) && ((TWSR&0xF8)!=TW_MR_SLA_ACK)) return 2;
    return 0;
}
void TWI_stop(void) { TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTO); while(TWCR&(1<<TWSTO)); }
uint8_t TWI_write(uint8_t d) {
    TWDR=d; TWCR=(1<<TWINT)|(1<<TWEN); while(!(TWCR&(1<<TWINT)));
    if ((TWSR&0xF8)!=TW_MT_DATA_ACK) return 1; return 0;
}
uint8_t TWI_ack_read(void) {
    TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWEA); while(!(TWCR&(1<<TWINT)));
    flags.TWI_ACK=1; if((TWSR&0xF8)!=TW_MR_DATA_ACK) flags.TWI_ACK=0; return TWDR;
}
uint8_t TWI_nack_read(void) {
    TWCR=(1<<TWINT)|(1<<TWEN); while(!(TWCR&(1<<TWINT)));
    flags.TWI_ACK=1; if((TWSR&0xF8)!=TW_MR_DATA_NACK){flags.TWI_ACK=0;return 0;} return TWDR;
}
uint8_t Read_Reg(uint8_t a, uint8_t r) {
    TWI_status=TWI_start(a,TW_WRITE); if(TWI_status) return 1;
    TWI_status=TWI_write(r);          if(TWI_status) return 2;
    TWI_status=TWI_start(a,TW_READ);  if(TWI_status) return 3;
    TWI_byte=TWI_nack_read(); TWI_stop();
    if(!flags.TWI_ACK) return 4; return 0;
}
uint8_t Read_Reg_N(uint8_t a, uint8_t r, uint8_t n, int16_t *d) {
    uint8_t *p=(uint8_t*)d;
    TWI_status=TWI_start(a,TW_WRITE); if(TWI_status) return 1;
    TWI_status=TWI_write(r);          if(TWI_status) return 2;
    TWI_status=TWI_start(a,TW_READ);  if(TWI_status) return 3;
    for (uint8_t i=0;i<n-1;i++) { *p=TWI_ack_read(); if(!flags.TWI_ACK) return 5; p++; }
    *p=TWI_nack_read(); TWI_stop(); if(!flags.TWI_ACK) return 4; return 0;
}
uint8_t Write_Reg(uint8_t a, uint8_t r, uint8_t v) {
    TWI_status=TWI_start(a,TW_WRITE); if(TWI_status) return 1;
    TWI_status=TWI_write(r);          if(TWI_status) return 2;
    TWI_status=TWI_write(v);          if(TWI_status) return 3;
    TWI_stop(); if(!flags.TWI_ACK) return 4; return 0;
}

/* ============================== Filter ============================== */
static float alpha = 0.3f;
float filter_distance(float newVal, float *state) {
    *state = alpha*newVal + (1.0f-alpha)*(*state);
    return *state;
}