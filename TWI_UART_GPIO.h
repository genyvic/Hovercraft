#ifndef TWI_UART_GPIO_H
#define TWI_UART_GPIO_H

/* =============================================================================
 * TWI_UART_GPIO.h  –  Shared header for the low-level hardware drivers:
 * UART (debug logging), GPIO, timers, ADC, and TWI/I2C (talks to the MPU6050).
 * =============================================================================*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/twi.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

/* ---------- UART ---------- */
void uart_init(uint16_t ubrr);
void uart_tx(char c);
void uart_print(const char *s);
void uart_print_int(int val);

/* ---------- GPIO ---------- */
void gpio_init(void);

/* ---------- Timers ---------- */
void timer1_init(void);   /* 50 Hz loop tick + servo PWM */
void timer2_init(void);   /* ultrasonic timing           */
void delay_ms(uint16_t ms);

extern volatile uint8_t  loop_flag;      /* set by Timer1 OVF ISR (~50 Hz) */
extern volatile uint32_t system_ticks;   /* incremented every 20 ms        */

/* ---------- ADC ---------- */
void     adc_init(void);
void     adc_start(uint8_t channel);
uint16_t adc_get(void);
extern volatile uint8_t  adc_ready;
extern volatile uint16_t adc_value;

/* ---------- TWI (I2C) ---------- */
void    twi_init(void);
uint8_t TWI_start(uint8_t twi_addr, uint8_t read_write);
void    TWI_stop(void);
uint8_t TWI_write(uint8_t tx_data);
uint8_t TWI_ack_read(void);
uint8_t TWI_nack_read(void);
uint8_t Read_Reg(uint8_t TWI_addr, uint8_t reg_addr);
uint8_t Read_Reg_N(uint8_t TWI_addr, uint8_t reg_addr, uint8_t bytes, int16_t *data);
uint8_t Write_Reg(uint8_t TWI_addr, uint8_t reg_addr, uint8_t value);

/* ---------- Distance filter (defined in TWI_UART_GPIO.c) ---------- */
float filter_distance(float newVal, float *state);

/* ---------- Fans (FANS.c) ---------- */
void init_fan_system(void);
void lift_fan_control(uint8_t speed);
void lift_fan_stop(void);
void thrust_fan_control(uint8_t speed);
void thrust_fan_off(void);

/* ---------- Ultrasonic (US_LEFT.c / US_RIGHT.c) ---------- */
uint16_t getLeftDistance(void);
uint16_t getRightDistance(void);

/* ---------- IR top sensor (IR_TOP.c) ---------- */
/* Converts a raw ADC reading to distance in cm. Not currently called --
 * navigation works off raw ADC counts (see IR_OBSTACLE_THRESHOLD in
 * main.c) -- kept around in case cm-based thresholds are useful later. */
uint16_t gp2y0a21yk_read_cm(uint16_t adc_value);

/* ---------- IMU (imu_yaw.c) ---------- */
extern int16_t gx_offset, gy_offset, gz_offset;
extern int16_t ax_offset, ay_offset, az_offset;
extern float   yaw;           /* integrated gyro-z (degrees) */
extern uint8_t gyro_setting;
extern uint8_t accel_setting;
extern uint8_t gyro_config[];
extern uint8_t accel_config[];
extern float   gyro_sens[];

void imu_calibrate(void);
void runServo(float dt);         /* updates yaw; does NOT write servo */
void servo_set_yaw(float angle); /* writes OCR1A directly             */

#endif /* TWI_UART_GPIO_H */