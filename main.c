// main.c - Hovercraft IR-guided navigation

#define F_CPU 16000000UL

#include "TWI_UART_GPIO.h"

#define BAUD  115200UL
#define UBRR  (((F_CPU)/(BAUD*8UL))-1)

#define MPU_ADDR        0x68
#define SAMPLE_RATE_DIV 9

// --- tuning constants ---
#define K_YAW               2.0f
#define MAX_HEADING_SERVO   30.0f

#define SCAN_HOLD_MS        300u
#define IR_OBSTACLE_THRESHOLD   60u
#define IR_OPEN_MIN_ADC         200u
#define IR_AVG_WINDOW           5u

#define LIFT_SPEED          200u
#define THRUST_BURST        200u
#define THRUST_CRUISE       120u
#define THRUST_TURN         80u
#define THRUST_MAX          255u

#define BURST_TIME_MS       1000u
#define TURN_STUCK_MS       3000u
#define TURN_SERVO_ANGLE    60.0f
#define TURN_COMPLETE_DEG   85.0f
#define TURN_TIMEOUT_MS     2400u   // 120 ticks * 20ms

#define FORWARD_BLIND_MS    3000u
#define POST_TURN_BLIND_MS  4000u

// scan angles for finding open directions
float scan_angles[5] = { -90.0f, -45.0f,0.0f, 45.0f,  90.0f };

// heading reference
float heading_ref = 0.0f;

// IR averaging buffer
uint16_t ir_buf[IR_AVG_WINDOW];
uint8_t  ir_idx  = 0;
uint8_t  ir_full = 0;

// -------------------------------------------------------
// sensor helpers
// -------------------------------------------------------

uint16_t read_ir(void) {
    adc_start(6);
    while (!adc_ready);
    return adc_get();
}

void clear_ir_buf(void) {
    ir_idx = 0;
    ir_full = 0;
    for (uint8_t i = 0; i < IR_AVG_WINDOW; i++) ir_buf[i] = 0;
}

// rolling average over the last IR_AVG_WINDOW samples, to smooth out noise
uint16_t get_avg_ir(void) {
    uint16_t new_value = read_ir();
    ir_buf[ir_idx] = new_value;

    ir_idx++;
    if (ir_idx >= IR_AVG_WINDOW) {
        ir_idx = 0;
        ir_full = 1;
    }

    uint8_t count = ir_full ? IR_AVG_WINDOW : ir_idx;
    if (count == 0) return new_value; // first call, buffer still empty

    uint32_t sum = 0;
    for (uint8_t i = 0; i < count; i++) sum += ir_buf[i];
    return (uint16_t)(sum / count);
}


// wait for the next 50 Hz tick and update the IMU

void wait_tick(float *dt) {
    static uint32_t last_ticks = 0;
    while (!loop_flag);
    loop_flag = 0;

    uint32_t now = system_ticks;
    if (last_ticks == 0) {
        *dt = 0.02f;
    } else {
        *dt = (float)(now - last_ticks) * 0.02f;
        if (*dt > 0.05f) *dt = 0.05f;
    }
    last_ticks = now;
    runServo(*dt);
}

// -------------------------------------------------------
// apply yaw correction to hold heading_ref
// -------------------------------------------------------

void apply_yaw_correction(void) {
    float err = heading_ref - yaw;
    if (err >  180.0f) err -= 360.0f;
    if (err < -180.0f) err += 360.0f;
    float cmd = K_YAW * err;
    if (cmd >  MAX_HEADING_SERVO) cmd =  MAX_HEADING_SERVO;
    if (cmd < -MAX_HEADING_SERVO) cmd = -MAX_HEADING_SERVO;
    servo_set_yaw(cmd);
}

// -------------------------------------------------------
// scan all angles and return the most open direction
// -------------------------------------------------------

float scan_direction(void) {
    float    best_angle = 0.0f;
    uint16_t best_adc   = 0xFFFF;
    uint8_t  found_open = 0;

    for (uint8_t i = 0; i < 5; i++) {
        servo_set_yaw(scan_angles[i]);
        delay_ms(SCAN_HOLD_MS);

        uint16_t ir_val = read_ir();

        uart_print("SCAN angle=");
        uart_print_int((int)scan_angles[i]);
        uart_print(" ADC=");
        uart_print_int((int)ir_val);
        uart_print("\r\n");

        if (ir_val < IR_OPEN_MIN_ADC) {
            found_open = 1;
            if (ir_val < best_adc) {
                best_adc   = ir_val;
                best_angle = scan_angles[i];
            }
        }
    }

    servo_set_yaw(0.0f);

    if (!found_open) {
        uart_print("SCAN: all blocked, defaulting to 0\r\n");
        return 0.0f;
    }

    uart_print("SCAN: best angle=");
    uart_print_int((int)best_angle);
    uart_print("\r\n");

    return best_angle;
}

// -------------------------------------------------------
// do a turn in direction dir (+1 = right, -1 = left)
// then wait 4 seconds for the craft to settle
// -------------------------------------------------------

void do_turn(int8_t dir) {
    float    turn_start_yaw   = heading_ref;
    uint32_t turn_start_ticks = system_ticks;
    float    dt               = 0.02f;

    uart_print("NAV: TURNING dir=");
    uart_print_int((int)dir);
    uart_print("\r\n");

    servo_set_yaw(dir >= 0 ? TURN_SERVO_ANGLE : -TURN_SERVO_ANGLE);
    thrust_fan_control(THRUST_TURN);

    // keep turning until rotated enough or timed out
    while (1) {
        wait_tick(&dt);

        float delta = yaw - turn_start_yaw;
        if (delta >  180.0f) delta -= 360.0f;
        if (delta < -180.0f) delta += 360.0f;

        float    progress   = (float)dir * delta;
        uint32_t elapsed_ms = (system_ticks - turn_start_ticks) * 20u;

        if (elapsed_ms >= TURN_STUCK_MS) {
            thrust_fan_control(THRUST_MAX);
        }

        if (progress >= TURN_COMPLETE_DEG) {
            uart_print("NAV: turn done\r\n");
            break;
        }

        if (elapsed_ms >= TURN_TIMEOUT_MS) {
            uart_print("NAV: turn timeout\r\n");
            break;
        }
    }

    // the craft coasts on momentum after the turn stops, so heading_ref
    // (a straight line) isn't a great reference right after turning --
    // just aim for "the opposite of where we started"
    heading_ref = turn_start_yaw + 180.0f;
    if (heading_ref >  180.0f) heading_ref -= 360.0f;
    if (heading_ref < -180.0f) heading_ref += 360.0f;

    uart_print("NAV: ref=");
    uart_print_int((int)heading_ref);
    uart_print("\r\n");

    // momentum kill: hold briefly, drop the lift cushion so friction
    // stops the spin, then re-inflate before handing off to go_forward()
    uart_print("NAV: stabilize before lift cut\r\n");
    uint32_t delay_start = system_ticks;
    while ((system_ticks - delay_start) * 20u < 1000u) {
        wait_tick(&dt);
        apply_yaw_correction();
    }

    uart_print("NAV: lift OFF (momentum kill)\r\n");
    lift_fan_stop();

    uint32_t cut_start = system_ticks;
    while ((system_ticks - cut_start) * 20u < 2000u) {
        wait_tick(&dt); // grounded, no point correcting yaw here
    }

    uart_print("NAV: lift ON\r\n");
    lift_fan_control(LIFT_SPEED);

    uint32_t recover_start = system_ticks;
    while ((system_ticks - recover_start) * 20u < 500u) {
        wait_tick(&dt);
        apply_yaw_correction();
    }
    uart_print("NAV: settling done\r\n");
}

// -------------------------------------------------------
// go forward until a wall is detected
// -------------------------------------------------------

void go_forward(void) {
    uint32_t entry_ticks = system_ticks;
    uint32_t burst_start = system_ticks;
    uint8_t  bursting    = 1;
    float    dt          = 0.02f;

    clear_ir_buf();

    lift_fan_control(LIFT_SPEED);
    thrust_fan_control(THRUST_BURST);

    uart_print("NAV: FORWARD ref=");
    uart_print_int((int)heading_ref);
    uart_print("\r\n");

    while (1) {
        wait_tick(&dt);

        // burst -> cruise transition
        if (bursting && (system_ticks - burst_start) * 20u >= BURST_TIME_MS) {
            bursting = 0;
            thrust_fan_control(THRUST_CRUISE);
            uart_print("NAV: CRUISE\r\n");
        }

        // skip IR check during the entry blind window
        if ((system_ticks - entry_ticks) * 20u < FORWARD_BLIND_MS) {
            apply_yaw_correction();
            continue;
        }

        uint16_t ir_adc = get_avg_ir();
        if (ir_adc >= IR_OBSTACLE_THRESHOLD) {
            uart_print("NAV: wall detected\r\n");
            break;
        }

        apply_yaw_correction();
    }
}

// -------------------------------------------------------
// main
// -------------------------------------------------------

int main(void) {

    // hardware init
    gpio_init();
    uart_init(UBRR);
    timer1_init();
    timer2_init();
    adc_init();
    init_fan_system();
    twi_init();

    // set up IMU registers
    Write_Reg(MPU_ADDR, 0x6B, 0x00);
    delay_ms(100);
    Write_Reg(MPU_ADDR, 0x1A, 0x01);
    Write_Reg(MPU_ADDR, 0x19, SAMPLE_RATE_DIV);
    Write_Reg(MPU_ADDR, 0x1B, gyro_config[gyro_setting]);
    Write_Reg(MPU_ADDR, 0x1C, accel_config[accel_setting]);

    // calibrate IMU
    uart_print("Starting Calibration\r\n");
    imu_calibrate();
    uart_print("Finished Calibration\r\n");

    // uncomment to calibrate IR sensor
    /*
    uart_print("ENTERING SENSOR CALIBRATION MODE...\r\n");
    while(1) {
        uint16_t raw_ir = read_ir();
        uart_print("FRONT IR ADC: ");
        uart_print_int(raw_ir);
        uart_print("\r\n");
        delay_ms(250);
    }
    */

    // start lift fan and centre servo
    lift_fan_control(LIFT_SPEED);
    servo_set_yaw(0.0f);

    uart_print("Waiting 1.5s for hardware to settle...\r\n");
    delay_ms(1500);
    read_ir();

    heading_ref = 0.0f;

    // main navigation loop:
    // go forward -> hit wall -> scan for open direction -> turn -> repeat
    while (1) {

        go_forward();

        // stop so the craft can settle before scanning
        thrust_fan_off();
        lift_fan_stop();
        servo_set_yaw(0.0f);
        uart_print("NAV: SCAN_STOP\r\n");
        delay_ms(500);

        // find the most open direction
        float best_angle = scan_direction();

        uart_print("NAV: scan best=");
        uart_print_int((int)best_angle);
        uart_print("\r\n");

        // pick turn direction
        int8_t dir;
        if (best_angle > 0.0f) {
            dir = +1;
        } else if (best_angle < 0.0f) {
            dir = -1;
        } else {
            dir = +1;   // default right if straight ahead is clearest
        }

        // re-inflate the air cushion before turning
        lift_fan_control(LIFT_SPEED);
        delay_ms(300);

        do_turn(dir);
    }

    return 0;
}