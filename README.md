# Hovercraft — Autonomous Navigation (ENGR 290)

Firmware for an air-cushion hovercraft that navigates on its own: drive forward
until it detects a wall, scan for the most open direction, turn to face it,
and repeat. Built for an ATmega328P (Arduino Uno-compatible board) in plain
AVR C — no Arduino API calls, register-level drivers throughout.

## How it navigates

1. **Go forward** — lift + thrust fans on, IMU-based yaw hold keeps it
   driving straight. A front-mounted IR sensor watches for an obstacle.
2. **Wall detected** — stop, center the steering servo.
3. **Scan** — sweep the servo through five angles (-90 to +90), sampling the
   front IR at each one, and pick whichever direction reads most open.
4. **Turn** — steer toward that angle and drive the thrust fan until the IMU
   reports enough rotation (or a timeout kicks in so it doesn't get stuck
   spinning in place). Afterward it briefly kills lift to bleed off momentum
   before continuing.
5. Repeat.

Debug info streams out over UART at 115200 baud the whole time (wall
detections, scan results, turn decisions, timeouts).

## Hardware

| Function              | Pin (Uno)     | Notes                              |
|-----------------------|---------------|-------------------------------------|
| Steering servo        | D9  (OC1A)    | 50 Hz, driven directly off Timer1  |
| Lift fan              | D6  (OC0A)    | PWM via Timer0                     |
| Thrust fan            | D5  (OC0B)    | PWM via Timer0                     |
| Front IR (GP2Y0A21YK) | A6            | obstacle detection                 |
| IMU (MPU6050)         | A4/A5 (I2C)   | yaw only, addr 0x68                |
| Left ultrasonic       | D11 trig / D2 echo | wired but unused (see below)  |
| Right ultrasonic      | D13 trig / D3 echo | wired but unused (see below)  |
| Debug UART            | D0/D1         | 115200 baud                        |

The left/right ultrasonic drivers (`US_LEFT.c` / `US_RIGHT.c`) and the IR
cm-conversion helper (`IR_TOP.c`'s `gp2y0a21yk_read_cm()`) are compiled in
but not called by the current navigation logic, which works off raw IR ADC
counts from the front sensor only. Kept around in case a future version
brings the side sensors back into the decision-making.

## Building / uploading

Open `Hovercraft.ino` in the Arduino IDE, board = **Arduino Uno** (or any
ATmega328P board wired the same way), and upload as normal. There's nothing
Arduino-specific in the code itself — `main()` is defined directly in
`main.c` and never touches `setup()`/`loop()`.

## Files

- `main.c` — navigation state machine (go forward / scan / turn), tuning
  constants, `main()`
- `IMU_YAW.c` — MPU6050 calibration + yaw integration, servo angle output
- `TWI_UART_GPIO.c/.h` — shared low-level drivers: UART, GPIO, timers, ADC,
  TWI/I2C
- `FANS.c` — lift/thrust fan PWM
- `IR_TOP.c` — front IR ADC-to-cm conversion
- `US_LEFT.c` / `US_RIGHT.c` — side ultrasonic distance sensing (currently
  unused, see above)
- `Hovercraft.ino` — sketch entry point (just a pointer to `main.c`)
