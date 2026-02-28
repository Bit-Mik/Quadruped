# Quadruped (Arduino + PCA9685)

A 2-DOF-per-leg quadruped controller for **Arduino Uno** using a **PCA9685 16-channel servo driver**.

This firmware uses:
- Inverse kinematics (IK) for each leg
- A phase-shifted walking gait
- Servo safety clamping + saturation reporting
- Startup posture initialization + emergency stop from Serial
- MPU-6050 IMU integration with Mahony AHRS orientation estimation

## Hardware

- Arduino Uno
- PCA9685 servo driver (I2C)
- 8 servo motors (Hip + Knee for each leg)
- External 5V servo power supply (recommended)

## Servo Channel Map

Leg order used in code:
- `LEG_BL` = 0 (Back Left)
- `LEG_FL` = 1 (Front Left)
- `LEG_BR` = 2 (Back Right)
- `LEG_FR` = 3 (Front Right)

PCA9685 channels (`src/globals.cpp`):
- Back Left: Hip `0`, Knee `1`
- Front Left: Hip `4`, Knee `5`
- Back Right: Hip `8`, Knee `9`
- Front Right: Hip `12`, Knee `13`

## I2C / Board Setup

- PCA9685 default address in code: `0x40`
- PWM frequency: `50 Hz`
- Serial monitor baud: `115200`

`platformio.ini` is configured for:
- `platform = atmelavr`
- `board = uno`
- `framework = arduino`

## Build and Upload

### Option 1: PlatformIO (VS Code)
1. Open this folder in VS Code.
2. Install PlatformIO IDE extension.
3. Build from PlatformIO panel.
4. Upload to the Uno.
5. Open Serial Monitor at `115200`.

### Option 2: PlatformIO CLI
```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

## Runtime Controls

Serial commands:
- Send `s` or `S` to trigger emergency stop.

Emergency stop behavior:
- Gait loop halts
- Legs move to neutral IK posture
- Saturation statistics are printed

## Code Architecture

The codebase is split by responsibility:

- `src/main.cpp`
  - System startup and main control loop
  - I2C device check and gait scheduler
- `src/globals.cpp`
  - Global state and leg configuration table
- `src/hardware.cpp`
  - I2C detection + angle-to-PWM tick conversion
- `src/ik.cpp`
  - 2-link inverse kinematics + reachability checks
- `src/servo_control.cpp`
  - IK-to-servo mapping, mirroring, clamp, saturation tracking
- `src/gait.cpp`
  - Swing/stance trajectory generation for each leg
- `src/safety.cpp`
  - Servo initialization, emergency stop, saturation reports
- `src/imu.cpp`
  - MPU-6050 readout, startup gyro calibration, Mahony AHRS, Euler angle state API

Headers are under `include/`.

## Gait and Kinematics Tuning

Primary tuning parameters are in `include/config.h`:

- Leg geometry
  - `UPPER_LEG_LENGTH`
  - `LOWER_LEG_LENGTH`
- Gait motion
  - `STEP_LENGTH`
  - `STEP_HEIGHT`
  - `GAIT_CYCLE_DURATION` (defined in `src/globals.cpp`)
- Control timing
  - `CONTROL_DT`
- Workspace target
  - `X_OFFSET`
  - `Y_GROUND`
- Mechanical mapping
  - `HIP_FRAME_ROTATION`
  - `legs[]` mechanical offsets (`hipMechOffset`, `kneeMechOffset`)

## Safety Model

Safety mechanisms currently implemented:
- IK reachability test (with singularity margin)
- Servo angle clamp to `[0, 180]`
- Saturation event counters per leg/joint
- Initialization mode disables saturation counting during startup posture

## Startup Sequence

1. Initialize Serial and I2C
2. Detect PCA9685 at `0x40`
3. Set PWM frequency to `50 Hz`
4. Move all legs to neutral standing posture
5. Start gait loop

## Notes

- Power servos from a dedicated supply; do not power multiple servos from Uno 5V.
- Keep grounds common between Uno and servo power supply.
- If you see frequent saturation warnings, retune offsets and/or gait targets.
- IMU implementation expects MPU-6050 at I2C address `0x68` and performs startup gyro calibration over initial samples while the robot is held still.

## Repository Layout

```text
QUADRUPED/
  include/
    config.h
    robot_types.h
    globals.h
    hardware.h
    ik.h
    servo_control.h
    gait.h
    safety.h
    imu.h
  src/
    main.cpp
    globals.cpp
    hardware.cpp
    ik.cpp
    servo_control.cpp
    gait.cpp
    safety.cpp
    imu.cpp
  platformio.ini
```
