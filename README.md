# Quadruped Firmware

ESP32-based quadruped controller using a PCA9685 servo driver, inverse kinematics, a crawl gait, IMU stabilization, FreeRTOS tasks, and a LittleFS web dashboard.

## Hardware Overview

- ESP32 DevKit board
- PCA9685 16-channel PWM servo driver at I2C address `0x40`
- 12 servos: shoulder, hip, and knee for each leg
- Razor-style serial IMU on UART2 (`RX2 = 16`, `TX2 = 17`)
- External servo power supply with common ground to the ESP32

## Firmware Flow

1. `src/main.cpp` starts Serial, I2C, Wi-Fi, LittleFS, the web UI, PCA9685, servos, IMU, stabilization, and FreeRTOS tasks.
2. `src/tasks.cpp` runs `imuUpdate()` in a pinned FreeRTOS task at `CONTROL_DT`.
3. `loop()` handles telemetry, Serial commands, timing, and the current `RobotMode`.
4. `MODE_STAND` keeps all feet at the standing target with IMU compensation.
5. `MODE_GAIT` advances gait phase, updates support shift, updates stabilization, and sends each leg through `squareGait()`.
6. `MODE_MANUAL` allows direct servo or leg IK commands without the gait loop overwriting them.

## Main Code Areas

- `include/config.h` contains physical geometry, gait timing constants, PID externs, and leg index definitions.
- `src/globals.cpp` stores runtime tuning values such as PID gains, body shift, gait duration, and leg calibration offsets.
- `src/ik.cpp` converts foot targets into shoulder/hip/knee angles.
- `src/servo_control.cpp` mirrors IK angles per side, applies offsets, clamps servo angles, and tracks saturation.
- `src/gait.cpp` creates foot trajectories and support-shift compensation for crawl walking.
- `src/stabilization.cpp` reads IMU roll/pitch and produces per-leg height corrections through PID controllers.
- `src/webserver.cpp` serves the dashboard and handles WebSocket commands for gait, manual IK, servo moves, and PID tuning.
- `src/telemetry.cpp` sends roll/pitch/yaw JSON to the dashboard.

## Leg Order and Servo Channels

Code leg order:

- `LEG_FR = 0`
- `LEG_FL = 1`
- `LEG_BR = 2`
- `LEG_BL = 3`

PCA9685 channels:

- Front Right: shoulder `0`, hip `1`, knee `2`
- Front Left: shoulder `4`, hip `5`, knee `6`
- Back Right: shoulder `8`, hip `9`, knee `10`
- Back Left: shoulder `12`, hip `13`, knee `14`

Calibration offsets live in `legs[]` inside `src/globals.cpp`.

## Gait Planning

The current walking gait is a crawl sequence:

```text
FR -> BR -> FL -> BL
```

Each leg has a phase offset of `0.25`, so only one leg should swing at a time. `SWING_PORTION` is `0.20`, meaning each leg spends 20% of its cycle in the air and 80% in stance.

`squareGait()` divides swing into:

- Lift: foot rises vertically.
- Forward transfer: foot moves through the air.
- Lower: foot returns to ground.
- Stance: foot moves backward relative to the body as the robot advances.

Directional commands use:

- `targetForward = 1.0` for forward.
- `targetForward = -1.0` for backward.
- `targetTurn = -1.0` for left.
- `targetTurn = 1.0` for right.

The gait starts at `GAIT_START_PHASE`, equal to `SWING_PORTION`, so the robot does not immediately lift a leg at phase zero.

## Back-Heavy Stability Fix

If the robot topples backward when walking starts, the center of mass is behind the support polygon. The firmware now includes three stability helpers:

- `BODY_X_TRIM` in `src/globals.cpp`: static fore/aft trim added to every foot target. Negative values place feet slightly behind the body, moving the body/center of mass forward over the feet.
- `BODY_SHIFT_X`: dynamic fore/aft support shift during gait.
- `BODY_SHIFT_Y`: dynamic left/right support shift during gait.

Current safer defaults:

```cpp
float BODY_SHIFT_X = 0.8f;
float BODY_SHIFT_Y = 0.8f;
float BODY_X_TRIM = -1.0f;
```

Suggested tuning order for a back-heavy robot:

1. Keep the robot on a stand or hold it safely.
2. Start with `BODY_X_TRIM = -1.0`.
3. If it still falls backward, try `-1.5`, then `-2.0`.
4. If front legs look overloaded or knees bind, move back toward `-0.5`.
5. Keep `STEP_LENGTH` small while tuning, around `2.0` to `3.0`.
6. Increase `BODY_SHIFT_X/Y` slowly if a leg lift unloads the wrong side.

Mechanical fixes help too: move the battery forward, lower the battery, widen the stance, or reduce rear payload.

## IMU Stabilization

`stabilizationInit()` calibrates the body pose after IMU startup. During stand and gait:

- Roll error changes left/right foot height compensation.
- Pitch error changes front/rear foot height compensation.
- Deadbands prevent constant tiny corrections.
- Max correction limits prevent aggressive servo movement.

Runtime PID values are stored in globals:

- `KP_ROLL`, `KI_ROLL`, `KD_ROLL`
- `KP_PITCH`, `KI_PITCH`, `KD_PITCH`
- `ROLL_DEADBAND`, `PITCH_DEADBAND`
- `MAX_ROLL_CORR`, `MAX_PITCH_CORR`

The web dashboard can update these values live through a WebSocket `type: "pid"` message. Applying new PID values resets both PID integrators.

## Web Dashboard

The dashboard is served from `data/` through LittleFS.

Controls:

- Movement buttons: `forward`, `backward`, `left`, `right`, `stop`, `stand`
- Telemetry display: roll, pitch, yaw
- PID Stabilization panel: roll/pitch PID, deadbands, max correction
- Servo Control: direct channel angle command
- Leg IK: move a named leg to X/Y/Z

Before uploading web files, use:

```bash
pio run -t uploadfs
```

Then upload firmware:

```bash
pio run -t upload
```

## Serial Commands

- `HELP`: print command list.
- `G`, `START`, or `GAIT`: start forward gait.
- `STOP` or `S`: stop gait and return to stand mode.
- `HOME`: put all servos at 90 degrees.
- `STATUS`: print cached servo angles.
- `S<n>=<angle>`: set servo channel directly.
- `FR=x,y,z`, `FL=x,y,z`, `BR=x,y,z`, `BL=x,y,z`: move a leg through IK.

## Build

The project is configured in `platformio.ini` for:

- `platform = espressif32`
- `board = esp32dev`
- `framework = arduino`
- `board_build.filesystem = littlefs`

Build:

```bash
pio run
```

Upload:

```bash
pio run -t upload
```

Open monitor:

```bash
pio device monitor -b 115200
```

## Safety Notes

- Power servos from a dedicated supply, not from the ESP32.
- Keep servo power ground and ESP32 ground common.
- Test new gait settings with the robot supported.
- Start with small `STEP_LENGTH` and slow `GAIT_CYCLE_DURATION`.
- Watch for saturation events; repeated saturation means the target position, offsets, or gait amplitude need tuning.
- If the robot leans backward at startup, tune `BODY_X_TRIM` before increasing PID gains.
