# Smart SafeTech Tractor — Full Documentation

**File:** `/home/g1/pap/final/main.py`  
**Platform:** Raspberry Pi  
**Language:** Python 3  
**Framework:** Flask (web server, port 5000)

---

## Table of Contents

1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Hardware — GPIO Pin Map](#hardware--gpio-pin-map)
4. [Hardware — MPU6050 IMU Registers](#hardware--mpu6050-imu-registers)
5. [Class: TractorPins](#class-tractorpins)
6. [Class: TractorController](#class-tractorcontroller)
   - [Initialization](#initialization)
   - [Motor Control](#motor-control)
   - [Servo Control](#servo-control)
   - [Sensor Reading](#sensor-reading)
   - [Programs](#programs)
   - [Cleanup](#cleanup)
7. [Flask Web Server](#flask-web-server)
   - [Page Routes](#page-routes)
   - [API Endpoints](#api-endpoints)
8. [Operating Modes](#operating-modes)
9. [Running the Application](#running-the-application)
10. [Development / Simulation Mode](#development--simulation-mode)
11. [Bug Fixes Applied](#bug-fixes-applied)
12. [Known Limitations](#known-limitations)

---

## Overview

A Raspberry Pi-based tractor control system that exposes a local web interface (Flask, port 5000) for:

- **Manual driving** via on-screen controls or mobile cockpit
- **Line following** using 3 IR sensors
- **Autonomous driving** with ultrasonic obstacle avoidance
- **Inclination safety** using an MPU6050 IMU that triggers a servo-controlled protection arc
- **Peripheral control** — lights, turn signals, buzzer, sound modules, relays

The system falls back to **simulation mode** automatically if GPIO or I2C hardware is unavailable.

---

## Architecture

```
                          ┌─────────────────────────────┐
                          │   Flask Web Server :5000     │
                          │                             │
         Browser/Mobile ──┤  Page routes  API routes   │
                          │  /manual      /api/move     │
                          │  /line_follower /api/control│
                          │  /autonomous  /api/sensors  │
                          └────────────┬────────────────┘
                                       │
                          ┌────────────▼────────────────┐
                          │     TractorController        │
                          │                             │
                          │  Motor A (Right)   Motor B  │
                          │  Servo             Line snsr│
                          │  Ultrasonic        MPU6050  │
                          │  Lights/Sound/Relay         │
                          └─────────────────────────────┘
                                       │
                          ┌────────────▼────────────────┐
                          │  Raspberry Pi GPIO (BCM)     │
                          └─────────────────────────────┘
```

---

## Hardware — GPIO Pin Map

All pin numbers use **BCM mode**.

### Motors (L298N driver)

| Signal | GPIO | Description |
|--------|------|-------------|
| ENA    | 18   | Motor A enable (PWM, 100 Hz) — Right motor |
| IN1    | 8    | Motor A direction 1 |
| IN2    | 11   | Motor A direction 2 |
| ENB    | 27   | Motor B enable (PWM, 100 Hz) — Left motor |
| IN3    | 0    | Motor B direction 1 |
| IN4    | 25   | Motor B direction 2 |

**Direction truth table:**

| IN1 | IN2 | Motor A |
|-----|-----|---------|
| HIGH | LOW | Forward |
| LOW | HIGH | Backward |
| LOW | LOW | Stop |

### Servo

| Signal | GPIO | Frequency |
|--------|------|-----------|
| SERVO_MOTOR | 22 | 50 Hz |

Duty cycle formula: `duty = 2 + (angle / 18)` — maps 0°→2%, 180°→12%.

### Lights & Signals

| Name | GPIO | Logic |
|------|------|-------|
| PISCA_RIGHT | 24 | HIGH = on |
| PISCA_LEFT | 23 | HIGH = on |
| PIRILÂMPO | 14 | HIGH = on |
| LUZ_FRENTE | 15 | HIGH = on |
| LUZ_TRAS | 4 | HIGH = on |

### Sound Modules (inverted logic)

| Name | GPIO | Note |
|------|------|------|
| BUZZER | 17 | HIGH = on (normal logic) |
| SOM_TRABALHAR | 13 | **LOW = on** (inverted) |
| SOM_SEGURANCA | 19 | **LOW = on** (inverted) |
| SOM_INCLINACAO | 26 | **LOW = on** (inverted) |
| SOM_4 | 12 | **LOW = on** (inverted) |

### Relays

| Name | GPIO |
|------|------|
| RELE_1 | 5 |
| RELE_2 | 6 |

### Sensors

| Name | GPIO | Type |
|------|------|------|
| SENSOR_LINHA_L | 16 | IR line sensor (left) — input, pull-down |
| SENSOR_LINHA_C | 20 | IR line sensor (center) — input, pull-down |
| SENSOR_LINHA_R | 21 | IR line sensor (right) — input, pull-down |
| TRIGGER | 7 | Ultrasonic trigger — output |
| ECHO | 1 | Ultrasonic echo — input |
| SDA | 2 | I2C data (MPU6050) |
| SCL | 3 | I2C clock (MPU6050) |

---

## Hardware — MPU6050 IMU Registers

| Register | Address | Purpose |
|----------|---------|---------|
| PWR_MGMT_1 | 0x6B | Power management |
| SMPLRT_DIV | 0x19 | Sample rate divider (set to 7) |
| CONFIG | 0x1A | DLPF configuration (set to 0) |
| GYRO_CONFIG | 0x1B | Gyro full-scale range (set to 24 = ±2000°/s) |
| INT_ENABLE | 0x38 | Interrupt enable (set to 1) |
| ACCEL_XOUT_H | 0x3B | Accelerometer X high byte |
| ACCEL_YOUT_H | 0x3D | Accelerometer Y high byte |
| ACCEL_ZOUT_H | 0x3F | Accelerometer Z high byte |
| GYRO_XOUT_H | 0x43 | Gyroscope X high byte |
| GYRO_YOUT_H | 0x45 | Gyroscope Y high byte |
| GYRO_ZOUT_H | 0x47 | Gyroscope Z high byte |

**I2C Address:** `0x68`  
**Sensitivity:** ±2g → 16384 LSB/g

---

## Class: TractorPins

A namespace class holding all hardware constants. No instances are created.

```python
class TractorPins:
    # MPU6050 register addresses
    # GPIO pin assignments (BCM)
    # SMBus instance (set by MPU_Init)
    bus = None
    Device_Address = 0x68
    current_angle = 0
```

---

## Class: TractorController

Main controller class. A single global instance `tractor` is created at module load.

### Initialization

```python
tractor = TractorController()
```

**`__init__`** sets the following instance flags:

| Attribute | Default | Purpose |
|-----------|---------|---------|
| `current_program` | `None` | Name of the active program |
| `program_thread` | `None` | Active program thread |
| `running` | `True` | Master run flag |
| `arco_running` | `True` | Stop flag for inclination sensor thread |
| `arco_thread` | set in setup | Reference to inclination thread |
| `line_following` | `False` | Line follower active flag |
| `autonomous_driving` | `False` | Autonomous mode active flag |
| `manual_mode` | `False` | Manual mode flag |
| `development_mode` | `False` | Simulation mode flag |
| `pwm_a` | GPIO.PWM | PWM for motor A (100 Hz, GPIO 18) |
| `pwm_b` | GPIO.PWM | PWM for motor B (100 Hz, GPIO 27) |
| `servo_pwm` | GPIO.PWM | PWM for servo (50 Hz, GPIO 22) |

If GPIO setup fails, the system enters `development_mode` and all hardware calls are simulated.

---

### Motor Control

#### `move_forward(speed=70)`
Both motors spin forward. Speed is a PWM duty cycle 0–100.

#### `move_backward(speed=70)`
Both motors spin backward.

#### `turn_left(speed=70)`
Motor A (right) forward + Motor B (left) backward → pivot left.

#### `turn_right(speed=70)`
Motor A (right) backward + Motor B (left) forward → pivot right.

#### `stop_all_motors()`
Sets all IN pins LOW and PWM to 0%.

All four movement methods return immediately with a simulation print if `development_mode` is active.

---

### Servo Control

#### `set_servo_angle(angle)`

Sets the servo to the given angle (0–180°).

```python
duty_cycle = 2 + (angle / 18)
```

Holds position for 1 second then cuts PWM signal to prevent jitter.

---

### Sensor Reading

#### `read_line_sensors() → dict`

Returns:
```python
{'left': bool, 'center': bool, 'right': bool}
```
In simulation mode returns random values.

#### `read_ultrasonic_distance() → float`

Measures distance via HC-SR04 on TRIGGER (GPIO 7) / ECHO (GPIO 1).  
Returns distance in cm, or `-1` on invalid/out-of-range reading (valid range: 2–400 cm).

#### `read_accel_data() → dict`

Reads raw accelerometer from MPU6050 and converts to g-force:
```python
{'x': float, 'y': float, 'z': float}  # units: g
```

#### `read_raw_data(addr) → int`

Reads a signed 16-bit value from the MPU6050 at the given register address.

---

### Programs

#### Inclination Protection — `sensor_active_arco()`

Runs in a dedicated daemon thread (`self.arco_thread`) started during GPIO setup.

- Reads accelerometer every 100 ms
- Computes **pitch** and **roll** from accelerometer:
  ```
  pitch = atan2(x, sqrt(y² + z²))
  roll  = atan2(y, sqrt(x² + z²))
  ```
- **DANGER_ANGLE = 15°** — if `max(|pitch|, |roll|) > 15°`, raises servo to 91° (protection up)
- **SAFE_ANGLE = 10°** — hysteresis; lowers servo to 20° when tilt drops below 10°
- Skips sensor logic when `current_program == "manual"`
- Stops cleanly when `self.arco_running` is set to `False`

#### Line Following — `line_following_program()`

Runs in a daemon thread. Uses 3 IR sensors to track a dark line on a light surface.

| Sensor state | Action | Speed |
|--------------|--------|-------|
| Center active | Move forward | 40% |
| Left active | Turn left | 25% |
| Right active | Turn right | 25% |
| None active | Recovery sequence | 20% |

**Recovery logic (no line detected):**
1. Stop
2. If last move was left/right → continue that direction briefly (200 ms)
3. If last move was center → back up, then steer toward `previous_last_move` direction
4. Fallback: back up

Stops when `self.line_following = False`.

#### Autonomous Driving — `autonomous_driving_program()`

Runs in a daemon thread. Uses the ultrasonic sensor for obstacle avoidance.

| Condition | Action |
|-----------|--------|
| `distance > 25 cm` | Move forward at 50% speed |
| `distance <= 25 cm` | Stop → random left/right turn (1.0 s) → resume |
| `distance == -1` | Sensor error → stop, retry after 500 ms |

Stops when `self.autonomous_driving = False`.

---

### Cleanup

#### `cleanup()`

Called on shutdown (KeyboardInterrupt or exception):

1. Sets `running = False`, `arco_running = False`, `line_following = False`, `autonomous_driving = False`
2. Stops all motors
3. Sets all audio pins HIGH (sound off)
4. Sets all light/relay pins LOW
5. Stops all PWM instances
6. Calls `GPIO.cleanup()`

---

## Flask Web Server

Runs on `0.0.0.0:5000` (all interfaces). Access from any device on the same network.

### Page Routes

| Route | Template | Description |
|-------|----------|-------------|
| `/` | `index.html` | Main menu — program selection |
| `/manual` | `manual.html` | Manual drive controls |
| `/line_follower` | `line_follower.html` | Line follower UI |
| `/control_panel` | `control_panel.html` | Show-off panel (all controls) |
| `/autonomous_radar` | `autonomous_radar.html` | Autonomous driving UI |
| `/mobile_cockpit` | `mobile_cockpit.html` | Mobile-optimised cockpit |

Visiting `/manual` also sets `tractor.current_program = "manual"` and stops line following.

---

### API Endpoints

All endpoints accept/return JSON.

---

#### `POST /api/move`

Control motor direction.

**Request:**
```json
{
  "direction": "forward" | "backward" | "left" | "right" | "stop",
  "speed": 70
}
```

**Response:**
```json
{"status": "ok"}
```

---

#### `POST /api/start_line_follower`

Starts line follower in a background thread.

**Response:**
```json
{"status": "started"}
// or
{"status": "already_running"}
```

---

#### `POST /api/stop_line_follower`

Stops line follower and halts motors.

**Response:**
```json
{"status": "stopped"}
```

---

#### `POST /api/start_autonomous_driving`

Starts autonomous driving in a background thread. Also stops line following.

**Response:**
```json
{"status": "started", "message": "Condução autónoma iniciada"}
// or
{"status": "already_running", "message": "Condução autónoma já ativa"}
```

---

#### `POST /api/stop_autonomous_driving`

Stops autonomous driving and halts motors.

**Response:**
```json
{"status": "stopped", "message": "Condução autónoma parada"}
```

---

#### `POST /api/control`

Toggle/set lights, signals, sounds, relays.

**Request:**
```json
{
  "component": "pisca_left" | "pisca_right" | "luz_frente" | "luz_tras" |
               "pirilâmpo" | "buzzer" | "som_trabalhar" | "som_seguranca" |
               "som_inclinacao" | "rele_1" | "rele_2",
  "state": "on" | "off" | "toggle"
}
```

Sound components (`som_*`) use inverted logic (LOW = on).

**Response:**
```json
{"status": "ok", "component": "...", "state": "..."}
// or in simulation:
{"status": "ok", "component": "...", "state": "...", "mode": "simulation"}
// on error:
{"status": "error", "message": "..."}
```

---

#### `POST /api/servo`

Set servo angle.

**Request:**
```json
{"angle": 90}
```

**Response:**
```json
{"status": "ok", "angle": 90}
```

---

#### `GET /api/sensors`

Read all sensors.

**Response:**
```json
{
  "line_sensors": {"left": false, "center": true, "right": false},
  "ultrasonic_distance": 42.15,
  "timestamp": "2026-05-28T14:32:00.123456",
  "development_mode": false
}
```

`ultrasonic_distance` is `null` if the reading was invalid or in simulation mode.

---

## Operating Modes

### Hardware Mode (default)

- Full GPIO and I2C access
- Real sensor readings
- MPU6050 inclination thread active

### Development / Simulation Mode

Activated automatically if GPIO setup fails, or forced with:

```bash
python3 main.py --simulation
```

In simulation mode:
- All movement and GPIO calls print `[SIM] ...` and return immediately
- `read_line_sensors()` returns random boolean values
- `read_ultrasonic_distance()` returns a random value between 5–150 cm

---

## Running the Application

### Normal start

```bash
cd /home/g1/pap/final
python3 main.py
```

### Simulation mode

```bash
python3 main.py --simulation
```

### Access the UI

```
http://pi.local:5000
http://<pi-ip-address>:5000
```

### Stop

Press `Ctrl+C`. The `finally` block calls `tractor.cleanup()` to release GPIO and PWM resources.

---

## Bug Fixes Applied

The following bugs were identified and fixed (2026-05-28):

| # | Severity | Description | Fix |
|---|----------|-------------|-----|
| 1 | Critical | `previusLastMove` undefined in `line_following_program` → `NameError` at runtime | Replaced with properly tracked `previous_last_move` variable |
| 2 | Critical | Dead code pasted inside `autonomous_driving_program` after the return point (copy of `read_line_sensors` body, unreachable) | Removed dead block |
| 3 | Bug | Variable name mismatch: `last_move` (outer scope) vs `lastMove` (camelCase in else block) — recovery logic always saw wrong state | Unified to `last_move` / `previous_last_move` throughout |
| 4 | Bug | `smbus.SMBus(1)` called at class definition time as a class variable — opened I2C bus before any GPIO/MPU init | Made `bus = None` at class level; actual init moved to `MPU_Init()` |
| 5 | Bug | `move_forward`, `move_backward`, `turn_left`, `turn_right` called `GPIO.output()` directly without checking `development_mode` — crash in simulation | Added `development_mode` guard at top of each method |
| 6 | Design | `sensor_active_arco` thread ran an infinite `while True:` loop with no stop mechanism; `cleanup()` could not terminate it | Added `self.arco_running` flag; thread checks the flag; `cleanup()` sets it `False` |
| 7 | Design | Inclination thread had no stored reference and was not marked as daemon | Stored as `self.arco_thread`; set `daemon = True` so it auto-exits with the main process |

---

## Known Limitations

| Issue | Detail |
|-------|--------|
| GPIO 1 used as ECHO | GPIO 1 is the hardware I2C SDA pin. Using it simultaneously as ultrasonic ECHO input may conflict with MPU6050 I2C communication. Consider rewiring ECHO to an unused GPIO. |
| No API authentication | All `/api/*` endpoints are open to any device on the local network. Acceptable for a local-only project; add token auth if exposed to a wider network. |
| Autonomous turn direction random | Obstacle avoidance picks left/right at random with no memory of past turns. A wall-following or histogram-based approach would be more robust. |
| Servo angle not clamped | `/api/servo` does not clamp `angle` to 0–180 (the clamp line is commented out). Out-of-range values may damage the servo. |
