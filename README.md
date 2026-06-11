# SmartTractor

A **Raspberry Pi-based smart tractor control system** that exposes a local web interface for manual driving, line following, and autonomous obstacle avoidance — with real-time safety monitoring via an IMU.

Built as a final project for **Smart SafeTech** (EPMS 2026).

---

## Overview

| Feature | Details |
|---------|---------|
| Platform | Raspberry Pi 3 |
| Language | Python 3 |
| Web framework | Flask (port 5000) |
| Motor driver | L298N (dual H-bridge) |
| Sensors | HC-SR04 ultrasonic, MPU6050 IMU, 3× IR line sensors |
| Actuators | 2× DC motors, 1× servo motor |
| Peripherals | Lights, turn signals, buzzer, sound modules, 2× relays |
| Network | Accessible from any device on the same Wi-Fi |

---

## Features

### Driving Modes

**Manual Control**
- On-screen D-pad with variable speed slider
- Keyboard shortcuts (`W A S D` / arrow keys)
- Mobile-optimised cockpit view

**Line Following**
- Uses 3 infrared sensors (left, center, right)
- Automatic recovery when the line is lost
- Start/stop from the web UI

**Autonomous Driving**
- HC-SR04 ultrasonic obstacle avoidance
- Stops and steers randomly on obstacles closer than 25 cm
- Start/stop from the web UI with live radar display

### Safety — Inclination Protection

A dedicated background thread reads the MPU6050 accelerometer 10× per second and computes pitch and roll:

- **Tilt > 15°** → servo raises the protection arc to 91° (blocking hazardous movement)
- **Tilt < 10°** → servo lowers the arc back to 20° (hysteresis prevents chatter)

This runs independently of the active driving mode and cannot be disabled from the UI.

### Peripheral Control

All outputs can be toggled via the control panel:

| Group | Components |
|-------|-----------|
| Lights | Front light, rear light |
| Signals | Left blinker, right blinker, strobe (pirilâmpo) |
| Audio | Buzzer, working sound, safety alarm, tilt alarm |
| Relays | RELE_1, RELE_2 |

Blink components (blinkers, strobe) run on their own timer threads at configurable intervals.

---

## Hardware — GPIO Pin Map (BCM)

### Motors (L298N)

| Signal | GPIO | Description |
|--------|------|-------------|
| ENA | 18 | Motor A PWM (100 Hz) — Right |
| IN1 | 8 | Motor A direction 1 |
| IN2 | 11 | Motor A direction 2 |
| ENB | 27 | Motor B PWM (100 Hz) — Left |
| IN3 | 0 | Motor B direction 1 |
| IN4 | 25 | Motor B direction 2 |

### Servo & Sensors

| Signal | GPIO | Notes |
|--------|------|-------|
| SERVO_MOTOR | 22 | 50 Hz, duty = 2 + (angle/18) |
| SENSOR_LINHA_L | 16 | IR line sensor left |
| SENSOR_LINHA_C | 20 | IR line sensor center |
| SENSOR_LINHA_R | 21 | IR line sensor right |
| TRIGGER | 7 | HC-SR04 trigger |
| ECHO | 1 | HC-SR04 echo |
| SDA | 2 | MPU6050 I2C data |
| SCL | 3 | MPU6050 I2C clock |

### Lights, Audio & Relays

| Name | GPIO | Logic |
|------|------|-------|
| LUZ_FRENTE | 15 | HIGH = on |
| LUZ_TRAS | 4 | HIGH = on |
| PISCA_LEFT | 23 | HIGH = on |
| PISCA_RIGHT | 24 | HIGH = on |
| PIRILÂMPO | 14 | HIGH = on |
| BUZZER | 17 | HIGH = on |
| SOM_TRABALHAR | 13 | **LOW = on** (inverted) |
| SOM_SEGURANCA | 19 | **LOW = on** (inverted) |
| SOM_INCLINACAO | 26 | **LOW = on** (inverted) |
| RELE_1 | 5 | HIGH = on |
| RELE_2 | 6 | HIGH = on |

---

## Project Structure

```
SmartTractor/
├── main.py                    # Flask app + all hardware logic
├── test_hardware.py           # Interactive hardware diagnostics
├── requirements.txt           # Python dependencies
├── setup_systemc.sh           # Systemd service installer
├── TRACTOR_DOCUMENTATION.md   # Full technical reference
├── templates/
│   ├── base.html              # Shared layout (dark theme, nav)
│   ├── index.html             # Mode selector dashboard
│   ├── manual.html            # Manual drive D-pad
│   ├── line_follower.html     # Line follower UI
│   ├── autonomous_radar.html  # Radar + autonomous controls
│   ├── control_panel.html     # All-peripherals show panel
│   └── mobile_cockpit.html    # Mobile-optimised full cockpit
└── static/
    └── images/                # UI assets
```

---

## Getting Started

### 1. Install dependencies

```bash
cd /home/g1/pap/final
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### 2. Run the server

```bash
python3 main.py
```

Open a browser on any device connected to the same network:

```
http://pi.local:5000
http://<pi-ip>:5000
```

### 3. Run as a system service (auto-start on boot)

```bash
bash setup_systemc.sh
sudo systemctl start tractor-control
```

### 4. Simulation / development mode

No Raspberry Pi? No problem — run in simulation mode on any machine:

```bash
python3 main.py --simulation
```

GPIO calls are replaced with console output (`[SIM] ...`), and sensors return random values.

---

## API Reference

All endpoints are JSON. Base URL: `http://pi.local:5000`

| Method | Endpoint | Description |
|--------|----------|-------------|
| `POST` | `/api/move` | Motor direction (`forward`, `backward`, `left`, `right`, `stop`) + optional `speed` (0–100) |
| `POST` | `/api/control` | Toggle peripheral: `component` + `state` (`on`/`off`/`toggle`) |
| `POST` | `/api/servo` | Set servo angle (`angle`: 0–180) |
| `POST` | `/api/start_line_follower` | Start line following thread |
| `POST` | `/api/stop_line_follower` | Stop line following |
| `POST` | `/api/start_autonomous_driving` | Start autonomous driving thread |
| `POST` | `/api/stop_autonomous_driving` | Stop autonomous driving |
| `GET` | `/api/sensors` | Read all sensors (line, ultrasonic, timestamp) |

### Example

```bash
# Drive forward at 80% speed for 2 seconds, then stop
curl -X POST http://pi.local:5000/api/move \
  -H "Content-Type: application/json" \
  -d '{"direction":"forward","speed":80}'

sleep 2

curl -X POST http://pi.local:5000/api/move \
  -H "Content-Type: application/json" \
  -d '{"direction":"stop"}'
```

---

## Web UI Screenshots

| Page | URL | Purpose |
|------|-----|---------|
| Dashboard | `/` | Mode selector with live sensor badges |
| Manual | `/manual` | D-pad + speed slider + peripheral toggles |
| Line Follower | `/line_follower` | Start/stop + live sensor indicators |
| Autonomous | `/autonomous_radar` | Radar sweep + obstacle distance |
| Control Panel | `/control_panel` | All peripherals in one view |
| Mobile Cockpit | `/mobile_cockpit` | Full cockpit optimised for phones |

---

## Architecture

```
                    ┌──────────────────────────────────┐
                    │      Flask Web Server :5000       │
    Browser/Phone ──│  / /manual /line_follower         │
                    │  /autonomous_radar /control_panel │
                    │  /api/move /api/control /api/...  │
                    └────────────────┬─────────────────┘
                                     │
                    ┌────────────────▼─────────────────┐
                    │         TractorController         │
                    │                                   │
                    │  Motor A (Right)   Motor B (Left) │
                    │  Servo (arc)       3× IR sensors  │
                    │  HC-SR04 (dist.)   MPU6050 (IMU)  │
                    │  Lights / Audio / Relays           │
                    │                                   │
                    │  Threads:                         │
                    │    arco_thread    — IMU safety    │
                    │    program_thread — active mode   │
                    └────────────────┬─────────────────┘
                                     │
                    ┌────────────────▼─────────────────┐
                    │     Raspberry Pi GPIO (BCM)        │
                    └───────────────────────────────────┘
```

---

## Dependencies

```
Flask==2.3.3
RPi.GPIO==0.7.1
```

The `smbus` library (I2C for MPU6050) is included with Raspberry Pi OS.

---

## License

Academic project — EPMS 2026. All rights reserved.
