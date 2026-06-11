#!/usr/bin/env python3
"""
Sistema de Controlo do Trator - Raspberry Pi
Servidor web local para controlo de múltiplos programas
"""

from flask import Flask, render_template, request, jsonify
import RPi.GPIO as GPIO
import time
import threading
import sys
from datetime import datetime
import smbus
import math


app = Flask(__name__)

# Components that blink when active (GPIO is toggled on/off repeatedly)
BLINK_COMPONENTS = {'pisca_left', 'pisca_right', 'pirilâmpo'}
BLINK_INTERVALS  = {'pisca_left': 0.5, 'pisca_right': 0.5, 'pirilâmpo': 0.3}


class TractorPins:
    PWR_MGMT_1   = 0x6B
    SMPLRT_DIV   = 0x19
    CONFIG       = 0x1A
    GYRO_CONFIG  = 0x1B
    INT_ENABLE   = 0x38
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    GYRO_XOUT_H  = 0x43
    GYRO_YOUT_H  = 0x45
    GYRO_ZOUT_H  = 0x47
    bus = None
    Device_Address = 0x68

    # Motor A - Right Motor
    ENA = 18
    IN1 = 8
    IN2 = 11

    # Motor B - Left Motor
    ENB = 27
    IN3 = 0
    IN4 = 25

    SERVO_MOTOR = 22

    PISCA_RIGHT    = 24
    PISCA_LEFT     = 23
    PIRILÂMPO      = 14
    LUZ_FRENTE     = 15
    LUZ_TRAS       = 4

    BUZZER         = 17
    SOM_TRABALHAR  = 13
    SOM_SEGURANCA  = 19
    SOM_INCLINACAO = 26
    SOM_4          = 12

    RELE_1 = 5
    RELE_2 = 6

    SENSOR_LINHA_R = 21
    SENSOR_LINHA_C = 20
    SENSOR_LINHA_L = 16
    TRIGGER        = 7
    ECHO           = 1
    SDA            = 2
    SCL            = 3
    current_angle  = 0


class TractorController:
    def __init__(self):
        self.current_program    = None
        self.program_thread     = None
        self.running            = True
        self.line_following     = False
        self.autonomous_driving = False
        self.manual_mode        = False
        self.development_mode   = False
        self.arco_running       = True
        self.last_command_time  = 0.0
        self.watchdog_active    = False
        self.blink_active        = {}   # component -> bool, drives blink threads
        self.hazard_active       = False
        self.component_states    = {}   # component -> bool, tracks on/off for UI sync
        self.line_follower_speed = 40   # base speed, set via API before starting

        _wd = threading.Thread(target=self._watchdog)
        _wd.daemon = True
        _wd.start()

        try:
            self.setup_gpio()
            print("✅ GPIO configurado com sucesso")
        except Exception as e:
            print(f"⚠️ Erro no GPIO: {e}")
            print("🔧 Entrando em modo de desenvolvimento (simulação)")
            self.development_mode = True
            self.setup_simulation()
            return

        try:
            self.pwm_a = GPIO.PWM(TractorPins.ENA, 100)
            self.pwm_b = GPIO.PWM(TractorPins.ENB, 100)
            self.pwm_a.start(0)
            self.pwm_b.start(0)
            self.servo_pwm = GPIO.PWM(TractorPins.SERVO_MOTOR, 50)
            self.servo_pwm.start(0)
            print("✅ PWM configurado (ENA=18, ENB=27)")
        except Exception as e:
            print(f"⚠️ Erro no PWM: {e}")
            self.development_mode = True

    # ── Blink control ────────────────────────────────────────────────────────

    def start_blink(self, component, pin, interval=0.5):
        """Start a background thread that toggles `pin` at `interval` seconds."""
        # Signal any existing thread for this component to exit
        self.blink_active[component] = False
        time.sleep(0.06)
        self.blink_active[component] = True

        def _loop():
            state = True
            while self.blink_active.get(component):
                if not self.development_mode:
                    GPIO.output(pin, GPIO.HIGH if state else GPIO.LOW)
                else:
                    print(f"[SIM BLINK] {component} {'ON' if state else 'OFF'}")
                state = not state
                time.sleep(interval)
            # Ensure pin is off when done
            if not self.development_mode:
                try:
                    GPIO.output(pin, GPIO.LOW)
                except Exception:
                    pass

        threading.Thread(target=_loop, daemon=True).start()

    def stop_blink(self, component, pin):
        """Stop the blink thread and ensure pin is off."""
        self.blink_active[component] = False
        if not self.development_mode:
            try:
                GPIO.output(pin, GPIO.LOW)
            except Exception:
                pass

    def start_hazard(self):
        """Blink both turn signals in a single thread so they are exactly in sync."""
        # Stop any individual pisca blink threads first
        self.blink_active['pisca_left']  = False
        self.blink_active['pisca_right'] = False
        self.hazard_active = False
        time.sleep(0.06)
        self.hazard_active = True

        def _loop():
            state = True
            while self.hazard_active:
                if not self.development_mode:
                    level = GPIO.HIGH if state else GPIO.LOW
                    GPIO.output(TractorPins.PISCA_LEFT,  level)
                    GPIO.output(TractorPins.PISCA_RIGHT, level)
                else:
                    print(f"[SIM HAZARD] {'ON' if state else 'OFF'}")
                state = not state
                time.sleep(0.5)
            if not self.development_mode:
                try:
                    GPIO.output(TractorPins.PISCA_LEFT,  GPIO.LOW)
                    GPIO.output(TractorPins.PISCA_RIGHT, GPIO.LOW)
                except Exception:
                    pass

        threading.Thread(target=_loop, daemon=True).start()

    def stop_hazard(self):
        self.hazard_active = False
        if not self.development_mode:
            try:
                GPIO.output(TractorPins.PISCA_LEFT,  GPIO.LOW)
                GPIO.output(TractorPins.PISCA_RIGHT, GPIO.LOW)
            except Exception:
                pass

    # ── MPU-6050 (IMU) ───────────────────────────────────────────────────────

    def sensor_active_arco(self):
        DANGER_ANGLE = 15
        SAFE_ANGLE   = 10
        protection_raised = False

        while self.arco_running:
            if self.current_program != "manual":
                try:
                    accel = self.read_accel_data()
                    x, y, z = accel['x'], accel['y'], accel['z']
                    pitch = math.degrees(math.atan2(x, math.sqrt(y**2 + z**2)))
                    roll  = math.degrees(math.atan2(y, math.sqrt(x**2 + z**2)))
                    max_angle = max(abs(pitch), abs(roll))

                    if not protection_raised and max_angle > DANGER_ANGLE:
                        self.set_servo_angle(91)
                        protection_raised = True
                    elif protection_raised and max_angle < SAFE_ANGLE:
                        self.set_servo_angle(20)
                        protection_raised = False
                except Exception:
                    pass  # ignore transient I2C errors
            time.sleep(0.1)

    def MPU_Init(self):
        TractorPins.bus = smbus.SMBus(1)
        TractorPins.bus.write_byte_data(TractorPins.Device_Address, TractorPins.SMPLRT_DIV, 7)
        TractorPins.bus.write_byte_data(TractorPins.Device_Address, TractorPins.PWR_MGMT_1, 1)
        TractorPins.bus.write_byte_data(TractorPins.Device_Address, TractorPins.CONFIG, 0)
        TractorPins.bus.write_byte_data(TractorPins.Device_Address, TractorPins.GYRO_CONFIG, 24)
        TractorPins.bus.write_byte_data(TractorPins.Device_Address, TractorPins.INT_ENABLE, 1)

    def read_raw_data(self, addr):
        high  = TractorPins.bus.read_byte_data(TractorPins.Device_Address, addr)
        low   = TractorPins.bus.read_byte_data(TractorPins.Device_Address, addr + 1)
        value = (high << 8) | low
        if value > 32767:
            value -= 65536
        return value

    def read_accel_data(self):
        return {
            'x': self.read_raw_data(TractorPins.ACCEL_XOUT_H) / 16384.0,
            'y': self.read_raw_data(TractorPins.ACCEL_YOUT_H) / 16384.0,
            'z': self.read_raw_data(TractorPins.ACCEL_ZOUT_H) / 16384.0,
        }

    # ── GPIO setup ───────────────────────────────────────────────────────────

    def setup_gpio(self):
        self.MPU_Init()
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        valid_pins = list(range(28))
        audio_pins = [TractorPins.SOM_TRABALHAR, TractorPins.SOM_SEGURANCA,
                      TractorPins.SOM_INCLINACAO, TractorPins.SOM_4]

        outputs = [
            TractorPins.ENA, TractorPins.IN1, TractorPins.IN2,
            TractorPins.ENB, TractorPins.IN3, TractorPins.IN4,
            TractorPins.SERVO_MOTOR, TractorPins.PISCA_RIGHT, TractorPins.PISCA_LEFT,
            TractorPins.PIRILÂMPO, TractorPins.LUZ_FRENTE, TractorPins.LUZ_TRAS,
            TractorPins.BUZZER, TractorPins.SOM_TRABALHAR, TractorPins.SOM_SEGURANCA,
            TractorPins.SOM_INCLINACAO, TractorPins.SOM_4, TractorPins.RELE_1, TractorPins.RELE_2
        ]

        print("🔧 Configurando pinos de saída (BCM)...")
        for pin in outputs:
            if pin in valid_pins:
                try:
                    GPIO.setup(pin, GPIO.OUT)
                    # Audio pins: HIGH = off (inverted logic)
                    GPIO.output(pin, GPIO.HIGH if pin in audio_pins else GPIO.LOW)
                    print(f"   ✅ GPIO{pin} OK")
                except Exception as e:
                    print(f"   ❌ GPIO{pin}: {e}")
                    raise

        inputs = [TractorPins.SENSOR_LINHA_R, TractorPins.SENSOR_LINHA_C, TractorPins.SENSOR_LINHA_L]
        print("🔧 Configurando pinos de entrada (BCM)...")
        for pin in inputs:
            if pin in valid_pins:
                try:
                    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
                    print(f"   ✅ GPIO{pin} OK")
                except Exception as e:
                    print(f"   ❌ GPIO{pin}: {e}")
                    raise

        GPIO.setup(TractorPins.TRIGGER, GPIO.OUT)
        GPIO.setup(TractorPins.ECHO, GPIO.IN)
        self.servo_pwm = None

        try:
            self.arco_thread = threading.Thread(target=self.sensor_active_arco)
            self.arco_thread.daemon = True
            self.arco_thread.start()
            print("Arco ready")
        except Exception as e:
            print(f"⚠️ Erro no arco: {e}")

    def setup_simulation(self):
        print("🎮 Configurando modo de simulação...")
        self.pwm_a = None
        self.pwm_b = None
        self.servo_pwm = None
        print("✅ Modo de simulação ativo")

    # ── Motor control ─────────────────────────────────────────────────────────

    def stop_all_motors(self):
        if self.development_mode:
            print("[SIM] stop")
            return
        GPIO.output(TractorPins.IN1, GPIO.LOW)
        GPIO.output(TractorPins.IN2, GPIO.LOW)
        GPIO.output(TractorPins.IN3, GPIO.LOW)
        GPIO.output(TractorPins.IN4, GPIO.LOW)
        if self.pwm_a and self.pwm_b:
            self.pwm_a.ChangeDutyCycle(0)
            self.pwm_b.ChangeDutyCycle(0)

    def move_forward(self, speed=70):
        if self.development_mode:
            print(f"[SIM] forward {speed}")
            return
        GPIO.output(TractorPins.IN1, GPIO.HIGH)
        GPIO.output(TractorPins.IN2, GPIO.LOW)
        GPIO.output(TractorPins.IN3, GPIO.HIGH)
        GPIO.output(TractorPins.IN4, GPIO.LOW)
        if self.pwm_a and self.pwm_b:
            self.pwm_a.ChangeDutyCycle(speed)
            self.pwm_b.ChangeDutyCycle(speed)

    def move_backward(self, speed=70):
        if self.development_mode:
            print(f"[SIM] backward {speed}")
            return
        GPIO.output(TractorPins.IN1, GPIO.LOW)
        GPIO.output(TractorPins.IN2, GPIO.HIGH)
        GPIO.output(TractorPins.IN3, GPIO.LOW)
        GPIO.output(TractorPins.IN4, GPIO.HIGH)
        if self.pwm_a and self.pwm_b:
            self.pwm_a.ChangeDutyCycle(speed)
            self.pwm_b.ChangeDutyCycle(speed)

    def turn_left(self, speed=70):
        if self.development_mode:
            print(f"[SIM] left {speed}")
            return
        GPIO.output(TractorPins.IN1, GPIO.LOW)
        GPIO.output(TractorPins.IN2, GPIO.HIGH)
        GPIO.output(TractorPins.IN3, GPIO.HIGH)
        GPIO.output(TractorPins.IN4, GPIO.LOW)
        if self.pwm_a and self.pwm_b:
            self.pwm_a.ChangeDutyCycle(speed)
            self.pwm_b.ChangeDutyCycle(speed)

    def turn_right(self, speed=70):
        if self.development_mode:
            print(f"[SIM] right {speed}")
            return
        GPIO.output(TractorPins.IN1, GPIO.HIGH)
        GPIO.output(TractorPins.IN2, GPIO.LOW)
        GPIO.output(TractorPins.IN3, GPIO.LOW)
        GPIO.output(TractorPins.IN4, GPIO.HIGH)
        if self.pwm_a and self.pwm_b:
            self.pwm_a.ChangeDutyCycle(speed)
            self.pwm_b.ChangeDutyCycle(speed)

    # ── Sensors ───────────────────────────────────────────────────────────────

    def read_line_sensors(self):
        if self.development_mode:
            import random
            return {
                'left':   random.choice([True, False]),
                'center': random.choice([True, False]),
                'right':  random.choice([True, False]),
            }
        return {
            'left':   bool(GPIO.input(TractorPins.SENSOR_LINHA_L)),
            'center': bool(GPIO.input(TractorPins.SENSOR_LINHA_C)),
            'right':  bool(GPIO.input(TractorPins.SENSOR_LINHA_R)),
        }

    def read_ultrasonic_distance(self):
        if self.development_mode:
            import random
            return random.uniform(5, 150)

        try:
            GPIO.output(TractorPins.TRIGGER, False)
            time.sleep(0.05)
            GPIO.output(TractorPins.TRIGGER, True)
            time.sleep(0.00001)
            GPIO.output(TractorPins.TRIGGER, False)

            # Wait for echo to go HIGH — timeout 100 ms
            timeout = time.time() + 0.1
            pulse_start = time.time()
            while GPIO.input(TractorPins.ECHO) == 0:
                pulse_start = time.time()
                if pulse_start > timeout:
                    return -1

            # Wait for echo to go LOW — timeout 100 ms
            timeout = time.time() + 0.1
            pulse_end = time.time()
            while GPIO.input(TractorPins.ECHO) == 1:
                pulse_end = time.time()
                if pulse_end > timeout:
                    return -1

            distance = (pulse_end - pulse_start) * 17150
            if distance > 400 or distance < 2:
                return -1
            return round(distance, 2)

        except Exception as e:
            print(f"Erro ultrassónico: {e}")
            return -1

    # ── Autonomous programs ───────────────────────────────────────────────────

    def autonomous_driving_program(self):
        print("Iniciando condução autónoma...")
        self.autonomous_driving = True
        import random

        while self.autonomous_driving and self.running:
            try:
                distance = self.read_ultrasonic_distance()
                if distance == -1:
                    self.stop_all_motors()
                    time.sleep(0.5)
                    continue

                if distance > 25:
                    self.move_forward(50)
                else:
                    self.stop_all_motors()
                    time.sleep(0.5)
                    if random.choice([True, False]):
                        self.turn_left(40)
                    else:
                        self.turn_right(40)
                    time.sleep(1.0)
                    self.stop_all_motors()
                    time.sleep(0.3)

                time.sleep(0.2)
            except Exception as e:
                print(f"Erro autónomo: {e}")
                self.stop_all_motors()
                time.sleep(1)

        self.stop_all_motors()
        print("Condução autónoma parada.")

    def line_following_program(self):
        print("Iniciando seguidor de linha...")
        self.line_following = True
        last_move = "center"
        previous_last_move = "center"

        while self.line_following and self.running:
            sensors = self.read_line_sensors()
            spd     = self.line_follower_speed
            turn    = max(20, int(spd * 0.6))
            recover = max(15, int(spd * 0.5))

            if sensors['center']:
                self.move_forward(spd)
                previous_last_move, last_move = last_move, "center"
            elif sensors['left']:
                self.turn_left(turn)
                previous_last_move, last_move = last_move, "left"
            elif sensors['right']:
                self.turn_right(turn)
                previous_last_move, last_move = last_move, "right"
            else:
                self.stop_all_motors()
                time.sleep(0.1)
                if last_move == "left":
                    self.turn_left(recover)
                    time.sleep(0.2)
                elif last_move == "right":
                    self.turn_right(recover)
                    time.sleep(0.2)
                else:
                    self.move_backward(recover)
                    time.sleep(0.1)
                    if previous_last_move == "left":
                        self.turn_left(recover)
                    elif previous_last_move == "right":
                        self.turn_right(recover)
                    else:
                        self.move_backward(recover)
                self.stop_all_motors()

            time.sleep(0.05)

        self.stop_all_motors()
        print("Seguidor de linha parado.")

    # ── Servo ─────────────────────────────────────────────────────────────────

    def set_servo_angle(self, angle):
        if not self.servo_pwm:
            return

        # Run in a thread so we don't block the Flask request handler
        def _move():
            duty = 2 + (angle / 18)
            self.servo_pwm.ChangeDutyCycle(duty)
            time.sleep(1)
            self.servo_pwm.ChangeDutyCycle(0)

        threading.Thread(target=_move, daemon=True).start()

    # ── Watchdog ──────────────────────────────────────────────────────────────

    def _watchdog(self):
        TIMEOUT = 0.5
        while self.running:
            if self.watchdog_active and (time.time() - self.last_command_time) > TIMEOUT:
                self.stop_all_motors()
                self.watchdog_active = False
                print("Watchdog: motores parados por segurança")
            time.sleep(0.1)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def cleanup(self):
        self.running            = False
        self.arco_running       = False
        self.line_following     = False
        self.autonomous_driving = False

        # Stop all blink/hazard threads
        self.hazard_active = False
        for component in list(self.blink_active.keys()):
            self.blink_active[component] = False

        self.stop_all_motors()

        if not self.development_mode:
            audio_pins = [TractorPins.SOM_TRABALHAR, TractorPins.SOM_SEGURANCA,
                          TractorPins.SOM_INCLINACAO, TractorPins.SOM_4, TractorPins.BUZZER]
            for pin in audio_pins:
                try:
                    GPIO.output(pin, GPIO.HIGH)
                except Exception:
                    pass

            light_pins = [TractorPins.PISCA_LEFT, TractorPins.PISCA_RIGHT,
                          TractorPins.LUZ_FRENTE, TractorPins.LUZ_TRAS,
                          TractorPins.PIRILÂMPO, TractorPins.RELE_1, TractorPins.RELE_2]
            for pin in light_pins:
                try:
                    GPIO.output(pin, GPIO.LOW)
                except Exception:
                    pass

            if self.pwm_a:   self.pwm_a.stop()
            if self.pwm_b:   self.pwm_b.stop()
            if self.servo_pwm: self.servo_pwm.stop()
            GPIO.cleanup()

        print("🧹 Cleanup concluído")


# ── App instance ──────────────────────────────────────────────────────────────

tractor = TractorController()


# ── Routes ────────────────────────────────────────────────────────────────────

@app.route('/')
def index():
    tractor.current_program = None   # re-enable arc auto-detection
    return render_template('index.html')

@app.route('/manual')
def manual_control():
    tractor.current_program = "manual"   # disables arc auto-detection while in manual
    tractor.line_following = False
    return render_template('manual.html')

@app.route('/line_follower')
def line_follower():
    tractor.current_program = "line_follower"
    return render_template('line_follower.html')

@app.route('/control_panel')
def control_panel():
    tractor.current_program = "control_panel"
    return render_template('control_panel.html')

@app.route('/autonomous_radar')
def autonomous_radar():
    tractor.current_program = "autonomous_radar"
    return render_template('autonomous_radar.html')

@app.route('/mobile_cockpit')
def mobile_cockpit():
    tractor.current_program = "mobile_cockpit"
    return render_template('mobile_cockpit.html')


# ── API ───────────────────────────────────────────────────────────────────────

@app.route('/api/move', methods=['POST'])
def api_move():
    data      = request.json
    direction = data.get('direction')
    speed     = int(data.get('speed', 70))

    if direction != 'stop':
        tractor.last_command_time = time.time()
        tractor.watchdog_active   = True
    else:
        tractor.watchdog_active = False

    if   direction == 'forward':  tractor.move_forward(speed)
    elif direction == 'backward': tractor.move_backward(speed)
    elif direction == 'left':     tractor.turn_left(speed)
    elif direction == 'right':    tractor.turn_right(speed)
    elif direction == 'stop':     tractor.stop_all_motors()

    return jsonify({'status': 'ok'})


@app.route('/api/start_line_follower', methods=['POST'])
def api_start_line_follower():
    tractor.watchdog_active = False
    data  = request.json or {}
    speed = int(data.get('speed', tractor.line_follower_speed))
    tractor.line_follower_speed = max(20, min(80, speed))
    if not tractor.line_following:
        t = threading.Thread(target=tractor.line_following_program)
        t.daemon = True
        t.start()
        return jsonify({'status': 'started', 'speed': tractor.line_follower_speed})
    return jsonify({'status': 'already_running'})


@app.route('/api/stop_line_follower', methods=['POST'])
def api_stop_line_follower():
    tractor.line_following = False
    tractor.stop_all_motors()
    return jsonify({'status': 'stopped'})


@app.route('/api/start_autonomous_driving', methods=['POST'])
def api_start_autonomous_driving():
    tractor.watchdog_active = False
    if not tractor.autonomous_driving:
        tractor.line_following = False
        t = threading.Thread(target=tractor.autonomous_driving_program)
        t.daemon = True
        t.start()
        return jsonify({'status': 'started'})
    return jsonify({'status': 'already_running'})


@app.route('/api/stop_autonomous_driving', methods=['POST'])
def api_stop_autonomous_driving():
    tractor.autonomous_driving = False
    tractor.stop_all_motors()
    return jsonify({'status': 'stopped'})


@app.route('/api/control', methods=['POST'])
def api_control():
    data      = request.json
    component = data.get('component')
    state     = data.get('state', 'toggle')

    pin_map = {
        'pisca_left':     TractorPins.PISCA_LEFT,
        'pisca_right':    TractorPins.PISCA_RIGHT,
        'luz_frente':     TractorPins.LUZ_FRENTE,
        'luz_tras':       TractorPins.LUZ_TRAS,
        'pirilâmpo':      TractorPins.PIRILÂMPO,
        'buzzer':         TractorPins.BUZZER,
        'som_trabalhar':  TractorPins.SOM_TRABALHAR,
        'som_seguranca':  TractorPins.SOM_SEGURANCA,
        'som_inclinacao': TractorPins.SOM_INCLINACAO,
        'rele_1':         TractorPins.RELE_1,
        'rele_2':         TractorPins.RELE_2,
    }
    # Audio pins: LOW = on, HIGH = off (inverted logic board)
    audio_components = {'som_trabalhar', 'som_seguranca', 'som_inclinacao'}

    if component not in pin_map:
        return jsonify({'status': 'error', 'message': 'Component not found'})

    pin = pin_map[component]

    if tractor.development_mode:
        print(f"[SIM] {component} -> {state}")
        return jsonify({'status': 'ok', 'component': component, 'state': state, 'mode': 'simulation'})

    try:
        if component in BLINK_COMPONENTS:
            if state == 'on':
                tractor.start_blink(component, pin, BLINK_INTERVALS.get(component, 0.5))
            elif state == 'off':
                tractor.stop_blink(component, pin)
            else:  # toggle
                if tractor.blink_active.get(component):
                    tractor.stop_blink(component, pin)
                else:
                    tractor.start_blink(component, pin, BLINK_INTERVALS.get(component, 0.5))

        elif component in audio_components:
            if   state == 'on':  GPIO.output(pin, GPIO.LOW)
            elif state == 'off': GPIO.output(pin, GPIO.HIGH)
            else:                GPIO.output(pin, not GPIO.input(pin))

        else:
            if   state == 'on':     GPIO.output(pin, GPIO.HIGH)
            elif state == 'off':    GPIO.output(pin, GPIO.LOW)
            else:                   GPIO.output(pin, not GPIO.input(pin))

    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)})

    # Track state so the frontend can sync on page load
    if component in BLINK_COMPONENTS:
        tractor.component_states[component] = tractor.blink_active.get(component, False)
    else:
        tractor.component_states[component] = (state == 'on')

    return jsonify({'status': 'ok', 'component': component, 'state': state})


@app.route('/api/hazard', methods=['POST'])
def api_hazard():
    data  = request.json or {}
    state = data.get('state', 'toggle')

    if state == 'on':
        tractor.start_hazard()
    elif state == 'off':
        tractor.stop_hazard()
    else:  # toggle
        if tractor.hazard_active:
            tractor.stop_hazard()
        else:
            tractor.start_hazard()

    tractor.component_states['hazard'] = tractor.hazard_active
    return jsonify({'status': 'ok', 'active': tractor.hazard_active})


@app.route('/api/component_states')
def api_component_states():
    """Returns the current on/off state of all controllable components."""
    states = dict(tractor.component_states)
    for c in BLINK_COMPONENTS:
        states[c] = tractor.blink_active.get(c, False)
    states['hazard'] = tractor.hazard_active
    return jsonify(states)


@app.route('/api/servo', methods=['POST'])
def api_servo():
    data  = request.json
    angle = data.get('angle', 90)
    tractor.set_servo_angle(angle)
    return jsonify({'status': 'ok', 'angle': angle})


@app.route('/api/sensors')
def api_sensors():
    line_sensors = tractor.read_line_sensors()
    distance     = tractor.read_ultrasonic_distance()
    return jsonify({
        'line_sensors':       line_sensors,
        'ultrasonic_distance': round(distance, 2) if distance > 0 else None,
        'timestamp':          datetime.now().isoformat(),
        'development_mode':   tractor.development_mode,
    })


if __name__ == '__main__':
    try:
        print("🚜 Iniciando Smart SafeTech Tractor...")

        if len(sys.argv) > 1 and sys.argv[1] == '--simulation':
            print("🎮 Modo simulação forçado via parâmetro")
            tractor.development_mode = True

        try:
            with open('/proc/cpuinfo', 'r') as f:
                for line in f:
                    if 'Model' in line:
                        print(f"📱 {line.strip()}")
                        break
        except Exception:
            pass

        print("⚡ MODO HARDWARE ATIVO")
        print("🌐 http://localhost:5000")

        try:
            import socket
            local_ip = socket.gethostbyname(socket.gethostname())
            print(f"🌐 http://{local_ip}:5000")
        except Exception:
            pass

        print("💡 Use Ctrl+C para parar o servidor")
        app.run(host='0.0.0.0', port=5000, debug=False)

    except KeyboardInterrupt:
        print("\n🛑 Servidor parado pelo utilizador")
    except Exception as e:
        print(f"\n❌ Erro: {e}")
    finally:
        tractor.cleanup()
