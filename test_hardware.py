#!/usr/bin/env python3
"""
Script de Teste do Hardware - Smart SafeTech Tractor
Testa todas as ligações GPIO conforme a tabela de pinos
"""

import RPi.GPIO as GPIO
import time
import sys

# Configuração dos pinos baseada na tabela
class TractorPins:
    # Motores (outputs)
    MOTOR_LEFT_1 = 27      # EN3
    MOTOR_LEFT_2 = 22      # EN4
    MOTOR_RIGHT_1 = 24     # EN1
    MOTOR_RIGHT_2 = 23     # EN2
    MOTOR_LEFT_PWM = 13    # ENB
    MOTOR_RIGHT_PWM = 12   # ENA
    
    # Servo (output)
    SERVO_MOTOR = 15       # GPIO 22
    
    # Luzes e sinalizadores (outputs)
    PISCA_RIGHT = 18       # GPIO 24
    PISCA_LEFT = 16        # GPIO 23
    PIRILÂMPO = 8          # GPIO 14
    LUZ_FRENTE = 10        # GPIO 15
    LUZ_TRAS = 7           # GPIO 4
    
    # Som (outputs)
    BUZZER = 11            # GPIO 17
    SOM_TRABALHAR = 33     # GPIO 13
    SOM_SEGURANCA = 35     # GPIO 19
    SOM_INCLINACAO = 37    # GPIO 26
    SOM_4 = 32             # GPIO 12
    
    # Relés (outputs)
    RELE_1 = 29            # GPIO 5
    RELE_2 = 31            # GPIO 6
    
    # Sensores (inputs)
    SENSOR_LINHA_R = 36    # GPIO 16
    SENSOR_LINHA_C = 38    # GPIO 20
    SENSOR_LINHA_L = 40    # GPIO 21
    TRIGGER = 26           # GPIO 7
    ECHO = 28              # GPIO 1
    SDA = 3                # GPIO 2
    SCL = 5                # GPIO 3

class HardwareTester:
    def __init__(self):
        print("🚜 Smart SafeTech Tractor - Teste de Hardware")
        print("=" * 50)
        
        # Configurar GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        
        # Definir pinos de saída
        self.output_pins = {
            'Motor Esquerdo 1': TractorPins.MOTOR_LEFT_1,
            'Motor Esquerdo 2': TractorPins.MOTOR_LEFT_2,
            'Motor Direito 1': TractorPins.MOTOR_RIGHT_1,
            'Motor Direito 2': TractorPins.MOTOR_RIGHT_2,
            'PWM Motor Esquerdo': TractorPins.MOTOR_LEFT_PWM,
            'PWM Motor Direito': TractorPins.MOTOR_RIGHT_PWM,
            'Servo Motor': TractorPins.SERVO_MOTOR,
            'Pisca Direita': TractorPins.PISCA_RIGHT,
            'Pisca Esquerda': TractorPins.PISCA_LEFT,
            'Pirilâmpo': TractorPins.PIRILÂMPO,
            'Luz Frente': TractorPins.LUZ_FRENTE,
            'Luz Trás': TractorPins.LUZ_TRAS,
            'Buzzer': TractorPins.BUZZER,
            'Som Trabalhar': TractorPins.SOM_TRABALHAR,
            'Som Segurança': TractorPins.SOM_SEGURANCA,
            'Som Inclinação': TractorPins.SOM_INCLINACAO,
            'Som 4': TractorPins.SOM_4,
            'Relé 1': TractorPins.RELE_1,
            'Relé 2': TractorPins.RELE_2
        }
        
        # Definir pinos de entrada
        self.input_pins = {
            'Sensor Linha Direita': TractorPins.SENSOR_LINHA_R,
            'Sensor Linha Centro': TractorPins.SENSOR_LINHA_C,
            'Sensor Linha Esquerda': TractorPins.SENSOR_LINHA_L,
            'Trigger Ultrassónico': TractorPins.TRIGGER,
            'Echo Ultrassónico': TractorPins.ECHO,
            'SDA (I2C)': TractorPins.SDA,
            'SCL (I2C)': TractorPins.SCL
        }
        
        self.setup_gpio()
    
    def setup_gpio(self):
        """Configurar todos os pinos GPIO"""
        print("🔧 Configurando GPIO...")
        
        # Configurar outputs
        for name, pin in self.output_pins.items():
            try:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)
                print(f"✅ {name}: Pino {pin} - OK")
            except Exception as e:
                print(f"❌ {name}: Pino {pin} - ERRO: {e}")
        
        # Configurar inputs
        for name, pin in self.input_pins.items():
            try:
                GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
                print(f"✅ {name}: Pino {pin} - OK")
            except Exception as e:
                print(f"❌ {name}: Pino {pin} - ERRO: {e}")
        
        print()
    
    def test_outputs(self):
        """Testar todos os pinos de saída"""
        print("🔌 Testando Saídas (Outputs)...")
        print("-" * 30)
        
        for name, pin in self.output_pins.items():
            try:
                print(f"🧪 Testando {name} (Pino {pin})...")
                
                # Ligar
                GPIO.output(pin, GPIO.HIGH)
                print(f"   ⚡ HIGH - Verifique se {name.lower()} ativou")
                time.sleep(1)
                
                # Desligar
                GPIO.output(pin, GPIO.LOW)
                print(f"   ⚫ LOW - Verifique se {name.lower()} desativou")
                time.sleep(0.5)
                
                # Perguntar ao utilizador
                response = input(f"   ✅ {name} funcionou corretamente? (y/n): ").lower().strip()
                if response == 'y':
                    print(f"   ✅ {name} - PASSOU")
                else:
                    print(f"   ❌ {name} - FALHOU")
                    
            except Exception as e:
                print(f"   ❌ Erro ao testar {name}: {e}")
            
            print()
    
    def test_inputs(self):
        """Testar todos os pinos de entrada"""
        print("📥 Testando Entradas (Inputs)...")
        print("-" * 30)
        
        for name, pin in self.input_pins.items():
            try:
                print(f"🧪 Testando {name} (Pino {pin})...")
                
                # Ler estado atual
                current_state = GPIO.input(pin)
                print(f"   📊 Estado atual: {current_state}")
                
                if 'Sensor Linha' in name:
                    print(f"   🛤️  Coloque uma linha preta sob o sensor e pressione Enter...")
                    input("   ")
                    line_state = GPIO.input(pin)
                    print(f"   📊 Com linha: {line_state}")
                    
                    print(f"   🛤️  Remova a linha e pressione Enter...")
                    input("   ")
                    no_line_state = GPIO.input(pin)
                    print(f"   📊 Sem linha: {no_line_state}")
                    
                    if line_state != no_line_state:
                        print(f"   ✅ {name} - PASSOU (detecta mudanças)")
                    else:
                        print(f"   ❌ {name} - FALHOU (não detecta mudanças)")
                
                elif 'Ultrassónico' in name:
                    print(f"   📏 Sensor ultrassónico - mova objetos e observe mudanças")
                    for i in range(5):
                        state = GPIO.input(pin)
                        print(f"   📊 Leitura {i+1}: {state}")
                        time.sleep(0.5)
                
                else:
                    print(f"   📊 Estado lido: {current_state}")
                    print(f"   ✅ {name} - Leitura OK")
                    
            except Exception as e:
                print(f"   ❌ Erro ao testar {name}: {e}")
            
            print()
    
    def test_pwm(self):
        """Testar PWM dos motores"""
        print("⚡ Testando PWM dos Motores...")
        print("-" * 30)
        
        try:
            # Configurar PWM
            left_pwm = GPIO.PWM(TractorPins.MOTOR_LEFT_PWM, 1000)  # 1kHz
            right_pwm = GPIO.PWM(TractorPins.MOTOR_RIGHT_PWM, 1000)
            
            left_pwm.start(0)
            right_pwm.start(0)
            
            print("🧪 Testando PWM Motor Esquerdo...")
            for duty in [25, 50, 75, 100]:
                left_pwm.ChangeDutyCycle(duty)
                print(f"   ⚡ PWM {duty}% - Verifique velocidade do motor")
                time.sleep(2)
            
            left_pwm.ChangeDutyCycle(0)
            
            print("🧪 Testando PWM Motor Direito...")
            for duty in [25, 50, 75, 100]:
                right_pwm.ChangeDutyCycle(duty)
                print(f"   ⚡ PWM {duty}% - Verifique velocidade do motor")
                time.sleep(2)
            
            right_pwm.ChangeDutyCycle(0)
            
            left_pwm.stop()
            right_pwm.stop()
            
            print("✅ Teste PWM concluído")
            
        except Exception as e:
            print(f"❌ Erro no teste PWM: {e}")
        
        print()
    
    def test_servo(self):
        """Testar servo motor"""
        print("🔄 Testando Servo Motor...")
        print("-" * 30)
        
        try:
            servo_pwm = GPIO.PWM(TractorPins.SERVO_MOTOR, 50)  # 50Hz para servo
            servo_pwm.start(0)
            
            angles = [0, 45, 90, 135, 180]
            
            for angle in angles:
                duty_cycle = 2 + (angle / 18)
                servo_pwm.ChangeDutyCycle(duty_cycle)
                print(f"🔄 Servo em {angle}° - Verifique posição")
                time.sleep(2)
                servo_pwm.ChangeDutyCycle(0)  # Parar sinal
                time.sleep(0.5)
            
            servo_pwm.stop()
            print("✅ Teste servo concluído")
            
        except Exception as e:
            print(f"❌ Erro no teste servo: {e}")
        
        print()
    
    def test_ultrasonic(self):
        """Testar sensor ultrassónico"""
        print("📏 Testando Sensor Ultrassónico...")
        print("-" * 30)
        
        try:
            for i in range(5):
                # Enviar trigger
                GPIO.output(TractorPins.TRIGGER, True)
                time.sleep(0.00001)
                GPIO.output(TractorPins.TRIGGER, False)
                
                # Medir tempo de echo
                start_time = time.time()
                stop_time = time.time()
                
                # Aguardar início do echo
                while GPIO.input(TractorPins.ECHO) == 0:
                    start_time = time.time()
                
                # Aguardar fim do echo
                while GPIO.input(TractorPins.ECHO) == 1:
                    stop_time = time.time()
                
                # Calcular distância
                time_elapsed = stop_time - start_time
                distance = (time_elapsed * 34300) / 2
                
                print(f"📏 Leitura {i+1}: {distance:.2f} cm")
                time.sleep(0.5)
            
            print("✅ Teste ultrassónico concluído")
            
        except Exception as e:
            print(f"❌ Erro no teste ultrassónico: {e}")
        
        print()
    
    def run_full_test(self):
        """Executar teste completo"""
        print("🚀 Iniciando Teste Completo do Hardware")
        print("=" * 50)
        
        try:
            # Menu de opções
            while True:
                print("\n📋 Menu de Testes:")
                print("1. Testar Saídas (Outputs)")
                print("2. Testar Entradas (Inputs)")
                print("3. Testar PWM dos Motores")
                print("4. Testar Servo Motor")
                print("5. Testar Sensor Ultrassónico")
                print("6. Teste Rápido de Todas as Saídas")
                print("7. Teste Completo")
                print("0. Sair")
                
                choice = input("\nEscolha uma opção: ").strip()
                
                if choice == '1':
                    self.test_outputs()
                elif choice == '2':
                    self.test_inputs()
                elif choice == '3':
                    self.test_pwm()
                elif choice == '4':
                    self.test_servo()
                elif choice == '5':
                    self.test_ultrasonic()
                elif choice == '6':
                    self.quick_output_test()
                elif choice == '7':
                    self.test_outputs()
                    self.test_inputs()
                    self.test_pwm()
                    self.test_servo()
                    self.test_ultrasonic()
                elif choice == '0':
                    break
                else:
                    print("❌ Opção inválida!")
        
        except KeyboardInterrupt:
            print("\n🛑 Teste interrompido pelo utilizador")
        
        finally:
            self.cleanup()
    
    def quick_output_test(self):
        """Teste rápido de todas as saídas"""
        print("⚡ Teste Rápido - Todas as Saídas...")
        print("-" * 30)
        
        for name, pin in self.output_pins.items():
            try:
                print(f"⚡ {name}")
                GPIO.output(pin, GPIO.HIGH)
                time.sleep(0.3)
                GPIO.output(pin, GPIO.LOW)
                time.sleep(0.1)
            except Exception as e:
                print(f"❌ Erro em {name}: {e}")
        
        print("✅ Teste rápido concluído")
    
    def cleanup(self):
        """Limpar configurações GPIO"""
        print("\n🧹 Limpando configurações GPIO...")
        
        # Desligar todas as saídas
        for pin in self.output_pins.values():
            try:
                GPIO.output(pin, GPIO.LOW)
            except:
                pass
        
        GPIO.cleanup()
        print("✅ Cleanup concluído")
        print("👋 Obrigado por usar o teste de hardware!")

def main():
    """Função principal"""
    if len(sys.argv) > 1 and sys.argv[1] == '--quick':
        # Teste rápido via linha de comando
        tester = HardwareTester()
        tester.quick_output_test()
        tester.cleanup()
    else:
        # Teste interativo
        tester = HardwareTester()
        tester.run_full_test()

if __name__ == "__main__":
    main()