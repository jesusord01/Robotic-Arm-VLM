### Archivo: arm_controller.py (LIMPIO PARA IK)

import serial
import time
import config

class ArmController:

    def __init__(self, port, baudrate):
        print("[INFO] Conectando al Arduino...")
        try:
            # write_timeout=2 es vital para que no se bloquee si enviamos muchos datos
            self.ser = serial.Serial(port, baudrate, timeout=1, write_timeout=2)
            time.sleep(2) 
        except Exception as e:
            print(f"[ERROR] No se pudo conectar: {e}")
            raise e

        # Estado inicial
        self.current_angles = config.HOME_POSITION.copy()
        self.base_angle = self.current_angles["base"]
        print("[OK] Conexión establecida")
        self.home()

    def close(self):
        self.ser.close()

    def clamp(self, name, angle):
        """Mantiene los ángulos dentro de los límites seguros definidos en config.py"""
        min_angle, max_angle = config.SERVO_LIMITS[name]
        return max(min(angle, max_angle), min_angle)

    # ================= MOVIMIENTO SUAVE (INTERPOLACIÓN) =================
    # Esta es la función clave que usa la Cinemática Inversa para no ser brusca
    
    def _ease_in_out(self, t):
        # Curva Quintic: Aceleración y frenado ultra suaves
        return 10 * t**3 - 15 * t**4 + 6 * t**5
    
    def smooth_move(self, targets, duration=1.5):
        """
        Mueve varios servos simultáneamente de forma interpolada.
        Recibe: diccionario {nombre_servo: angulo_objetivo}
        """
        start_angles = {}
        changes = {}
        
        # 1. Planificar movimiento
        for name, target_angle in targets.items():
            target_angle = self.clamp(name, target_angle)
            start_angle = self.current_angles.get(name, target_angle)
            start_angles[name] = start_angle
            changes[name] = target_angle - start_angle

        start_time = time.time()
        
        # 2. Ejecutar animación
        while True:
            elapsed = time.time() - start_time
            progress = elapsed / duration
            if progress >= 1.0: progress = 1.0; break 

            ease = self._ease_in_out(progress)
            
            # Enviar pasos intermedios
            for name, change in changes.items():
                current_target = start_angles[name] + (change * ease)
                servo_id = config.SERVO_IDS[name]
                try:
                    self.ser.write(f"{servo_id},{int(current_target)}\n".encode())
                except:
                    pass # Si el Arduino se satura, saltamos este micro-paso
            
            # Pausa para no saturar al Arduino (50ms es seguro)
            time.sleep(0.05) 

        # 3. Asegurar posición final exacta
        for name, target_angle in targets.items():
            target_angle = self.clamp(name, target_angle)
            servo_id = config.SERVO_IDS[name]
            try:
                self.ser.write(f"{servo_id},{int(target_angle)}\n".encode())
            except:
                pass 
            
            # Actualizar memoria interna
            self.current_angles[name] = target_angle
            if name == "base": self.base_angle = target_angle
            
            time.sleep(0.02)

    def move(self, name, target, delay=0.0):
        """Mueve un solo servo (útil para pruebas manuales)"""
        target = self.clamp(name, target)
        
        if name == "gripper":
            # La garra se envía varias veces para asegurar fuerza de agarre
            servo_id = config.SERVO_IDS[name]
            cmd = f"{servo_id},{int(target)}\n"
            for _ in range(3): 
                try: self.ser.write(cmd.encode())
                except: pass
                time.sleep(0.05)
            self.current_angles[name] = target
            time.sleep(delay if delay > 0 else 0.5)
        else:
            # Los demás usan movimiento suave por defecto
            self.smooth_move({name: target}, duration=0.5)

    def home(self):
        """Regresa a la posición definida en config.py"""
        print("[INFO] Yendo a HOME...")
        try: self.ser.reset_output_buffer()
        except: pass
        self.smooth_move(config.HOME_POSITION, duration=2.0)