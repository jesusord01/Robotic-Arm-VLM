# config.py

# Puerto serial (Configuración)
PORT = "COM8"  # En Linux puede ser "/dev/ttyUSB0" o similar
BAUDRATE = 115200

# ---------- LIMITES DE CADA SERVO (GRADOS) ----------
# Estos se usan para proteger los motores de choques
SERVO_LIMITS = {
    "base":     (0, 180),
    "shoulder": (0, 150),
    "elbow":    (0, 120), #0 80
    "wrist":    (0, 90),
    "gripper":  (40, 85)
}

# -------- POSICION DE BÚSQUEDA (HOME) ---------------
# Posición inicial al encender o tras soltar un objeto
HOME_POSITION = {
    "base": 90,
    "shoulder": 90,
    "elbow": 0,
    "wrist": 0,
    "gripper": 85
}

# -------- ID DE CADA SERVO EN ARDUINO ---------------
# Mapeo de nombres a números de Pin/ID
SERVO_IDS = {
    "base": 1,
    "shoulder": 2,
    "elbow": 3,
    "wrist": 4,
    "gripper": 5
}