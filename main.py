from ultralytics import YOLO
import cv2
import numpy as np
import time
from arm_controller import ArmController
import kinematics_lib as kine

# ================= NUEVO: IMPORTAMOS MATPLOTLIB =================
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ================= CLASE PARA VISUALIZACIÓN 3D =================
class VisualizadorRobot:
    def __init__(self):
        # Configuramos la figura en modo interactivo
        plt.ion()
        self.fig = plt.figure(figsize=(5, 5))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_title("Cinemática en Tiempo Real")
        
        # Ajusta estos límites visuales según el tamaño de tu robot (en cm)
        self.limite_espacio = 40 

    def actualizar(self, q):
        # Usamos librería para calcular dónde están los puntos
        puntos = kine.calcular_fk(q)
        
        # Separamos X, Y, Z
        xs = [p[0] for p in puntos]
        ys = [p[1] for p in puntos]
        zs = [p[2] for p in puntos]

        self.ax.cla() # Limpiar gráfico anterior
        
        # Dibujamos el esqueleto (Línea azul con puntos rojos en las uniones)
        self.ax.plot(xs, ys, zs, '-o', linewidth=4, markersize=6, color='blue', markeredgecolor='red')
        
        # Dibujar la base (punto 0,0,0) para referencia
        self.ax.scatter([0], [0], [0], color='black', s=50, label='Base')

        # Fijamos los límites para que el gráfico no "salte"
        self.ax.set_xlim(-self.limite_espacio, self.limite_espacio)
        self.ax.set_ylim(0, self.limite_espacio + 10) # Asumiendo que trabaja hacia adelante
        self.ax.set_zlim(0, self.limite_espacio)
        
        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Y (cm)')
        self.ax.set_zlabel('Z (cm)')
        
        # Pausa breve para permitir que se dibuje sin bloquear OpenCV
        plt.draw()
        plt.pause(0.001)

# ===================== 1. CONFIGURACIÓN =====================
CALIBRACION = {
    "base":     {"id": 1, "offset": 90,   "dir": 1},
    "shoulder": {"id": 2, "offset": 170,  "dir": -1},
    "elbow":    {"id": 3, "offset": 174,  "dir": 1},
    "wrist":    {"id": 4, "offset": -100, "dir": -1},
    "gripper":  {"id": 5, "offset": 0,    "dir": 1}
}

Q_HOME_RADS = np.deg2rad([0, 80, -174, 180, -100, 0])
q_actual = Q_HOME_RADS.copy() 

# ===================== 2. MAPEO VISIÓN =====================
def pixel_a_coords_reales(pixel_y):
    P_MIN, P_MAX = 90, 360  
    DIST_LEJOS = 23.0    
    DIST_CERCA = 12.0 
    
    factor = (pixel_y - P_MIN) / (P_MAX - P_MIN)
    distancia_y = DIST_LEJOS + factor * (DIST_CERCA - DIST_LEJOS)
    return np.array([0, distancia_y, 0.0])

# ===================== 3. FUNCIONES DE MOVIMIENTO =====================

def ir_a_home_seguro(arm):
    global q_actual
    print("🏠 Yendo a HOME (Modo Seguro)...")
    try:
        arm.ser.reset_output_buffer()
        time.sleep(0.1)
    except: pass

    q_actual = Q_HOME_RADS.copy()
    
    # Movimiento físico
    arm.ser.write(f"{CALIBRACION['gripper']['id']},85\n".encode()); time.sleep(0.1)
    arm.ser.write(f"{CALIBRACION['elbow']['id']},0\n".encode()); time.sleep(0.1)
    arm.ser.write(f"{CALIBRACION['shoulder']['id']},90\n".encode()); time.sleep(0.1)
    arm.ser.write(f"{CALIBRACION['wrist']['id']},0\n".encode()); time.sleep(0.1)
    arm.base_angle = 90 
    arm.ser.write(f"{CALIBRACION['base']['id']},90\n".encode()); time.sleep(0.5)
    
    print("✅ En HOME.")

def mover_brazo_suave_ik(arm, objetivo_xyz):
    global q_actual
    
    # 1. Calculamos IK
    q_nuevo = kine.calcular_ik(q_actual, objetivo_xyz)
    q_actual = q_nuevo
    
    # 2. Preparamos smooth_move
    math_deg = np.degrees(q_actual)
    objetivos = {}
    mapping = [("shoulder", 1), ("elbow", 2), ("wrist", 4)]
    
    for nombre, idx in mapping:
        cfg = CALIBRACION[nombre]
        val_fisico = int(max(0, min(180, (math_deg[idx] / cfg['dir']) + cfg['offset'])))
        objetivos[nombre] = val_fisico

    try: arm.ser.reset_output_buffer()
    except: pass
    
    print(f"🐌 Movimiento suave a {objetivo_xyz}...")
    arm.smooth_move(objetivos, duration=1.5)
    
    return True

# ===================== 4. INIT =====================
class Stabilizer:
    def __init__(self, alpha=0.2): self.alpha=alpha; self.value=None
    def update(self, n): 
        self.value = n if self.value is None else (self.alpha*n)+((1-self.alpha)*self.value)
        return int(self.value)

print("🔌 Conectando...")
try:
    arm = ArmController("COM8", 115200)
    arm.ser.write_timeout = 2 
except Exception as e:
    print(f"Error conectando: {e}")
    exit()

# === INICIALIZAR VISUALIZADOR ===
viz = VisualizadorRobot() 
# ================================

model = YOLO("best.pt")
stab_x, stab_y = Stabilizer(0.3), Stabilizer(0.3)
cap = cv2.VideoCapture(0) 

BASE_MIN, BASE_MAX = 50, 160
KP, DEAD_ZONE, MAX_STEP = 0.3, 15, 0.3
DIR_CORRECCION = -1

mode, search_dir = "search", 1
base_locked, start_time = False, None
last_seen_time = time.time()

def smart_sleep(sec):
    s = time.time()
    while time.time()-s < sec:
        # ACTUALIZAR GRAFICO MIENTRAS ESPERAMOS
        viz.actualizar(q_actual) 
        if cv2.waitKey(1)&0xFF in [ord('e'),ord('q')]: return True
    return False

# ===================== 5. BUCLE PRINCIPAL =====================
print("✅ SISTEMA LISTO")
ir_a_home_seguro(arm) 

while True:
    # === ACTUALIZAR VISUALIZACIÓN EN CADA FRAME ===
    viz.actualizar(q_actual) 
    # ==============================================

    ret, frame = cap.read()
    if not ret: break
    h, w, _ = frame.shape
    
    results = model(frame, verbose=False, stream=True)
    best_box, max_conf = None, 0

    for r in results:
        for box in r.boxes:
            if box.conf[0] > max_conf: max_conf = box.conf[0]; best_box = box

    if best_box:
        last_seen_time = time.time()
        
        x1, y1, x2, y2 = map(int, best_box.xyxy[0])
        cx, cy = stab_x.update((x1+x2)//2), stab_y.update((y1+y2)//2)
        error = cx - (w//2)

        cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
        cv2.putText(frame, f"Y: {cy}", (x1,y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0),2)

        if mode == "search": mode = "track"

        if mode == "track":
            if not base_locked: 
                # FASE 1: ALINEAR BASE
                if abs(error) > DEAD_ZONE:
                    start_time = None
                    step = np.clip(int(round(error*KP*DIR_CORRECCION)), -MAX_STEP, MAX_STEP)
                    if step == 0: step = 1 if error*DIR_CORRECCION > 0 else -1
                    
                    new_base = np.clip(arm.base_angle+step, BASE_MIN, BASE_MAX)
                    arm.base_angle = new_base
                    arm.ser.write(f"{CALIBRACION['base']['id']},{int(arm.base_angle)}\n".encode())
                    
                    # Sincronizamos matemática PARA QUE EL GRÁFICO SE MUEVA
                    q_actual[0] = np.deg2rad(new_base - 90)
                    
                    cv2.putText(frame, f"ERR: {error}", (10,80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,165,255), 2)
                else:
                    if start_time is None: start_time = time.time()
                    if time.time()-start_time >= 1.5: base_locked = True
                    else: cv2.putText(frame, "LOCKING...", (10,80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            else: 
                # FASE 2: AGARRAR
                try:
                    print(f"🏁 INICIANDO AGARRE (Y: {cy})")
                    target = pixel_a_coords_reales(cy)
                    
                    arm.ser.write(f"{CALIBRACION['gripper']['id']},85\n".encode()); smart_sleep(0.3)
                    
                    # Al llamar a esto, q_actual se actualiza y la siguiente vuelta del bucle actualizará el gráfico
                    mover_brazo_suave_ik(arm, target)
                    if smart_sleep(1.0): raise KeyboardInterrupt
                    
                    arm.ser.write(f"{CALIBRACION['gripper']['id']},55\n".encode()); smart_sleep(0.5)
                    
                    mover_brazo_suave_ik(arm, [0, 20, 20]) 
                    if smart_sleep(1.0): raise KeyboardInterrupt
                    
                    arm.ser.write(f"{CALIBRACION['gripper']['id']},85\n".encode()); smart_sleep(0.5)
                    
                    ir_a_home_seguro(arm) 
                    
                    base_locked = False
                    start_time = None
                    mode = "search"

                except KeyboardInterrupt:
                    ir_a_home_seguro(arm)
                    break

    # --- LÓGICA DE BÚSQUEDA ---
    if not best_box or (time.time() - last_seen_time > 1.0):
        mode = "search"
        base_locked = False
        
        cv2.putText(frame, "BUSCANDO...", (10,80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
        
        arm.base_angle += 1 * search_dir
        if arm.base_angle >= BASE_MAX: search_dir = -1
        elif arm.base_angle <= BASE_MIN: search_dir = 1
        
        arm.ser.write(f"{CALIBRACION['base']['id']},{int(arm.base_angle)}\n".encode())
        # Actualizamos q_actual para el gráfico
        q_actual[0] = np.deg2rad(arm.base_angle - 90)

    cv2.imshow("Vision", frame)
    if cv2.waitKey(1) & 0xFF in [ord('q'), ord('e')]: break

arm.close()
cap.release()
cv2.destroyAllWindows()
plt.close() # Cerrar gráfico al salir