import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import kinematics_lib as kine
from arm_controller import ArmController

# ================= CONFIGURACIÓN =================
PUERTO = "COM8"
BAUD = 115200

# Parámetros de Calibración solo para el sentido y angulo, omitir el simulado (Mapping Servo <-> Math)
AJUSTES = {
    "base":     {"id": 1, "offset": 90,  "dir": 1,  "idx": 0}, 
    "hombro":   {"id": 2, "offset": 180, "dir": -1, "idx": 1}, 
    "codo":     {"id": 3, "offset": 90,  "dir": 1,  "idx": 2}, 
    "muneca":   {"id": 4, "offset": -90, "dir": -1, "idx": 4}, 
    "garra":    {"id": 5, "offset": 0,   "dir": 1,  "idx": 5}  
}

# ================= CONEXIÓN =================
print(f"🔌 Conectando a {PUERTO}...")
try:
    arm = ArmController(PUERTO, BAUD)
    print("✅ Conexión establecida.")
except Exception as e:
    arm = None
    print(f"⚠️ Modo Simulación (Error: {e})")

# ================= VISUALIZACIÓN =================
plt.ion()
fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111, projection='3d')

def dibujar_brazo(q_rads):
    ax.cla()
    puntos = kine.calcular_fk(q_rads)
    X = [p[0] for p in puntos]; Y = [p[1] for p in puntos]; Z = [p[2] for p in puntos]
    
    # Dibujo del esqueleto
    ax.plot(X, Y, Z, 'o-', linewidth=5, markersize=8, color='#007acc')
    ax.scatter(X[0], Y[0], Z[0], s=100, c='black')
    ax.scatter(X[-1], Y[-1], Z[-1], s=120, c='red', marker='^')

    ax.set_xlim([-30, 30]); ax.set_ylim([-30, 30]); ax.set_zlim([0, 45])
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    plt.draw()
    plt.pause(0.001)

def nothing(x): pass

# ================= LÓGICA DE TEST =================
def probar_articulacion(nombre):
    cfg = AJUSTES[nombre]
    win_name = f"TEST: {nombre.upper()}"
    
    cv2.namedWindow(win_name)
    cv2.resizeWindow(win_name, 500, 150)
    cv2.createTrackbar("Servo", win_name, cfg["offset"], 180, nothing)

    print(f"\n--- TEST {nombre.upper()} ---")
    print("Espacio: Volver | Q: Salir")

    while True:
        # 1. Lectura del Slider (Valor Físico)
        val_servo = cv2.getTrackbarPos("Servo", win_name)
        
        # 2. Conversión a Matemático (Radianes)
        deg_math = (val_servo - cfg["offset"]) * cfg["dir"]
        rad_math = np.deg2rad(deg_math)
        
        # 3. Construcción del vector Q (Resto en Home/Neutro)
        q_sim = [0, 0, 0, np.pi, 0, 0]
        if "idx" in cfg: q_sim[cfg["idx"]] = rad_math
        
        # 4. Actualización Visual y Física
        dibujar_brazo(q_sim)
        
        if arm:
            arm.ser.write(f"{cfg['id']},{val_servo}\n".encode())
            time.sleep(0.01)

        # 5. Interfaz Info
        img = np.zeros((150, 500, 3), dtype=np.uint8)
        cv2.putText(img, f"Servo: {val_servo} | Math: {int(deg_math)} deg", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
        cv2.putText(img, f"Cfg: Off={cfg['offset']} Dir={cfg['dir']}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150,150,150), 1)
        cv2.imshow(win_name, img)
        
        k = cv2.waitKey(1)
        if k == 32: cv2.destroyWindow(win_name); return True
        if k == ord('q'): return False

# ================= MAIN LOOP =================
while True:
    print("\n=== MENU DIAGNOSTICO ===")
    print("1. Base\n2. Hombro\n3. Codo\n4. Muñeca\n5. Salir")
    op = input("Opción: ")
    
    continuar = True
    if op == '1': continuar = probar_articulacion("base")
    elif op == '2': continuar = probar_articulacion("hombro")
    elif op == '3': continuar = probar_articulacion("codo")
    elif op == '4': continuar = probar_articulacion("muneca")
    elif op == '5': break
    
    if not continuar: break

if arm: arm.close()
plt.close()
cv2.destroyAllWindows()