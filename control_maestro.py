import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import kinematics_lib as kine
from arm_controller import ArmController 

# ================= CONFIGURACIÓN =================
PUERTO = "COM8"
BAUD = 115200

# Calibración: Relación entre Cinemática (Math) y Servos (Físico)
# Formula: Servo = (Math / Dir) + Offset
CALIBRACION = {
    "base":     {"id": 1, "offset": 90,   "dir": 1},
    "shoulder": {"id": 2, "offset": 170,  "dir": -1},
    "elbow":    {"id": 3, "offset": 174,  "dir": 1},
    "wrist":    {"id": 4, "offset": -100, "dir": -1},
    "gripper":  {"id": 5, "offset": 0,    "dir": 1}
}

# ================= CONEXIÓN =================
print("🔌 Iniciando conexión...")
try:
    robot = ArmController(PUERTO, BAUD)
    conectado = True
    print("✅ Robot conectado.")
except Exception as e:
    print(f"⚠️ Modo Simulación (Error: {e})")
    conectado = False

# ================= GRAFICOS =================
plt.ion() 
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

def actualizar_grafico(q_rads, objetivo=None):
    ax.cla()
    
    # --- Parche Visual (Simulación) ---
    # Corrección solo para la vista 3D, no afecta al físico
    q_visual = q_rads.copy()
    OFFSET_VISUAL_MUNECA = -7
    q_visual[4] += np.deg2rad(OFFSET_VISUAL_MUNECA)
    # ----------------------------------

    puntos = kine.calcular_fk(q_visual)
    X = [p[0] for p in puntos]; Y = [p[1] for p in puntos]; Z = [p[2] for p in puntos]
    
    # Dibujar Esqueleto
    ax.plot(X, Y, Z, 'o-', linewidth=5, markersize=8, color='#007acc', label='Robot')
    ax.scatter(X[-1], Y[-1], Z[-1], s=120, c='red', label='Punta')
    ax.scatter(X[0], Y[0], Z[0], s=100, c='black', marker='s')

    if objetivo is not None:
        ax.scatter(objetivo[0], objetivo[1], objetivo[2], s=100, c='green', marker='^', label='Meta')

    ax.set_xlim([-35, 35]); ax.set_ylim([-35, 35]); ax.set_zlim([0, 45])
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.set_title(f"Control Maestro ")
    plt.draw()
    plt.pause(0.01)

# ================= MAIN =================

# Estado Inicial (Home Sincronizado)
q_actual = np.deg2rad([0, 80, -174, 180, -100, 0]) 

print("\n🚀 Sistema Listo. Ingresa coordenadas 'X Y Z' o 'q' para salir.")
actualizar_grafico(q_actual)

while True:
    entrada = input("\n📍 Meta [X Y Z]: ")
    if entrada.lower() == 'q': break
    
    try:
        vals = list(map(float, entrada.split()))
        if len(vals) != 3: continue
        objetivo_xyz = np.array(vals)
        
        # 1. Cinemática Inversa
        q_nuevo = kine.calcular_ik(q_actual, objetivo_xyz)
        q_actual = q_nuevo 
        actualizar_grafico(q_actual, objetivo_xyz)
        
        # 2. Control Físico
        if conectado:
            math_deg = np.degrees(q_actual)
            print("-" * 50)
            print(f"{'JOINT':<10} | {'MATH':<10} | {'SERVO (0-180)'}")
            
            mapping = [("base", 0), ("shoulder", 1), ("elbow", 2), ("wrist", 4)]
            cmds = []

            for nombre, idx in mapping:
                cfg = CALIBRACION[nombre]
                
                # Conversión Math -> Servo + Clamping
                val_math = math_deg[idx]
                val_fisico = int((val_math / cfg['dir']) + cfg['offset'])
                val_fisico = max(0, min(180, val_fisico))
                
                print(f"{nombre:<10} | {int(val_math):<10} | {val_fisico}")
                cmds.append(f"{cfg['id']},{val_fisico}")

            print("-" * 50)

            # Envío de comandos
            for cmd in cmds:
                robot.ser.write((cmd + "\n").encode())
                time.sleep(0.015) 

    except ValueError:
        print("Formato incorrecto.")
    except Exception as e:
        print(f"Error: {e}")

if conectado: robot.close()
plt.close()