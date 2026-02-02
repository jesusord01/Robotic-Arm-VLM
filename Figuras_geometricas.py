import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import kinematics_lib as kine
from arm_controller import ArmController 

# ================= CONFIGURACIÓN =================
PUERTO = "COM8"
BAUD = 115200

# AJUSTES DE DIBUJO
Y_PIZARRA = 22.0  
Z_CENTRO  = 20.0  

# --- MEJORA DE FLUIDEZ ---
# Bajamos esto de 1.0 a 0.2. 
# Esto permite que el robot haga micro-correcciones suaves en lugar de esperar a dar un salto grande.
UMB_MOVIMIENTO = 0.1

# --- CALIBRACIÓN (Con el ajuste del offset para evitar el choque en X negativo) ---
CALIBRACION = {
    "base":     {"id": 1, "offset": 70,   "dir": 1}, 
    "shoulder": {"id": 2, "offset": 170,  "dir": -1},
    "elbow":    {"id": 3, "offset": 174,  "dir": 1},
    "wrist":    {"id": 4, "offset": -100, "dir": -1},
    "gripper":  {"id": 5, "offset": 0,    "dir": 1}
}

# ================= CONEXIÓN =================
print("🔌 Conectando...")
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

def actualizar_grafico(q_rads, trayectoria_completa=None):
    ax.cla()
    q_visual = q_rads.copy()
    
    puntos = kine.calcular_fk(q_visual)
    X, Y, Z = zip(*puntos)
    
    # Dibujar Robot
    ax.plot(X, Y, Z, 'o-', linewidth=5, markersize=8, color='#007acc')
    ax.scatter(X[-1], Y[-1], Z[-1], s=120, c='red')
    
    # Dibujar la figura
    if trayectoria_completa is not None:
        tx, ty, tz = zip(*trayectoria_completa)
        ax.plot(tx, ty, tz, 'g--', linewidth=1, alpha=0.5)

    ax.set_xlim([-30, 30]); ax.set_ylim([0, 40]); ax.set_zlim([0, 40])
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    plt.draw()
    plt.pause(0.001)

# ================= GENERADORES DE FIGURAS (ALTA RESOLUCIÓN) =================

def generar_linea(p1, p2, pasos=60):
    return np.linspace(p1, p2, pasos)

def figura_cuadrado_vertical(lado):
    mitad = lado / 2
    p1 = np.array([-mitad, Y_PIZARRA, Z_CENTRO - mitad]) 
    p2 = np.array([-mitad, Y_PIZARRA, Z_CENTRO + mitad]) 
    p3 = np.array([ mitad, Y_PIZARRA, Z_CENTRO + mitad]) 
    p4 = np.array([ mitad, Y_PIZARRA, Z_CENTRO - mitad]) 
    
    trayectoria = []
    trayectoria.extend(generar_linea(p1, p2)) 
    trayectoria.extend(generar_linea(p2, p3)) 
    trayectoria.extend(generar_linea(p3, p4)) 
    trayectoria.extend(generar_linea(p4, p1)) 
    return trayectoria

def figura_triangulo_vertical(lado):
    altura = (np.sqrt(3)/2) * lado
    p1 = np.array([-lado/2, Y_PIZARRA, Z_CENTRO - altura/3])
    p2 = np.array([0,       Y_PIZARRA, Z_CENTRO + 2*altura/3])
    p3 = np.array([lado/2,  Y_PIZARRA, Z_CENTRO - altura/3])
    
    trayectoria = []
    trayectoria.extend(generar_linea(p1, p2))
    trayectoria.extend(generar_linea(p2, p3))
    trayectoria.extend(generar_linea(p3, p1))
    return trayectoria

def figura_circulo_vertical(radio, pasos=180): 
    trayectoria = []
    for i in range(pasos + 1):
        angulo = 2 * np.pi * i / pasos
        x = radio * np.cos(angulo)
        z = Z_CENTRO + radio * np.sin(angulo)
        trayectoria.append(np.array([x, Y_PIZARRA, z]))
    return trayectoria

# ================= FUNCIONES DE CONTROL ROBUSTO =================

def enviar_comando_fisico(q_objetivo_rads, q_anterior_grados):
    if not conectado: return q_anterior_grados

    math_deg = np.degrees(q_objetivo_rads)
    mapping = [("base", 0), ("shoulder", 1), ("elbow", 2), ("wrist", 4)]
    
    nuevos_grados = []
    grados_a_enviar = {}
    
    for nombre, idx in mapping:
        cfg = CALIBRACION[nombre]
        raw_val = (math_deg[idx] / cfg['dir']) + cfg['offset']
        val_limitado = int(max(0, min(180, raw_val)))

        # Solo imprimimos alerta si es grave (fuera de rango > 5 grados)
        if nombre == "base" and (raw_val < -5 or raw_val > 185):
             print(f"⚠️ {nombre} SATURADO: {raw_val:.1f}")

        grados_a_enviar[cfg['id']] = val_limitado
        nuevos_grados.append(val_limitado)

    # Filtrado suavizado
    if q_anterior_grados is not None:
        diff = np.max(np.abs(np.array(nuevos_grados) - np.array(q_anterior_grados)))
        if diff < UMB_MOVIMIENTO:
            return q_anterior_grados 

    # Envío al robot
    for id_servo, angulo in grados_a_enviar.items():
        try:
            robot.ser.write(f"{id_servo},{angulo}\n".encode())
        except:
            pass
    
    return nuevos_grados


# ================= MAIN =================

q_actual = np.deg2rad([0, 20, 20, 180, 0, 0]) 
actualizar_grafico(q_actual)
estado_fisico_anterior = [90, 90, 90, 90] 

print("\n MODO ARTISTA FLUIDO")
print(f"Distancia Pizarra: {Y_PIZARRA}cm | Centro Z: {Z_CENTRO}cm")

if conectado:
    print(" Cerrando garra...")
    id_garra = CALIBRACION["gripper"]["id"]
    robot.ser.write(f"{id_garra},40\n".encode()) 
    time.sleep(1.0) 

    print(" Postura inicial...")
    pos_inicial = np.array([0, Y_PIZARRA, Z_CENTRO])
    try:
        q_actual = kine.calcular_ik(q_actual, pos_inicial)
        estado_fisico_anterior = enviar_comando_fisico(q_actual, [0,0,0,0]) 
    except:
        print("⚠️ No se pudo alcanzar el centro inicial.")
    time.sleep(2.0)

while True:
    print("\n--- MENÚ (TAMAÑOS AUMENTADOS) ---")
    print("1. Cuadrado")
    print("2. Triángulo")
    print("3. Círculo")
    print("4. Salir")
    op = input("Elige: ")
    
    if op == '4': break
    
    trayectoria_xyz = []
    # --- AUMENTAMOS TAMAÑOS AQUÍ ---
    if op == '1': trayectoria_xyz = figura_cuadrado_vertical(lado=15) # Antes 10
    elif op == '2': trayectoria_xyz = figura_triangulo_vertical(lado=15) # Antes 10
    elif op == '3': trayectoria_xyz = figura_circulo_vertical(radio=8)   # Antes 5
    else: continue

    print(f"🔄 Calculando cinemática de alta resolución ({len(trayectoria_xyz)} puntos)...")
    
    ruta_angulos = []
    q_simulado = q_actual.copy()
    error_ik = False

    for punto in trayectoria_xyz:
        try:
            q_next = kine.calcular_ik(q_simulado, punto)
            ruta_angulos.append(q_next)
            q_simulado = q_next
        except:
            print("Punto fuera de alcance, abortando trazo.")
            error_ik = True
            break
    
    if error_ik: continue

    print("▶️ Ejecutando trazo fluido...")
    
    for i, q_target in enumerate(ruta_angulos):
        # Actualizar gráfico cada 5 pasos para no alentar el bucle físico
        if i % 5 == 0: 
            actualizar_grafico(q_target, trayectoria_xyz)
        
        q_actual = q_target
        
        if conectado:
            estado_fisico_anterior = enviar_comando_fisico(q_target, estado_fisico_anterior)
            # Bajamos el tiempo de espera. Cuanto menor sea (sin llegar a 0), más fluido.
            # Si el robot "tiembla", sube esto a 0.03
            time.sleep(0.015) 

print("Fin.")
if conectado: robot.close()
plt.close()