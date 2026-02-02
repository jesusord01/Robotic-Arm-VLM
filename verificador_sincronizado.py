import cv2  # Biblioteca para interfaz gráfica y manejo de ventanas
import numpy as np # Biblioteca para cálculos numéricos
import matplotlib.pyplot as plt  # Biblioteca para gráficos 3D
import time  # Biblioteca para manejo de tiempos y pausas (sleep , para no saturar el puerto serial)
import kinematics_lib as kine  # Biblioteca personalizada de cinemática
from arm_controller import ArmController # Biblioteca personalizada para control del brazo

# =============================================================
# CONFIGURACIÓN DEL SISTEMA
# =============================================================

# Definición de offsets y direcciones para alinear el motor real con la matemática
# Fórmula: Angulo_Math = (Valor_Slider - Offset) * Direccion
CONFIG_SERVOS = {
    "base":     {"id": 1, "offset": 90,  "dir": 1}, #id significa el ID del servo en el controlador
    "shoulder": {"id": 2, "offset": 170, "dir": -1}, # offset significa el valor en el slider que corresponde a 0 radianes en la matemática
    "elbow":    {"id": 3, "offset": 174, "dir": 1},  # 0 en slider = Codo horizontal
    "wrist":    {"id": 4, "offset": -100, "dir": -1}, #dir significa si el movimiento del servo es directo (1) o invertido (-1) respecto a la matemática
    "gripper":  {"id": 5, "offset": 0,   "dir": 1}   
}

PUERTO_SERIAL = "COM8" # Ajustar según el sistema operativo y configuración local , linux: "/dev/ttyUSB0"
BAUD_RATE = 115200 # Velocidad de comunicación serial, si es menos de 115200 puede generar latencia

# =============================================================
# INICIALIZACIÓN DE COMPONENTES
# =============================================================

# Conexión con el controlador del brazo
try:
    arm = ArmController(PUERTO_SERIAL, BAUD_RATE) # Inicializa la comunicación serial
    print("Hardware detectado: Brazo conectado exitosamente.")
except Exception: # Captura cualquier error en la conexión serial
    print("Aviso: Hardware no detectado. Iniciando en MODO SIMULACIÓN.")
    arm = None #Significa que no hay conexión con el brazo físico pero el simulador funcionará

# Configuración estética del gráfico 3D
plt.ion() # Modo interactivo para actualización en tiempo real
fig = plt.figure(figsize=(8, 7)) # Crear figura de matplotlib 
ax = fig.add_subplot(111, projection='3d') # Añadir subplot 3D (111 significa 1 fila, 1 columna, primer subplot), subloat es el área de dibujo

def actualizar_visualizacion(q_rads):
    """Renderiza el estado actual del robot en el espacio 3D.""" 
    ax.cla() # Limpiar el gráfico antes de redibujar
    puntos = kine.calcular_fk(q_rads) # Obtener posiciones de las articulaciones usando cinemática directa
    
    # Extraer coordenadas
    x, y, z = zip(*puntos) #zip es una función que agrupa elementos de listas
    
    # Dibujar estructura (Links y Articulaciones)
    ax.plot(x, y, z, 'o-', linewidth=5, markersize=7, color="#F35E0D", label="Estructura") #Se dibuja la estructura del brazo
    #Linewidth es el grosor de la línea, markersize es el tamaño de los puntos en las articulaciones
    ax.scatter(x[-1], y[-1], z[-1], s=100, c="#000000", label="End-Effector") 
    #ax.scatter dibuja un punto especial para el efector final (la "punta final de la mano" del robot)
    
    # Límites y etiquetas
    ax.set_xlim([-40, 40]); ax.set_ylim([-40, 40]); ax.set_zlim([0, 40])
    ax.set_xlabel('X (cm)'); ax.set_ylabel('Y (cm)'); ax.set_zlabel('Z (cm)')
    ax.set_title("Cinemática: Virtual vs Real", fontsize=12, pad=20)
    #fontsize es el tamaño del título, pad es la distancia del título al gráfico
    plt.draw() # Actualiza el gráfico con los nuevos datos
    plt.pause(0.001) # Pausa breve para permitir la actualización visual

# =============================================================
# INTERFAZ DE CONTROL (GUI)
# =============================================================

def placeholder(x): pass

cv2.namedWindow("Panel de Control") # Crear ventana para los sliders (control de ángulos deslizantes)
cv2.resizeWindow("Panel de Control", 400, 220) # Ajustar tamaño de la ventana de panel

# Creación de Sliders (Trackbars)
cv2.createTrackbar("Base (q1)",   "Panel de Control", 90, 180, placeholder)
cv2.createTrackbar("Hombro (q2)", "Panel de Control", 90, 180, placeholder)
cv2.createTrackbar("Codo (q3)",   "Panel de Control", 0,  180, placeholder) 
cv2.createTrackbar("Muneca (q5)", "Panel de Control", 0,  180, placeholder) 
cv2.createTrackbar("Pinza",       "Panel de Control", 40, 70,  placeholder)

def transform_to_math(val, key): #Para convertir el valor del slider a radianes matemáticos
    """Convierte el valor del slider (0-180) a radianes matemáticos."""
    cfg = CONFIG_SERVOS[key] # Obtener configuración del servo, key es el nombre del servo
    deg = (val - cfg["offset"]) * cfg["dir"] # Aplicar offset y dirección
    return np.deg2rad(deg) # Convertir a radianes

# =============================================================
# BUCLE PRINCIPAL DE EJECUCIÓN
# =============================================================

print("\n Sistema Operativo. Presiona 'q' en la ventana de datos para salir.")

try:
    while True: # Bucle infinito hasta que el usuario decida salir, sirve para actualizar continuamente la interfaz y el robot
        # 1. Captura de valores desde la interfaz
        v_base = cv2.getTrackbarPos("Base (q1)",   "Panel de Control")
        v_shou = cv2.getTrackbarPos("Hombro (q2)", "Panel de Control")
        v_elbo = cv2.getTrackbarPos("Codo (q3)",   "Panel de Control")
        v_wris = cv2.getTrackbarPos("Muneca (q5)", "Panel de Control")
        v_grip = cv2.getTrackbarPos("Pinza",       "Panel de Control")

        # 2. Conversión a espacio de configuración (Radianes)
        q_math = [
            transform_to_math(v_base, "base"), #sirve para convertir el valor del slider a radianes matemáticos
            transform_to_math(v_shou, "shoulder"),
            transform_to_math(v_elbo, "elbow"),
            np.deg2rad(180), # q4 se mantiene constante según diseño
            transform_to_math(v_wris, "wrist"),
            0 # Actuador final
        ]

        # 3. Actualización de Simulador
        actualizar_visualizacion(q_math)
        
        # 4. Sincronización con Robot Físico
        if arm:
            # Enviamos los comandos de forma iterativa y limpia
            movimientos = [
                (CONFIG_SERVOS['base']['id'], v_base),
                (CONFIG_SERVOS['shoulder']['id'], v_shou),
                (CONFIG_SERVOS['elbow']['id'], v_elbo),
                (CONFIG_SERVOS['wrist']['id'], v_wris),
                (CONFIG_SERVOS['gripper']['id'], v_grip)
            ]
            
            for id_servo, angulo in movimientos:
                arm.ser.write(f"{id_servo},{angulo}\n".encode())
            
            time.sleep(0.01) # Pequeño respiro para el buffer serial

        # 5. Visualización de Ventana de Texto
        info_panel = np.zeros((150, 450, 3), dtype=np.uint8) + 30 # Fondo gris oscuro
        fuente = cv2.FONT_HERSHEY_SIMPLEX
        
        #Nos muestra los valores actuales de los servos en la ventana de telemetría
        cv2.putText(info_panel, f"Base: {v_base} | Hombro: {v_shou}", (20, 35), fuente, 0.5, (255, 255, 255), 1) 
        cv2.putText(info_panel, f"Codo (Math): {int(np.degrees(q_math[2]))} deg", (20, 70), fuente, 0.5, (0, 255, 255), 1)
        cv2.putText(info_panel, f"Muneca: {v_wris} | Pinza: {v_grip}", (20, 105), fuente, 0.5, (255, 255, 255), 1)
        cv2.putText(info_panel, "Presione 'q' para salir", (20, 135), fuente, 0.4, (150, 150, 150), 1)
        
        cv2.imshow("Telemetria del Brazo", info_panel)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Cierre seguro de recursos
    if arm: 
        arm.close()
    plt.close()
    cv2.destroyAllWindows()
    print("\n✅ Recursos liberados. Proceso finalizado.")