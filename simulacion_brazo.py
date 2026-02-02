import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# ================= CONFIGURACIÓN (Como en tu main.m y config.py) =================

# Longitudes del robot (L) - Basado en  archivo calcular_fk.m
L = np.array([12.5, 12.0, 8.5, 3.0, 4.0, 9.5])

# LÍMITES DE LOS SERVOS (Radianes)
# He extraído estos valores de tu archivo 'calcular_ik.m'.
# Asegúrate de que coincidan con tu hardware real.
SERVO_LIMITS = {
    # [Minimo, Maximo]
    0: np.deg2rad([-180, 180]), # Base
    1: np.deg2rad([90, 180]),   # Hombro
    2: np.deg2rad([-90, 10]),   # Codo
    3: np.deg2rad([179, 181]),  # Antebrazo (Fijo en código)
    4: np.deg2rad([90, 180]),   # Muñeca
    5: np.deg2rad([0, 70])      # Gripper
}

# Configuración de simulación
ERROR_TOLERANCIA = 0.2  # cm
MAX_ITERACIONES = 150
VALOR_Q4_FIJO = np.deg2rad(180) # Tu restricción mecánica

# ================= MATEMÁTICA (Traducción de tus archivos .m) =================

def matriz_dh(theta, d, a, alpha):
    """Traducción de matriz_dh.m"""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    return np.array([
        [ct, -ca*st,  sa*st, a*ct],
        [st,  ca*ct, -sa*ct, a*st],
        [0,   sa,     ca,    d],
        [0,   0,      0,     1]
    ])

def calcular_fk(L, q):
    """Traducción de calcular_fk.m"""
    # Desempaquetar longitudes
    L1, L2, L3, L4, L5, L6 = L
    
    # Parámetros DH (theta, d, a, alpha)
    DH = np.array([
        [q[0], L1,      0,   np.pi/2],
        [q[1], 0,       L2,  0],
        [q[2], 0,       0,  -np.pi/2],
        [q[3], L3+L4,   0,   np.pi/2],
        [q[4], 0,       0,   np.pi/2],
        [q[5], L5+L6,   0,   0]
    ])
    
    T_acumulada = np.eye(4)
    Puntos = []
    Puntos.append(np.array([0, 0, 0])) # P0 (Base)
    
    for i in range(6):
        th, d, a, alp = DH[i]
        H = matriz_dh(th, d, a, alp)
        T_acumulada = np.dot(T_acumulada, H)
        # Extraer posición (Columna 3, filas 0:3)
        Puntos.append(T_acumulada[0:3, 3])
        
    return Puntos, T_acumulada

def calcular_jacobiano(L, q):
    """Método de perturbación numérica (igual que en tu script MATLAB)"""
    delta = 0.0001
    J = np.zeros((3, 6))
    
    # Posición actual
    Puntos, _ = calcular_fk(L, q)
    P0 = Puntos[-1] # Último punto (End Effector)
    
    for i in range(6):
        q_temp = q.copy()
        q_temp[i] += delta
        Puntos_temp, _ = calcular_fk(L, q_temp)
        P_nueva = Puntos_temp[-1]
        
        J[:, i] = (P_nueva - P0) / delta
        
    return J

def calcular_ik(L, q_inicial, P_objetivo):
    """
    Traducción de calcular_ik.m
    Cinemática Inversa Iterativa con Jacobiano Inverso y Límites
    """
    q = q_inicial.copy()
    
    # Arrays de límites para facilitar el 'clip' de numpy
    lim_min = np.array([SERVO_LIMITS[i][0] for i in range(6)])
    lim_max = np.array([SERVO_LIMITS[i][1] for i in range(6)])

    print(f"🔄 Iniciando cálculo IK para llegar a: {P_objetivo}")

    for k in range(MAX_ITERACIONES):
        # 1. FK Actual
        Puntos, _ = calcular_fk(L, q)
        P_actual = Puntos[-1]
        
        # 2. Calcular Error
        error_pos = P_objetivo - P_actual
        norma_error = np.linalg.norm(error_pos)
        
        if norma_error < ERROR_TOLERANCIA:
            print(f"✅ Convergencia alcanzada en iteración {k}. Error: {norma_error:.4f}")
            break
            
        # 3. Jacobiano e Inversa (Pseudoinversa de Moore-Penrose)
        J = calcular_jacobiano(L, q)
        delta_q = np.dot(np.linalg.pinv(J), error_pos)
        
        # 4. Bloqueo de q4 (Eslabón Fijo) #Fijo debido a que no usamos 
        delta_q[3] = 0 
        
        # 5. Actualizar q (con factor de aprendizaje 0.2 para suavidad)
        q = q + delta_q * 0.2
        
        # 6. Aplicar Jaula de Límites (Clamping)
        q = np.clip(q, lim_min, lim_max)
        
        # Reforzar q4 fijo manualmente por seguridad
        q[3] = VALOR_Q4_FIJO
        
    return q

# ================= VISUALIZACIÓN =================

def dibujar_robot(L, q, objetivo=None):
    Puntos, _ = calcular_fk(L, q)
    
    # Convertir lista de arrays a arrays separados para ploteo
    X = [p[0] for p in Puntos]
    Y = [p[1] for p in Puntos]
    Z = [p[2] for p in Puntos]
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Dibujar eslabones (Líneas grises gruesas)
    ax.plot(X, Y, Z, linewidth=5, color='gray', marker='o', markersize=8, markerfacecolor='blue')
    
    # Dibujar End Effector (Punta roja)
    ax.scatter(X[-1], Y[-1], Z[-1], s=100, c='red', label='End Effector')
    
    # Dibujar Objetivo si existe
    if objetivo is not None:
        ax.scatter(objetivo[0], objetivo[1], objetivo[2], s=100, c='green', marker='^', label='Objetivo')
        # Línea punteada del final al objetivo
        ax.plot([X[-1], objetivo[0]], [Y[-1], objetivo[1]], [Z[-1], objetivo[2]], 'g--', alpha=0.5)

    # Configuración del gráfico
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.set_zlabel('Z (cm)')
    ax.set_title(f'Simulación Robot 6GDL\nÁngulos: {np.degrees(q).astype(int)}')
    
    # Límites fijos para que no se deforme la visualización
    ax.set_xlim([-30, 30])
    ax.set_ylim([-30, 30])
    ax.set_zlim([0, 40])
    ax.legend()
    
    plt.show()

# ================= MAIN (Ejecución) =================

if __name__ == "__main__":
    # --- 1. Definir Posición Inicial (HOME) ---
    # Convertimos los grados de tu código a radianes
    q_actual = np.radians([0, 90, 0, 180, 90, 0])
    
    # --- 2. Definir OBJETIVO ---
    # Modifica esto para probar diferentes coordenadas XYZ
    Objetivo_XYZ = np.array([-15, 10, 5]) 
    
    # --- 3. Calcular Cinemática Inversa ---
    q_calculado = calcular_ik(L, q_actual, Objetivo_XYZ)
    
    # --- 4. Reporte ---
    Puntos_finales, _ = calcular_fk(L, q_calculado)
    pos_final = Puntos_finales[-1]
    
    print("-" * 40)
    print(f"📍 Objetivo Deseado: {Objetivo_XYZ}")
    print(f"🤖 Posición Lograda: {np.round(pos_final, 2)}")
    print(f"📐 Ángulos Resultantes (Grados):")
    print(np.degrees(q_calculado).astype(int))
    print("-" * 40)
    
    # --- 5. Visualizar ---
    dibujar_robot(L, q_calculado, Objetivo_XYZ)