import numpy as np

# ================= CONFIGURACIÓN DEL ROBOT =================
# Longitudes en cm
L = np.array([12.5, 12.0, 8.5, 3.0, 4.0, 9.5])

# Límites Matemáticos (Radianes)
# NOTA: Estos son los límites "teóricos" para el cálculo.
# Los hemos ampliado para que el algoritmo encuentre solución siempre.
SERVO_LIMITS = [
    np.deg2rad([-180, 180]), # q1: Base
    np.deg2rad([-180, 180]), # q2: Hombro
    np.deg2rad([-150, 150]), # q3: Codo 
    np.deg2rad([179, 181]),  # q4: Antebrazo (Fijo por el momento)
    np.deg2rad([-180, 180]), # q5: Muñeca
    np.deg2rad([30, 70])      # q6: Gripper
]
#Formula matriz dh, que sirve para calcular la matriz de transformacion de cada eslabon 
def matriz_dh(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -ca*st,  sa*st, a*ct],
        [st,  ca*ct, -sa*ct, a*st],
        [0,   sa,     ca,    d],
        [0,   0,      0,     1]
    ])


def calcular_fk(q):
    """Calcula la posición de todos los eslabones"""
    L1, L2, L3, L4, L5, L6 = L
    
    # Tabla DH qie sirve para calcular la matriz de transformacion de cada eslabon
    DH = np.array([
        [q[0], L1,      0,   np.pi/2],
        [q[1], 0,       L2,  0],
        [q[2], 0,       0,  -np.pi/2],
        [q[3], L3+L4,   0,   np.pi/2],
        [q[4], 0,       0,   np.pi/2],
        [q[5], L5+L6,   0,   0]
    ])
    
    T = np.eye(4)
    Puntos = [np.array([0, 0, 0])]
    
    for i in range(6):
        H = matriz_dh(DH[i,0], DH[i,1], DH[i,2], DH[i,3])
        T = np.dot(T, H)
        Puntos.append(T[0:3, 3])
        
    return Puntos

def calcular_ik(q_actual, objetivo_xyz):
    """Calcula los ángulos necesarios para llegar a XYZ"""
    q = q_actual.copy()
    objetivo = np.array(objetivo_xyz) # Convertir a array numpy
    
    # Parámetros de simulación
    MAX_ITER = 150
    TOLERANCIA = 0.2
    Q4_FIJO = np.deg2rad(180) 

    # Límites para 'clip'
    lim_min = np.array([l[0] for l in SERVO_LIMITS])
    lim_max = np.array([l[1] for l in SERVO_LIMITS])

    for _ in range(MAX_ITER):
        puntos = calcular_fk(q)
        actual = puntos[-1]
        error = objetivo - actual
        
        if np.linalg.norm(error) < TOLERANCIA:
            break
            
        # Jacobiano Numérico
        J = np.zeros((3, 6))
        delta = 0.0001
        for i in range(6):
            q_temp = q.copy()
            q_temp[i] += delta
            p_temp = calcular_fk(q_temp)[-1]
            J[:, i] = (p_temp - actual) / delta
            
        # Actualización (Pseudoinversa + Bloqueo Q4)
        delta_q = np.dot(np.linalg.pinv(J), error)
        delta_q[3] = 0 # Bloquear Q4
        
        q = q + delta_q * 0.2
        q = np.clip(q, lim_min, lim_max) # Respetar límites físicos
        q[3] = Q4_FIJO # Reforzar bloqueo
        
    return q