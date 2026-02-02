#  Brazo Robótico 5-DOF con Visión Artificial y Cinemática Inversa

Este proyecto consiste en el diseño y control de un brazo robótico de 5 grados de libertad (DOF) capaz de identificar, rastrear y manipular objetos de forma autónoma utilizando Inteligencia Artificial.

El sistema integra un "Gemelo Digital" (Digital Twin) que visualiza la posición del robot en tiempo real mediante cálculo matemático, sin necesidad de sensores externos en las articulaciones.



##  Características Principales

* **Visión Artificial (YOLOv8):** Detección y clasificación de objetos en tiempo real con corrección de perspectiva.
* **Cinemática Inversa (IK):** Implementación matemática propia usando el método del **Jacobiano Pseudo-Inverso** para calcular los ángulos de las articulaciones basándose en coordenadas (X, Y, Z).
* **Control de Movimiento Suave:** Interpolación de trayectoria usando curvas de quinto grado (Quintic Easing) para evitar movimientos bruscos y proteger los servos.
* **Visualización en Tiempo Real:** Un entorno 3D (Matplotlib) que simula el movimiento físico del robot simultáneamente.

##  Tecnologías Utilizadas

### Hardware
* **Microcontrolador:** Arduino Mega 2560
* **Actuadores:** Servomotores (Configuración 5 DOF + Gripper)
* **Sensores:** Webcam (Visión por Computadora)
* **Estructura:** Diseño impreso en 3D (PLA)

### Software
* **Lenguaje:** Python 3.9+
* **Librerías Clave:**
    * `OpenCV` & `Ultralytics (YOLO)`: Procesamiento de imagen.
    * `NumPy`: Cálculos matriciales para la cinemática (Matrices Denavit-Hartenberg).
    * `PySerial`: Comunicación UART de alta velocidad con el microcontrolador.
    * `Matplotlib`: Renderizado 3D del esqueleto del robot.

##  ¿Cómo funciona?

1.  **Detección:** La cámara captura el entorno y el modelo YOLO identifica el objeto objetivo.
2.  **Mapeo:** Se convierten las coordenadas 2D (píxeles) a coordenadas 3D (mundo real) estimando la profundidad.
3.  **Cálculo IK:** El algoritmo de Cinemática Inversa calcula iterativamente los ángulos necesarios para que el "end-effector" (la garra) llegue al objeto.
4.  **Ejecución:** Se envían los comandos al Arduino, que gestiona los servos con aceleración controlada.

##  Estructura del Código

* `main.py`: Bucle principal, visión por computadora y lógica de decisión.
* `kinematics_lib.py`: El "cerebro matemático". Contiene las fórmulas de Cinemática Directa (FK) e Inversa (IK).
* `arm_controller.py`: Driver de comunicación con el Arduino y generador de trayectorias suaves.
* `config.py`: Parámetros físicos del robot, límites de seguridad y calibración.

---
*Desarrollado por Edin Jesus Ordoñez Diaz - Estudiante de Ingeniería Mecatrónica UTP en conjunto con el grupo Crisálida* 
