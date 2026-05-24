# Controlador en Python para Webots
# Laboratorio 2: Navegación reactiva con filtrado y fusión de sensores

from controller import Robot
import math

# --- Constantes del Robot y Simulación ---
TIME_STEP = 64
WHEEL_RADIUS = 0.0205 # Metros
CRUISE_SPEED = 3.0    # Rad/s
TURN_SPEED = 2.0      # Rad/s

MAX_SENSOR_DIST = 0.07 # Rango máximo confiable del e-puck IR (7 cm)
SAFE_DISTANCE = 0.035  # Umbral de decisión para evadir obstáculos

# --- Parámetros del Filtro de Kalman ---
KALMAN_Q = 0.0001 # Varianza del proceso (confianza en encoders)
KALMAN_R = 0.005  # Varianza de la medición (ruido del sensor)

# Tabla de conversión IR a metros
LOOKUP_TABLE = [
    (0.000, 4095.0), (0.005, 2133.3), (0.010, 1465.7), (0.015, 601.5),
    (0.020, 383.8),  (0.030, 234.9),  (0.040, 158.0),  (0.050, 120.0),
    (0.060, 104.1),  (0.070, 67.2)
]

# --- Funciones Auxiliares ---

def clamp(value, min_val, max_val):
    """Limita un valor dentro de un rango determinado."""
    return max(min_val, min(value, max_val))

def raw_to_meters(raw):
    """Convierte el valor crudo del sensor IR a metros usando interpolación."""
    if raw <= 67.2:
        return MAX_SENSOR_DIST # Sin obstáculo en rango
    if raw >= 4095.0:
        return 0.0
    
    for i in range(len(LOOKUP_TABLE) - 1):
        d1, r1 = LOOKUP_TABLE[i]
        d2, r2 = LOOKUP_TABLE[i+1]
        if r1 >= raw >= r2:
            alpha = (raw - r1) / (r2 - r1)
            return d1 + alpha * (d2 - d1)
            
    return MAX_SENSOR_DIST

# --- Clase del Filtro de Kalman ---

class KalmanFilter:
    def __init__(self, initial_x, initial_p):
        self.x = initial_x
        self.p = initial_p

    def predict(self, delta_s):
        """Etapa de Predicción: El obstáculo se acerca según lo que avanza el robot."""
        self.x = clamp(self.x - delta_s, 0.0, MAX_SENSOR_DIST)
        self.p += KALMAN_Q

    def update(self, measurement):
        """Etapa de Corrección: Ajustar la predicción con la lectura real."""
        k = self.p / (self.p + KALMAN_R) # Ganancia de Kalman
        self.x = self.x + k * (measurement - self.x)
        self.p = (1.0 - k) * self.p


# --- Inicialización del Robot ---
robot = Robot()

# Inicialización de motores
motor_left = robot.getDevice("left wheel motor")
motor_right = robot.getDevice("right wheel motor")
motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))
motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)

# Inicialización de encoders
encoder_left = robot.getDevice("left wheel sensor")
encoder_right = robot.getDevice("right wheel sensor")
encoder_left.enable(TIME_STEP)
encoder_right.enable(TIME_STEP)

# Inicialización de sensores de distancia (ps0 a ps7)
ps = []
for i in range(8):
    sensor = robot.getDevice(f"ps{i}")
    sensor.enable(TIME_STEP)
    ps.append(sensor)

# Variables de estado
prev_enc_left = 0.0
prev_enc_right = 0.0
encoders_initialized = False

# Filtro Simple (Media Móvil Exponencial - EMA)
ema_front = MAX_SENSOR_DIST
alpha_ema = 0.4 # Factor de suavizado

# Instancia del Filtro de Kalman
kf = KalmanFilter(MAX_SENSOR_DIST, 1.0)

print(f"[*] Controlador Python iniciado. Frecuencia: {1000.0/TIME_STEP:.1f} Hz")

# --- Bucle Principal ---
while robot.step(TIME_STEP) != -1:
    
    # 1. LECTURA DE ENCODERS Y PREDICCIÓN (Modelo cinemático)
    curr_enc_left = encoder_left.getValue()
    curr_enc_right = encoder_right.getValue()
    delta_s = 0.0

    if not encoders_initialized:
        prev_enc_left = curr_enc_left
        prev_enc_right = curr_enc_right
        encoders_initialized = True
    else:
        delta_l = curr_enc_left - prev_enc_left
        delta_r = curr_enc_right - prev_enc_right
        delta_s = ((delta_l + delta_r) / 2.0) * WHEEL_RADIUS
        
        prev_enc_left = curr_enc_left
        prev_enc_right = curr_enc_right

    # Ejecutar predicción
    kf.predict(delta_s)

    # 2. LECTURA DE SENSORES Y FILTRADO SIMPLE
    raw_ps0 = ps[0].getValue() # Frontal derecho
    raw_ps7 = ps[7].getValue() # Frontal izquierdo
    
    dist_front_right = raw_to_meters(raw_ps0)
    dist_front_left = raw_to_meters(raw_ps7)
    
    # Tomamos la distancia del obstáculo más cercano al frente
    raw_measurement = min(dist_front_right, dist_front_left)
    
    # Aplicar Filtro Simple (EMA)
    ema_front = alpha_ema * raw_measurement + (1.0 - alpha_ema) * ema_front

    # 3. ACTUALIZACIÓN DEL FILTRO DE KALMAN
    kf.update(raw_measurement)

    # 4. LÓGICA DE NAVEGACIÓN REACTIVA
    speed_l = CRUISE_SPEED
    speed_r = CRUISE_SPEED

    if kf.x < SAFE_DISTANCE:
        # Evaluar laterales para decidir giro
        raw_ps5 = ps[5].getValue() # Lateral izquierdo
        raw_ps2 = ps[2].getValue() # Lateral derecho
        
        if raw_ps5 > raw_ps2:
            # Obstáculo más cerca por izquierda -> Girar derecha
            speed_l = CRUISE_SPEED
            speed_r = -TURN_SPEED
        else:
            # Obstáculo más cerca por derecha -> Girar izquierda
            speed_l = -TURN_SPEED
            speed_r = CRUISE_SPEED

    # Aplicar velocidades
    motor_left.setVelocity(speed_l)
    motor_right.setVelocity(speed_r)

    # 5. REGISTRO (Imprimir cada ~0.5 segundos)
    step_count = int(robot.getTime() * 1000) // TIME_STEP
    if step_count % 8 == 0:
        print(f"[Data] Crudo: {raw_measurement:.3f}m | EMA: {ema_front:.3f}m | Kalman: {kf.x:.3f}m | Avance dS: {delta_s:.4f}m")