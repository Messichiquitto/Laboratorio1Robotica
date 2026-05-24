# Laboratorio 2 Robótica

## Contenido
1. [Descripción](#descripción)
2. [Materiales, Herramientas](#materiales-herramientas)
3. [Configuración Temporal y Muestreo](#configuración-temporal-y-muestreo)
4. [Análisis de Señales Registradas](#análisis-de-señales-registradas)
5. [Implementación del Controlador](#implementación-del-controlador)

## Descripción

Este segundo laboratorio consiste en implementar un sistema básico de navegación reactiva en Webots para el robot móvil diferencial e-puck. A diferencia del laboratorio anterior, el foco está en la percepción del entorno mediante el procesamiento de sensores de distancia e infrarrojos y encoders de rueda. El controlador aplica técnicas de filtrado simple (EMA) y fusión sensorial mediante un Filtro de Kalman escalar para mitigar el ruido de las lecturas, logrando estimar con precisión la distancia frontal a obstáculos y optimizar la toma de decisiones autónomas de movimiento en tiempo real.

## Materiales, Herramientas

 - Simulador: Webots R2025a.
 - Lenguaje: Python.
 - Robot: Modelo e-puck.
 - Mundos: Ambos son arenas con dimensiones de 2x2 metros.
	 - Simple: Tiene 3 cajas que funcionan como obstáculo.
    <img src="https://github.com/Messichiquitto/Laboratorio1Robotica/blob/aed98f336570c488ef7a9d3205a254c7921d110d/capturasWebots/Mundo1Simple_1.png" alt="Mundo Simple" width="300" height="300">
    
	 - Complejo: El robot inicia encerrado en un pasillo hecho por 3 paredes.
    <img src="https://github.com/Messichiquitto/Laboratorio1Robotica/blob/aed98f336570c488ef7a9d3205a254c7921d110d/capturasWebots/Mundo2Complejo.png" alt="Mundo Complejo" width="400" height="400">

## Configuración Temporal y Muestreo

Para garantizar un registro robusto y continuo de las señales físicas, la simulación se ejecuta bajo un paso de tiempo fijo y controlado mediante la constante `TIME_STEP`. 

De acuerdo con el código fuente del controlador:
* Paso de tiempo (`TIME_STEP`): $64\text{ ms}$ (mecanismo síncrono del robot).
* Tiempo de muestreo ($T_s$): $0.064\text{ s}$.
* Frecuencia de muestreo ($f_s$): $$f_s = \frac{1}{T_s} = \frac{1}{0.064\text{ s}} \approx 15.625\text{ Hz}$$
* Muestras por minuto: Se registran exactamente $937.5$ muestras por cada minuto de simulación, lo que permite mapear de manera óptima las tendencias y variaciones de las señales sin comprometer el rendimiento.

## Análisis de Señales Registradas

Las lecturas directas entregadas por los sensores infrarrojos de proximidad del e-puck se reciben originalmente como magnitudes adimensionales (en un rango de $0.0$ a $4095.0$). Mediante una tabla de búsqueda (`LOOKUP_TABLE`) basada en la interpolación de voltaje/distancia, el controlador convierte estos valores crudos a metros, acotando el rango de confianza hasta los $0.07\text{ m}$ ($7\text{ cm}$).

### Desafíos de la Señal Cruda (Incertidumbre y Ruido)
A pesar de la conversión matemática, las mediciones crudas presentan serias limitaciones para el control reactivo directo:
1. Fluctuaciones Gaussianas: El simulador inyecta ruido electrónico en las lecturas, lo que genera oscilaciones en los valores incluso con el robot completamente detenido frente a una pared.
2. Incertidumbre por Ángulo de Incidencia: Al aproximarse de forma oblicua a esquinas o superficies rugosas, los haces infrarrojos sufren dispersión, provocando "picos" o caídas abruptas en la distancia percibida.

Si el robot navegara utilizando únicamente estas lecturas crudas (`raw_measurement`), los picos ruidosos provocarían respuestas erráticas, tales como giros innecesarios o un comportamiento de "titubeo" al aproximarse a un umbral de seguridad.

### Análisis y Explicación del Gráfico de Señales

<img src="https://github.com/Messichiquitto/Laboratorio1Robotica/blob/aed98f336570c488ef7a9d3205a254c7921d110d/capturasWebots/comparativa_senales_lab2.png" alt="Gráfico Señales">
El gráfico expone el comportamiento dinámico de la distancia frontal estimada durante un evento real de aproximación y evasión de un obstáculo en un intervalo de 11 muestras ($64\text{ ms}$ por paso). El análisis permite contrastar la respuesta de la medición cruda frente a las dos técnicas de filtrado implementadas.

#### 1. Transición Inicial e Inicialización del Filtro (Muestras 1 a 3)
En las primeras dos muestras, tanto la Medición Cruda ($z_k$) como el Filtro Simple (EMA) saturan en el rango máximo confiable del e-puck de $0.07\text{ m}$. Sin embargo, el Filtro de Kalman ($\hat{d}_k$) inicia rezagado en $0.043\text{ m}$. 

Este fenómeno se debe a la alta incertidumbre asignada en la covarianza inicial ($P = 1.0$) en combinación con la etapa de predicción cinemática. Al avanzar el robot a velocidad de crucero constante, los encoders registran de forma síncrona un desplazamiento lineal neto ($\Delta d_k \approx 0.004\text{ m}$), forzando al filtro de Kalman a proyectar matemáticamente un acercamiento continuo antes de que la medición del sensor IR converja y predomine sobre la estimación.

#### 2. Zona Crítica de Evasión (Muestras 4 a 6)
En la muestra 3, el sensor detecta físicamente la barrera, cayendo la lectura cruda a $0.048\text{ m}$. Al llegar a la muestra 4, el robot cruza el Umbral de Seguridad ($0.035\text{ m}$), registrando una distancia crítica de $0.018\text{ m}$. 

En esta fase se observa la convergencia de los tres métodos. El algoritmo detecta la proximidad del objeto y reduce el avance lineal (`Avance dS` disminuye drásticamente a $0.0007\text{ m}$), lo que evidencia el frenado y el inicio del pivoteo sobre el eje del e-puck para evadir la colisión. El Filtro de Kalman absorbe el impacto de la caída brusca y estabiliza la lectura en $0.017\text{ m}$ en la muestra 6, impidiendo lecturas falsas por rebotes de señal.

#### 3. Liberación del Frente y Efecto Amortiguador (Muestras 7 a 11)
Una vez que la lógica de navegación reactiva procesa la evasión (apoyada en los sensores laterales), el frente del robot se despeja bruscamente. En las muestras 7 y 8, la señal cruda experimenta una discontinuidad matemática saltando instantáneamente de $0.019\text{ m}$ a su límite de $0.069\text{ m}$.

Es en este escenario donde se justifican ambas técnicas de filtrado:
* Filtro Simple (EMA): Presenta un desfase temporal adaptativo debido a su factor de suavizado ($\alpha = 0.4$), tardando una muestra más en recuperar el valor real de régimen libre.
* Filtro de Kalman: Realiza una transición asintótica y suave ($0.017 \to 0.022 \to 0.040 \to 0.042\text{ m}$). Esto mitiga las discontinuidades y los "picos" transitorios de la lectura cruda, asegurando que el controlador del robot retome la velocidad de crucero de forma progresiva, previniendo oscilaciones mecánicas bruscas o el efecto de "titubeo" ante variaciones instantáneas del entorno.
  
## Implementación del Controlador

```python
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
```
