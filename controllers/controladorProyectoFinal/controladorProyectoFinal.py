"""
controlador_proyecto.py
=======================
Proyecto Final – ICI 4150 Robótica y Sistemas Autónomos
Módulo de Control, Fusión Sensorial, Navegación Local y Planificación Global (A*)

Implementa:
  - Planificador global A* sobre grilla de ocupación (reemplaza waypoints hardcodeados).
  - Modelo cinemático diferencial y odometría.
  - Filtrado de percepción (EMA y Kalman 1D).
  - Controlador proporcional con alineación y escalado antisaturación.
  - Navegación reactiva de emergencia basada en sensores IR.
"""

from controller import Robot
import math
import csv
import os
import heapq

# ──────────────────────────────────────────────────────────────
# 1. GRILLA DE OCUPACIÓN (MAPA DEL ESCENARIO)
# ──────────────────────────────────────────────────────────────
# Cada matriz representa el arena de 2x2 m discretizada en celdas de
# CELL_SIZE m. 0 = celda libre, -1 = celda ocupada por un obstáculo.
# Verificado contra las coordenadas reales de los Solid del .wbt:
# p.ej. en el escenario complejo, la pared "1" (x:[-0.6,-0.4], y:[-0.2,1.0])
# cae exactamente sobre la columna j=2, filas i=0..5 de ESCENARIO_C.
ESCENARIO_S = [( 0, 0, 0, 0,-1, 0, 0, 0, 0, 0),( 0, 0, 0, 0,-1, 0, 0, 0, 0, 0),( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),( 0, 0, 0, 0,-1, 0, 0, 0, 0, 0),(-1,-1, 0, 0,-1, 0, 0, 0, 0, 0),( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)]

ESCENARIO_C = [( 0, 0,-1, 0, 0, 0, 0, 0, 0, 0),( 0, 0,-1, 0, 0, 0, 0, 0, 0, 0),( 0, 0,-1, 0, 0,-1,-1,-1, 0, 0),( 0, 0,-1, 0, 0,-1, 0, 0, 0, 0),( 0, 0,-1, 0, 0,-1, 0, 0, 0, 0),( 0, 0,-1, 0, 0,-1, 0,-1,-1,-1),( 0, 0, 0, 0, 0,-1, 0,-1, 0, 0),( 0, 0, 0, 0, 0,-1, 0, 0, 0,-1),( 0, 0,-1, 0, 0,-1, 0,-1, 0, 0),( 0, 0,-1, 0, 0,-1, 0,-1, 0, 0)]

# Selección de escenario activo: cambiar a ESCENARIO_S para el mapa simple.
MATRIZ = ESCENARIO_S

# Posición real del robot E-puck en el mundo .wbt (translation del nodo E-puck).
# Es el origen físico que ancla la celda (0,0) de la grilla.
ROBOT_START_WORLD = (-0.9, 0.9)

# Punto de meta deseado en coordenadas del mundo (esquina opuesta del arena
# en ambos escenarios de prueba). Cambiar aquí para apuntar a otra meta;
# ya no es necesario hardcodear la ruta completa.
GOAL_WORLD = (0.9, -0.9)

# ──────────────────────────────────────────────────────────────
# 2. PARÁMETROS DEL ROBOT E-PUCK
# ──────────────────────────────────────────────────────────────
WHEEL_RADIUS   = 0.0205   # [m] Radio de rueda
WHEEL_BASE     = 0.05910  # [m] Distancia entre ruedas — calibrado empíricamente en Webots
MAX_SPEED      = 6.28     # [rad/s] Velocidad máxima del motor
TIME_STEP      = 64       # [ms] Paso de simulación sincronizado

# ──────────────────────────────────────────────────────────────
# 3. PARÁMETROS DE NAVEGACIÓN Y CONTROL
# ──────────────────────────────────────────────────────────────
GOAL_RADIUS    = 0.05     # [m] Radio de tolerancia para alcanzar un waypoint
SAFE_DISTANCE  = 0.025    # [m] Umbral de colisión frontal (Filtro Kalman)
CRUISE_SPEED   = 3.0      # [rad/s] Velocidad de avance estándar
TURN_SPEED     = 2.0      # [rad/s] Velocidad diferencial para giro reactivo
K_LINEAR       = 3.0      # Ganancia proporcional lineal
K_ANGULAR      = 6.0      # Ganancia proporcional angular
OBS_THRESHOLD  = 200.0    # Valor crudo mínimo para considerar obstáculo cercano
CELL_SIZE      = 0.2      # Tamaño de las celdas de la grilla de ocupación [m]
TURN_SPEED_NAV  = 0.35    # [rad/s] velocidad de giro — baja para limitar sobreimpulso por paso
                           # a 64ms/paso: 0.35*0.064 = 0.022 rad/paso ≈ 1.3° máx sobreimpulso
DIST_TOL        = 0.003   # [m] tolerancia de parada final (3 mm)
ENC_TOL         = 0.01    # [rad encoder] tolerancia de parada (ajustado para WB calibrado)

# Corrección de heading durante avance
K_HEADING        = 4.0    # Ganancia P para corrección angular suave diferencial (vl/vr)
MAX_HEADING_CORR = 0.6    # Corrección máxima [rad/s] aplicada a cada rueda (asimétrica)
# Micro-corrección post-giro: más exigente — 1° en vez de 2° para salir mejor alineado
POST_TURN_TOL   = 0.0175  # ~1° en radianes
POST_TURN_SPEED = 0.3     # [rad/s] velocidad muy baja para micro-ajuste angular
FINE_TURN_STABLE_STEPS = 3  # pasos consecutivos dentro de tolerancia para salir de FINE_TURN

# Fase APPROACH: control punto-a-punto puro en los últimos centímetros
# CELL_SIZE = 0.2m → tramos típicos de 0.5–1.3m. APPROACH activa en los últimos 3 cm.
APPROACH_DIST   = 0.03    # [m] activa aproximación final solo en los últimos 3 cm
APPROACH_SPEED  = 0.8     # [rad/s] velocidad en aproximación — permite frenado natural
K_APPROACH_ANG  = 5.0     # Ganancia angular en fase de aproximación (más agresiva)
# ODOM_SNAP eliminado: corregir solo x,y sin corregir phi creaba inconsistencia
# entre posición "ideal" y heading real, amplificando el error en vez de reducirlo

# ──────────────────────────────────────────────────────────────
# 4. PARÁMETROS FILTRO DE KALMAN 1D
# ──────────────────────────────────────────────────────────────
KF_Q = 1e-4   # Varianza del modelo cinemático
KF_R = 1e-2   # Varianza de la medición IR

# ══════════════════════════════════════════════════════════════
# PLANIFICADOR GLOBAL: A* SOBRE GRILLA DE OCUPACIÓN
# ══════════════════════════════════════════════════════════════
#
# Convención de la grilla (validada contra las coordenadas reales de los
# Solid en los archivos .wbt provistos):
#   - Columna j crece hacia +x:  x_centro(j) = ROBOT_START_WORLD.x + CELL_SIZE * j
#   - Fila    i crece hacia -y:  y_centro(i) = ROBOT_START_WORLD.y - CELL_SIZE * i
#   - La celda (0,0) coincide con la posición física inicial del robot.
#
# Movimiento restringido a 4-conectividad (sin diagonales): así se evita el
# clásico problema de "corner cutting" en A* sobre grillas (cruzar en
# diagonal entre dos celdas obstáculo adyacentes), y el resultado son tramos
# rectos horizontales/verticales — exactamente el tipo de trayectoria que el
# controlador de navegación (TURNING → FINE_TURN → MOVING → APPROACH) espera.

def world_to_grid(x: float, y: float) -> tuple[int, int]:
    """Convierte una coordenada del mundo (x, y) al índice de celda (i, j) más cercano."""
    j = round((x - ROBOT_START_WORLD[0]) / CELL_SIZE)
    i = round((ROBOT_START_WORLD[1] - y) / CELL_SIZE)
    return i, j

def grid_to_world(i: int, j: int) -> tuple[float, float]:
    """Convierte un índice de celda (i, j) al centro de esa celda en coordenadas del mundo."""
    x = ROBOT_START_WORLD[0] + CELL_SIZE * j
    y = ROBOT_START_WORLD[1] - CELL_SIZE * i
    return x, y

def _es_libre(matriz: list, i: int, j: int) -> bool:
    filas, cols = len(matriz), len(matriz[0])
    return 0 <= i < filas and 0 <= j < cols and matriz[i][j] == 0

def astar(matriz: list, start: tuple[int, int], goal: tuple[int, int]) -> list:
    """
    A* clásico en grilla 4-conectada.
    Heurística: distancia Manhattan (admisible y consistente para este tipo
    de movimiento, ya que el costo real de cada paso es siempre 1).
    Retorna la lista de celdas (i, j) desde start hasta goal, o None si no
    existe un camino libre de obstáculos.
    """
    if not _es_libre(matriz, *start) or not _es_libre(matriz, *goal):
        return None

    def h(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    contador = 0  # desempata entradas con igual f_score en el heap
    open_set = [(h(start, goal), contador, start)]
    g_score = {start: 0}
    came_from = {}
    cerrados = set()

    while open_set:
        _, _, actual = heapq.heappop(open_set)
        if actual in cerrados:
            continue
        cerrados.add(actual)

        if actual == goal:
            camino = [actual]
            while camino[-1] in came_from:
                camino.append(came_from[camino[-1]])
            camino.reverse()
            return camino

        i, j = actual
        for di, dj in ((-1, 0), (1, 0), (0, -1), (0, 1)):  # N, S, O, E — sin diagonales
            vecino = (i + di, j + dj)
            if not _es_libre(matriz, *vecino):
                continue
            g_tentativo = g_score[actual] + 1
            if g_tentativo < g_score.get(vecino, math.inf):
                g_score[vecino] = g_tentativo
                came_from[vecino] = actual
                contador += 1
                heapq.heappush(open_set, (g_tentativo + h(vecino, goal), contador, vecino))

    return None  # No existe camino: grilla desconectada u obstáculos bloquean la meta

def simplificar_camino(celdas: list) -> list:
    """
    Reduce la secuencia completa de celdas del A* a solo los puntos donde
    cambia la dirección de movimiento (más el punto inicial y final).
    Como el A* es 4-conectado, cada tramo entre puntos de giro es una línea
    recta horizontal o vertical: el controlador de navegación solo necesita
    los vértices, no cada celda intermedia.
    """
    if len(celdas) <= 2:
        return celdas
    simplificado = [celdas[0]]
    direccion_prev = None
    for k in range(1, len(celdas)):
        direccion_actual = (celdas[k][0] - celdas[k - 1][0], celdas[k][1] - celdas[k - 1][1])
        if direccion_prev is not None and direccion_actual != direccion_prev:
            simplificado.append(celdas[k - 1])
        direccion_prev = direccion_actual
    simplificado.append(celdas[-1])
    return simplificado

def generar_waypoints(matriz: list, inicio_world: tuple, meta_world: tuple) -> list:
    """
    Planifica la ruta global con A* y la traduce a una lista de waypoints en
    coordenadas del mundo, lista para ser consumida por la máquina de
    estados de navegación local (TURNING/FINE_TURN/MOVING/APPROACH).

    El primer punto (la propia celda donde ya está el robot) se descarta:
    no tiene sentido pedirle que "navegue" hacia donde ya se encuentra.
    """
    start_cell = world_to_grid(*inicio_world)
    goal_cell  = world_to_grid(*meta_world)

    camino = astar(matriz, start_cell, goal_cell)
    if camino is None:
        raise RuntimeError(
            f"[A*] No se encontró ruta libre de obstáculos entre {start_cell} "
            f"y {goal_cell} en la grilla activa. Revisa MATRIZ / GOAL_WORLD."
        )

    vertices = simplificar_camino(camino)
    waypoints = [grid_to_world(i, j) for (i, j) in vertices[1:]]
    return waypoints

# Ruta global calculada en tiempo de carga del módulo: ya no hay puntos
# hardcodeados, se derivan de MATRIZ + ROBOT_START_WORLD + GOAL_WORLD.
WAYPOINTS = generar_waypoints(MATRIZ, ROBOT_START_WORLD, GOAL_WORLD)

# ══════════════════════════════════════════════════════════════
# CLASES DE ESTIMACIÓN Y FILTRADO
# ══════════════════════════════════════════════════════════════

class KalmanFilter1D:
    """Filtro de Kalman escalar para estimar distancia frontal al obstáculo."""
    def __init__(self, q=KF_Q, r=KF_R, x0=0.07, p0=1.0):
        self.x = x0
        self.P = p0
        self.Q = q
        self.R = r

    def predict(self, delta_s: float):
        """Predicción cinemática: decremento de distancia según avance odómetrico."""
        self.x -= delta_s
        self.P += self.Q

    def update(self, z: float) -> float:
        """Corrección basada en la medición sensorial actual."""
        K = self.P / (self.P + self.R)
        self.x += K * (z - self.x)
        self.P *= (1 - K)
        return self.x

class EMAFilter:
    """Filtro de Media Móvil Exponencial (EMA) de primer orden."""
    def __init__(self, alpha=0.4, x0=0.07):
        self.alpha = alpha
        self.x = x0

    def update(self, z: float) -> float:
        self.x = self.alpha * z + (1 - self.alpha) * self.x
        return self.x

class Odometry:
    """Estimación de postura (x, y, φ) mediante modelo diferencial acoplado al simulador."""
    def __init__(self, x0=0.0, y0=0.0, phi0=0.0):
        self.x = x0
        self.y = y0
        self.phi = phi0

    def update(self, delta_theta_r: float, delta_theta_l: float):
        delta_sr = WHEEL_RADIUS * delta_theta_r
        delta_sl = WHEEL_RADIUS * delta_theta_l

        delta_s   = (delta_sr + delta_sl) / 2.0
        delta_phi = (delta_sr - delta_sl) / WHEEL_BASE
        mid_phi   = self.phi + delta_phi / 2.0

        # Mapeo al sistema de coordenadas físico de Webots
        self.x   += delta_s * math.cos(mid_phi)
        self.y   += delta_s * math.sin(mid_phi)
        self.phi  = self._normalize(self.phi + delta_phi)

        return self.x, self.y, self.phi, delta_s, delta_phi

    @staticmethod
    def _normalize(angle: float) -> float:
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

# ══════════════════════════════════════════════════════════════
# CONTROLADORES DE NAVEGACIÓN
# ══════════════════════════════════════════════════════════════

def compute_wheel_speeds(x: float, y: float, phi: float, goal_x: float, goal_y: float) -> tuple:
    """
    Controlador proporcional punto a punto.
    Incluye alineación prioritaria y escalado para prevenir saturación asimétrica.
    """
    dx = goal_x - x
    dy = goal_y - y
    dist = math.hypot(dx, dy)
    
    angle_to_goal = math.atan2(dy, dx)
    error_phi = Odometry._normalize(angle_to_goal - phi)

    # Factor de alineación: reduce avance lineal si el error angular es pronunciado
    alignment_factor = max(0.0, math.cos(error_phi))
    
    v_linear = min(K_LINEAR * dist, CRUISE_SPEED) * alignment_factor
    v_angular = K_ANGULAR * error_phi

    # Cinemática inversa
    half_omega_L = v_angular * WHEEL_BASE / 2.0
    vr = (v_linear + half_omega_L) / WHEEL_RADIUS
    vl = (v_linear - half_omega_L) / WHEEL_RADIUS

    # Escalado proporcional si se excede el límite físico del motor
    max_requested = max(abs(vl), abs(vr))
    if max_requested > MAX_SPEED:
        scale_factor = MAX_SPEED / max_requested
        vl *= scale_factor
        vr *= scale_factor

    return vl, vr

def sensor_detection(ps_values: list) -> tuple:
    """
    Evaluación reactiva basada en sensores IR laterales y frontales.
    Retorna velocidades de evasión o None si la ruta es segura. ***
    """
    front_left, front_right = ps_values[7], ps_values[0]
    side_left, side_right   = ps_values[6], ps_values[1]
    
    front = max(front_left, front_right)

    if front > OBS_THRESHOLD:
        if front_left > front_right or side_left > side_right:
            return TURN_SPEED, -TURN_SPEED
        return -TURN_SPEED, TURN_SPEED

    if side_left > OBS_THRESHOLD * 1.5:
        return CRUISE_SPEED, CRUISE_SPEED * 0.5

    if side_right > OBS_THRESHOLD * 1.5:
        return CRUISE_SPEED * 0.5, CRUISE_SPEED

    return None

# ══════════════════════════════════════════════════════════════
# UTILIDADES SENSORIALES
# ══════════════════════════════════════════════════════════════

LOOKUP_TABLE = [
    (4095, 0.002), (2133, 0.010), (1475, 0.020),
    (1062, 0.030), ( 858, 0.040), ( 770, 0.050),
    ( 724, 0.060), ( 700, 0.070),
]

def raw_to_meters(raw: float) -> float:
    """Conversión de lectura analógica a distancia métrica por interpolación lineal."""
    if raw >= LOOKUP_TABLE[0][0]: return LOOKUP_TABLE[0][1]
    if raw <= LOOKUP_TABLE[-1][0]: return LOOKUP_TABLE[-1][1]
    
    for i in range(len(LOOKUP_TABLE) - 1):
        r1, d1 = LOOKUP_TABLE[i]
        r2, d2 = LOOKUP_TABLE[i + 1]
        if r2 <= raw <= r1:
            t = (raw - r1) / (r2 - r1)
            return d1 + t * (d2 - d1)
    return LOOKUP_TABLE[-1][1]

# ══════════════════════════════════════════════════════════════
# FUNCIONES DE MOVIMIENTO
# ══════════════════════════════════════════════════════════════

def encoder_arc_for_angle(angle_rad: float) -> float:
    """Radianes que debe girar cada rueda para rotar el robot angle_rad en el lugar.
    arc = |angulo| * (WHEEL_BASE/2) / WHEEL_RADIUS
    """
    return abs(angle_rad) * (WHEEL_BASE / 2.0) / WHEEL_RADIUS


def snap_to_cardinal(phi: float) -> float:
    """Redondea phi al ángulo cardinal más cercano (0, ±π/2, π).
    Elimina error acumulado de giro forzando el heading a la cuadrícula exacta.
    """
    cardinals = [0.0, math.pi/2, math.pi, -math.pi/2]
    return min(cardinals, key=lambda c: abs(Odometry._normalize(phi - c)))

def nav_target_heading(wx: float, wy: float,
                       rx: float, ry: float) -> float:
    """Ángulo hacia el waypoint destino desde la posición actual del robot."""
    return math.atan2(wy - ry, wx - rx)
 
 
def nav_angle_error(target_heading: float, current_phi: float) -> float:
    """Error angular normalizado a (-π, π]."""
    return Odometry._normalize(target_heading - current_phi)
 
 
def nav_compute_turn(angle_err: float) -> tuple[float, float]:
    """
    Giro en el lugar a velocidad constante baja (TURN_SPEED_NAV).
    Velocidad reducida minimiza el sobreimpulso por inercia en cada step,
    permitiendo usar ANGLE_TOL muy ajustado (~0.005 rad / 0.3°).
    Retorna (vel_izq, vel_der).
    """
    if angle_err > 0:   # girar CCW → rueda izq retrocede
        return -TURN_SPEED_NAV, TURN_SPEED_NAV
    else:               # girar CW  → rueda der retrocede
        return TURN_SPEED_NAV, -TURN_SPEED_NAV

 
def nav_compute_move(dist_remaining: float,
                     angle_err: float) -> tuple[float, float]:
    """
    Avance recto con corrección diferencial suave de heading.

    La corrección angular actúa asimétricamente sobre vl/vr: una rueda va
    ligeramente más rápido que la otra, sin invertir ninguna, de modo que
    el robot curva suavemente hacia el ángulo deseado mientras avanza.

    Fases:
      - dist > 0.06 m : velocidad crucero completa.
      - dist < 0.06 m : rampa de frenado progresivo.
      - dist < 0.02 m : velocidad mínima (10 %) para precisión de parada.
    """
    # ── Velocidad base con rampa de frenado ──────────────────────────────
    if dist_remaining > 0.06:
        base = CRUISE_SPEED
    elif dist_remaining > 0.02:
        ramp = (dist_remaining - 0.02) / 0.04   # 0→1 entre 2 cm y 6 cm
        base = CRUISE_SPEED * (0.10 + 0.90 * ramp)
    else:
        base = CRUISE_SPEED * 0.10               # mínimo 10 % para llegar exacto

    # ── Corrección diferencial acotada (no invierte ruedas) ──────────────
    raw_corr  = K_HEADING * angle_err
    correction = max(-MAX_HEADING_CORR, min(MAX_HEADING_CORR, raw_corr))

    v_l = base - correction
    v_r = base + correction

    # Garantizar velocidad mínima positiva en ambas ruedas
    min_wheel = CRUISE_SPEED * 0.05
    v_l = max(v_l, min_wheel)
    v_r = max(v_r, min_wheel)

    # Saturar sin romper la relación diferencial
    top = max(abs(v_l), abs(v_r))
    if top > MAX_SPEED:
        v_l *= MAX_SPEED / top
        v_r *= MAX_SPEED / top
    return v_l, v_r
 
 
def nav_compute_approach(dist_remaining: float,
                         angle_err: float) -> tuple[float, float]:
    """
    Control de aproximación final para los últimos APPROACH_DIST metros (3 cm).

    A esta distancia ya no se necesita rampa: el robot llega lento desde MOVING
    y simplemente mantiene APPROACH_SPEED con corrección angular agresiva para
    converger al punto exacto. La rampa sería de 3cm a 0, demasiado corta para
    tener efecto real dado el TIME_STEP de 64ms.
    """
    base = APPROACH_SPEED

    raw_corr   = K_APPROACH_ANG * angle_err
    correction = max(-MAX_HEADING_CORR, min(MAX_HEADING_CORR, raw_corr))

    v_l = base - correction
    v_r = base + correction

    # Ambas ruedas siempre hacia adelante
    min_wheel = APPROACH_SPEED * 0.10
    v_l = max(v_l, min_wheel)
    v_r = max(v_r, min_wheel)

    top = max(abs(v_l), abs(v_r))
    if top > MAX_SPEED:
        v_l *= MAX_SPEED / top
        v_r *= MAX_SPEED / top
    return v_l, v_r


def nav_dist_to_waypoint(wx: float, wy: float,
                         rx: float, ry: float) -> float:
    """Distancia euclidiana entre robot y waypoint."""
    return math.hypot(wx - rx, wy - ry)


# ══════════════════════════════════════════════════════════════
# BUCLE DE CONTROL PRINCIPAL
# ══════════════════════════════════════════════════════════════

def main():
    robot = Robot()
    ts = int(robot.getBasicTimeStep())

    # Inicialización de hardware
    left_motor, right_motor = robot.getDevice('left wheel motor'), robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    enc_left, enc_right = robot.getDevice('left wheel sensor'), robot.getDevice('right wheel sensor')
    enc_left.enable(ts)
    enc_right.enable(ts)

    ps = [robot.getDevice(f'ps{i}') for i in range(8)]
    for sensor in ps:
        sensor.enable(ts)

    # IMU: fuente de heading absoluto, inmune al deslizamiento de ruedas que
    # sesga el phi calculado por odometría pura durante los giros en el lugar.
    imu = robot.getDevice('inertial unit')
    imu.enable(ts)

    # Sincronizar un primer paso para que el IMU entregue una lectura válida
    # antes de inicializar la odometría con el heading real (no asumido).
    robot.step(ts)
    _, _, yaw0 = imu.getRollPitchYaw()

    # Subsistemas
    odom = Odometry(x0=ROBOT_START_WORLD[0], y0=ROBOT_START_WORLD[1], phi0=yaw0)
    kf   = KalmanFilter1D()
    ema  = EMAFilter(alpha=0.4)

    print(f"[A*] Ruta global planificada: {len(WAYPOINTS)} waypoints "
          f"desde {ROBOT_START_WORLD} hasta {GOAL_WORLD}")
    for idx, wp in enumerate(WAYPOINTS):
        print(f"  WP{idx}: ({wp[0]:.3f}, {wp[1]:.3f})")

    # Logging
    log_path = os.path.join(os.path.dirname(__file__), 'datos_trayectoria.csv')
    log_file = open(log_path, 'w', newline='')
    writer = csv.writer(log_file)
    writer.writerow([
        'tiempo_s', 'x_est', 'y_est', 'phi_est',
        'dist_raw_m', 'dist_ema_m', 'dist_kalman_m',
        'vl', 'vr', 'waypoint_idx', 'modo'
    ])

    # Inicialización síncrona
    robot.step(ts)
    prev_enc_l = enc_left.getValue()
    prev_enc_r = enc_right.getValue()
    if math.isnan(prev_enc_l): prev_enc_l = 0.0
    if math.isnan(prev_enc_r): prev_enc_r = 0.0

    step_count = 0
    modo            = 'TURNING'
    waypoint_idx    = 0
    vl, vr          = 0.0, 0.0
    enc_turn_target = 0.0
    enc_turn_accum  = 0.0
    enc_turn_sign   = 1
    fine_turn_done  = False   # bandera: ¿ya se hizo la micro-corrección post-giro?
    fine_stable     = 0       # contador de pasos consecutivos dentro de POST_TURN_TOL
    diag_moving     = False   # bandera: imprimir diagnóstico al entrar a MOVING

    print("[INFO] Controlador de Navegación Autónomo Iniciado.")

    while robot.step(ts) != -1:
        t = robot.getTime()
        step_count += 1

        # 1. Odometría
        curr_enc_l, curr_enc_r = enc_left.getValue(), enc_right.getValue()
        if math.isnan(curr_enc_l) or math.isnan(curr_enc_r):
            continue

        delta_theta_l, delta_theta_r = curr_enc_l - prev_enc_l, curr_enc_r - prev_enc_r
        prev_enc_l, prev_enc_r = curr_enc_l, curr_enc_r

        # Heading absoluto desde el IMU: inmune al deslizamiento de ruedas que
        # sesga el delta_phi calculado por encoders durante los giros en el lugar
        # (ver datos_trayectoria.csv: odometría reportaba ~90° de giro cuando el
        # giro físico real era ~80°, ratio ~1.12 de sobreestimación).
        _, _, phi_imu = imu.getRollPitchYaw()

        # Se sincroniza odom.phi con el IMU ANTES de update(): así mid_phi
        # (usado internamente para proyectar delta_s en x,y) también queda
        # corregido, en vez de heredar el sesgo de la odometría angular pura.
        odom.phi = phi_imu
        x, y, _, delta_s, delta_phi = odom.update(delta_theta_r, delta_theta_l)
        phi = phi_imu  # heading que usará toda la navegación de aquí en más

        # 2. Percepción y Filtrado
        ps_raw = [sensor.getValue() for sensor in ps]
        raw_front_avg = (ps_raw[7] + ps_raw[0]) / 2.0
        raw_dist_m = raw_to_meters(raw_front_avg)

        ema_dist = ema.update(raw_dist_m)
        kf.predict(delta_s)
        kf_dist = kf.update(raw_dist_m)
        dist_frontal = max(kf_dist, 0.0)

        # 3. Máquina de Estados: Waypoints
        if waypoint_idx >= len(WAYPOINTS):
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            print("[INFO] Trayectoria global completada.")
            break

        # 4. Navegación Local
        
        if modo != 'FINISHED':
            wx, wy = WAYPOINTS[waypoint_idx]
            dist   = nav_dist_to_waypoint(wx, wy, x, y)
            t_hdg  = nav_target_heading(wx, wy, x, y)
            a_err  = nav_angle_error(t_hdg, phi)
 
            if modo == 'TURNING':
                # ── Giro principal por encoder ───────────────────────────────────
                if enc_turn_target == 0.0 and abs(a_err) > 0.001:
                    enc_turn_target = encoder_arc_for_angle(a_err)
                    enc_turn_accum  = 0.0
                    enc_turn_sign   = 1 if a_err > 0 else -1
                    fine_turn_done  = False
                    print(f"[NAV] Giro iniciado: {math.degrees(a_err):.1f}° "
                          f"→ arco encoder objetivo={enc_turn_target:.4f} rad")

                # Acumular arco: rueda exterior es la que avanza
                if enc_turn_sign > 0:
                    enc_turn_accum += abs(delta_theta_r)
                else:
                    enc_turn_accum += abs(delta_theta_l)

                if enc_turn_accum >= enc_turn_target - ENC_TOL:
                    left_motor.setVelocity(0.0)
                    right_motor.setVelocity(0.0)
                    enc_turn_target = 0.0
                    # Siempre pasar por FINE_TURN: el giro principal tiene sobreimpulso
                    # variable (±1.5°) que la odometría no puede medir con exactitud.
                    # FINE_TURN usa el ángulo al waypoint como referencia absoluta.
                    modo = 'FINE_TURN'
                    t_hdg = nav_target_heading(wx, wy, x, y)
                    a_err = nav_angle_error(t_hdg, phi)
                    print(f"[NAV] Giro principal completo. phi_imu={math.degrees(phi):.2f}° "
                          f"err_residual={math.degrees(a_err):.2f}° → FINE_TURN")
                else:
                    vl, vr = nav_compute_turn(a_err)
                    left_motor.setVelocity(vl)
                    right_motor.setVelocity(vr)

            elif modo == 'FINE_TURN':
                # ── Micro-corrección angular proporcional post-giro ────────────
                # Referencia: ángulo al waypoint (absoluta, no odométrica).
                # Sale solo cuando el error se mantiene dentro de tolerancia
                # FINE_TURN_STABLE_STEPS pasos consecutivos → evita falsos positivos.
                t_hdg = nav_target_heading(wx, wy, x, y)
                a_err = nav_angle_error(t_hdg, phi)
                if abs(a_err) <= POST_TURN_TOL:
                    fine_stable += 1
                else:
                    fine_stable = 0

                if fine_stable >= FINE_TURN_STABLE_STEPS:
                    left_motor.setVelocity(0.0)
                    right_motor.setVelocity(0.0)
                    fine_stable = 0
                    modo = 'MOVING'
                    diag_moving = True
                    print(f"[NAV] FINE_TURN OK. phi={math.degrees(phi):.2f}° "
                          f"err_final={math.degrees(a_err):.2f}°. Avanzando a WP{waypoint_idx}")
                else:
                    # Velocidad proporcional: frena al acercarse, mínimo 0.12 rad/s
                    fine_spd = max(0.12, min(POST_TURN_SPEED, abs(a_err) * 5.0))
                    if a_err > 0:
                        vl, vr = -fine_spd, fine_spd
                    else:
                        vl, vr = fine_spd, -fine_spd
                    left_motor.setVelocity(vl)
                    right_motor.setVelocity(vr)

            elif modo == 'MOVING':
                # Diagnóstico: imprime UNA vez al entrar al estado (bandera diag_moving)
                if diag_moving:
                    diag_moving = False
                    _t = nav_target_heading(wx, wy, x, y)
                    _e = nav_angle_error(_t, phi)
                    print(f"[DIAG] WP{waypoint_idx} inicio MOVING: "
                          f"phi={math.degrees(phi):.2f}° "
                          f"hdg={math.degrees(_t):.2f}° "
                          f"err_ang={math.degrees(_e):.2f}°  dist={dist:.4f}m")
                if dist < DIST_TOL:
                    left_motor.setVelocity(0.0)
                    right_motor.setVelocity(0.0)
                    print(f"[NAV] WP{waypoint_idx} alcanzado. "
                          f"Pos=({x:.4f}, {y:.4f})  error=({x-wx:.4f}, {y-wy:.4f})")
                    waypoint_idx += 1
                    if waypoint_idx >= len(WAYPOINTS):
                        modo = 'FINISHED'
                        print("[NAV] ¡Meta alcanzada!")
                    else:
                        modo = 'TURNING'
                elif dist < APPROACH_DIST:
                    modo = 'APPROACH'
                else:
                    vl, vr = nav_compute_move(dist, a_err)
                    left_motor.setVelocity(vl)
                    right_motor.setVelocity(vr)

            elif modo == 'APPROACH':
                # ── Aproximación precisa punto-a-punto ──────────────────────────
                t_hdg = nav_target_heading(wx, wy, x, y)
                a_err = nav_angle_error(t_hdg, phi)
                if dist < DIST_TOL:
                    left_motor.setVelocity(0.0)
                    right_motor.setVelocity(0.0)
                    print(f"[NAV] WP{waypoint_idx} alcanzado (APPROACH). "
                          f"Pos=({x:.4f}, {y:.4f})  error=({x-wx:.4f}, {y-wy:.4f})")
                    waypoint_idx += 1
                    if waypoint_idx >= len(WAYPOINTS):
                        modo = 'FINISHED'
                        print("[NAV] ¡Meta alcanzada!")
                    else:
                        modo = 'TURNING'
                else:
                    vl, vr = nav_compute_approach(dist, a_err)
                    left_motor.setVelocity(vl)
                    right_motor.setVelocity(vr)

        # 5. Registro de Datos (2 Hz)
        if step_count % 8 == 0:
            writer.writerow([
                f'{t:.3f}', f'{x:.4f}', f'{y:.4f}', f'{phi:.4f}',
                f'{raw_dist_m:.4f}', f'{ema_dist:.4f}', f'{kf_dist:.4f}',
                f'{vl:.3f}', f'{vr:.3f}', waypoint_idx, modo
            ])
            # Monitoreo básico en consola
            print(f"[{t:6.2f}s] pos=({x:.3f}, {y:.3f}) φ={math.degrees(phi):.1f}° | wp={waypoint_idx}/{len(WAYPOINTS)} | modo={modo}")

    log_file.close()
    print(f"[INFO] Sesión finalizada. Log exportado a {log_path}")

if __name__ == '__main__':
    main()