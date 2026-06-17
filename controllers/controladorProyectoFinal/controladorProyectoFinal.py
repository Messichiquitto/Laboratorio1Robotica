"""
controlador_proyecto.py
=======================
Proyecto Final – ICI 4150 Robótica y Sistemas Autónomos
Módulo de Control, Fusión Sensorial y Navegación Local

Implementa:
  - Modelo cinemático diferencial y odometría.
  - Filtrado de percepción (EMA y Kalman 1D).
  - Controlador proporcional con alineación y escalado antisaturación.
  - Navegación reactiva de emergencia basada en sensores IR.
"""

from controller import Robot
import math
import csv
import os
from planificador import obtener_waypoints

# ──────────────────────────────────────────────────────────────
# 1. PARÁMETROS DEL ROBOT E-PUCK
# ──────────────────────────────────────────────────────────────
WHEEL_RADIUS   = 0.0205   # [m] Radio de rueda
WHEEL_BASE     = 0.052    # [m] Distancia entre ruedas (L)
MAX_SPEED      = 6.28     # [rad/s] Velocidad máxima del motor
TIME_STEP      = 64       # [ms] Paso de simulación sincronizado

# ──────────────────────────────────────────────────────────────
# 2. PARÁMETROS DE NAVEGACIÓN Y CONTROL
# ──────────────────────────────────────────────────────────────
GOAL_RADIUS    = 0.05     # [m] Radio de tolerancia para alcanzar un waypoint
SAFE_DISTANCE  = 0.025    # [m] Umbral de colisión frontal (Filtro Kalman)
CRUISE_SPEED   = 3.0      # [rad/s] Velocidad de avance estándar
TURN_SPEED     = 2.5      # [rad/s] Velocidad diferencial para giro reactivo
K_LINEAR       = 3.0      # Ganancia proporcional lineal
K_ANGULAR      = 6.0      # Ganancia proporcional angular
OBS_THRESHOLD  = 200.0    # Valor crudo mínimo para considerar obstáculo cercano

# ──────────────────────────────────────────────────────────────
# 3. RUTA GLOBAL (WAYPOINTS)
# ──────────────────────────────────────────────────────────────
# Lista dinámica generada por el planificador global (A* / Dijkstra)
ESCENARIO = 'simple'  # O 'complejo', según el escenario elegido
INICIO = (-0.905, 0.905)   # Punto de partida (x, y) en metros
META   = (0.9, -0.9)   # Punto objetivo (x  y) en metros
WAYPOINTS = obtener_waypoints(ESCENARIO, INICIO, META)  # Función que lee el archivo de ruta generado por el planificador
print (f"[INFO] Waypoints cargados: {len(WAYPOINTS)} puntos desde {INICIO} hasta {META}.")

# ──────────────────────────────────────────────────────────────
# 4. PARÁMETROS FILTRO DE KALMAN 1D
# ──────────────────────────────────────────────────────────────
KF_Q = 1e-4   # Varianza del modelo cinemático
KF_R = 1e-2   # Varianza de la medición IR

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

        return self.x, self.y, self.phi, delta_s

    @staticmethod
    def _normalize(angle: float) -> float:
        while angle >  math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
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

def reactive_avoidance(ps_values: list) -> tuple:
    """
    Evaluación reactiva basada en sensores IR laterales y frontales.
    Retorna velocidades de evasión o None si la ruta es segura.
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

    # Subsistemas
    odom = Odometry(x0=0.0, y0=0.0, phi0=0.0)
    kf   = KalmanFilter1D()
    ema  = EMAFilter(alpha=0.4)

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

    waypoint_idx = 0
    step_count = 0

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

        x, y, phi, delta_s = odom.update(delta_theta_r, delta_theta_l)

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

        goal_x, goal_y = WAYPOINTS[waypoint_idx]
        if math.hypot(goal_x - x, goal_y - y) < GOAL_RADIUS:
            waypoint_idx += 1
            continue

        # 4. Navegación Local
        reactive = reactive_avoidance(ps_raw)
        
        if reactive is not None or dist_frontal < SAFE_DISTANCE:
            modo = 'reactivo'
            if reactive is not None:
                vl, vr = reactive
            else:
                vl, vr = (TURN_SPEED, -TURN_SPEED) if ps_raw[7] > ps_raw[0] else (-TURN_SPEED, TURN_SPEED)
        else:
            modo = 'waypoint'
            vl, vr = compute_wheel_speeds(x, y, phi, goal_x, goal_y)

        # Saturación final
        vl = max(-MAX_SPEED, min(MAX_SPEED, vl))
        vr = max(-MAX_SPEED, min(MAX_SPEED, vr))
        
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