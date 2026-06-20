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

# ──────────────────────────────────────────────────────────────
# 1. RUTA GLOBAL (WAYPOINTS)
# ──────────────────────────────────────────────────────────────
ESCENARIO_S = [( 0, 0, 0, 0,-1, 0, 0, 0, 0, 0),( 0, 0, 0, 0,-1, 0, 0, 0, 0, 0),( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),( 0, 0, 0, 0,-1, 0, 0, 0, 0, 0),(-1,-1, 0, 0,-1, 0, 0, 0, 0, 0),( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),( 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)]

ESCENARIO_C = [( 0, 0,-1, 0, 0, 0, 0, 0, 0, 0),( 0, 0,-1, 0, 0, 0, 0, 0, 0, 0),( 0, 0,-1, 0, 0,-1,-1,-1, 0, 0),( 0, 0,-1, 0, 0,-1, 0, 0, 0, 0),( 0, 0,-1, 0, 0,-1, 0, 0, 0, 0),( 0, 0,-1, 0, 0,-1, 0,-1,-1,-1),( 0, 0, 0, 0, 0,-1, 0,-1, 0, 0),( 0, 0, 0, 0, 0,-1, 0, 0, 0,-1),( 0, 0,-1, 0, 0,-1, 0,-1, 0, 0),( 0, 0,-1, 0, 0,-1, 0,-1, 0, 0)]

MATRIZ = ESCENARIO_C

PUNTOS_S = 0

PUNTOS_C = [(-0.7, 0.9),(-0.7, -0.4),(-0.2, -0.4),(-0.2, 0.8),( 0.8,  0.8),( 0.8,  0.2),( 0.3,  0.2),( 0.3,  -0.5),( 0.7,  -0.5),( 0.7,  -0.9),( 0.9,  -0.9)]

WAYPOINTS = PUNTOS_C

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
CELL_SIZE      = 0.2      # Tamaño de las celdas
TURN_SPEED_NAV  = 0.8     # [rad/s] velocidad de giro
DIST_TOL        = 0.025   # [m]  tolerancia para dar avance por terminado
ENC_TOL         = 0.01    # [rad encoder] tolerancia de parada (ajustado para WB calibrado)

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
    Velocidades para avance recto con corrección proporcional de heading.
    Incluye rampa de frenado en el último tramo (< 3 cm).
    Retorna (vel_izq, vel_der).
    """
    ramp      = min(1.0, dist_remaining / 0.03)
    base      = CRUISE_SPEED * (0.4 + 0.6 * ramp)   # mínimo 40% al frenar
    correction = K_LINEAR * angle_err
    v_l = base - correction
    v_r = base + correction
    # Saturar sin perder la relación diferencial
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

    # Subsistemas
    odom = Odometry(x0=-0.9, y0=0.9, phi0=0.0)
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

    step_count = 0
    modo            = 'TURNING'
    waypoint_idx    = 0
    vl, vr          = 0.0, 0.0
    enc_turn_target = 0.0
    enc_turn_accum  = 0.0
    enc_turn_sign   = 1

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

        x, y, phi, delta_s, delta_phi = odom.update(delta_theta_r, delta_theta_l)

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
                # Inicio de giro: calcular arco objetivo una sola vez
                if enc_turn_target == 0.0 and abs(a_err) > 0.001:
                    enc_turn_target = encoder_arc_for_angle(a_err)
                    enc_turn_accum  = 0.0
                    enc_turn_sign   = 1 if a_err > 0 else -1
                    print(f"[NAV] Giro iniciado: {math.degrees(a_err):.1f}° "
                          f"→ arco encoder objetivo={enc_turn_target:.4f} rad")

                # Acumular arco: rueda exterior es la que avanza
                # CCW (sign>0): rueda der avanza; CW (sign<0): rueda izq avanza
                if enc_turn_sign > 0:
                    enc_turn_accum += abs(delta_theta_r)
                else:
                    enc_turn_accum += abs(delta_theta_l)

                if enc_turn_accum >= enc_turn_target - ENC_TOL:
                    left_motor.setVelocity(0.0)
                    right_motor.setVelocity(0.0)
                    # Corregir phi al cardinal exacto para evitar acumulación de error
                    odom.phi = snap_to_cardinal(odom.phi)
                    enc_turn_target = 0.0
                    modo = 'MOVING'
                    print(f"[NAV] Giro completado. phi corregido a "
                          f"{math.degrees(odom.phi):.1f}°. Avanzando a WP{waypoint_idx}")
                else:
                    vl, vr = nav_compute_turn(a_err)
                    left_motor.setVelocity(vl)
                    right_motor.setVelocity(vr)
 
            elif modo == 'MOVING':
                if dist < DIST_TOL:
                    # Waypoint alcanzado
                    left_motor.setVelocity(0.0)
                    right_motor.setVelocity(0.0)
                    print(f"[NAV] WP{waypoint_idx} alcanzado. "
                          f"Pos=({x:.3f}, {y:.3f})")
                    waypoint_idx += 1
                    if waypoint_idx >= len(WAYPOINTS):
                        modo = 'FINISHED'
                        print("[NAV] ¡Meta alcanzada!")
                    else:
                        modo = 'TURNING'   # calcular nuevo heading
                else:
                    vl, vr = nav_compute_move(dist, a_err)
                    left_motor.setVelocity(vl)
                    right_motor.setVelocity(vr)

        '''reactive = reactive_avoidance(ps_raw)
        
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
        '''

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