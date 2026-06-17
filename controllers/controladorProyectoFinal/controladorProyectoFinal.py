"""
controlador_proyecto.py
=======================
Proyecto Final – ICI 4150 Robótica y Sistemas Autónomos
Módulo de Control, Fusión Sensorial y Navegación Local

Correcciones aplicadas respecto a la versión original:
  Bug 1 — Kalman reiniciado al detectar campo libre.
  Bug 2 — OBS_THRESHOLD subido de 200 → 800.
  Bug 3 — Odometría inicializada en INICIO = (-0.905, 0.905).
  Bug 4 — Escape del modo reactivo: si el robot lleva demasiados pasos
           consecutivos en modo reactivo (atrapado en esquina), fuerza un
           avance corto para salir del callejón.
  Bug 5 — GOAL_RADIUS subido de 0.05 → 0.12 m para absorber el error
           odométrico acumulado y el desfase celda_a_mundo vs META real.
           Adicionalmente, planificador.py reemplaza el último waypoint
           por la META exacta (ver ese archivo).
"""

from controller import Robot
import math
import csv
import os
from planificador import obtener_waypoints

# ──────────────────────────────────────────────────────────────
# 1. PARÁMETROS DEL ROBOT E-PUCK
# ──────────────────────────────────────────────────────────────
WHEEL_RADIUS = 0.0205   # [m]
WHEEL_BASE   = 0.052    # [m]
MAX_SPEED    = 6.28     # [rad/s]
TIME_STEP    = 64       # [ms]

# ──────────────────────────────────────────────────────────────
# 2. PARÁMETROS DE NAVEGACIÓN Y CONTROL
# ──────────────────────────────────────────────────────────────
# BUG 5 CORREGIDO: tolerancia subida de 0.05 → 0.12 m
# Cubre el error odométrico acumulado (~5-8 cm) más el desfase
# de media celda entre celda_a_mundo y la META real.
GOAL_RADIUS   = 0.12
OBS_THRESHOLD = 800.0   # BUG 2 ya corregido
SAFE_DISTANCE = 0.05    # [m]
CRUISE_SPEED  = 3.0     # [rad/s]
TURN_SPEED    = 2.5     # [rad/s]
K_LINEAR      = 3.0
K_ANGULAR     = 6.0

# BUG 4: límite de pasos reactivos consecutivos antes de forzar escape
MAX_REACTIVE_STEPS = 25   # ≈ 1.6 s a 64 ms/paso
ESCAPE_STEPS       = 10   # pasos de avance forzado

# ──────────────────────────────────────────────────────────────
# 3. RUTA GLOBAL (WAYPOINTS)
# ──────────────────────────────────────────────────────────────
ESCENARIO = 'simple'
INICIO = (-0.905, 0.905)
META   = (0.9, -0.9)
WAYPOINTS = obtener_waypoints(ESCENARIO, INICIO, META)
print(f"[INFO] Waypoints cargados: {len(WAYPOINTS)} puntos desde {INICIO} hasta {META}.")

# ──────────────────────────────────────────────────────────────
# 4. PARÁMETROS FILTRO DE KALMAN 1D
# ──────────────────────────────────────────────────────────────
KF_Q = 1e-4
KF_R = 1e-2


# ══════════════════════════════════════════════════════════════
# CLASES DE ESTIMACIÓN Y FILTRADO
# ══════════════════════════════════════════════════════════════

class KalmanFilter1D:
    """
    Filtro de Kalman escalar para estimar distancia frontal al obstáculo.
    BUG 1 CORREGIDO: reset_if_free() evita deriva acumulativa.
    """
    def __init__(self, q=KF_Q, r=KF_R, x0=0.07, p0=1.0):
        self.x   = x0
        self.P   = p0
        self.Q   = q
        self.R   = r

    def predict(self, delta_s: float):
        self.x -= delta_s
        self.P += self.Q

    def update(self, z: float) -> float:
        K      = self.P / (self.P + self.R)
        self.x = self.x + K * (z - self.x)
        self.P = self.P * (1 - K)
        return self.x

    def reset_if_free(self, ema_dist: float, threshold: float = 0.06):
        if ema_dist > threshold:
            self.x = ema_dist
            self.P = 1.0


class EMAFilter:
    """Filtro de Media Móvil Exponencial."""
    def __init__(self, alpha=0.4, x0=0.07):
        self.alpha = alpha
        self.x     = x0

    def update(self, z: float) -> float:
        self.x = self.alpha * z + (1 - self.alpha) * self.x
        return self.x


class Odometry:
    """
    Estimación de postura (x, y, φ) mediante modelo cinemático diferencial.
    BUG 3 CORREGIDO: inicializar con la posición real del robot en el .wbt.
    """
    def __init__(self, x0=0.0, y0=0.0, phi0=0.0):
        self.x   = x0
        self.y   = y0
        self.phi = phi0

    def update(self, delta_theta_r: float, delta_theta_l: float):
        delta_sr  = WHEEL_RADIUS * delta_theta_r
        delta_sl  = WHEEL_RADIUS * delta_theta_l
        delta_s   = (delta_sr + delta_sl) / 2.0
        delta_phi = (delta_sr - delta_sl) / WHEEL_BASE
        mid_phi   = self.phi + delta_phi / 2.0

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

def compute_wheel_speeds(x, y, phi, goal_x, goal_y):
    """Controlador proporcional punto a punto con alineación prioritaria."""
    dx   = goal_x - x
    dy   = goal_y - y
    dist = math.hypot(dx, dy)

    angle_to_goal = math.atan2(dy, dx)
    error_phi     = Odometry._normalize(angle_to_goal - phi)
    align         = max(0.0, math.cos(error_phi))

    v_linear  = min(K_LINEAR * dist, CRUISE_SPEED) * align
    v_angular = K_ANGULAR * error_phi

    half_L = WHEEL_BASE / 2.0
    vr = (v_linear + v_angular * half_L) / WHEEL_RADIUS
    vl = (v_linear - v_angular * half_L) / WHEEL_RADIUS

    peak = max(abs(vl), abs(vr))
    if peak > MAX_SPEED:
        scale = MAX_SPEED / peak
        vl *= scale
        vr *= scale

    return vl, vr


def reactive_avoidance(ps_values: list):
    """
    Evaluación reactiva basada en sensores IR.
    Retorna (vl, vr) de evasión o None si la ruta es segura.
    BUG 2 CORREGIDO: OBS_THRESHOLD = 800.
    """
    fl, fr = ps_values[7], ps_values[0]
    sl, sr = ps_values[6], ps_values[1]

    front = max(fl, fr)

    if front > OBS_THRESHOLD:
        if fl > fr or sl > sr:
            return  TURN_SPEED, -TURN_SPEED
        return -TURN_SPEED,  TURN_SPEED

    if sl > OBS_THRESHOLD * 1.5:
        return CRUISE_SPEED, CRUISE_SPEED * 0.5

    if sr > OBS_THRESHOLD * 1.5:
        return CRUISE_SPEED * 0.5, CRUISE_SPEED

    return None


# ══════════════════════════════════════════════════════════════
# CONVERSIÓN SENSOR IR → METROS
# ══════════════════════════════════════════════════════════════

LOOKUP_TABLE = [
    (4095, 0.002), (2133, 0.010), (1475, 0.020),
    (1062, 0.030), ( 858, 0.040), ( 770, 0.050),
    ( 724, 0.060), ( 700, 0.070),
]

def raw_to_meters(raw: float) -> float:
    if raw >= LOOKUP_TABLE[0][0]:  return LOOKUP_TABLE[0][1]
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
    ts    = int(robot.getBasicTimeStep())

    lm = robot.getDevice('left wheel motor')
    rm = robot.getDevice('right wheel motor')
    lm.setPosition(float('inf'))
    rm.setPosition(float('inf'))
    lm.setVelocity(0.0)
    rm.setVelocity(0.0)

    el = robot.getDevice('left wheel sensor')
    er = robot.getDevice('right wheel sensor')
    el.enable(ts)
    er.enable(ts)

    ps = [robot.getDevice(f'ps{i}') for i in range(8)]
    for s in ps: s.enable(ts)

    # BUG 3 CORREGIDO
    odom = Odometry(x0=INICIO[0], y0=INICIO[1], phi0=0.0)
    kf   = KalmanFilter1D()
    ema  = EMAFilter(alpha=0.4)

    log_path = os.path.join(os.path.dirname(__file__), 'datos_trayectoria.csv')
    log_file = open(log_path, 'w', newline='')
    writer   = csv.writer(log_file)
    writer.writerow([
        'tiempo_s', 'x_est', 'y_est', 'phi_est',
        'dist_raw_m', 'dist_ema_m', 'dist_kalman_m',
        'vl', 'vr', 'waypoint_idx', 'modo'
    ])

    robot.step(ts)
    prev_l = el.getValue()
    prev_r = er.getValue()
    if math.isnan(prev_l): prev_l = 0.0
    if math.isnan(prev_r): prev_r = 0.0

    wp_idx         = 0
    step_count     = 0
    # BUG 4: contadores de escape reactivo
    reactive_steps = 0
    escape_count   = 0

    print("[INFO] Controlador de Navegación Autónomo Iniciado.")

    while robot.step(ts) != -1:
        t = robot.getTime()
        step_count += 1

        # ── Odometría ───────────────────────────────────────────
        cl, cr = el.getValue(), er.getValue()
        if math.isnan(cl) or math.isnan(cr):
            continue
        dth_l, dth_r = cl - prev_l, cr - prev_r
        prev_l, prev_r = cl, cr
        x, y, phi, delta_s = odom.update(dth_r, dth_l)

        # ── Percepción y Filtrado ────────────────────────────────
        ps_raw     = [s.getValue() for s in ps]
        raw_front  = (ps_raw[7] + ps_raw[0]) / 2.0
        raw_dist_m = raw_to_meters(raw_front)

        ema_dist   = ema.update(raw_dist_m)

        # BUG 1 CORREGIDO
        kf.reset_if_free(ema_dist)
        kf.predict(delta_s)
        kf_dist = kf.update(raw_dist_m)

        dist_frontal = max(ema_dist, 0.0)

        # ── Máquina de estados: Waypoints ───────────────────────
        if wp_idx >= len(WAYPOINTS):
            lm.setVelocity(0.0)
            rm.setVelocity(0.0)
            print("[INFO] Trayectoria global completada.")
            break

        goal_x, goal_y = WAYPOINTS[wp_idx]
        # BUG 5 CORREGIDO: GOAL_RADIUS = 0.12 absorbe error odométrico
        if math.hypot(goal_x - x, goal_y - y) < GOAL_RADIUS:
            print(f"[INFO] Waypoint {wp_idx} alcanzado → ({goal_x:.3f}, {goal_y:.3f})")
            wp_idx += 1
            reactive_steps = 0
            escape_count   = 0
            continue

        # ── Navegación Local ─────────────────────────────────────
        reactive = reactive_avoidance(ps_raw)

        # BUG 4 CORREGIDO: escape de bucle reactivo infinito
        if escape_count > 0:
            # Modo escape: avance forzado para salir de la esquina
            modo = 'escape'
            vl   = CRUISE_SPEED * 0.5
            vr   = CRUISE_SPEED * 0.5
            escape_count -= 1
            if escape_count == 0:
                reactive_steps = 0   # resetear para volver a evaluar

        elif reactive is not None or dist_frontal < SAFE_DISTANCE:
            modo = 'reactivo'
            reactive_steps += 1
            if reactive is not None:
                vl, vr = reactive
            else:
                vl, vr = (TURN_SPEED, -TURN_SPEED) if ps_raw[7] > ps_raw[0] \
                          else (-TURN_SPEED, TURN_SPEED)

            # BUG 4: si lleva demasiado tiempo girando en esquina, escapar
            if reactive_steps > MAX_REACTIVE_STEPS:
                print(f"[WARN] {MAX_REACTIVE_STEPS} pasos reactivos consecutivos "
                      f"— activando modo escape.")
                escape_count   = ESCAPE_STEPS
                reactive_steps = 0

        else:
            modo = 'waypoint'
            reactive_steps = 0
            escape_count   = 0
            vl, vr = compute_wheel_speeds(x, y, phi, goal_x, goal_y)

        vl = max(-MAX_SPEED, min(MAX_SPEED, vl))
        vr = max(-MAX_SPEED, min(MAX_SPEED, vr))
        lm.setVelocity(vl)
        rm.setVelocity(vr)

        # ── Registro de Datos (≈2 Hz) ────────────────────────────
        if step_count % 8 == 0:
            writer.writerow([
                f'{t:.3f}', f'{x:.4f}', f'{y:.4f}', f'{phi:.4f}',
                f'{raw_dist_m:.4f}', f'{ema_dist:.4f}', f'{kf_dist:.4f}',
                f'{vl:.3f}', f'{vr:.3f}', wp_idx, modo
            ])
            print(f"[{t:6.2f}s] pos=({x:.3f},{y:.3f}) φ={math.degrees(phi):.1f}° "
                  f"| wp={wp_idx}/{len(WAYPOINTS)} | modo={modo} "
                  f"| d_ema={ema_dist:.3f}m")

    log_file.close()
    print(f"[INFO] Sesión finalizada. Log exportado a {log_path}")


if __name__ == '__main__':
    main()