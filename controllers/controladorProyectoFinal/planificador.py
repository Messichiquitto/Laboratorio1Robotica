#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
planificador.py
===============
Módulo de planificación global — Integrante 2: Algoritmo y Lógica Global
Proyecto Final: Navegación Autónoma con Planificación de Rutas en Webots
Robótica y Sistemas Autónomos 2026-01 | ICI 4150

Correcciones aplicadas:
  Bug 5 — El último waypoint se reemplaza por la META exacta (en metros)
           para eliminar el desfase de hasta media celda entre
           celda_a_mundo() y la posición real del objetivo.
           Sin esta corrección, con res=0.10 m el último wp puede estar
           hasta 5 cm lejos de la META real, lo que impide la detección
           de llegada incluso con GOAL_RADIUS = 0.12 m en casos límite.
"""

import math
import heapq
from typing import List, Tuple, Optional, Dict

# ══════════════════════════════════════════════════════════════════════
# PARÁMETROS POR ESCENARIO
# ══════════════════════════════════════════════════════════════════════

ORIGEN_X: float = -1.0
ORIGEN_Y: float = -1.0
ANCHO:    float =  2.0
ALTO:     float =  2.0

_CONFIG: Dict[str, Dict] = {
    'simple': {
        'resolucion': 0.10,
        'inflacion':  0.04,
    },
    'complejo': {
        'resolucion': 0.05,
        'inflacion':  0.02,
    },
}

# ══════════════════════════════════════════════════════════════════════
# DEFINICIÓN DE OBSTÁCULOS
# ══════════════════════════════════════════════════════════════════════

OBSTACULOS_SIMPLE: List[Tuple[float, float, float, float]] = [
    ( 0.1,  0.1, 0.2, 0.2),
    ( 0.1, -0.1, 0.2, 0.2),
    ( 0.1,  0.9, 0.2, 0.2),
    ( 0.1,  0.7, 0.2, 0.2),
    (-0.9, -0.1, 0.2, 0.2),
    (-0.7, -0.1, 0.2, 0.2),
]

OBSTACULOS_COMPLEJO: List[Tuple[float, float, float, float]] = [
    (-0.5,  0.4, 0.2, 1.2),
    (-0.5, -0.8, 0.2, 0.4),
    ( 0.1, -0.2, 0.2, 1.6),
    ( 0.4,  0.5, 0.4, 0.2),
    ( 0.5, -0.8, 0.2, 0.4),
    ( 0.5, -0.2, 0.2, 0.4),
    ( 0.8, -0.1, 0.4, 0.2),
    ( 0.9, -0.5, 0.2, 0.2),
]


# ══════════════════════════════════════════════════════════════════════
# CONSTRUCCIÓN DE LA GRILLA DE OCUPACIÓN
# ══════════════════════════════════════════════════════════════════════

def construir_grilla(
        obstaculos: List[Tuple[float, float, float, float]],
        resolucion: float,
        inflacion:  float,
        origen_x:   float = ORIGEN_X,
        origen_y:   float = ORIGEN_Y,
        ancho:      float = ANCHO,
        alto:       float = ALTO,
) -> List[List[int]]:
    nx = int(round(ancho / resolucion))
    ny = int(round(alto  / resolucion))
    grid: List[List[int]] = [[0] * nx for _ in range(ny)]

    for cx, cy, sx, sy in obstaculos:
        x_min = cx - sx / 2.0 - inflacion
        x_max = cx + sx / 2.0 + inflacion
        y_min = cy - sy / 2.0 - inflacion
        y_max = cy + sy / 2.0 + inflacion

        col_min = max(0,    math.floor((x_min - origen_x) / resolucion))
        col_max = min(nx-1, math.ceil ((x_max - origen_x) / resolucion) - 1)
        row_min = max(0,    math.floor((y_min - origen_y) / resolucion))
        row_max = min(ny-1, math.ceil ((y_max - origen_y) / resolucion) - 1)

        for row in range(row_min, row_max + 1):
            for col in range(col_min, col_max + 1):
                grid[row][col] = 1

    return grid


# ══════════════════════════════════════════════════════════════════════
# CONVERSIONES MUNDO ↔ GRILLA
# ══════════════════════════════════════════════════════════════════════

def mundo_a_celda(
        x: float, y: float,
        resolucion: float,
        origen_x: float = ORIGEN_X,
        origen_y: float = ORIGEN_Y,
        nx: int = None, ny: int = None,
) -> Tuple[int, int]:
    col = int((x - origen_x) / resolucion)
    row = int((y - origen_y) / resolucion)
    if nx is not None: col = max(0, min(nx - 1, col))
    if ny is not None: row = max(0, min(ny - 1, row))
    return col, row


def celda_a_mundo(
        col: int, row: int,
        resolucion: float,
        origen_x: float = ORIGEN_X,
        origen_y: float = ORIGEN_Y,
) -> Tuple[float, float]:
    """Centro geométrico de la celda en coordenadas del mundo."""
    x = origen_x + (col + 0.5) * resolucion
    y = origen_y + (row + 0.5) * resolucion
    return x, y


# ══════════════════════════════════════════════════════════════════════
# ALGORITMO A*
# ══════════════════════════════════════════════════════════════════════

_SQRT2 = math.sqrt(2)

_MOVIMIENTOS: List[Tuple[int, int, float]] = [
    ( 1,  0, 1.0),    (-1,  0, 1.0),
    ( 0,  1, 1.0),    ( 0, -1, 1.0),
    ( 1,  1, _SQRT2), (-1,  1, _SQRT2),
    ( 1, -1, _SQRT2), (-1, -1, _SQRT2),
]


def _heuristica_octil(a: Tuple[int, int], b: Tuple[int, int]) -> float:
    dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
    return (dx + dy) + (_SQRT2 - 2.0) * min(dx, dy)


def astar(
        grid:   List[List[int]],
        inicio: Tuple[int, int],
        meta:   Tuple[int, int],
) -> Optional[List[Tuple[int, int]]]:
    ny = len(grid)
    nx = len(grid[0]) if ny > 0 else 0

    def libre(c: int, r: int) -> bool:
        return 0 <= c < nx and 0 <= r < ny and grid[r][c] == 0

    if not libre(*inicio):
        raise ValueError(f"Posición inicial {inicio} está sobre obstáculo o fuera de la grilla.")
    if not libre(*meta):
        raise ValueError(f"Meta {meta} está sobre obstáculo o fuera de la grilla.")

    heap: List[Tuple[float, float, int, int]] = [(0.0, 0.0, inicio[0], inicio[1])]
    g_score: Dict[Tuple[int, int], float] = {inicio: 0.0}
    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}

    while heap:
        f, g, c, r = heapq.heappop(heap)
        current = (c, r)

        if current == meta:
            path: List[Tuple[int, int]] = []
            node = current
            while node in came_from:
                path.append(node)
                node = came_from[node]
            path.append(inicio)
            path.reverse()
            return path

        if g > g_score.get(current, math.inf):
            continue

        for dc, dr, costo in _MOVIMIENTOS:
            vecino = (c + dc, r + dr)
            if not libre(*vecino):
                continue
            nuevo_g = g_score[current] + costo
            if nuevo_g < g_score.get(vecino, math.inf):
                came_from[vecino] = current
                g_score[vecino] = nuevo_g
                h = _heuristica_octil(vecino, meta)
                heapq.heappush(heap, (nuevo_g + h, nuevo_g, vecino[0], vecino[1]))

    return None


# ══════════════════════════════════════════════════════════════════════
# SUAVIZADO DE RUTA
# ══════════════════════════════════════════════════════════════════════

def _linea_de_vision_libre(
        grid: List[List[int]],
        a: Tuple[int, int],
        b: Tuple[int, int],
) -> bool:
    ny = len(grid)
    nx = len(grid[0]) if ny > 0 else 0
    c0, r0 = a
    c1, r1 = b
    dc, dr = abs(c1 - c0), abs(r1 - r0)
    sc = 1 if c0 < c1 else -1
    sr = 1 if r0 < r1 else -1
    err = dc - dr

    while True:
        if not (0 <= c0 < nx and 0 <= r0 < ny):
            return False
        if grid[r0][c0] == 1:
            return False
        if c0 == c1 and r0 == r1:
            return True
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c0  += sc
        if e2 < dc:
            err += dc
            r0  += sr


def suavizar_ruta(
        grid: List[List[int]],
        ruta: List[Tuple[int, int]],
) -> List[Tuple[int, int]]:
    if len(ruta) <= 2:
        return ruta

    suavizada: List[Tuple[int, int]] = [ruta[0]]
    i = 0
    while i < len(ruta) - 1:
        j = len(ruta) - 1
        while j > i + 1:
            if _linea_de_vision_libre(grid, ruta[i], ruta[j]):
                break
            j -= 1
        suavizada.append(ruta[j])
        i = j
    return suavizada


# ══════════════════════════════════════════════════════════════════════
# FUNCIÓN PRINCIPAL DE ALTO NIVEL
# ══════════════════════════════════════════════════════════════════════

def obtener_waypoints(
        escenario: str,
        inicio:    Tuple[float, float],
        meta:      Tuple[float, float],
        suavizar:  bool = True,
) -> List[Tuple[float, float]]:
    """
    Calcula y devuelve la lista de waypoints (x, y) en metros.

    BUG 5 CORREGIDO: el último waypoint se reemplaza por 'meta' exacta
    para evitar el desfase de media celda entre celda_a_mundo y la
    posición real del objetivo.
    """
    if escenario not in _CONFIG:
        raise ValueError(f"Escenario '{escenario}' desconocido. Use 'simple' o 'complejo'.")

    cfg   = _CONFIG[escenario]
    res   = cfg['resolucion']
    inf   = cfg['inflacion']
    obs   = OBSTACULOS_SIMPLE if escenario == 'simple' else OBSTACULOS_COMPLEJO
    nx    = int(round(ANCHO / res))
    ny    = int(round(ALTO  / res))

    grid  = construir_grilla(obs, resolucion=res, inflacion=inf)

    ci, ri = mundo_a_celda(*inicio, resolucion=res, nx=nx, ny=ny)
    cm, rm = mundo_a_celda(*meta,   resolucion=res, nx=nx, ny=ny)

    ruta = astar(grid, (ci, ri), (cm, rm))

    if ruta is None:
        raise RuntimeError(
            f"A* no encontró ruta de {inicio} a {meta} en escenario '{escenario}'.")

    if suavizar:
        ruta = suavizar_ruta(grid, ruta)

    waypoints = [celda_a_mundo(c, r, resolucion=res) for c, r in ruta]

    # BUG 5 CORREGIDO: reemplazar último waypoint por la META exacta.
    # celda_a_mundo devuelve el centro de celda, que puede distar hasta
    # res/2 de la posición real. Con META=(0.9,-0.9) y res=0.10 el wp
    # generado era (0.95,-0.95), fuera del arena, que nunca se alcanzaba.
    if waypoints:
        waypoints[-1] = meta

    return waypoints


# ══════════════════════════════════════════════════════════════════════
# MÉTRICAS
# ══════════════════════════════════════════════════════════════════════

def longitud_ruta(waypoints: List[Tuple[float, float]]) -> float:
    return sum(
        math.hypot(waypoints[i][0] - waypoints[i-1][0],
                   waypoints[i][1] - waypoints[i-1][1])
        for i in range(1, len(waypoints))
    )


# ══════════════════════════════════════════════════════════════════════
# VISUALIZACIÓN EN CONSOLA (solo depuración)
# ══════════════════════════════════════════════════════════════════════

def visualizar(
        escenario: str,
        inicio:    Tuple[float, float],
        meta:      Tuple[float, float],
) -> None:
    cfg  = _CONFIG[escenario]
    res  = cfg['resolucion']
    inf  = cfg['inflacion']
    obs  = OBSTACULOS_SIMPLE if escenario == 'simple' else OBSTACULOS_COMPLEJO
    nx   = int(round(ANCHO / res))
    ny   = int(round(ALTO  / res))

    grid = construir_grilla(obs, resolucion=res, inflacion=inf)

    ci, ri = mundo_a_celda(*inicio, resolucion=res, nx=nx, ny=ny)
    cm, rm = mundo_a_celda(*meta,   resolucion=res, nx=nx, ny=ny)

    ruta = astar(grid, (ci, ri), (cm, rm))
    ruta_set = set(ruta) if ruta else set()

    print(f"\n{'═'*60}")
    print(f"  ESCENARIO: {escenario.upper()}  "
          f"(res={res}m, inflación={inf}m, grilla {ny}×{nx})")
    print(f"  Inicio : {inicio}  → celda ({ci},{ri})")
    print(f"  Meta   : {meta}   → celda ({cm},{rm})")
    print(f"  Ruta   : {'No encontrada' if ruta is None else str(len(ruta))+' celdas'}")
    print(f"{'═'*60}")
    for row in range(ny - 1, -1, -1):
        line = []
        for col in range(nx):
            if   (col, row) == (ci, ri): line.append('S')
            elif (col, row) == (cm, rm): line.append('G')
            elif (col, row) in ruta_set: line.append('*')
            elif grid[row][col] == 1:   line.append('#')
            else:                        line.append('.')
        print(' '.join(line))
    print()


# ══════════════════════════════════════════════════════════════════════
# BLOQUE DE PRUEBA
# ══════════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    INICIO = (-0.905,  0.905)
    META   = ( 0.85,  -0.85)

    for esc in ('simple', 'complejo'):
        visualizar(esc, INICIO, META)

        wp_raw    = obtener_waypoints(esc, INICIO, META, suavizar=False)
        wp_smooth = obtener_waypoints(esc, INICIO, META, suavizar=True)

        print(f"  Waypoints sin suavizar : {len(wp_raw):3d} pts  "
              f"| longitud = {longitud_ruta(wp_raw):.3f} m")
        print(f"  Waypoints suavizados   : {len(wp_smooth):3d} pts  "
              f"| longitud = {longitud_ruta(wp_smooth):.3f} m")
        print(f"\n  Lista de waypoints (suavizados):")
        for i, (x, y) in enumerate(wp_smooth):
            print(f"    [{i:2d}]  x={x:+.4f} m   y={y:+.4f} m")
        print()