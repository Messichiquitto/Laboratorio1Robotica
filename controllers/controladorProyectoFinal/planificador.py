#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
planificador.py
===============
Módulo de planificación global para el Proyecto Final.

Construye la grilla de ocupación a partir de los obstáculos definidos
en los archivos .wbt (simple y complejo). Implementa A* para encontrar
la ruta óptima desde un punto inicial hasta una meta.
"""

import math
import heapq
from typing import List, Tuple, Optional

# ----------------------------------------------------------------------
# PARÁMETROS DE LA GRILLA (arena de 2x2 metros, centrada en el origen)
# ----------------------------------------------------------------------
RESOLUCION = 0.20          # [m] Tamaño de celda (20 cm)
ORIGEN_X = -1.0            # [m] Esquina inferior izquierda de la arena
ORIGEN_Y = -1.0
ANCHO = 2.0                # [m] Tamaño de la arena
ALTO = 2.0

# Número de celdas
N_CELDAS_X = int(ANCHO / RESOLUCION)  # 10
N_CELDAS_Y = int(ALTO / RESOLUCION)   # 10

# ----------------------------------------------------------------------
# DEFINICIÓN DE OBSTÁCULOS PARA CADA ESCENARIO
# Cada obstáculo: (x, y, sx, sy) = centro (m) y tamaño (m) en x e y
# ----------------------------------------------------------------------

OBSTACULOS_SIMPLE = [
    ( 0.1,  0.1, 0.2, 0.2),
    ( 0.1, -0.1, 0.2, 0.2),
    ( 0.1,  0.9, 0.2, 0.2),
    ( 0.1,  0.7, 0.2, 0.2),
    (-0.9, -0.1, 0.2, 0.2),
    (-0.7, -0.1, 0.2, 0.2),
]

OBSTACULOS_COMPLEJO = [
    (-0.5,  0.4, 0.2, 1.2),
    (-0.5, -0.8, 0.2, 0.4),
    ( 0.1, -0.2, 0.2, 1.6),
    ( 0.4,  0.5, 0.4, 0.2),
    ( 0.5, -0.8, 0.2, 0.4),
    ( 0.5, -0.2, 0.2, 0.4),
    ( 0.8, -0.1, 0.4, 0.2),
    ( 0.9, -0.5, 0.2, 0.2),
]

# ----------------------------------------------------------------------
# FUNCIONES PARA CONSTRUIR LA GRILLA DE OCUPACIÓN
# ----------------------------------------------------------------------

def construir_grilla(obstaculos: List[Tuple[float, float, float, float]],
                     resolucion: float = RESOLUCION,
                     origen_x: float = ORIGEN_X,
                     origen_y: float = ORIGEN_Y,
                     nx: int = N_CELDAS_X,
                     ny: int = N_CELDAS_Y) -> List[List[int]]:
    """
    Crea una grilla 2D (ny filas, nx columnas) con 0=libre, 1=obstáculo.
    Cada obstáculo es un rectángulo dado por (cx, cy, sx, sy).
    Se marcan como ocupadas todas las celdas cuyo centro cae dentro del rectángulo.
    """
    grid = [[0] * nx for _ in range(ny)]

    for cx, cy, sx, sy in obstaculos:
        # Mitades
        hx = sx / 2.0
        hy = sy / 2.0
        # Rango en metros
        x_min = cx - hx
        x_max = cx + hx
        y_min = cy - hy
        y_max = cy + hy

        # Convertir a índices de celda (col, row)
        col_min = int((x_min - origen_x) / resolucion)
        col_max = int((x_max - origen_x) / resolucion)
        row_min = int((y_min - origen_y) / resolucion)
        row_max = int((y_max - origen_y) / resolucion)

        # Asegurar límites
        col_min = max(0, col_min)
        col_max = min(nx - 1, col_max)
        row_min = max(0, row_min)
        row_max = min(ny - 1, row_max)

        for row in range(row_min, row_max + 1):
            for col in range(col_min, col_max + 1):
                grid[row][col] = 1

    return grid

# ----------------------------------------------------------------------
# CLASE PLANIFICADOR A*
# ----------------------------------------------------------------------

class AStarPlanner:
    def __init__(self, grid: List[List[int]], resolucion: float = RESOLUCION,
                 origen_x: float = ORIGEN_X, origen_y: float = ORIGEN_Y):
        self.grid = grid
        self.ny = len(grid)
        self.nx = len(grid[0]) if self.ny > 0 else 0
        self.resolucion = resolucion
        self.origen_x = origen_x
        self.origen_y = origen_y

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        col = int((x - self.origen_x) / self.resolucion)
        row = int((y - self.origen_y) / self.resolucion)
        # Recortar a los límites
        col = max(0, min(self.nx - 1, col))
        row = max(0, min(self.ny - 1, row))
        return col, row

    def grid_to_world(self, col: int, row: int) -> Tuple[float, float]:
        x = self.origen_x + (col + 0.5) * self.resolucion
        y = self.origen_y + (row + 0.5) * self.resolucion
        return x, y

    def is_free(self, col: int, row: int) -> bool:
        if 0 <= col < self.nx and 0 <= row < self.ny:
            return self.grid[row][col] == 0
        return False

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan

    def get_neighbors(self, col: int, row: int) -> List[Tuple[int, int]]:
        candidates = [(col-1, row), (col+1, row), (col, row-1), (col, row+1)]
        return [(c, r) for c, r in candidates if self.is_free(c, r)]

    def plan(self, start_world: Tuple[float, float], goal_world: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        start_col, start_row = self.world_to_grid(*start_world)
        goal_col, goal_row = self.world_to_grid(*goal_world)

        if not self.is_free(start_col, start_row):
            raise ValueError("Posición inicial sobre obstáculo.")
        if not self.is_free(goal_col, goal_row):
            raise ValueError("Meta sobre obstáculo.")

        open_set = []
        heapq.heappush(open_set, (0, start_col, start_row))
        came_from = {}
        g_score = {(start_col, start_row): 0}
        f_score = {(start_col, start_row): self.heuristic((start_col, start_row), (goal_col, goal_row))}

        while open_set:
            _, c, r = heapq.heappop(open_set)
            current = (c, r)

            if current == (goal_col, goal_row):
                # Reconstruir camino
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append((start_col, start_row))
                path.reverse()
                return [self.grid_to_world(col, row) for col, row in path]

            for neighbor in self.get_neighbors(c, r):
                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, (goal_col, goal_row))
                    f_score[neighbor] = f
                    heapq.heappush(open_set, (f, neighbor[0], neighbor[1]))

        return None  # No se encontró ruta

# ----------------------------------------------------------------------
# FUNCIÓN DE ALTO NIVEL PARA OBTENER WAYPOINTS
# ----------------------------------------------------------------------

def obtener_waypoints(escenario: str,
                      inicio: Tuple[float, float],
                      meta: Tuple[float, float]) -> List[Tuple[float, float]]:
    """
    Devuelve una lista de waypoints (x, y) en metros para el escenario dado.
    escenario: 'simple' o 'complejo'
    """
    if escenario == 'simple':
        obstaculos = OBSTACULOS_SIMPLE
    elif escenario == 'complejo':
        obstaculos = OBSTACULOS_COMPLEJO
    else:
        raise ValueError("Escenario debe ser 'simple' o 'complejo'.")

    grid = construir_grilla(obstaculos)
    planner = AStarPlanner(grid)
    waypoints = planner.plan(inicio, meta)
    if waypoints is None:
        raise RuntimeError("No se pudo encontrar una ruta entre inicio y meta.")
    return waypoints

# ----------------------------------------------------------------------
# EJEMPLO DE USO (prueba)
# ----------------------------------------------------------------------
if __name__ == "__main__":
    # Escenario simple: inicio en (-0.905, 0.905) (posición del e-puck)
    # meta en (0.9, -0.9) (esquina inferior derecha)
    inicio = (-0.905, 0.905)
    meta = (0.9, -0.9)
    wp = obtener_waypoints('simple', inicio, meta)
    print("Waypoints (simple):", wp)

    # Escenario complejo
    wp = obtener_waypoints('complejo', inicio, meta)
    print("Waypoints (complejo):", wp)