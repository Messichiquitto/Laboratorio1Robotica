# Laboratorio1Robotica

## Contenido

## Descripción

Este primer laboratorio consiste en programar un controlador en un lenguaje de programación (en este caso Python) para un robot que utiliza dos ruedas motrices independientes. Las cuales a través de la manipulación de sus velocidades se observan distintos tipos de trayectorias y comportamientos por partes el robot que analizaremos más adelante.

## Materiales, Herramientas

 - Simulador: Webots R2025a.
 - Lenguaje: Python.
 - Robot: Modelo e-puck.
 - Entorno: Área Rectangular 2 x 2.

## Modelo Cinemático
El movimiento del robot se rige por las siguientes ecuaciones, donde v_r es la velocidad de la rueda derecha, v_l de la izquierda y L la distancia entre ellas.

 - Velocidad Lineal: 
-> 				v = (v_r + v_l) 	/ 2
 - Velocidad Angular: 
 -> w = (v_r - v_l) / L
- Posición:
-> x_t + 1 = x_t + v * t_f - t_i

## Experimentos Realizados

El controlador nos permite alternar entre 4 distintos modos de experimentos mediante la variable modo_experimento
<!--stackedit_data:
eyJoaXN0b3J5IjpbMTQwMTc4NjI1NywtMjExMDU5NTgxLC0xNz
czMzg5NDA4LDkxMzMzNTAxLC0xNTU2NDkyOTM4LDExNTg1MzQ2
NzksMjA5ODgxMTEwNCwzNzg2MzU5NDgsLTc3NzA4NTYzMCwtMT
U3NTg0MjQ2N119
-->