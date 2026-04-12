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

El controlador nos permite alternar entre 5 distintos modos de experimentos mediante la variable modo_experimento.

| MODO | Resultado | 
|--|--| 
| 1 | El robot avanza recto|
| 2 | El robot realiza una curva s |
| 3 |  |
| 4 |  |
| 5 |  |

<!--stackedit_data:
eyJoaXN0b3J5IjpbLTc0NzgyMjg1MSwxNDAxNzg2MjU3LC0yMT
EwNTk1ODEsLTE3NzMzODk0MDgsOTEzMzM1MDEsLTE1NTY0OTI5
MzgsMTE1ODUzNDY3OSwyMDk4ODExMTA0LDM3ODYzNTk0OCwtNz
c3MDg1NjMwLC0xNTc1ODQyNDY3XX0=
-->