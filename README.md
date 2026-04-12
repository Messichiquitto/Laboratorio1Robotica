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

<!--stackedit_data:
eyJoaXN0b3J5IjpbLTQ5NTM1NDU5MCwtMTc3MzM4OTQwOCw5MT
MzMzUwMSwtMTU1NjQ5MjkzOCwxMTU4NTM0Njc5LDIwOTg4MTEx
MDQsMzc4NjM1OTQ4LC03NzcwODU2MzAsLTE1NzU4NDI0NjddfQ
==
-->