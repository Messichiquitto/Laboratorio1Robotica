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
El movimiento del robot se rige por las siguientes ecuaciones, donde $\large v_r$ es la velocidad de la rueda derecha, $\large v_l$ de la izquierda y $\large L$ la distancia entre ellas.

 - Velocidad Lineal: 
-> $v=\frac{(v_r+v_l)}{2}$
 - Velocidad Angular: 
-> $w = \frac{(v_r - v_l)}{L}$
- Posición:
-> $x_t + 1 = x_t + v * t_f - t_i$

## Experimentos Realizados

El controlador nos permite alternar entre 5 distintos modos de experimentos mediante la variable modo_experimento.

| MODO | Resultado Esperado | 
|--|--|
| 1 | El robot avanza recto |
| 2 | El robot realiza una curva |
| 3 |El robot gira sobre su propio eje  |
| 4 | Trayectoria circular constante |
| 5 | Simula un ruido |

Para todos los experimentos se inicia desde el mismo punto tal que:

<img src="https://github.com/Messichiquitto/Laboratorio1Robotica/blob/main/testing%20images/Estado%20base.png" alt="Estado base" width="300" height="300">

### Linea recta

Cuando las velocidades de ambas ruedas es igual: $v_r = v_l$

<img src="https://github.com/Messichiquitto/Laboratorio1Robotica/blob/main/testing%20images/Linea%20recta.png" alt="Linea recta" width="300" height="300">

### Linea curva

Cuando las velocidades de ambas ruedas son distintas: $v_r \neq v_l$

<div style="display: flex; gap: 20px">
    <img src="https://github.com/Messichiquitto/Laboratorio1Robotica/blob/main/testing%20images/Linea%20curva%201.png" alt="Linea curva 1" width="300" height="300">
    <img src="https://github.com/Messichiquitto/Laboratorio1Robotica/blob/main/testing%20images/Linea%20curva%202.png" alt="Linea curva 2" width="300" height="300">
</div>

### Giro sobre eje

Cuando las velocidades de las ruedas son opuestas: $v_r = -v_l$

<div style="display: flex; gap: 20px">
    <img src="https://github.com/Messichiquitto/Laboratorio1Robotica/blob/main/testing%20images/Rotacion%201.png" alt="Rotacion sobre eje 1" width="300" height="300">
    <img src="https://github.com/Messichiquitto/Laboratorio1Robotica/blob/main/testing%20images/Rotacion%202.png" alt="Rotacion sobre eje 2" width="300" height="300">
</div>


### Circulo

El robot realiza un circulo

![alt text](https://github.com/Messichiquitto/Laboratorio1Robotica/blob/main/testing%20images/Circulo%201.png "Circulo 1")
![alt text](https://github.com/Messichiquitto/Laboratorio1Robotica/blob/main/testing%20images/Circulo%202.png "Circulo 2")
![alt text](https://github.com/Messichiquitto/Laboratorio1Robotica/blob/main/testing%20images/Circulo%203.png "Circulo 3")
![alt text](https://github.com/Messichiquitto/Laboratorio1Robotica/blob/main/testing%20images/Circulo%204.png "Circulo 4")

## Implementación del controlador

	controlador_lab1 controller
	from controller import Robot
	import random
	
	robot = Robot()
	timestep = int(robot.getBasicTimeStep())

	left_motor = robot.getDevice('left wheel motor')
	right_motor = robot.getDevice('right wheel motor')

	left_motor.setPosition(float('inf'))
	right_motor.setPosition(float('inf'))
	left_motor.setVelocity(0.0)
	right_motor.setVelocity(0.0)

	#Variables de control para los experimentos
	#Cambiando el número el robot se comportará de formas distintas:
	#1: Recto, 2: Curva, 3: Rotación, 4: Círculo, 5: Perturbaciones
	modo_experimento = 1

	#Constante de velocidad
	V_BASE = 2.0

	def set_robot_velocity(vl, vr):
    #Aplica las velocidades a los motores izquierdo y derecho
    left_motor.setVelocity(vl)
    right_motor.setVelocity(vr)

	#Bucle principal de simulación 
	while robot.step(timestep) != -1:
    
    if modo_experimento == 1:
        # Movimiento recto: vr = vl 
        set_robot_velocity(V_BASE, V_BASE)
        
    elif modo_experimento == 2:
        # Trayectoria curva: vr != vl
        set_robot_velocity(V_BASE * 0.8, V_BASE)
        
    elif modo_experimento == 3:
        # Rotación en el lugar: vr = -vl
        set_robot_velocity(-V_BASE, V_BASE)
        
    elif modo_experimento == 4:
        # Dibujar un círculo: velocidades constantes diferentes
        set_robot_velocity(1.5, 3.0)
        
    elif modo_experimento == 5:
        # Extensión: Simular perturbaciones en los actuadores
        # Se añade un ruido aleatorio a la velocidad base de ambos motores
        ruido_l = random.uniform(0.9, 1.1)
        ruido_r = random.uniform(0.9, 1.1)
        set_robot_velocity(V_BASE * ruido_l, V_BASE * ruido_r)
