# Proyecto Final Robótica

- Gabriel Reyes
- Benjamín Soto
- Diego Zúñiga
  
## Contenido
1. [Descripción y Objetivo](#descripción-y-objetivo)
2. [Materiales, Herramientas y Escenarios](#materiales-herramientas-y-escenarios)
3. [Configuración Temporal y Muestreo](#configuración-temporal-y-muestreo)
4. [Relación Explícita con Laboratorios 1 y 2](#relación-explícita-con-laboratorios-1-y-2)
5. [Explicación del Algoritmo y Diagrama de Flujo](#explicación-del-algoritmo-y-diagrama-de-flujo)
6. [Análisis de Señales y Resultados Obtenidos](#análisis-de-señales-y-resultados-obtenidos)
7. [Conclusiones](#conclusiones)
8. [Instrucciones para Ejecutar la Simulación](#instrucciones-para-ejecutar-la-simulación)

## Descripción y Objetivo

Este proyecto final consiste en diseñar, implementar y evaluar en Webots un sistema de navegación autónoma para el robot móvil diferencial e-puck. A diferencia de los laboratorios anteriores, el sistema evoluciona hacia una meta global, integrando el control cinemático, la percepción sensorial local, la estimación de postura mediante odometría y la toma de decisiones basada en un algoritmo de planificación de rutas sobre un mapa discreto. El objetivo es que el robot alcance una coordenada final de forma autónoma siguiendo un conjunto de puntos intermedios (waypoints) planificados, mientras conserva la capacidad de evadir obstáculos de última hora mediante reactividad.

## Materiales, Herramientas y Escenarios

 - Simulador: Webots R2025a.
 - Lenguaje: Python.
 - Robot: Modelo e-puck. El chasis tiene un radio de rueda calibrado de $r = 0.0205\text{ m}$ y una distancia entre ejes de $L = 0.052\text{ m}$.
 - Sensores: Encoders de posición en ambas ruedas y un anillo frontal/lateral de 8 sensores infrarrojos de proximidad (`ps0` a `ps7`).
 - Escenarios de Prueba:
   - *Escenario Simple:* 
   - *Escenario Complejo:* 

## Configuración Temporal y Muestreo

Para garantizar un registro robusto y continuo de las señales físicas y de la estimación odométrica, la simulación se ejecuta bajo un paso de tiempo fijo y controlado mediante la constante `TIME_STEP`. 

De acuerdo con el código fuente del controlador:
* Paso de tiempo (`TIME_STEP`): $64\text{ ms}$ (mecanismo síncrono del robot).
* Frecuencia de muestreo ($f_s$): $\approx 15.625\text{ Hz}$.
* Muestreo de Datos (Data Logging): Se ha configurado un recolector de datos que registra el vector de estado del robot $(t, x, y, \phi)$, las mediciones de los filtros y las velocidades de los motores una vez cada 8 pasos de simulación. Esto arroja una tasa de escritura en el archivo `.csv` de aproximadamente $2\text{ Hz}$ ($0.512\text{ s}$), optimizando la densidad de los datos sin afectar el rendimiento de los algoritmos de búsqueda.

## Relación Explícita con Laboratorios 1 y 2

El diseño de este controlador de navegación consolida directamente las herramientas matemáticas y lógicas validadas en las experiencias previas:

### Evolución del Laboratorio 1 (Control Cinemático)
Se reutilizaron las ecuaciones del modelo cinemático diferencial para integrar la postura global $(x, y, \phi)$ mediante la lectura incremental de los encoders (Ecuaciones 1 a 7). Para gobernar el seguimiento de la ruta, se reemplazó la configuración a lazo abierto por un controlador proporcional punto a punto ($K_{linear} = 3.0$, $K_{angular} = 6.0$)**. Se implementó una estrategia de alineación que penaliza el avance lineal $\cos(\Delta\phi)$ ante errores de rumbo, forzando giros prioritarios. Adicionalmente, se programó un escalador proporcional anti saturación que preserva el radio de curvatura en caso de que la cinemática inversa exija valores superiores a la velocidad máxima del motor ($\pm 6.28\text{ rad/s}$).

### Evolución del Laboratorio 2 (Percepción y Estimación)
La arquitectura de percepción local conserva la interpolación en unidades métricas y la posterior fusión sensorial mediante un Filtro de Media Móvil Exponencial (EMA) y un Filtro de Kalman escalar 1D. El modelo de predicción cinemática x̂ₖ = x̂ₖ₋₁ − Δs permite amortiguar los picos de ruido causados por la dispersión infrarroja en superficies diagonales. Esta señal filtrada dicta el cruce hacia la lógica de evasión reactiva ante cualquier objeto detectado a menos de $0.025\text{ m}$ (Umbral de Seguridad), evitando falsos gatillos durante la navegación en el espacio libre.

## Explicación del Algoritmo y Diagrama de Flujo



## Análisis de Señales y Resultados Obtenidos


## Conclusiones


## Instrucciones para Ejecutar la Simulación

### Requisitos previos
- Simulador Webots versión R2025a.
- Intérprete de Python 3.x configurado en las variables de entorno.
- Librerías base de Python (math, csv, os).

### Pasos de ejecución

1.  Clonar repositorio (o descargar los archivos)
    
    Bash
    
    ```
    git clone https://github.com/Messichiquitto/ProyectoFinalRobotica.git
    cd ProyectoFinalRobotica
    ```
    
2.  Abrir Webots y cargar el mundo deseado:
    
    -   Mundo simple: Archivo → Abrir mundos → `worlds/escenario_simple.wbt`
        
    -   Mundo complejo: Archivo → Abrir mundos → `worlds/escenario_complejo.wbt`
        
3.  Configurar el controlador en Python:
    
    -   El código fuente en Python debe ubicarse en `controllers/controladorProyectoFinal/controladorProyectoFinal.py`.
        
    -   Al tratarse de un script interpretado, Webots lo ejecutará directamente al iniciar la simulación. Asegúrese de que en el nodo E-puck el campo `controller` esté configurado como `"controladorProyectoFinal"` (sin extensión).
        
4.  Ejecutar la simulación:
    
    -   Presione el botón «Run» (o Ctrl+R).
        
    -   El robot comenzará a moverse navegando secuencialmente por la ruta establecida. En la consola de Webots se imprimirá cada ~0.5 s la telemetría del robot (posición, filtros, orientación y modo actual), y se exportará en segundo plano el archivo `datos_trayectoria.csv`.
        
5.  Verificación de resultados:
    
    -   Se puede observar el comportamiento en la vista 3D: debe alcanzar la meta final siguiendo la ruta mientras esquiva obstáculos sin colisiones.
        
    -   Para modificar la ruta planificada, reemplace la lista `WAYPOINTS` en la cabecera del archivo `controladorProyectoFinal.py`. Para modificar parámetros dinámicos (por ejemplo, `SAFE_DISTANCE` o las ganancias `K_LINEAR`/`K_ANGULAR`), edite el script, guarde y Webots aplicará los cambios en la siguiente ejecución.
