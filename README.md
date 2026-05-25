# Laboratorio 2 Robótica

- Alex Parada
- Gabriel Reyes
- Benjamín Soto
- Diego Zuñiga
  
## Contenido
1. [Descripción](#descripción)
2. [Materiales, Herramientas](#materiales-herramientas)
3. [Configuración Temporal y Muestreo](#configuración-temporal-y-muestreo)
4. [Análisis de Señales Registradas](#análisis-de-señales-registradas)
5. [Implementación del Controlador](#implementación-del-controlador)

## Descripción

Este segundo laboratorio consiste en implementar un sistema básico de navegación reactiva en Webots para el robot móvil diferencial e-puck. A diferencia del laboratorio anterior, el foco está en la percepción del entorno mediante el procesamiento de sensores de distancia e infrarrojos y encoders de rueda. El controlador aplica técnicas de filtrado simple (EMA) y fusión sensorial mediante un Filtro de Kalman escalar para mitigar el ruido de las lecturas, logrando estimar con precisión la distancia frontal a obstáculos y optimizar la toma de decisiones autónomas de movimiento en tiempo real.

## Materiales, Herramientas

 - Simulador: Webots R2025a.
 - Lenguaje: Python.
 - Robot: Modelo e-puck.
 - Mundos: Ambos son arenas con dimensiones de 2x2 metros.
	 - Simple: Tiene 3 cajas que funcionan como obstáculo.
    <img src="https://github.com/Messichiquitto/Laboratorio1Robotica/blob/aed98f336570c488ef7a9d3205a254c7921d110d/capturasWebots/Mundo1Simple_1.png" alt="Mundo Simple" width="300" height="300">
    
	 - Complejo: El robot inicia encerrado en un pasillo hecho por 3 paredes.
    <img src="https://github.com/Messichiquitto/Laboratorio1Robotica/blob/aed98f336570c488ef7a9d3205a254c7921d110d/capturasWebots/Mundo2Complejo.png" alt="Mundo Complejo" width="400" height="400">

## Configuración Temporal y Muestreo

Para garantizar un registro robusto y continuo de las señales físicas, la simulación se ejecuta bajo un paso de tiempo fijo y controlado mediante la constante `TIME_STEP`. 

De acuerdo con el código fuente del controlador:
* Paso de tiempo (`TIME_STEP`): $64\text{ ms}$ (mecanismo síncrono del robot).
* Tiempo de muestreo ($T_s$): $0.064\text{ s}$.
* Frecuencia de muestreo ($f_s$): $$f_s = \frac{1}{T_s} = \frac{1}{0.064\text{ s}} \approx 15.625\text{ Hz}$$
* Muestras por minuto: Se registran exactamente $937.5$ muestras por cada minuto de simulación, lo que permite mapear de manera óptima las tendencias y variaciones de las señales sin comprometer el rendimiento.

## Análisis de Señales Registradas

Las lecturas directas entregadas por los sensores infrarrojos de proximidad del e-puck se reciben originalmente como magnitudes adimensionales (en un rango de $0.0$ a $4095.0$). Mediante una tabla de búsqueda (`LOOKUP_TABLE`) basada en la interpolación de voltaje/distancia, el controlador convierte estos valores crudos a metros, acotando el rango de confianza hasta los $0.07\text{ m}$ ($7\text{ cm}$).

### Desafíos de la Señal Cruda (Incertidumbre y Ruido)
A pesar de la conversión matemática, las mediciones crudas presentan serias limitaciones para el control reactivo directo:
1. Fluctuaciones Gaussianas: El simulador inyecta ruido electrónico en las lecturas, lo que genera oscilaciones en los valores incluso con el robot completamente detenido frente a una pared.
2. Incertidumbre por Ángulo de Incidencia: Al aproximarse de forma oblicua a esquinas o superficies rugosas, los haces infrarrojos sufren dispersión, provocando "picos" o caídas abruptas en la distancia percibida.

Si el robot navegara utilizando únicamente estas lecturas crudas (`raw_measurement`), los picos ruidosos provocarían respuestas erráticas, tales como giros innecesarios o un comportamiento de "titubeo" al aproximarse a un umbral de seguridad.

### Análisis y Explicación del Gráfico de Señales

<img src="https://github.com/Messichiquitto/Laboratorio1Robotica/blob/aed98f336570c488ef7a9d3205a254c7921d110d/capturasWebots/comparativa_senales_lab2.png" alt="Gráfico Señales">

El gráfico expone el comportamiento dinámico de la distancia frontal estimada durante un evento real de aproximación y evasión de un obstáculo en un intervalo de 11 muestras ($64\text{ ms}$ por paso). El análisis permite contrastar la respuesta de la medición cruda frente a las dos técnicas de filtrado implementadas.

#### 1. Transición Inicial e Inicialización del Filtro (Muestras 1 a 3)
En las primeras dos muestras, tanto la Medición Cruda ($z_k$) como el Filtro Simple (EMA) saturan en el rango máximo confiable del e-puck de $0.07\text{ m}$. Sin embargo, el Filtro de Kalman ($\hat{d}_k$) inicia rezagado en $0.043\text{ m}$. 

Este fenómeno se debe a la alta incertidumbre asignada en la covarianza inicial ($P = 1.0$) en combinación con la etapa de predicción cinemática. Al avanzar el robot a velocidad de crucero constante, los encoders registran de forma síncrona un desplazamiento lineal neto ($\Delta d_k \approx 0.004\text{ m}$), forzando al filtro de Kalman a proyectar matemáticamente un acercamiento continuo antes de que la medición del sensor IR converja y predomine sobre la estimación.

#### 2. Zona Crítica de Evasión (Muestras 4 a 6)
En la muestra 3, el sensor detecta físicamente la barrera, cayendo la lectura cruda a $0.048\text{ m}$. Al llegar a la muestra 4, el robot cruza el Umbral de Seguridad ($0.035\text{ m}$), registrando una distancia crítica de $0.018\text{ m}$. 

En esta fase se observa la convergencia de los tres métodos. El algoritmo detecta la proximidad del objeto y reduce el avance lineal (`Avance dS` disminuye drásticamente a $0.0007\text{ m}$), lo que evidencia el frenado y el inicio del pivoteo sobre el eje del e-puck para evadir la colisión. El Filtro de Kalman absorbe el impacto de la caída brusca y estabiliza la lectura en $0.017\text{ m}$ en la muestra 6, impidiendo lecturas falsas por rebotes de señal.

#### 3. Liberación del Frente y Efecto Amortiguador (Muestras 7 a 11)
Una vez que la lógica de navegación reactiva procesa la evasión (apoyada en los sensores laterales), el frente del robot se despeja bruscamente. En las muestras 7 y 8, la señal cruda experimenta una discontinuidad matemática saltando instantáneamente de $0.019\text{ m}$ a su límite de $0.069\text{ m}$.

Es en este escenario donde se justifican ambas técnicas de filtrado:
* Filtro Simple (EMA): Presenta un desfase temporal adaptativo debido a su factor de suavizado ($\alpha = 0.4$), tardando una muestra más en recuperar el valor real de régimen libre.
* Filtro de Kalman: Realiza una transición asintótica y suave ($0.017 \to 0.022 \to 0.040 \to 0.042\text{ m}$). Esto mitiga las discontinuidades y los "picos" transitorios de la lectura cruda, asegurando que el controlador del robot retome la velocidad de crucero de forma progresiva, previniendo oscilaciones mecánicas bruscas o el efecto de "titubeo" ante variaciones instantáneas del entorno.

## Conversión de los encoders a desplazamiento lineal

Los encoders del e-puck miden el ángulo girado por cada rueda en radianes. Para convertir eso a metros, se usa la relación arco-ángulo:

$$
s=r \cdot \theta
$$

Donde $r = 0.0205m$ (radio de la rueda) y $\theta$ es el ángulo medido por el encoder. En el controlador, esto se aplica de forma diferencial entre dos instantes consecutivos:

$$
\Delta\theta_L = \theta_L^{(k)} - \theta_L^{(k-1)},\quad \Delta\theta_R = \theta_R^{(k)} - \theta_R^{(k-1)}
$$

El desplazamiento lineal de cada rueda es:

$$
\Delta s_L = r \cdot \Delta\theta_L, \quad \Delta s_R = r \cdot \Delta\theta_R
$$

Y el avance lineal del robot (promedio de ambas ruedas, modelo diferencial):

$$
\Delta s = \frac{\Delta s_L + \Delta s_R}{2} = \frac{(\Delta\theta_L + \Delta\theta_R)}{2} \cdot r
$$

En el código esto aparece exactamente así:

```c
// Diferencia de theta izquierda y derecha en radianes
double delta_l = curr_enc_left - prev_enc_left; 
double delta_r = curr_enc_right - prev_enc_right;
delta_s = ((delta_l + delta_r) / 2.0) * WHEEL_RADIUS;
```

## Filtrado simple - Media Móvil Exponencial (EMA)

El EMA es un filtro que combina la medicición actual con el historial acumulado ponderado en un factor $\alpha$:

$$
EMA_{k} = \alpha \cdot z_{k} + (1 - \alpha) \cdot EMA_{k-1}
$$

Donde $z_k$ es la distancia frontal cruda del instante actual. En el controlador se utiliza $\alpha = 0.4$:

```c
ema_front = alpha_ema * raw_measurement + (1.0 - alpha_ema) * ema_front;
```

El valor de $\alpha$ controla el balance entre suavizado y velocidad de respuesta. Con el valor otorgado, cada nueva estimación pondera un $40\%$ la medición nueva y un $60\%$ el historial, reduciendo el efecto del ruido del sensor IR.

## Filtro de Kalman

### Predicción

Si el robot avanza $\Delta s$ metros, el obstáculo se encuentra esa misma distancia más cerca del robot. El estado se corrige con el modelo cinemático y la incertidumbre crece por el error acumulado de los encoders:

$$
\hat{x}_k = \hat{x}_{k-1} - \Delta s
$$


$$
P_{k}= P_{k-1} + Q
$$

$Q = 0.0001$, lo que indica una alta confianza en lo encoders, esto significa que la incertidumbre tiene muy poco crecimiento por cada ciclo.

## Implementación del Controlador

```c
/*
 * Controlador en C para Webots
 * Laboratorio 2: Navegación reactiva con filtrado y fusión de sensores
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>

// --- Constantes del Robot y Simulación ---
#define TIME_STEP 64
#define WHEEL_RADIUS 0.0205 // Metros
#define MAX_SPEED 6.28      // Rad/s
#define CRUISE_SPEED 3.0    // Rad/s

// --- CONSTANTES CORREGIDAS PARA EVITAR CHOQUES ---
#define TURN_SPEED 3.0       // Aumentado para girar más rápido
#define SAFE_DISTANCE 0.047  // Aumentado a 4.7 cm para reaccionar con más anticipación

#define MAX_SENSOR_DIST 0.07 // Rango máximo confiable del e-puck IR (7 cm)

// --- Parámetros del Filtro de Kalman ---
#define KALMAN_Q 0.0001 // Varianza del proceso (confianza en encoders)
#define KALMAN_R 0.005  // Varianza de la medición (ruido del sensor)

// Estructura para el Filtro de Kalman 1D
typedef struct {
    double x; // Estimación de la distancia
    double p; // Varianza (incertidumbre)
} KalmanFilter;

// Estructura para la tabla de conversión IR a metros
typedef struct {
    double distance;
    double raw;
} SensorLookup;

SensorLookup lookup_table[] = {
    {0.000, 4095.0}, {0.005, 2133.3}, {0.010, 1465.7}, {0.015, 601.5},
    {0.020, 383.8},  {0.030, 234.9},  {0.040, 158.0},  {0.050, 120.0},
    {0.060, 104.1},  {0.070, 67.2}
};
const int TABLE_SIZE = 10;

// --- Funciones Auxiliares ---

// Limitar valores dentro de un rango
double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// Convertir valor crudo del sensor IR a metros usando interpolación lineal
double raw_to_meters(double raw) {
    if (raw <= 67.2) return MAX_SENSOR_DIST; // Sin obstáculo en rango
    if (raw >= 4095.0) return 0.0;
    
    for (int i = 0; i < TABLE_SIZE - 1; i++) {
        if (raw <= lookup_table[i].raw && raw >= lookup_table[i+1].raw) {
            double alpha = (raw - lookup_table[i].raw) / (lookup_table[i+1].raw - lookup_table[i].raw);
            return lookup_table[i].distance + alpha * (lookup_table[i+1].distance - lookup_table[i].distance);
        }
    }
    return MAX_SENSOR_DIST;
}

// --- Funciones del Filtro de Kalman ---

// Etapa de Predicción: El obstáculo se acerca según lo que avanza el robot
void kf_predict(KalmanFilter *kf, double delta_s) {
    kf->x = clamp(kf->x - delta_s, 0.0, MAX_SENSOR_DIST);
    kf->p = kf->p + KALMAN_Q;
}

// Etapa de Corrección: Ajustar la predicción con la lectura real del sensor
void kf_update(KalmanFilter *kf, double measurement) {
    double K = kf->p / (kf->p + KALMAN_R); // Ganancia de Kalman
    kf->x = kf->x + K * (measurement - kf->x);
    kf->p = (1.0 - K) * kf->p;
}

// --- Función Principal ---
int main(int argc, char **argv) {
    wb_robot_init();

    // Inicialización de motores
    WbDeviceTag motor_left = wb_robot_get_device("left wheel motor");
    WbDeviceTag motor_right = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(motor_left, INFINITY);
    wb_motor_set_position(motor_right, INFINITY);
    wb_motor_set_velocity(motor_left, 0.0);
    wb_motor_set_velocity(motor_right, 0.0);

    // Inicialización de encoders
    WbDeviceTag encoder_left = wb_robot_get_device("left wheel sensor");
    WbDeviceTag encoder_right = wb_robot_get_device("right wheel sensor");
    wb_position_sensor_enable(encoder_left, TIME_STEP);
    wb_position_sensor_enable(encoder_right, TIME_STEP);

    // Inicialización de sensores de distancia (ps0 y ps7 frontales; ps2 y ps5 laterales)
    WbDeviceTag ps[8];
    char ps_names[8][4];
    for (int i = 0; i < 8; i++) {
        sprintf(ps_names[i], "ps%d", i);
        ps[i] = wb_robot_get_device(ps_names[i]);
        wb_distance_sensor_enable(ps[i], TIME_STEP);
    }

    // Variables de estado
    double prev_enc_left = 0.0, prev_enc_right = 0.0;
    bool encoders_initialized = false;
    
    // Variables para el filtro simple (Media Móvil Exponencial - EMA)
    double ema_front = MAX_SENSOR_DIST;
    double alpha_ema = 0.4; // Factor de suavizado (0 a 1)

    // Inicializar Filtro de Kalman
    KalmanFilter kf = {MAX_SENSOR_DIST, 1.0};

    printf("[*] Controlador iniciado con ajustes antimuros.\n");
    printf("[*] Frecuencia de muestreo: %.1f Hz\n", 1000.0/TIME_STEP);

    // Bucle principal
    while (wb_robot_step(TIME_STEP) != -1) {
        
        // 1. LECTURA DE ENCODERS Y PREDICCIÓN (Modelo cinemático)
        double curr_enc_left = wb_position_sensor_get_value(encoder_left);
        double curr_enc_right = wb_position_sensor_get_value(encoder_right);
        double delta_s = 0.0;

        if (!encoders_initialized) {
            prev_enc_left = curr_enc_left;
            prev_enc_right = curr_enc_right;
            encoders_initialized = true;
        } else {
            double delta_l = curr_enc_left - prev_enc_left;
            double delta_r = curr_enc_right - prev_enc_right;
            delta_s = ((delta_l + delta_r) / 2.0) * WHEEL_RADIUS; // Avance lineal estimado
            
            prev_enc_left = curr_enc_left;
            prev_enc_right = curr_enc_right;
        }

        // Ejecutar predicción de Kalman con el avance
        kf_predict(&kf, delta_s);

        // 2. LECTURA DE SENSORES Y FILTRADO SIMPLE
        double raw_ps0 = wb_distance_sensor_get_value(ps[0]); // Frontal derecho
        double raw_ps7 = wb_distance_sensor_get_value(ps[7]); // Frontal izquierdo
        
        double dist_front_right = raw_to_meters(raw_ps0);
        double dist_front_left = raw_to_meters(raw_ps7);
        
        // Tomamos la distancia del obstáculo más cercano al frente
        double raw_measurement = fmin(dist_front_right, dist_front_left);
        
        // Aplicar Filtro Simple (EMA) a las lecturas
        ema_front = alpha_ema * raw_measurement + (1.0 - alpha_ema) * ema_front;

        // 3. ACTUALIZACIÓN DEL FILTRO DE KALMAN
        kf_update(&kf, raw_measurement);

        // 4. LÓGICA DE NAVEGACIÓN REACTIVA (Corregida para rotación pura)
        double speed_l = CRUISE_SPEED;
        double speed_r = CRUISE_SPEED;

        // Decisión usando la estimación de Kalman fusionada
        if (kf.x < SAFE_DISTANCE) {
            // Obstáculo detectado, verificar sensores laterales para decidir giro
            double raw_ps5 = wb_distance_sensor_get_value(ps[5]); // Izquierda
            double raw_ps2 = wb_distance_sensor_get_value(ps[2]); // Derecha
            
            // En e-puck, mayor valor crudo = obstáculo más cerca
            if (raw_ps5 > raw_ps2) {
                // Obstáculo más cerca por la izquierda -> Rotar sobre su eje hacia la derecha
                speed_l = TURN_SPEED;
                speed_r = -TURN_SPEED;
            } else {
                // Obstáculo más cerca por la derecha -> Rotar sobre su eje hacia la izquierda
                speed_l = -TURN_SPEED;
                speed_r = TURN_SPEED;
            }
        }

        wb_motor_set_velocity(motor_left, speed_l);
        wb_motor_set_velocity(motor_right, speed_r);

        // 5. REGISTRO Y MONITOREO (Imprimir cada ~0.5 segundos)
        int step_count = (int)(wb_robot_get_time() * 1000) / TIME_STEP;
        if (step_count % 8 == 0) {
            printf("[Data] Crudo: %.3fm | Filtro Simple(EMA): %.3fm | Kalman: %.3fm | Avance dS: %.4fm\n", 
                   raw_measurement, ema_front, kf.x, delta_s);
        }
    }

    wb_robot_cleanup();
    return 0;
}
```
## Comparativa del rendimiento de dos mundos

A continuación se analiza el comportamiento del robot en cada escenario, contrastando el uso exclusivo de mediciones crudas, el filtro simple (EMA) y la estimación del Filtro de Kalman.

### Mundo simple: tres cajas con obstaculos aislados
- Descripción: Arena de 2 x 2 metros con tres cajas dispuestas de forma que el robor debe esquivarlas una tras otra.
- Comportamiento observado:
  	- Medición cruda: El robot mostraba un movimiento entrecortado (“titubeo”) al acercarse a cada caja. Los picos de ruido provocaban giros innecesarios incluso antes de alcanzar
  	  el umbral real de seguridad. Sin embargo logra pasar las cajas sin lograr colisiones en estas.
  	- Filtro simple (EMA): Se redujeron las oscilaciones, pero persistió un leve retardo en la detección de los bordes de las cajas. El robot logró esquivar las tres cajas sin
  	  colisiones, aunque realizó algunos giros “indecisos” cuando la distancia frontal estimada por EMA fluctuaba cerca del umbral (aproximadamente 3‑5 giros extra por recorrido).
  	- Filtro de Kalman: La estimación fusionada proporcionó una transición suave y estable. El robot avanzó mayormente con velocidad constante y, al cruzar el umbral de seguridad,
  	  giró de forma decisiva hacia el lado con mayor espacio. La incorporación de la predicción por encoders evitó que variaciones puntuales del sensor IR desencadenaran acciones
  	  incorrectas.

### Mundo complejo: pasillo estrecho formado por tres paredes.


