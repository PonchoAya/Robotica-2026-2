#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from math import cos, sin, atan2, acos, sqrt, pi

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# Parámetros de trayectoria ──────────────────────────────────────────────

# Aumentar PUNTOS_POR_SEGMENTO da más suavidad pero más tiempo por segmento.
# Aumentar PERIODO_TIMER hace el movimiento más lento.
# Tiempo total por segmento = PUNTOS_POR_SEGMENTO * PERIODO_TIMER segundos.
PUNTOS_POR_SEGMENTO = 300     # puntos intermedios entre P1 y P2
PERIODO_TIMER       = 0.03   # [s] periodo del timer de control

# Parámetros geométricos del robot (del URDF) ────────────────────────────

L1, L2, L3 = 0.45, 0.45, 0.30   # longitudes de eslabones [m]

# Vértices cartesianos de la trayectoria [x, y, theta] ──────────────────

VERTICES = [
    (-1.159,  0.6,  3*pi/4),   # P1 — posicion inicial
    ( 1.159,  0.6,    pi/4),   # P2 — posicion final
]


# Utilidades angulares ───────────────────────────────────────────────────

def normalizar(a):
    """
    Envuelve un ángulo al rango (-π, π] usando atan2(sin, cos).
    Se usa tanto en cin_inv() como al publicar θ1 al joint, ya que
    el controlador de Gazebo acepta únicamente valores en ese rango.
    """
    return atan2(sin(a), cos(a))


def unwrap_incremental(raw, prev_continuo):
    """
    Corrige saltos de ±2π en la secuencia de θ1 calculada por cin_inv().

    Parámetros
    ----------
    raw          : nuevo θ1 calculado por cin_inv(), normalizado en (-π, π]
    prev_continuo: acumulado anterior, puede estar fuera de (-π, π]

    Retorna el nuevo valor acumulado continuo (sin normalizar).
    """
    delta = atan2(sin(raw - prev_continuo), cos(raw - prev_continuo))
    return prev_continuo + delta


# Cinemática inversa ─────────────────────────────────────────────────────

def cin_inv(x_in, y_in, theta_in):
    """
    Calcula la configuración de juntas (θ1, θ2, θ3) para una pose
    cartesiana dada del efector final, usando el método geométrico de
    desacoplamiento de muñeca.

    Pasos
    -----
    1. Desacoplamiento: restar L3 para obtener la posición de la muñeca P3.
    2. Radio efectivo r = distancia euclidiana desde la base hasta P3.
    3. θ2 por ley de cosenos sobre el triángulo (L1, L2, r):
           β  = acos((L1² + L2² − r²) / (2·L1·L2))
           θ2 = π − β        [configuración codo abajo]
    4. θ1 por descomposición del vector radial:
           α  = acos((L1² + r² − L2²) / (2·L1·r))
           ψ  = atan2(y3, x3)
           θ1 = ψ − α        [configuración codo abajo]
    5. θ3 por cierre de la cadena cinemática:
           θ3 = θ_efector − θ1 − θ2

    Los argumentos de acos se limitan a [-1, 1] con max/min para evitar
    errores numéricos por redondeo cerca de los límites del workspace.

    Parámetros
    ----------
    x_in, y_in : posición cartesiana del efector final [m]
    theta_in   : orientación del efector en el plano XY [rad]

    """
    # Paso 1: Desacoplamiento de la muñeca
    x3 = x_in - L3 * cos(theta_in)
    y3 = y_in  - L3 * sin(theta_in)

    # Paso 2: Radio efectivo hasta la muñeca
    r = sqrt(x3**2 + y3**2)

    # Paso 3: Ángulo del codo (θ2)
    arg_beta  = max(-1.0, min(1.0, (L1**2 + L2**2 - r**2) / (2*L1*L2)))
    arg_alpha = max(-1.0, min(1.0, (L1**2 + r**2 - L2**2) / (2*L1*r)))
    beta      = acos(arg_beta)
    alpha     = acos(arg_alpha)
    psi       = atan2(y3, x3)

    # Paso 4: Ángulo de la base (θ1) — configuración codo abajo
    theta_1 = normalizar(psi - alpha)

    # θ2 normalizado
    theta_2 = normalizar(pi - beta)

    # Paso 5: Orientación final (θ3)
    theta_3 = normalizar(theta_in - theta_1 - theta_2)

    return theta_1, theta_2, theta_3


# ── Pre-cálculo de la trayectoria completa ─────────────────────────────────

def construir_trayectoria(vertices, n_puntos):
    """
    Genera la lista completa de ángulos de juntas para la trayectoria
    de ida (P1 a P2) y vuelta (P2 a P1), aplicando unwrap global de θ1.

    La interpolación se realiza en espacio cartesiano, no en espacio de
    juntas. Esto garantiza que el efector siga una línea recta visible,
    a diferencia del script básico donde Gazebo interpolaba en espacio
    de juntas produciendo arcos curvos.

    El unwrap se aplica de forma global y continua a lo largo de todos
    los segmentos sin resetear nunca el acumulado, lo que mantiene la
    continuidad de θ1 incluso entre segmentos consecutivos.

    Parámetros
    ----------
    vertices : lista de poses [x, y, theta] que definen los extremos
    n_puntos : número de puntos intermedios por segmento

    """
    trayectoria = []
    t1_continuo = None   # acumulado global — nunca se resetea entre segmentos

    # Segmento 1: ida P1 a P2 | Segmento 2: vuelta P2 a P1
    segmentos = [
        (vertices[0], vertices[1]),
        (vertices[1], vertices[0]),
    ]

    for seg_idx, (inicio, fin) in enumerate(segmentos):
        x0, y0, th0 = inicio
        x1, y1, th1 = fin

        # Diferencia angular más corta para interpolar theta del efector
        dth = atan2(sin(th1 - th0), cos(th1 - th0))

        for k in range(n_puntos + 1):
            # Parámetro de interpolación lineal s ∈ [0, 1]
            s  = k / n_puntos
            x  = x0 + s * (x1 - x0)
            y  = y0 + s * (y1 - y0)
            th = normalizar(th0 + s * dth)

            # Cinemática inversa para este punto cartesiano
            t1_raw, t2, t3 = cin_inv(x, y, th)

            # Unwrap incremental de θ1
            if t1_continuo is None:
                t1_continuo = t1_raw # primer punto: inicializar
            else:
                t1_continuo = unwrap_incremental(t1_raw, t1_continuo)

            es_inicio = (k == 0)
            trayectoria.append((t1_continuo, t2, t3, es_inicio, seg_idx + 1))

    return trayectoria


# Pre-cálculo al importar el módulo ─────────────────────────────────────

# Toda la trayectoria se calcula una sola vez antes de que el nodo arranque.
# El callback del timer solo recorre la lista, sin cálculos en tiempo real.
TRAYECTORIA = construir_trayectoria(VERTICES, PUNTOS_POR_SEGMENTO)


# Nodo ROS2 ──────────────────────────────────────────────────────────────

class ScaraControl(Node):
    """
    Nodo ROS2 que publica comandos de posición a las tres juntas del robot
    SCARA siguiendo la trayectoria pre-calculada en TRAYECTORIA.

    """

    def __init__(self):
        super().__init__('q_plan_node')

        # Publishers — uno por junta, mismos topics que el script básico
        self.pub_joint01_ = self.create_publisher(msg_type=Float64,
                            topic='/joint1/cmd_pos', qos_profile=10)
        self.pub_joint02_ = self.create_publisher(msg_type=Float64,
                            topic='/joint2/cmd_pos', qos_profile=10)
        self.pub_joint03_ = self.create_publisher(msg_type=Float64,
                            topic='/joint3/cmd_pos', qos_profile=10)

        # Timer que dispara el callback a PERIODO_TIMER segundos
        self.timer_control_ = self.create_timer(timer_period_sec=PERIODO_TIMER,
                              callback=self.cbck_scara_control)

        # Índice actual en la lista TRAYECTORIA
        self.indice_ = 0
        self.total_  = len(TRAYECTORIA)

        self.get_logger().info('Nodo controlador scara')

        # Publisher del marcador de trayectoria para RViz
        self.pub_marker_ = self.create_publisher(
            msg_type=Marker,
            topic='/trayectoria_marker',
            qos_profile=10
        )
        self.puntos_visitados_ = []   # acumula los puntos recorridos


    def cbck_scara_control(self):
        """
        Callback del timer. En cada disparo publica los ángulos del
        punto actual de la trayectoria y avanza al siguiente.

        """
        # Definicion de las variables de la posicion inicial
        theta_O_1_p1 = Float64()
        theta_1_2_p1 = Float64()
        theta_2_3_p1 = Float64()
        # Definicion de las variables de la posicion final
        theta_O_1_p2 = Float64()
        theta_1_2_p2 = Float64()
        theta_2_3_p2 = Float64()

        # Obtener el punto actual de la trayectoria pre-calculada
        t1_cont, t2, t3, es_inicio, seg_num = TRAYECTORIA[self.indice_]

        # Normalizar θ1 al rango (-π, π] antes de publicar.
        # t1_cont puede estar fuera de ese rango (ej. -3.9 rad) porque
        # el unwrap acumula sin normalizar. Gazebo requiere (-π, π].
        t1_pub = normalizar(t1_cont)

        if es_inicio:
            # Enviar comandos de la posicion inicial
            self.get_logger().info(f'Segmento {seg_num} — Posicion inicio')
            theta_O_1_p1.data = float(t1_pub)
            self.pub_joint01_.publish(theta_O_1_p1)
            theta_1_2_p1.data = float(t2)
            self.pub_joint02_.publish(theta_1_2_p1)
            theta_2_3_p1.data = float(t3)
            self.pub_joint03_.publish(theta_2_3_p1)
        else:
            # Enviar comandos de la posicion final
            theta_O_1_p2.data = float(t1_pub)
            self.pub_joint01_.publish(theta_O_1_p2)
            theta_1_2_p2.data = float(t2)
            self.pub_joint02_.publish(theta_1_2_p2)
            theta_2_3_p2.data = float(t3)
            self.pub_joint03_.publish(theta_2_3_p2)

            # Log al llegar al último punto del segmento
            if self.indice_ + 1 < self.total_:
                _, _, _, prox_es_inicio, _ = TRAYECTORIA[self.indice_ + 1]
                if prox_es_inicio:
                    self.get_logger().info(f'Segmento {seg_num} — Posicion final')


        # Reconstruir posición cartesiana actual desde los ángulos publicados
        x_act = (L1 * cos(t1_pub) 
            + L2 * cos(t1_pub + t2) 
            + L3 * cos(t1_pub + t2 + t3))
        y_act = (L1 * sin(t1_pub) 
            + L2 * sin(t1_pub + t2) 
            + L3 * sin(t1_pub + t2 + t3))

        # Acumular el punto
        p = Point()
        p.x = x_act
        p.y = y_act
        p.z = 0.225   # altura del plano de trabajo (origin del link_1_joint en el URDF)
        self.puntos_visitados_.append(p)

        # Construir y publicar el marcador
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp    = self.get_clock().now().to_msg()
        marker.ns              = 'trayectoria'
        marker.id              = 0
        marker.type            = Marker.LINE_STRIP
        marker.action          = Marker.ADD
        marker.scale.x         = 0.005   # grosor de la línea [m]
        marker.color.r         = 1.0
        marker.color.g         = 1.0
        marker.color.b         = 0.0     # amarillo
        marker.color.a         = 1.0
        marker.points          = self.puntos_visitados_
        self.pub_marker_.publish(marker)

        # Avanzar índice de forma cíclica
        self.indice_ = (self.indice_ + 1) % self.total_
        if self.indice_ == 0:
            self.get_logger().info('Ciclo completado, reiniciando...')


def main(args=None):
    rclpy.init(args=args)
    node = ScaraControl()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()