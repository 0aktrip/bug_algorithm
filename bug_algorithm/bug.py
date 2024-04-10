import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from math import pow, atan2, sqrt, cos, sin
import os,sys

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        self.get_logger().info("Algoritmo TangentBug para Navegacion Reactiva")
        
        # GENERAR PUBLISHERS Y SUBSCRIBERS
        self.cmd_vel_pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.odom_subs = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.laser_subs = self.create_subscription(LaserScan,'/scan',self.laserscan_callback,10)
        self.timer = self.create_timer(0.5, self.move_to_point)

        # Parametros de Control
        self.k1 = 0.4
        self.k2 = 0.8
        self.safe_distance = 0.2
        self.sensor_offset = 0

        # Posicion del robot en todo momento
        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.theta = 0.0

        # Posicion objetivo
        self.goal_pose = Pose2D()
        self.goal_pose.x = 0.0
        self.goal_pose.y = 0.0
        
        self.start_path = False
        self.min_dist_to_obstacle = float('inf')
        self.angle_to_obstacle = 0
        self.ranges = []

    def laserscan_callback(self, msg):
        self.ranges = msg.ranges
        min_dist = min(msg.ranges)
        self.min_dist_to_obstacle = min_dist
        self.angle_to_obstacle = msg.ranges.index(min_dist)
    
    def odom_callback(self, msg_odom):
        self.pose.x = round(msg_odom.pose.pose.position.x,4)
        self.pose.y = round(msg_odom.pose.pose.position.y,4)

        orientation = msg_odom.pose.pose.orientation
        
        # Conversion de cuaterniones a angulos de Euler
        (xq,yq,zq,wq) = (orientation.x, orientation.y, orientation.z, orientation.w)
        t3 = 2.0 * (wq*zq+xq*yq)
        t4 = 1.0 - 2.0 * (yq*yq+zq*zq)
        
        self.theta = math.atan2(t3,t4)

    # Error hacia el objetivo
    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x),2)+pow((goal_pose.y-self.pose.y),2))
    
    def ley_control(self):
        # Error d
        x_err = self.goal_pose.x - self.pose.x
        y_err = self.goal_pose.y - self.pose.y
        
        d = sqrt(x_err**2 + y_err**2)
        
        if(self.theta > math.pi):
            self.theta = self.theta - 2 * math.pi
        if(self.theta < -math.pi):
            self.theta = self.theta + 2 * math.pi
            
        # Error alpha
        alpha = atan2(y_err,x_err) - self.theta
        
        # Ley de control para la velocidad lineal
        v = self.k1 * d * cos(alpha)
        
        if v > 1:
            v = 1.0
        if v < -1:
            v = -1.0
        
        # Ley de control para la velocidad angular
        w = self.k2 * alpha + self.k1 * sin(alpha) * cos(alpha)
        
        if w > 1.0:
            w = 1.0
        if w < -1.0:
            w = -1.0

        self.get_logger().info(f'goal pose: ({self.goal_pose.x} , {self.goal_pose.y}) - robo pose: ({round(self.pose.x,2)},{round(self.pose.y,2)}) - theta: {round(self.theta,2)} - {round(self.theta*180/math.pi,2)}', throttle_duration_sec=1)
        
        return v,w
    
    def is_path_clear(self):
        # Calcula el ángulo hacia el objetivo desde la posición actual del robot
        goal_angle = atan2(self.goal_pose.y - self.pose.y, self.goal_pose.x - self.pose.x)

        # Convertir el ángulo a grados y ajustar para que coincida con el rango del sensor
        goal_angle_degrees = math.degrees(goal_angle) % 360
        sensor_angle = int((goal_angle_degrees + self.sensor_offset) % 360) # Ajustar según la orientación del sensor

        # Definir una distancia mínima segura al objetivo (en metros)
        safe_distance = 0.2

        # Comprobar si hay obstáculos en la dirección del objetivo
        if 0 < self.ranges[sensor_angle] < safe_distance:
            return False # Hay un obstáculo en la dirección del objetivo dentro de la distancia segura
        
        return True # El camino está despejado

    def control(self):
        v = 0.0
        w = 0.0
        obstacle_threshold = 2.0    # Define el umbral para considerar algo como un obstáculo

        # Analiza los rangos del sensor para detectar obstáculos
        obstacle_right = any(r < obstacle_threshold for r in self.ranges[90:135])
        obstacle_center = any(r < obstacle_threshold for r in self.ranges[135:225])
        obstacle_left = any(r < obstacle_threshold for r in self.ranges[225:270])

        if obstacle_center:
            if not obstacle_left and obstacle_right:
                w = self.k2
            elif obstacle_left and not obstacle_right:
                w = -self.k2
            else:
                w = 0.0
        
            v = self.k1 * 0.5

        else:
            v, w = self.ley_control()

        # Asegura que las velocidades estén dentro de los límites aceptables
        v = max(min(v, 1.0), -1.0)
        w = max(min(w, 1.0), -1.0)

        return v, w

    def move_to_point(self):
        if not self.start_path:
            #parar robot
            self.stop_robot()
            
            # Solicitud del destino
            point_x,point_y = input("Ingrese las coordenadas x,y destino: ").split(",")
            self.goal_pose.x = float(point_x)
            self.goal_pose.y = float(point_y)
            
            self.start_path = True

        distance_tolerance = 0.15
        
        if self.euclidean_distance(self.goal_pose) >= distance_tolerance:
            v,w = self.control()
            self.mover(v,w)                       
        else:
            self.stop_robot()
            self.start_path = False

            while True:
                answer = input("Desea ingresar otro punto? s/n:   ")
                
                if answer.lower() not in ('s', 'n'):
                    print("Respuesta no valida")
                else:
                    break
            if answer == "s":
                pass
            else:
                sys.exit()

    # FUNCION PARA MOVER EL ROBOT
    def mover(self,pot_l,pot_a): 
        msg = Twist()
        msg.linear.x  = pot_l
        msg.linear.y = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = pot_a
        self.cmd_vel_pub.publish(msg)
    
    # FUNCION PARA DETENER ROBOT
    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info("ROBOT DETENIDO")
        
def main(args=None):

    #limpiar pantalla
    os.system('clear')
    
    rclpy.init()
    
    robot_controller = RobotController()
    
    #FIXME - Salir del programa ctrl + c
    try:
        rclpy.spin(robot_controller)
    except:
        sys.exit()
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()