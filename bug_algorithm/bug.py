import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from math import pow, atan2, sqrt, cos, sin
import threading

import os,sys

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        self.get_logger().info("Algoritmo TangentBug para Navegacion Reactiva")
        
        # GENERAR PUBLISHERS Y SUBSCRIBERS
        self.cmd_vel_pub = self.create_publisher(Twist,'/cmd_vel',10)
        self.odom_subs = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.laser_subs = self.create_subscription(LaserScan,'/scan',self.laserscan_callback,10)

        self.timer = self.create_timer(0.1, self.move2goal)

        # Parametros de Control
        self.k1 = 0.4
        self.k2 = 0.8

        # Posicion del robot en todo momento
        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.theta = 0.0

        # Posicion objetivo
        self.goal_pose = Pose2D()

        # Boundary-following state
        self.start_path = False
        self.first_time = True
        self.prev_dot_product = 0
        self.umbral = 2.0
        self.colision_pose = Pose2D()
        self.obstacle = {'obstacle': False, 'orientation': None}

    def laserscan_callback(self, msg):
        '''
        TODO: HACER UN ANILLO DE SEGURIDAD
              SENSAR OBSTACULOS DE FRENTE
        '''
        obstacle_right = any(r < self.umbral for r in msg.ranges[90:179])
        obstacle_left = any(r < self.umbral for r in msg.ranges[180:270])

        self.obstacle['min_right'] = min(msg.ranges[90:179])
        self.obstacle['max_right'] = max(msg.ranges[90:179])

        self.obstacle['min_left'] = min(msg.ranges[180:270])
        self.obstacle['max_left'] = max(msg.ranges[180:270])

        if obstacle_right:
            self.obstacle['orientation'] = 'derecha'
            self.obstacle['obstacle'] = True

        if obstacle_left:
            self.obstacle['orientation'] = 'izquierda'
            self.obstacle['obstacle'] = True
    
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
    
    def move2goal(self):
        if not self.start_path:
            self.stop_robot()

            #??? -  no es necesario un hilo para pedir datos
            thread = threading.Thread(target=self.get_goal_input)
            thread.start()
            thread.join()

        distance_tolerance = 0.15

        if self.euclidean_distance(self.goal_pose) >= distance_tolerance:
            if self.obstacle['obstacle']:
                self.follow_b()
            else:
                self.ley_control()
        else:
            self.stop_robot()
            self.start_path = False
            self.query_new_goal()

    def get_goal_input(self):
        try:
            # Solicitud del destino
            point_x,point_y = input("Ingrese las coordenadas x,y destino: ").split(",")
            self.goal_pose.x = float(point_x)
            self.goal_pose.y = float(point_y)
            self.start_path = True
        except Exception as e:
            self.get_logger().info(f"Entrada invalida: {str(e)}")

    def query_new_goal(self):
        while True:
            answer = input("Desea ingresar otro punto? s/n:   ").strip()
            if answer.lower() not in ('s', 'n'):
                print("Respuesta no valida")
            else:
                break
        if answer == 's':
            pass
        else:
            sys.exit()

    def follow_b(self):
        if self.first_time:
            self.colision_pose.x = self.pose.x
            self.colision_pose.y = self.pose.y
            self.first_time = False

        v = 0.3

        error = self.calculate_obstacle_error()

        #w = max(min(error, 1.0), -1.0) if abs(error) > 0.5 else 0.1 * error

        if error > 1:
            error = 1.0
            
        if error < -1:
            error = -1.0

        w = error
        
        self.mover(v,w)
        
        self.check_vector_sign_change()

    def calculate_obstacle_error(self):
        if self.obstacle['orientation'] == 'derecha':
            return -1 * ((self.obstacle['min_right'] - self.umbral) * 2)
        elif self.obstacle['orientation'] == 'izquierda':
            return (self.obstacle['min_left'] - self.umbral) * 2
        return 0
    
    def check_vector_sign_change(self):
        # Verificación del cambio de signo utilizando el producto punto
        vector_goal = (self.goal_pose.x - self.colision_pose.x, self.goal_pose.y - self.colision_pose.y)
        vector_goal = (-vector_goal[1],vector_goal[0])
        
        vector_position = (self.pose.x - self.colision_pose.x, self.pose.y - self.colision_pose.y)
        
        dot_product = vector_goal[0] * vector_position[0] + vector_goal[1] * vector_position[1]
        
        if (dot_product >= 0 and self.prev_dot_product < 0):
            self.obstacle["obstacle"] = False
            self.first_time = True
            self.get_logger().info("Contour following completed")
            
        self.get_logger().info(f'DOT: ({dot_product})', throttle_duration_sec=1)
        
                    
        self.prev_dot_product = dot_product

    def ley_control(self):
        # Error d
        x_err = self.goal_pose.x - self.pose.x
        y_err = self.goal_pose.y - self.pose.y
        
        d = sqrt(x_err**2 + y_err**2)
        
        # Error alpha
        alpha = atan2(y_err,x_err) - self.theta

        if(alpha > math.pi):
            alpha = alpha - 2 * math.pi
        if(alpha < -math.pi):
            alpha = alpha + 2 * math.pi
        
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
        
        self.mover(v, w)

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
