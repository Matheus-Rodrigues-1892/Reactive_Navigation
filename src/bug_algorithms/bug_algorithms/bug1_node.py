#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion


class Bug1Node(Node):
    def __init__(self):
        super().__init__('bug1_node')
        
        # ==================== SUBSCRIBERS & PUBLISHERS ====================
        self.create_subscription(Odometry, '/odom', self.clbk_odom, 10)
        self.create_subscription(LaserScan, '/base_scan', self.clbk_laser, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # ==================== TIMER ====================
        self.timer = self.create_timer(0.1, self.main_loop)  # 10 Hz
        
        # ==================== ROBOT STATE ====================
        self.position_ = Point()
        self.yaw_ = 0.0
        self.regions_ = None
        
        # ==================== NAVIGATION PARAMETERS ====================
        # Coordenadas do mundo (Stage world coordinates)
        start = (0.0, 0.0)    # Posição inicial na odometria
        goal = (-7.0, -7.0)  
        
        # Meta do Bug1
        self.goal = Point()
        self.goal.x = goal[0] - start[0]  
        self.goal.y = goal[1] - start[1]  
        self.goal.z = 0.0
        
        # ==================== BUG1 ALGORITHM VARIABLES ====================
        self.state_ = 0                   
        self.closest_point_ = Point()     
        self.start_point_ = Point()       
        self.goal_tol = 0.3              
        # Inicializar pontos
        self.closest_point_.x = self.goal.x
        self.closest_point_.y = self.goal.y
        self.start_point_.x = self.goal.x
        self.start_point_.y = self.goal.y
        
        # ==================== CONTROL PARAMETERS ====================
        self.linear_speed = 0.4
        self.angular_speed = 0.5
        self.wall_distance_threshold = 0.5
        self.wall_follow_distance = 0.7
        
        # ==================== INITIALIZATION LOG ====================
        self.get_logger().info('Bug1 Navigation Node initialized!')
        self.get_logger().info(f'Goal: ({self.goal.x:.2f}, {self.goal.y:.2f})')
        self.get_logger().info(f'Goal tolerance: {self.goal_tol}m')
        
    # ==================== CALLBACK FUNCTIONS ====================
    
    def clbk_odom(self, msg):
        """Callback para dados de odometria"""
        # Atualizar posição
        self.position_ = msg.pose.pose.position
        
        # Atualizar orientação (yaw)
        q = msg.pose.pose.orientation
        _, _, self.yaw_ = euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    def clbk_laser(self, msg):
        """Callback para dados do laser scanner"""
        if len(msg.ranges) == 0:
            return
            
        n = len(msg.ranges)
        step = n // 5
        
        # Dividir laser em regiões
        self.regions_ = {
            'right': min(min(msg.ranges[0:step]), 10.0),
            'front_right': min(min(msg.ranges[step:2*step]), 10.0), 
            'front': min(min(msg.ranges[2*step:3*step]), 10.0),
            'front_left': min(min(msg.ranges[3*step:4*step]), 10.0),
            'left': min(min(msg.ranges[4*step:]), 10.0),
        }
    
    # ==================== UTILITY FUNCTIONS ====================
    
    def dist(self, p1, p2):
        """Calcular distância euclidiana entre dois pontos"""
        return math.hypot(p1.x - p2.x, p1.y - p2.y)
    
    def normalize_angle(self, angle):
        """Normalizar ângulo para [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def is_obstacle_ahead(self):
        """Verificar se há obstáculo à frente"""
        if self.regions_ is None:
            return True
            
        # Considerar obstáculo se qualquer região frontal estiver bloqueada
        front_regions = [
            self.regions_['front'],
            self.regions_['front_left'], 
            self.regions_['front_right']
        ]
        
        return any(distance < self.wall_distance_threshold for distance in front_regions)
    
    # ==================== NAVIGATION BEHAVIORS ====================
    
    def go_to_point(self):
        """Comportamento: ir diretamente ao ponto meta"""
        cmd = Twist()
        
        # Calcular ângulo para a meta
        angle_to_goal = math.atan2(
            self.goal.y - self.position_.y,
            self.goal.x - self.position_.x
        )
        
        # Diferença angular
        angle_diff = self.normalize_angle(angle_to_goal - self.yaw_)
        
        # Verificar se chegou na meta
        distance_to_goal = self.dist(self.position_, self.goal)
        if distance_to_goal < self.goal_tol:
            self.get_logger().info(f'Objetivo alcançado! Distância: {distance_to_goal:.3f}m')
            return None
        
        # Controle de movimento
        if abs(angle_diff) > 0.2: 
            # Rotacionar para alinhar com meta
            cmd.angular.z = self.angular_speed * angle_diff
            cmd.linear.x = 0.0
        else:
            # Mover em direção à meta
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.1 * angle_diff 
        
        return cmd
    
    def wall_follow(self):
        """Comportamento: seguir parede (algoritmo right-hand)"""
        cmd = Twist()
        
        if self.regions_ is None:
            return cmd
        
        # Estratégia: seguir parede do lado direito
        front_dist = self.regions_['front']
        right_dist = self.regions_['right']
        
        # Prioridade 1: Evitar colisão frontal
        if front_dist < self.wall_distance_threshold:
            # Virar à esquerda para evitar obstáculo
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed
            self.get_logger().debug('Wall follow: Turning left (obstacle ahead)')
            
        # Prioridade 2: Manter distância da parede direita
        elif right_dist > self.wall_follow_distance:
            # Parede muito longe - aproximar (virar direita)
            cmd.linear.x = self.linear_speed * 0.5
            cmd.angular.z = -self.angular_speed * 0.6
            self.get_logger().debug('Wall follow: Turning right (wall too far)')
            
        elif right_dist < self.wall_distance_threshold:
            # Parede muito perto - afastar (virar esquerda)
            cmd.linear.x = self.linear_speed * 0.3
            cmd.angular.z = self.angular_speed * 0.4
            self.get_logger().debug('Wall follow: Turning left (wall too close)')
            
        else:
            # Distância boa da parede - seguir em frente
            cmd.linear.x = self.linear_speed * 0.7
            cmd.angular.z = 0.0
            self.get_logger().debug('Wall follow: Moving forward')
        
        return cmd
    
    # ==================== MAIN CONTROL LOOP ====================
    
    def main_loop(self):
        """Loop principal do algoritmo Bug1"""
        if self.regions_ is None:
            return
        
        cmd = Twist()
        
        # ==================== STATE 0: GO TO GOAL ====================
        if self.state_ == 0:
            
            if self.is_obstacle_ahead():
                
                self.start_point_ = Point()
                self.start_point_.x = self.position_.x
                self.start_point_.y = self.position_.y
                self.start_point_.z = self.position_.z
                
                self.closest_point_ = Point()
                self.closest_point_.x = self.position_.x
                self.closest_point_.y = self.position_.y
                self.closest_point_.z = self.position_.z
                
                self.state_ = 1
                self.get_logger().info('Obstacle detected! Starting wall following...')
                self.get_logger().info(f'Start point: ({self.start_point_.x:.2f}, {self.start_point_.y:.2f})')
            else:
            
                cmd = self.go_to_point() or Twist()
        
        # ==================== STATE 1: WALL FOLLOWING ====================
        elif self.state_ == 1:
           
            current_dist = self.dist(self.position_, self.goal)
            closest_dist = self.dist(self.closest_point_, self.goal)
            
            if current_dist < closest_dist:
                self.closest_point_.x = self.position_.x
                self.closest_point_.y = self.position_.y
                self.closest_point_.z = self.position_.z
                self.get_logger().info(f'New closest point: ({self.closest_point_.x:.2f}, {self.closest_point_.y:.2f})')
                self.get_logger().info(f'Distance to goal: {current_dist:.3f}m')
            
            # Verificar se voltou ao ponto de início (completou circunavegação)
            dist_to_start = self.dist(self.position_, self.start_point_)
            if dist_to_start < 0.3:
                self.state_ = 2
                self.get_logger().info('Circumnavigation complete! Going to closest point...')
                self.get_logger().info(f'Closest point: ({self.closest_point_.x:.2f}, {self.closest_point_.y:.2f})')
            
            # Executar wall following
            cmd = self.wall_follow()
        
        # ==================== STATE 2: GO TO CLOSEST POINT ====================
        elif self.state_ == 2:
            # Verificar se chegou no ponto mais próximo
            dist_to_closest = self.dist(self.position_, self.closest_point_)
            if dist_to_closest < 0.3:
                self.state_ = 0
                self.get_logger().info('Reached closest point! Resuming direct navigation...')
            
            # Continuar wall following até chegar no ponto mais próximo
            cmd = self.wall_follow()
        
        # ==================== PUBLISH COMMAND ====================
        self.cmd_pub.publish(cmd)


# ==================== MAIN FUNCTION ====================

def main(args=None):
    """Função principal"""
    rclpy.init(args=args)
    
    node = Bug1Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Bug1 Node stopped by user')
    finally:
        # Parar robô antes de encerrar
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()