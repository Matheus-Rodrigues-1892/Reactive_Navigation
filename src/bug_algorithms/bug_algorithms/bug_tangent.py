#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion


class BugTangentNode(Node):
    def __init__(self):
        super().__init__('bug_tangent_node')
        
        # Subs e pubs
        self.create_subscription(Odometry, '/odom', self.clbk_odom, 10)
        self.create_subscription(LaserScan, '/base_scan', self.clbk_laser, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.05, self.main_loop)  # 20Hz para melhor responsividade

        # Estado do robô
        self.position_ = Point()
        self.yaw_ = 0.0
        self.laser_ranges = []
        self.laser_angle_min = 0.0
        self.laser_angle_increment = 0.0

        # Goal
        self.goal = Point()
        self.goal.x = 7.0
        self.goal.y = 7.0
        self.goal_tol = 0.3

        # Estado do algoritmo Bug Tangent clássico
        # 0 = motion_to_goal (indo direto ao goal)
        # 1 = boundary_following (seguindo contorno)
        self.state_ = 0
        
        # Variáveis do algoritmo Bug Tangent
        self.current_best_point = None
        self.current_heuristic = float('inf')
        self.follow_start_distance = None  # Distância do robô ao goal quando iniciou boundary following
        self.heuristic_increasing_count = 0  # Contador para detectar se heurística está aumentando
        
        # Histórico para detectar aumento da heurística
        self.heuristic_history = []
        self.max_heuristic_history = 10
        
        # Parâmetros específicos do Bug Tangent
        self.range_sensor_limit = 3.0  # Limite do sensor de distância
        self.heuristic_threshold = 0.05  # Threshold para considerar aumento da heurística

        # Parâmetros de navegação
        self.linear_speed = 0.4
        self.angular_speed = 0.6
        self.obstacle_threshold = 0.8
        self.safety_distance = 0.4
        self.wall_follow_distance = 0.6
        
        # Sistema anti-travamento simplificado
        self.stuck_counter = 0
        self.last_positions = []

        self.get_logger().info(f"Bug Tangent inicializado. Goal=({self.goal.x:.2f},{self.goal.y:.2f})")

    # ========== Callbacks ==========
    def clbk_odom(self, msg):
        self.position_ = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, self.yaw_ = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def clbk_laser(self, msg):
        self.laser_ranges = msg.ranges
        self.laser_angle_min = msg.angle_min
        self.laser_angle_increment = msg.angle_increment

    # ========== Funções auxiliares ==========
    def dist(self, p1, p2):
        return math.hypot(p1.x - p2.x, p1.y - p2.y)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def get_laser_reading(self, angle_deg):
        """Obter leitura do laser em um ângulo específico (em graus)"""
        if not self.laser_ranges:
            return float('inf')
        
        angle_rad = math.radians(angle_deg)
        index = int((angle_rad - self.laser_angle_min) / self.laser_angle_increment)
        index = max(0, min(index, len(self.laser_ranges) - 1))
        
        reading = self.laser_ranges[index]
        return reading if not math.isinf(reading) and not math.isnan(reading) else 10.0

    def obstacle_in_range(self):
        """Detectar se há obstáculo dentro do alcance do sensor"""
        if not self.laser_ranges:
            return False
        
        # Verificar se há obstáculos em qualquer direção dentro do range limit
        for distance in self.laser_ranges:
            if 0.1 < distance < self.range_sensor_limit:
                return True
        return False
    
    def compute_heuristic_point(self):
        """Encontrar ponto no obstáculo que minimiza função heurística"""
        if not self.laser_ranges:
            return None, float('inf')
        
        best_point = None
        best_heuristic = float('inf')
        
        # Examinar todos os pontos detectados pelo sensor de alcance
        for i, distance in enumerate(self.laser_ranges):
            # Filtrar pontos válidos (dentro do range do sensor)
            if distance < 0.2 or distance > self.range_sensor_limit:
                continue
            
            # Calcular ângulo e posição do ponto
            angle = self.laser_angle_min + i * self.laser_angle_increment
            world_angle = self.yaw_ + angle
            
            point_x = self.position_.x + distance * math.cos(world_angle)
            point_y = self.position_.y + distance * math.sin(world_angle)
            obstacle_point = Point(x=point_x, y=point_y, z=0.0)
            
            # Calcular função heurística: distância(robô, ponto) + distância(ponto, goal)
            dist_robot_to_point = self.dist(self.position_, obstacle_point)
            dist_point_to_goal = self.dist(obstacle_point, self.goal)
            heuristic_cost = dist_robot_to_point + dist_point_to_goal
            
            # Atualizar melhor ponto
            if heuristic_cost < best_heuristic:
                best_heuristic = heuristic_cost
                best_point = obstacle_point
        
        return best_point, best_heuristic
    
    def is_heuristic_increasing(self, current_heuristic):
        """Detectar se função heurística está aumentando"""
        self.heuristic_history.append(current_heuristic)
        
        # Manter apenas histórico recente
        if len(self.heuristic_history) > self.max_heuristic_history:
            self.heuristic_history.pop(0)
        
        # Precisamos de pelo menos 5 amostras para decidir
        if len(self.heuristic_history) < 5:
            return False
        
        # Verificar se há tendência crescente nas últimas amostras
        recent_values = self.heuristic_history[-5:]
        increasing_count = 0
        
        for i in range(1, len(recent_values)):
            if recent_values[i] > recent_values[i-1] + self.heuristic_threshold:
                increasing_count += 1
        
        # Se mais da metade das comparações mostram aumento, consideramos que está aumentando
        return increasing_count >= 3

    def line_of_sight_clear(self):
        """Checar se não há obstáculo na direção do goal (melhorado)"""
        if not self.laser_ranges:
            return False
        
        # Calcular ângulo para o goal
        angle_to_goal = math.atan2(self.goal.y - self.position_.y,
                                   self.goal.x - self.position_.x)
        angle_to_goal_deg = math.degrees(self.normalize_angle(angle_to_goal - self.yaw_))
        
        # Verificar múltiplos ângulos ao redor da direção do goal
        check_angles = [angle_to_goal_deg - 15, angle_to_goal_deg, angle_to_goal_deg + 15]
        
        for angle in check_angles:
            distance = self.get_laser_reading(angle)
            goal_distance = self.dist(self.position_, self.goal)
            
            # Se obstáculo está mais perto que o goal, caminho não está livre
            if distance < min(goal_distance, 2.0):  # Pelo menos 2m de visão livre
                return False
        
        return True

    def compute_best_tangent_point(self):
        """Escolhe ponto da borda que minimiza custo heurístico (melhorado)"""
        if not self.laser_ranges:
            return None
        
        best_point = None
        best_cost = float('inf')
    # ========== Comportamentos de Navegação Bug Tangent ==========
    
    def motion_to_goal(self):
        """Comportamento: Motion-to-goal - ir diretamente ao goal"""
        cmd = Twist()
        
        # Verificar se chegou ao goal
        dist_to_goal = self.dist(self.position_, self.goal)
        if dist_to_goal < self.goal_tol:
            self.get_logger().info("🎯 Goal alcançado!")
            return Twist()
        
        # Calcular direção para o goal
        angle_to_goal = math.atan2(self.goal.y - self.position_.y,
                                   self.goal.x - self.position_.x)
        angle_diff = self.normalize_angle(angle_to_goal - self.yaw_)
        
        # Controle de movimento
        if abs(angle_diff) > 0.15:  # Precisa rotacionar
            cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, 2.5 * angle_diff))
            cmd.linear.x = self.linear_speed * 0.3
        else:
            # Ir em frente
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.3 * angle_diff
        
        return cmd
    
    def motion_to_best_point(self):
        """Comportamento: ir ao ponto que minimiza heurística"""
        cmd = Twist()
        
        if not self.current_best_point:
            return cmd
        
        # Verificar se chegou próximo ao ponto
        dist_to_point = self.dist(self.position_, self.current_best_point)
        if dist_to_point < 0.3:
            # Chegou ao ponto - recalcular
            self.current_best_point, self.current_heuristic = self.compute_heuristic_point()
            if not self.current_best_point:
                return self.boundary_following()
        
        # Calcular direção para o ponto
        angle_to_point = math.atan2(self.current_best_point.y - self.position_.y,
                                    self.current_best_point.x - self.position_.x)
        angle_diff = self.normalize_angle(angle_to_point - self.yaw_)
        
        # Verificar segurança frontal
        front_distance = self.get_laser_reading(0)
        
        if front_distance < self.safety_distance:
            # Muito próximo de obstáculo - apenas rotacionar
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
        elif abs(angle_diff) > 0.2:
            # Precisa rotacionar
            cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, 2.0 * angle_diff))
            cmd.linear.x = self.linear_speed * 0.4
        else:
            # Movimento normal
            cmd.linear.x = self.linear_speed * 0.7
            cmd.angular.z = 0.5 * angle_diff
        
        return cmd
    
    def boundary_following(self):
        """Comportamento: seguir contorno do obstáculo (wall following)"""
        cmd = Twist()
        
        # Usar wall following baseado na parede mais próxima
        front_dist = self.get_laser_reading(0)
        right_dist = self.get_laser_reading(-90)
        left_dist = self.get_laser_reading(90)
        front_right = self.get_laser_reading(-45)
        front_left = self.get_laser_reading(45)
        
        # Prioridade 1: Evitar colisão frontal
        if front_dist < self.safety_distance:
            cmd.linear.x = 0.0
            # Escolher direção com mais espaço
            if left_dist > right_dist:
                cmd.angular.z = self.angular_speed
            else:
                cmd.angular.z = -self.angular_speed
                
        # Prioridade 2: Seguir parede (preferir direita para consistência)
        elif right_dist < self.wall_follow_distance and right_dist > 0.1:
            # Seguir parede direita
            if right_dist > self.wall_follow_distance * 1.3:
                # Parede longe - aproximar
                cmd.angular.z = -0.4
                cmd.linear.x = self.linear_speed * 0.6
            elif right_dist < self.wall_follow_distance * 0.7:
                # Parede perto - afastar
                cmd.angular.z = 0.4
                cmd.linear.x = self.linear_speed * 0.5
            else:
                # Distância boa - seguir paralelo
                cmd.linear.x = self.linear_speed * 0.7
                cmd.angular.z = -0.1  # Leve tendência para direita
                
        elif left_dist < self.wall_follow_distance and left_dist > 0.1:
            # Seguir parede esquerda
            if left_dist > self.wall_follow_distance * 1.3:
                # Parede longe - aproximar
                cmd.angular.z = 0.4
                cmd.linear.x = self.linear_speed * 0.6
            elif left_dist < self.wall_follow_distance * 0.7:
                # Parede perto - afastar
                cmd.angular.z = -0.4
                cmd.linear.x = self.linear_speed * 0.5
            else:
                # Distância boa - seguir paralelo
                cmd.linear.x = self.linear_speed * 0.7
                cmd.angular.z = 0.1  # Leve tendência para esquerda
        else:
            # Sem paredes próximas - navegar conservativamente
            cmd.linear.x = self.linear_speed * 0.5
            
            # Se há obstáculo diagonal, evitá-lo
            if front_right < front_left:
                cmd.angular.z = 0.3  # Virar esquerda
            elif front_left < front_right:
                cmd.angular.z = -0.3  # Virar direita
            else:
                cmd.angular.z = 0.0
        
        return cmd
        if dist_to_goal < self.goal_tol:
            self.get_logger().info("Goal alcançado!")
            return Twist()
        
        # Calcular ângulo para o goal
        angle_to_goal = math.atan2(self.goal.y - self.position_.y,
                                   self.goal.x - self.position_.x)
        angle_diff = self.normalize_angle(angle_to_goal - self.yaw_)
        
        # Verificar obstáculos frontais
        front_distance = self.get_laser_reading(0)
        
        # Controle de movimento adaptativo
        if abs(angle_diff) > 0.2:  # Precisa rotacionar
            cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, 3.0 * angle_diff))
            cmd.linear.x = self.linear_speed * 0.3  # Reduzir velocidade ao girar
        else:
            # Movimento para frente com segurança
            if front_distance > 1.0:
                cmd.linear.x = self.linear_speed
            elif front_distance > 0.5:
                cmd.linear.x = self.linear_speed * 0.6
            else:
                cmd.linear.x = self.linear_speed * 0.3
                
            cmd.angular.z = 0.5 * angle_diff  # Correção fina
        
        return cmd

    def follow_tangent(self):
        """Seguir ponto tangente (melhorado)"""
        cmd = Twist()
        
        if not self.tangent_point:
            # Se não tem ponto tangente, tentar wall following básico
            return self.basic_wall_follow()
        
        # Verificar se chegou próximo ao ponto tangente
        dist_to_tangent = self.dist(self.position_, self.tangent_point)
        if dist_to_tangent < 0.4:
            # Chegou ao ponto tangente, atualizar
            self.tangent_point = self.compute_best_tangent_point()
            if not self.tangent_point:
                return self.basic_wall_follow()
        
        # Calcular direção para o ponto tangente
        angle_to_point = math.atan2(self.tangent_point.y - self.position_.y,
                                    self.tangent_point.x - self.position_.x)
        angle_diff = self.normalize_angle(angle_to_point - self.yaw_)
        
        # Verificar segurança frontal
        front_distance = self.get_laser_reading(0)
        
        if front_distance < self.safety_distance:
            # Obstáculo muito próximo - parar e girar
            cmd.linear.x = 0.0
            cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
        elif abs(angle_diff) > 0.3:
            # Precisa girar significativamente
            cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, 2.0 * angle_diff))
            cmd.linear.x = self.linear_speed * 0.4
        else:
            # Movimento normal
            cmd.linear.x = self.linear_speed * 0.7
            cmd.angular.z = 0.8 * angle_diff
        
        return cmd

    def basic_wall_follow(self):
        """Wall following básico quando tangent falha"""
        cmd = Twist()
        
        front_dist = self.get_laser_reading(0)
        right_dist = self.get_laser_reading(-90)
        left_dist = self.get_laser_reading(90)
        
        if front_dist < self.safety_distance:
            # Obstáculo à frente - virar
            cmd.linear.x = 0.0
            if left_dist > right_dist:
                cmd.angular.z = self.angular_speed
            else:
                cmd.angular.z = -self.angular_speed
        else:
            # Seguir parede mais próxima
            if right_dist < left_dist and right_dist < 1.5:
                # Seguir parede direita
                if right_dist > 0.8:
                    cmd.angular.z = -0.3  # Aproximar da parede
                elif right_dist < 0.4:
                    cmd.angular.z = 0.3   # Afastar da parede
                else:
                    cmd.angular.z = 0.0   # Seguir paralelo
                cmd.linear.x = self.linear_speed * 0.6
            elif left_dist < 1.5:
                # Seguir parede esquerda
                if left_dist > 0.8:
                    cmd.angular.z = 0.3   # Aproximar da parede
                elif left_dist < 0.4:
                    cmd.angular.z = -0.3  # Afastar da parede
                else:
                    cmd.angular.z = 0.0   # Seguir paralelo
                cmd.linear.x = self.linear_speed * 0.6
            else:
                # Sem paredes próximas - ir em direção ao goal
                angle_to_goal = math.atan2(self.goal.y - self.position_.y,
                                           self.goal.x - self.position_.x)
                angle_diff = self.normalize_angle(angle_to_goal - self.yaw_)
                cmd.angular.z = 0.5 * angle_diff
                cmd.linear.x = self.linear_speed * 0.5
        
        return cmd

    # ========== Loop Principal Bug Tangent Clássico ==========
    def main_loop(self):
        """Loop principal do algoritmo Bug Tangent seguindo especificação clássica"""
        if not self.laser_ranges:
            return

        cmd = Twist()
        
        # Verificar se chegou ao goal
        current_dist_to_goal = self.dist(self.position_, self.goal)
        if current_dist_to_goal < self.goal_tol:
            self.get_logger().info("Goal alcançado!")
            self.cmd_pub.publish(Twist())
            return

        # ========== STATE 0: MOTION-TO-GOAL ==========
        if self.state_ == 0:
            # Verificar se detecta obstáculo no sensor de alcance
            if self.obstacle_in_range():
                # Obstáculo detectado - calcular ponto que minimiza heurística
                self.current_best_point, self.current_heuristic = self.compute_heuristic_point()
                
                if self.current_best_point:
                    self.get_logger().info(f"Obstáculo detectado! Indo para ponto heurístico (custo: {self.current_heuristic:.2f})")
                    self.get_logger().info(f"Ponto alvo: ({self.current_best_point.x:.2f}, {self.current_best_point.y:.2f})")
                    
                    # Ir ao ponto que minimiza heurística
                    cmd = self.motion_to_best_point()
                else:
                    # Não conseguiu calcular ponto - usar boundary following
                    self.get_logger().warn("Não conseguiu calcular ponto heurístico, iniciando boundary following")
                    self.state_ = 1
                    self.follow_start_distance = current_dist_to_goal
                    cmd = self.boundary_following()
            else:
                # Caminho livre - ir diretamente ao goal
                cmd = self.motion_to_goal()
            
            # Verificar se heurística começou a aumentar
            if self.current_best_point and self.current_heuristic != float('inf'):
                if self.is_heuristic_increasing(self.current_heuristic):
                    self.get_logger().info("Heurística aumentando! Iniciando boundary following...")
                    self.state_ = 1
                    self.follow_start_distance = current_dist_to_goal
                    cmd = self.boundary_following()

        # ========== STATE 1: BOUNDARY-FOLLOWING ==========
        elif self.state_ == 1:
            cmd = self.boundary_following()
            
            # Condição para voltar ao motion-to-goal:
            # Distância atual ao goal < distância quando iniciou boundary following
            if current_dist_to_goal < self.follow_start_distance - 0.1:  # 10cm de margem
                self.get_logger().info(f"Progresso feito! Voltando ao motion-to-goal...")
                self.get_logger().info(f"Distância atual: {current_dist_to_goal:.2f}m | Distância inicial: {self.follow_start_distance:.2f}m")
                
                # Reset para estado motion-to-goal
                self.state_ = 0
                self.current_best_point = None
                self.current_heuristic = float('inf')
                self.heuristic_history.clear()
                self.follow_start_distance = None
                
                cmd = self.motion_to_goal()

        # Aplicar limites de segurança
        cmd.linear.x = max(-self.linear_speed, min(self.linear_speed, cmd.linear.x))
        cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, cmd.angular.z))
        
        # Debug periódico
        if not hasattr(self, '_debug_counter'):
            self._debug_counter = 0
        self._debug_counter += 1
        
        if self._debug_counter % 40 == 0:  # A cada 2 segundos (20Hz)
            state_names = ['MOTION-TO-GOAL', 'BOUNDARY-FOLLOWING']
            self.get_logger().info(f"Estado: {state_names[self.state_]} | Distância ao goal: {current_dist_to_goal:.2f}m")
            if self.current_heuristic != float('inf'):
                self.get_logger().info(f"Heurística atual: {self.current_heuristic:.2f}")
        
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BugTangentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Bug Tangent encerrado.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
