#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion
import time


class BugTangent(Node):
    """
    Bug Tangent Simplificado:
    - Navega diretamente ao goal quando possível
    - Segue parede quando obstáculo bloqueia caminho
    - Mantém direção consistente em obstáculos côncavos
    - Anti-loop básico com mudança de direção
    """

    def __init__(self):
        super().__init__('bug_tangent')

        # topics
        self.create_subscription(Odometry, '/odom', self.clbk_odom, 10)
        self.create_subscription(LaserScan, '/base_scan', self.clbk_laser, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # loop rate
        self.timer = self.create_timer(0.05, self.main_loop)  # 20 Hz

        # robot state
        self.position_ = Point()
        self.yaw_ = 0.0
        self.laser_ranges = []
        self.laser_angle_min = 0.0
        self.laser_angle_increment = 0.0

        # goal
        self.goal = Point(x=-7.0, y=-7.0, z=0.0)
        self.goal_tol = 0.25

        # algorithm state: 0=motion_to_goal, 1=boundary_following
        self.state_ = 0

        # navigation parameters
        self.linear_speed = 0.3
        self.angular_speed = 0.8
        self.safety_distance = 0.5
        self.wall_follow_distance = 0.6
        self.obstacle_threshold = 1.2

        # timing / debug
        self._debug_counter = 0
        self.last_progress_time = time.time()
        self.best_dist_to_goal = float('inf')

        # simple anti-loop
        self.preferred_wall_side = 'right'  # mantém consistência
        self.stuck_count = 0
        self.follow_start_dist = None

        self.get_logger().info(f"Bug Tangent Simple inicializado. Goal=({self.goal.x:.2f},{self.goal.y:.2f})")

    # ================= callbacks =================
    def clbk_odom(self, msg):
        self.position_ = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, self.yaw_ = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def clbk_laser(self, msg):
        # convert to list and sanitize
        self.laser_ranges = [float(r) if (r is not None and not math.isnan(r) and not math.isinf(r)) else float('inf')
                             for r in list(msg.ranges)]
        self.laser_angle_min = msg.angle_min
        self.laser_angle_increment = msg.angle_increment

    # ================= util functions =================
    def dist(self, p1, p2):
        return math.hypot(p1.x - p2.x, p1.y - p2.y)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def get_laser_reading(self, angle_deg):
        """
        angle_deg: em graus relativos à frente do robô (0 frente, + esquerda, - direita)
        retorna distância (float) — se fora do scan ou inválido retorna inf
        """
        if not self.laser_ranges:
            return float('inf')
        angle_rad = math.radians(angle_deg)
        idx = int(round((angle_rad - self.laser_angle_min) / self.laser_angle_increment))
        if idx < 0 or idx >= len(self.laser_ranges):
            return float('inf')
        val = self.laser_ranges[idx]
        return val if val > 0.0 else float('inf')

    def line_of_sight_clear(self):
        """
        Verifica LOS na direção do goal. Checa alguns raios próximos.
        """
        if not self.laser_ranges:
            return False
        angle_to_goal = math.atan2(self.goal.y - self.position_.y, self.goal.x - self.position_.x)
        rel_deg = math.degrees(self.normalize_angle(angle_to_goal - self.yaw_))
        goal_dist = self.dist(self.position_, self.goal)
        sample_angles = [rel_deg, rel_deg - 10, rel_deg + 10]

        for a in sample_angles:
            d = self.get_laser_reading(a)
            # se obstáculo mais próximo que goal dentro de um limite prático, LOS bloqueada
            if d < min(goal_dist, self.obstacle_threshold):
                return False
        return True

    # ================= wall following behaviors =================
    def boundary_following(self):
        """
        Wall-following simples com preferência pela direita.
        Evita oscilações em obstáculos côncavos mantendo direção preferida.
        """
        cmd = Twist()

        front = self.get_laser_reading(0)
        right = self.get_laser_reading(-90)
        left = self.get_laser_reading(90)
        fr = self.get_laser_reading(-45)
        fl = self.get_laser_reading(45)

        # parada de emergência
        if front < self.safety_distance:
            cmd.linear.x = 0.0
            # escolha direção com mais espaço
            cmd.angular.z = self.angular_speed if left > right else -self.angular_speed
            return cmd

        # Seguir parede na direção preferida
        if self.preferred_wall_side == 'right' and right < 2.0:
            # erro em relação ao desejado (positivo -> precisa se afastar da parede)
            err = right - self.wall_follow_distance
            # controle proporcional simples
            cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, -err))
            cmd.linear.x = self.linear_speed * 0.6
        elif self.preferred_wall_side == 'left' and left < 2.0:
            err = left - self.wall_follow_distance
            cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, err))
            cmd.linear.x = self.linear_speed * 0.6
        else:
            # sem paredes ou mudança de direção necessária
            if right < left and right < 1.5:
                self.preferred_wall_side = 'right'
            elif left < 1.5:
                self.preferred_wall_side = 'left'
            
            # navegar conservadoramente
            cmd.linear.x = self.linear_speed * 0.5
            if fr < fl:
                cmd.angular.z = 0.3  # virar esquerda
            elif fl < fr:
                cmd.angular.z = -0.3  # virar direita
        
        return cmd

    def motion_to_goal(self):
        """
        Movimento direto ao goal com verificação de segurança.
        """
        cmd = Twist()
        
        # Calcular direção para o goal
        angle_to_goal = math.atan2(self.goal.y - self.position_.y, self.goal.x - self.position_.x)
        angle_diff = self.normalize_angle(angle_to_goal - self.yaw_)
        
        # Controle de movimento
        if abs(angle_diff) > 0.2:  # Precisa rotacionar
            cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, 2.0 * angle_diff))
            cmd.linear.x = self.linear_speed * 0.3
        else:
            # Ir em frente
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.3 * angle_diff

        # segurança frontal
        if self.get_laser_reading(0) < self.safety_distance:
            cmd.linear.x = 0.0
        
        return cmd

    # ================= main loop =================
    def main_loop(self):
        if not self.laser_ranges:
            return

        cmd = Twist()
        current_dist_to_goal = self.dist(self.position_, self.goal)

        # update best distance seen so far (progress detector)
        if current_dist_to_goal + 0.01 < self.best_dist_to_goal:
            self.best_dist_to_goal = current_dist_to_goal
            self.last_progress_time = time.time()

        # GOAL reached
        if current_dist_to_goal < self.goal_tol:
            self.get_logger().info("Goal alcançado!")
            self.cmd_pub.publish(Twist())
            return

        # STATE 0: motion to goal
        if self.state_ == 0:
            if self.line_of_sight_clear():
                cmd = self.motion_to_goal()
            else:
                # obstáculo detectado -> switch para boundary following
                self.get_logger().info("Obstáculo detectado -> boundary following")
                self.state_ = 1
                self.follow_start_dist = current_dist_to_goal
                cmd = self.boundary_following()

        # STATE 1: boundary following
        elif self.state_ == 1:
            cmd = self.boundary_following()

            # if we made progress (distance smaller than when we started following) AND LOS is clear, return to motion
            if (self.follow_start_dist is not None and
                current_dist_to_goal < self.follow_start_dist - 0.1 and
                self.line_of_sight_clear()):
                self.get_logger().info("Progresso detectado -> voltando ao motion")
                self.state_ = 0

            # Anti-loop simples: se ficou preso muito tempo, muda direção preferida
            if time.time() - self.last_progress_time > 8.0:
                self.stuck_count += 1
                if self.stuck_count % 3 == 0:  # a cada 3 tentativas, muda direção
                    self.preferred_wall_side = 'left' if self.preferred_wall_side == 'right' else 'right'
                    self.get_logger().warn(f"Anti-loop: mudando para {self.preferred_wall_side}")
                self.last_progress_time = time.time()  # reset timer

        # saturate velocities
        cmd.linear.x = max(-self.linear_speed, min(self.linear_speed, cmd.linear.x))
        cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, cmd.angular.z))

        self.cmd_pub.publish(cmd)
        
        # Debug periódico
        if not hasattr(self, '_debug_counter'):
            self._debug_counter = 0
        self._debug_counter += 1
        
        if self._debug_counter % 40 == 0:  # A cada ~2s (20Hz)
            state_names = ['MOTION-TO-GOAL', 'BOUNDARY-FOLLOWING']
            self.get_logger().info(f"Estado: {state_names[self.state_]} | Dist: {current_dist_to_goal:.2f}m ")

def main(args=None):
    rclpy.init(args=args)
    node = BugTangent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Bug Tangent encerrado.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
