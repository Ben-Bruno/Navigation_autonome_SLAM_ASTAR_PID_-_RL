import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf_transformations import euler_from_quaternion
import numpy as np
import math
import time

class TurtleBotEnv(Node):
    def __init__(self, goal_x, goal_y):
        super().__init__('turtlebot_rl_env')
        
        # --- PUBLISHERS & SUBSCRIBERS ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        # Service pour réinitialiser la simulation Gazebo à chaque épisode
        self.reset_client = self.create_client(Empty, '/reset_simulation')
        
        # --- PARAMÈTRES DU BUT ---
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.max_distance = math.hypot(10.0, 10.0) # Normalisation
        
        # --- VARIABLES D'ÉTAT ---
        self.laser_state = np.zeros(24) # 24 rayons LiDAR
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.collision = False
        self.min_distance_mur = 0.15 # 15 cm
        self.dist_to_goal_prec = 0.0

        # --- ACTIONS DISCRÈTES (Action Space) ---
        # [v, w] : Avancer, Tourner Gauche, Tourner Droite, Pivoter G, Pivoter D
        self.action_space = [
            [0.22, 0.0],
            [0.15, 0.5],
            [0.15, -0.5],
            [0.0, 1.0],
            [0.0, -1.0]
        ]

    def scan_cb(self, msg):
        """Compresse les 360 rayons du LiDAR en 24 rayons pour simplifier l'état."""
        ranges = np.array(msg.ranges)
        ranges[np.isnan(ranges) | np.isinf(ranges)] = 3.5 # Remplacer les erreurs par la portée max
        
        # On découpe en 24 secteurs de 15 degrés et on prend le minimum de chaque secteur
        secteurs = np.array_split(ranges, 24)
        self.laser_state = np.array([np.min(sec) for sec in secteurs])
        
        # Détection de collision
        if np.min(self.laser_state) < self.min_distance_mur:
            self.collision = True

    def odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.robot_theta = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def get_state(self):
        """Construit le vecteur d'état (State Representation)."""
        dist_to_goal = math.hypot(self.goal_x - self.robot_x, self.goal_y - self.robot_y)
        angle_to_goal = math.atan2(self.goal_y - self.robot_y, self.goal_x - self.robot_x)
        heading_error = angle_to_goal - self.robot_theta
        
        # Normalisation de l'angle entre -pi et pi
        while heading_error > math.pi: heading_error -= 2.0 * math.pi
        while heading_error < -math.pi: heading_error += 2.0 * math.pi
        
        # L'état = 24 rayons laser + distance normalisée + erreur d'angle
        state = np.append(self.laser_state / 3.5, [dist_to_goal / self.max_distance, heading_error / math.pi])
        return state, dist_to_goal

    def reset(self):
        """Réinitialise l'épisode : replace le robot au centre et renvoie l'état initial."""
        req = Empty.Request()
        while not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Attente du service /reset_simulation...')
        self.reset_client.call_async(req)
        
        # Stopper le robot
        self.cmd_pub.publish(Twist())
        time.sleep(0.5) # Laisser le temps à Gazebo de se remettre en place
        
        self.collision = False
        state, self.dist_to_goal_prec = self.get_state()
        return state

    def step(self, action_index):
        """Exécute une action, calcule la récompense et renvoie le nouvel état."""
        # 1. Envoyer l'action au robot
        action = self.action_space[action_index]
        cmd = Twist()
        cmd.linear.x = action[0]
        cmd.angular.z = action[1]
        self.cmd_pub.publish(cmd)
        
        # Laisser l'action s'exécuter un court instant (ex: 0.1 seconde)
        time.sleep(0.1)
        
        # 2. Lire le nouvel état
        next_state, dist_to_goal = self.get_state()
        done = False
        reward = 0.0
        
        # 3. Calculer la récompense (Reward Function)
        if self.collision:
            reward = -100.0
            done = True
            self.get_logger().info("CRASH ! Mur touché.")
        elif dist_to_goal < 0.2:
            reward = 100.0
            done = True
            self.get_logger().info(" OBJECTIF ATTEINT !")
        else:
            # Récompense de progression (le robot gagne des points s'il s'approche)
            reward = (self.dist_to_goal_prec - dist_to_goal) * 100.0
            # Pénalité de temps pour l'inciter à aller vite
            reward -= 0.5 
            
        self.dist_to_goal_prec = dist_to_goal
        
        return next_state, reward, done