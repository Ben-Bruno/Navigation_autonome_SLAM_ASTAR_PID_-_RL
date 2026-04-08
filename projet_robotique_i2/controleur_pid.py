#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import time
import numpy as np
import matplotlib.pyplot as plt
from tf_transformations import euler_from_quaternion

class PIDControllerNode(Node):
    def __init__(self):
        super().__init__('controleur_pid')
        
        # PUBLISHERS & SUBSCRIBERS
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # PARAMÈTRES DU ROBOT
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.obstacle_devant = False
        self.distance_securite = 0.4  # m
        
        # CHEMIN À SUIVRE (Waypoints issus de A*) 
        # Remplace ces points par ceux générés par ton script A*
        self.waypoints = [(-9.100000000000001, 5.900000000000002), (-9.05, 5.900000000000002), (-9.0, 5.900000000000002), (-8.950000000000001, 5.900000000000002), (-8.9, 5.900000000000002), (-8.850000000000001, 5.900000000000002), (-8.8, 5.900000000000002), (-8.75, 5.900000000000002), (-8.700000000000001, 5.900000000000002), (-8.65, 5.900000000000002), (-8.600000000000001, 5.900000000000002), (-8.55, 5.900000000000002), (-8.5, 5.900000000000002), (-8.450000000000001, 5.900000000000002), (-8.4, 5.900000000000002), (-8.350000000000001, 5.950000000000003), (-8.3, 6.0), (-8.25, 6.0), (-8.200000000000001, 6.0), (-8.15, 6.0), (-8.100000000000001, 6.0), (-8.05, 6.0), (-8.0, 6.0), (-7.950000000000001, 6.0), (-7.9, 6.0), (-7.8500000000000005, 6.0), (-7.800000000000001, 6.0), (-7.75, 6.0), (-7.700000000000001, 6.0), (-7.65, 6.0), (-7.6000000000000005, 6.0), (-7.550000000000001, 6.0), (-7.5, 6.050000000000001), (-7.450000000000001, 6.050000000000001), (-7.4, 6.050000000000001), (-7.3500000000000005, 6.050000000000001), (-7.300000000000001, 6.050000000000001), (-7.25, 6.050000000000001), (-7.200000000000001, 6.050000000000001), (-7.15, 6.050000000000001), (-7.1000000000000005, 6.050000000000001), (-7.050000000000001, 6.050000000000001), (-7.0, 6.050000000000001), (-6.950000000000001, 6.050000000000001), (-6.9, 6.050000000000001), (-6.8500000000000005, 6.050000000000001), (-6.800000000000001, 6.050000000000001), (-6.75, 6.050000000000001), (-6.700000000000001, 6.050000000000001), (-6.65, 6.050000000000001), (-6.6000000000000005, 6.050000000000001), (-6.550000000000001, 6.050000000000001), (-6.5, 6.050000000000001), (-6.450000000000001, 6.050000000000001), (-6.4, 6.050000000000001), (-6.3500000000000005, 6.050000000000001), (-6.300000000000001, 6.050000000000001), (-6.250000000000001, 6.050000000000001), (-6.2, 6.050000000000001), (-6.15, 6.050000000000001), (-6.1000000000000005, 6.050000000000001), (-6.050000000000001, 6.050000000000001), (-6.000000000000001, 6.050000000000001), (-5.95, 6.050000000000001), (-5.9, 6.050000000000001), (-5.8500000000000005, 6.050000000000001), (-5.800000000000001, 6.050000000000001), (-5.750000000000001, 6.050000000000001), (-5.7, 6.050000000000001), (-5.65, 6.050000000000001), (-5.6000000000000005, 6.050000000000001), (-5.550000000000001, 6.050000000000001), (-5.5, 6.050000000000001), (-5.45, 6.050000000000001), (-5.4, 6.050000000000001), (-5.3500000000000005, 6.050000000000001), (-5.300000000000001, 6.050000000000001), (-5.25, 6.050000000000001), (-5.2, 6.050000000000001), (-5.15, 6.050000000000001), (-5.1000000000000005, 6.050000000000001), (-5.050000000000001, 6.050000000000001), (-5.0, 6.050000000000001), (-4.95, 6.050000000000001), (-4.9, 6.050000000000001), (-4.8500000000000005, 6.050000000000001), (-4.800000000000001, 6.050000000000001), (-4.75, 6.050000000000001), (-4.7, 6.050000000000001), (-4.65, 6.050000000000001), (-4.6000000000000005, 6.050000000000001), (-4.550000000000001, 6.050000000000001), (-4.5, 6.050000000000001), (-4.45, 6.050000000000001), (-4.4, 6.050000000000001), (-4.3500000000000005, 6.050000000000001), (-4.300000000000001, 6.050000000000001), (-4.25, 6.050000000000001), (-4.2, 6.050000000000001), (-4.15, 6.050000000000001), (-4.1000000000000005, 6.050000000000001), (-4.050000000000001, 6.050000000000001), (-4.0, 6.050000000000001), (-3.95, 6.050000000000001), (-3.9000000000000004, 6.050000000000001), (-3.8500000000000005, 6.050000000000001), (-3.8000000000000007, 6.050000000000001), (-3.75, 6.050000000000001), (-3.7, 6.050000000000001), (-3.6500000000000004, 6.050000000000001), (-3.6000000000000005, 6.050000000000001), (-3.5500000000000007, 6.050000000000001), (-3.5, 6.050000000000001), (-3.45, 6.050000000000001), (-3.4000000000000004, 6.050000000000001), (-3.3500000000000005, 6.050000000000001), (-3.3000000000000007, 6.050000000000001), (-3.25, 6.050000000000001), (-3.2, 6.050000000000001), (-3.1500000000000004, 6.050000000000001), (-3.1000000000000005, 6.050000000000001), (-3.0500000000000007, 6.050000000000001), (-3.0, 6.050000000000001), (-2.95, 6.050000000000001), (-2.9000000000000004, 6.050000000000001), (-2.8500000000000005, 6.050000000000001), (-2.8000000000000007, 6.050000000000001), (-2.75, 6.050000000000001), (-2.7, 6.050000000000001), (-2.6500000000000004, 6.050000000000001), (-2.6000000000000005, 6.050000000000001), (-2.5500000000000007, 6.050000000000001), (-2.5, 6.050000000000001), (-2.45, 6.050000000000001), (-2.4000000000000004, 6.050000000000001), (-2.3500000000000005, 6.050000000000001), (-2.3000000000000007, 6.050000000000001), (-2.25, 6.050000000000001), (-2.200000000000001, 6.050000000000001), (-2.1500000000000004, 6.050000000000001), (-2.0999999999999996, 6.050000000000001), (-2.0500000000000007, 6.050000000000001), (-2.0, 6.050000000000001), (-1.950000000000001, 6.050000000000001), (-1.9000000000000004, 6.050000000000001), (-1.8499999999999996, 6.050000000000001), (-1.8000000000000007, 6.050000000000001), (-1.75, 6.0), (-1.700000000000001, 6.0), (-1.6500000000000004, 6.0), (-1.5999999999999996, 6.0), (-1.5500000000000007, 6.0), (-1.5, 6.0), (-1.450000000000001, 6.0), (-1.4000000000000004, 6.0), (-1.3499999999999996, 6.0), (-1.3000000000000007, 6.0), (-1.25, 6.0), (-1.200000000000001, 6.0), (-1.1500000000000004, 5.950000000000003), (-1.0999999999999996, 5.950000000000003), (-1.0500000000000007, 5.900000000000002), (-1.0, 5.900000000000002), (-0.9500000000000011, 5.850000000000001), (-0.9000000000000004, 5.850000000000001)]
        self.current_wp_index = 0
        
        # GAINS DU PID
        # Kp (Proportionnel), Ki (Intégral), Kd (Dérivé)
        self.Kp_dist = 0.5
        self.Ki_dist = 0.01
        self.Kd_dist = 0.1
        
        self.Kp_angle = 1.5
        self.Ki_angle = 0.0
        self.Kd_angle = 0.2
        
        # Variables pour les calculs PID
        self.erreur_dist_prec = 0.0
        self.erreur_angle_prec = 0.0
        self.somme_erreur_dist = 0.0
        self.somme_erreur_angle = 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9
        
        # --- DONNÉES POUR L'ANALYSE (Graphiques) ---
        self.history_time = []
        self.history_error_dist = []
        self.history_error_angle = []
        self.history_v = []
        self.history_w = []
        self.start_time = time.time()
        
        # Boucle de contrôle à 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Contrôleur PID démarré !")

    def odom_callback(self, msg):
        """Récupère la position et l'orientation actuelles du robot."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        _, _, self.theta = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])

    def scan_callback(self, msg):
        """Vérifie s'il y a un obstacle droit devant (cône de -20° à +20°)."""
        # Le Turtlebot3 LiDAR a 360 rayons (1 par degré). On regarde devant.
        secteur_avant = msg.ranges[0:20] + msg.ranges[340:359]
        # Nettoyage des valeurs inf/NaN
        secteur_avant = [r for r in secteur_avant if not math.isinf(r) and not math.isnan(r)]
        
        if secteur_avant and min(secteur_avant) < self.distance_securite:
            self.obstacle_devant = True
        else:
            self.obstacle_devant = False

    def control_loop(self):
        """Calcul du PID et publication des vitesses."""
        if self.current_wp_index >= len(self.waypoints):
            self.get_logger().info("Objectif final atteint !")
            self.cmd_pub.publish(Twist()) # Arrêt du robot
            self.timer.cancel()
            self.generer_graphiques_analyse()
            return
            
        cible_x, cible_y = self.waypoints[self.current_wp_index]
        
        # 1. Calcul des erreurs
        erreur_x = cible_x - self.x
        erreur_y = cible_y - self.y
        erreur_dist = math.sqrt(erreur_x**2 + erreur_y**2)
        
        angle_vers_cible = math.atan2(erreur_y, erreur_x)
        erreur_angle = angle_vers_cible - self.theta
        
        # Normalisation de l'angle entre -pi et pi
        while erreur_angle > math.pi: erreur_angle -= 2.0 * math.pi
        while erreur_angle < -math.pi: erreur_angle += 2.0 * math.pi

        # Si le robot est assez proche du waypoint, on passe au suivant
        if erreur_dist < 0.15:
            self.get_logger().info(f"Waypoint {self.current_wp_index + 1} atteint !")
            self.current_wp_index += 1
            return

        # 2. Calcul du temps écoulé (dt)
        current_time = self.get_clock().now().nanoseconds / 1e9
        dt = current_time - self.last_time
        if dt <= 0.0: return
        
        # 3. Évitement d'obstacles (Priorité Absolue)
        cmd = Twist()
        if self.obstacle_devant:
            self.get_logger().warn("OBSTACLE ! Évitement en cours...")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Rotation sur place pour dégager la voie
            self.cmd_pub.publish(cmd)
            return

        # 4. Calcul PID
        # Termes Intégraux
        self.somme_erreur_dist += erreur_dist * dt
        self.somme_erreur_angle += erreur_angle * dt
        
        # Termes Dérivés
        deriv_dist = (erreur_dist - self.erreur_dist_prec) / dt
        deriv_angle = (erreur_angle - self.erreur_angle_prec) / dt

        # Loi de commande
        v = (self.Kp_dist * erreur_dist) + (self.Ki_dist * self.somme_erreur_dist) + (self.Kd_dist * deriv_dist)
        w = (self.Kp_angle * erreur_angle) + (self.Ki_angle * self.somme_erreur_angle) + (self.Kd_angle * deriv_angle)

        # Limitation des vitesses (Saturations pour le TurtleBot3)
        v = max(min(v, 0.22), -0.22)
        w = max(min(w, 2.84), -2.84)
        
        # Si le robot est très mal orienté, on l'oblige à tourner sur place avant d'avancer
        if abs(erreur_angle) > 0.5:
            v = 0.0 

        # 5. Envoi des commandes
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)
        
        # Mise à jour pour le prochain cycle
        self.erreur_dist_prec = erreur_dist
        self.erreur_angle_prec = erreur_angle
        self.last_time = current_time
        
        # Sauvegarde des données pour l'analyse
        self.history_time.append(time.time() - self.start_time)
        self.history_error_dist.append(erreur_dist)
        self.history_error_angle.append(erreur_angle)
        self.history_v.append(v)
        self.history_w.append(w)

    def generer_graphiques_analyse(self):
        """Génère les figures demandées pour le rapport."""
        plt.figure(figsize=(12, 8))

        # 1. Analyse de l'erreur de suivi (Tracking Error & Stability)
        plt.subplot(2, 1, 1)
        plt.plot(self.history_time, self.history_error_dist, label="Erreur de Distance (m)", color="blue")
        plt.plot(self.history_time, self.history_error_angle, label="Erreur d'Angle (rad)", color="red", alpha=0.6)
        plt.axhline(0, color='black', linestyle='--')
        plt.title("Analyse PID : Erreur de suivi de trajectoire et Stabilité")
        plt.xlabel("Temps (s)")
        plt.ylabel("Erreur")
        plt.legend()
        plt.grid()

        # 2. Analyse de la fluidité (Smoothness of motion)
        plt.subplot(2, 1, 2)
        plt.plot(self.history_time, self.history_v, label="Vitesse Linéaire (v)", color="green")
        plt.plot(self.history_time, self.history_w, label="Vitesse Angulaire (w)", color="orange", alpha=0.8)
        plt.title("Analyse PID : Fluidité du mouvement (Commandes de vitesse)")
        plt.xlabel("Temps (s)")
        plt.ylabel("Vitesse")
        plt.legend()
        plt.grid()

        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()