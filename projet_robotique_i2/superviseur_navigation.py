#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Superviseur_navigation
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import time
import os

# Importation de ton module d'affichage modulaire
from projet_robotique_i2.traceur_graphiques import afficher_bilan_navigation

class ChefOrchestreNavigation:
    """
    Nœud superviseur qui commande la stack Nav2 et récolte la télémétrie.
    """
    def __init__(self):
        self.navigateur = BasicNavigator()
        
    def attendre_systeme_pret(self):
        """Bloque l'exécution tant que le SLAM, A* et DWA ne sont pas actifs."""
        print("⏳ Attente de l'initialisation des serveurs de navigation...")
        self.navigateur.nav_to_pose_client.wait_for_server()
        print("✅ Système de navigation prêt !")

    def attendre_fin_exploration(self):
        print("🌍 Exploration en cours...")
    # Attendre que explore_node disparaisse du réseau
        import subprocess
        while True:
            result = subprocess.run(
                ['ros2', 'node', 'list'], 
                capture_output=True, text=True
            )
            if '/explore_node' not in result.stdout:
                print("🏁 Fin de l'exploration détectée !")
                break
            time.sleep(2.0)
    def lancer_retour_base(self):
        """
        Envoie le robot à (0,0) avec A* et DWA, récolte les données et trace les graphes.
        """
        cible_x = 0.0
        cible_y = 0.0
        
        pose_cible = PoseStamped()
        pose_cible.header.frame_id = 'map'
        pose_cible.header.stamp = self.navigateur.get_clock().now().to_msg()
        pose_cible.pose.position.x = cible_x
        pose_cible.pose.position.y = cible_y
        pose_cible.pose.orientation.w = 1.0 # Orientation neutre

        print(f"🚀 Ordre envoyé : Retour à la base (X={cible_x}, Y={cible_y}) pour fermeture de boucle SLAM")
        self.navigateur.goToPose(pose_cible)

        # Variables pour stocker les données du graphique
        historique_temps = []
        historique_x = []
        historique_y = []
        historique_distances = []
        
        temps_debut = time.time()

        # Boucle de surveillance pendant le déplacement
        while not self.navigateur.isTaskComplete():
            retour_info = self.navigateur.getFeedback()
            
            if retour_info:
                temps_ecoule = time.time() - temps_debut
                distance_restante = retour_info.distance_remaining
                
                # Récupération de la position exacte calculée par l'odométrie et AMCL
                pos_actuelle = retour_info.current_pose.pose.position
                
                # Sauvegarde des données pour le graphique post-mission
                historique_temps.append(temps_ecoule)
                historique_x.append(pos_actuelle.x)
                historique_y.append(pos_actuelle.y)
                historique_distances.append(distance_restante)

                print(f"En route... Distance : {distance_restante:.2f} m | Temps : {temps_ecoule:.1f} s", end="\r")
                time.sleep(0.1) # Échantillonnage à 10 Hz

        # Fin de la mission, analyse du résultat
        resultat = self.navigateur.getResult()
        print("\n") 
        
        if resultat == TaskResult.SUCCEEDED:
            print("🎉 Retour à la base réussi ! Fermeture de boucle SLAM effectuée.")
            
            # --- 1. SAUVEGARDE DE LA CARTE ---
            print("📸 Sauvegarde de la carte en cours...")
            chemin_carte = os.path.expanduser('~/turtlebot3_ws/src/projet_robotique_i2/maps/carte_finale')
            os.system(f"ros2 run nav2_map_server map_saver_cli -f {chemin_carte} --ros-args -p use_sim_time:=true")
            print(f"✅ Carte sauvegardée dans : {chemin_carte}")
            
            # --- 2. GÉNÉRATION DES GRAPHIQUES ---
            print("📊 Génération des graphiques...")
            afficher_bilan_navigation(
                historique_temps, historique_x, historique_y, historique_distances, 
                cible_x, cible_y
            )
        elif resultat == TaskResult.CANCELED:
            print("⚠️ Mission annulée.")
        elif resultat == TaskResult.FAILED:
            print("❌ Échec de la navigation vers la base.")

def main():
    rclpy.init()
    
    superviseur = ChefOrchestreNavigation()
    
    # 1. Attendre que Nav2 soit prêt
    superviseur.attendre_systeme_pret()
    
    # 2. Attendre que le robot ait fini d'explorer la carte
    superviseur.attendre_fin_exploration()
    
    # 3. Prendre le contrôle, revenir, sauvegarder et tracer !
    superviseur.lancer_retour_base()

    rclpy.shutdown()

if __name__ == '__main__':
    main()