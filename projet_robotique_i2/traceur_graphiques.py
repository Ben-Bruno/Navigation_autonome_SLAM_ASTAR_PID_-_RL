#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Mar 28 21:17:44 2026

@author: benbruno
"""
# Traceur graphiques 

import matplotlib.pyplot as plt

def afficher_bilan_navigation(temps, positions_x, positions_y, distances, cible_x, cible_y):
    """
    Génère deux graphiques post-déplacement :
    1. La trajectoire géométrique (X, Y)
    2. L'évolution de la distance par rapport au temps
    """
    # Création d'une fenêtre avec deux sous-graphiques
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # --- Graphique 1 : Trajectoire (Vue de dessus) ---
    ax1.plot(positions_x, positions_y, label='Trajectoire réelle du robot', color='blue', linewidth=2)
    ax1.scatter(positions_x[0], positions_y[0], color='green', s=100, label='Départ', zorder=5)
    ax1.scatter(cible_x, cible_y, color='red', s=100, label='Cible (A*)', zorder=5)
    
    ax1.set_title("Trajectoire suivie sur la carte")
    ax1.set_xlabel("Position X (m)")
    ax1.set_ylabel("Position Y (m)")
    ax1.grid(True)
    ax1.legend()
    ax1.axis('equal') # Pour que les mètres en X et Y aient la même échelle

    # --- Graphique 2 : Distance en fonction du temps ---
    ax2.plot(temps, distances, label='Distance restante', color='orange', linewidth=2)
    ax2.set_title("Évolution de la distance vers la cible")
    ax2.set_xlabel("Temps (secondes)")
    ax2.set_ylabel("Distance (m)")
    ax2.grid(True)
    ax2.legend()

    # Affichage des graphiques
    plt.tight_layout()
    plt.show()