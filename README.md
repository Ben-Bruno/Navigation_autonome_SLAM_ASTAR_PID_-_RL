# Projet Robotique I2 — Navigation Autonome TurtleBot3

## Description

Ce projet implémente un système de navigation et d'exploration autonome pour le robot TurtleBot3 Waffle dans un environnement simulé Gazebo. Le robot est capable de cartographier un espace inconnu de manière entierement autonome, d'eviter les obstacles, et de sauvegarder la carte produite.

La stack est construite sur ROS2 Humble et integre les composants suivants : Gazebo pour la simulation physique, SLAM Toolbox pour la cartographie simultanee et la localisation, NAV2 pour la planification de trajectoire et le controle, ainsi qu'explore_lite pour la detection et le ciblage automatique des frontieres inexplorees.

---

## Architecture du Systeme

```
Gazebo (simulation)
    |-- TurtleBot3 Waffle (lidar, odometrie, camera)
    |
    v
SLAM Toolbox (online_async)
    |-- Publie : /map (carte incrementale)
    |-- Consomme : /scan, /odom, /tf
    |
    v
NAV2
    |-- global_costmap  (planification A*)
    |-- local_costmap   (evitement DWB)
    |-- bt_navigator    (orchestration)
    |
    v
explore_lite
    |-- Detecte les frontieres sur global_costmap/costmap
    |-- Envoie des goals successifs a NAV2
    |
    v
Superviseur (optionnel)
    |-- Attend la fin d'explore_lite
    |-- Ordonne le retour a la base
    |-- Sauvegarde la carte finale
    |-- Genere les graphiques de mission
```

---

## Prerequis

- Ubuntu 22.04
- ROS2 Humble (installation complete)
- Gazebo 11 (inclus avec ROS2 Humble Desktop)
- TurtleBot3 packages
- SLAM Toolbox
- NAV2
- explore_lite (fork compatible ROS2)
- Python 3.10+
- matplotlib (pour les graphiques du superviseur)

---

## Structure du Depot

```
projet_robotique_i2/
|-- config/
|   |-- nav2_params.yaml          # Parametres NAV2 (DWB, costmaps, A*)
|   `-- slam_toolbox_params.yaml  # Parametres SLAM (lidar, solver Ceres)
|-- launch/
|   `-- bringup_autonomy.launch.py  # Point d'entree unique
|-- maps/
|   `-- map.pgm / map.yaml        # Carte generee (apres exploration)
|-- worlds/
|   `-- exploration_world.world   # Monde Gazebo ferme 20x20 m
|-- projet_robotique_i2/
|   |-- superviseur_navigation.py # Noeud superviseur post-exploration
|   `-- traceur_graphiques.py     # Module de visualisation matplotlib
|-- CMakeLists.txt
`-- package.xml
```

---

## Configuration Cle

### Monde Gazebo

Le monde `exploration_world.world` est une salle fermee de 20 x 20 m avec :

- Robot spawne a (0, 0) avec 4 m d'espace libre dans toutes les directions
- Quatre murs exterieurs a +/- 10 m (indispensables pour forcer la couverture totale)
- Quatre cloisons interieures formant des couloirs avec ouvertures de 2 m
- Douze colonnes cylindriques dans les salles d'angle
- Huit boites dans les couloirs pour creer des obstacles intermediaires

### Parametres NAV2 critiques

```yaml
global_costmap:
  track_unknown_space: true   # Obligatoire pour explore_lite
  inflation_radius: 0.30      # Superieur au rayon inscrit du waffle (0.225 m)

planner_server:
  GridBased:
    use_astar: true
    allow_unknown: true
```

### Parametres explore_lite critiques

```python
"costmap_topic": "global_costmap/costmap",  # Ne pas remapper vers /map
"min_frontier_size": 0.1,
"progress_timeout": 120.0,
"gain_scale": 5.0,
```

---

## Problemes Resolus

| Probleme | Cause | Solution |
|---|---|---|
| Crash gzserver (exit -11) | Double plugin diff_drive | Supprimer le modele waffle du .world |
| TF base_link avec ${namespace} | Xacro non resolu | Command(["xacro ", urdf_xacro]) |
| TF odom manquant | GAZEBO_MODEL_PATH vide | Forcer os.environ dans le launch |
| NAV2 FATAL : No critics | Indentation YAML cassee | Reecriture complete du fichier |
| explore_lite immobile | Remapping vers /map (carte figee) | Utiliser global_costmap/costmap |
| Exploration incomplete | Monde ouvert sans murs | Monde ferme 20x20 m |
| Robot s'arrete trop tot | obstacles trop pres du spawn | Obstacles a > 4 m du centre |

---

## Avertissements Non Bloquants

Ces messages apparaissent dans les logs mais n'affectent pas le fonctionnement :

- `minimum laser range (0.0 m) exceeds lidar (0.1 m)` : plage clippee automatiquement
- `BehaviorTree tick rate 100Hz exceeded` : CPU charge, navigation fonctionnelle
- `Control loop missed desired rate 20Hz` : charge CPU, non bloquant
- `GLSL link result : active samplers` : bug OpenGL RViz2, purement cosmétique

---

## Auteur

 Bruno, Hugues et Marcelle — Projet I2, 2026
