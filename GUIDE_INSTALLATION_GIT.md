# Guide d'Installation, de Lancement et de Depot Git

---

## Partie 1 — Installation et Lancement sur Machine

### 1.1 Prerequis systeme

Verifier que ROS2 Humble est installe et source :

```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

Installer les dependances ROS2 si ce n'est pas deja fait :

```bash
sudo apt update
sudo apt install -y \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-description \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-nav2-msgs \
  ros-humble-gazebo-ros-pkgs \
  python3-pip

pip3 install matplotlib
```

### 1.2 Cloner le projet

```bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone https://github.com/<votre-compte>/projet_robotique_i2.git
```

Cloner egalement les dependances externes si elles ne sont pas dans le depot :

```bash
# explore_lite (fork ROS2)
git clone https://github.com/robo-friends/m-explore-ros2.git

# (optionnel) multirobot_map_merge si present dans le workspace
git clone https://github.com/robo-friends/m-explore-ros2.git
```

### 1.3 Construire le workspace

```bash
cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash
```

Ajouter le source au bashrc pour ne pas avoir a le refaire :

```bash
echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc
```

### 1.4 Verifier l'installation

```bash
# Verifier que le package est visible
ros2 pkg list | grep projet_robotique_i2

# Verifier que le launch est accessible
ros2 launch projet_robotique_i2 bringup_autonomy.launch.py --show-args
```

### 1.5 Lancer le systeme complet

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch projet_robotique_i2 bringup_autonomy.launch.py
```

La sequence de demarrage automatique est la suivante :

| Delai | Composant |
|---|---|
| 0 s | Gazebo + monde |
| 3 s | Spawn du robot |
| 6 s | SLAM Toolbox |
| 10 s | NAV2 + RViz2 |
| 18 s | explore_lite |
| 300 s | Sauvegarde automatique de la carte |

### 1.6 Lancer le superviseur (terminal separe, optionnel)

Dans un second terminal, apres que explore_lite a demarre :

```bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run projet_robotique_i2 superviseur_navigation
```

Le superviseur attend la fin de l'exploration, puis ordonne le retour a la base, sauvegarde la carte et genere les graphiques de mission.

### 1.7 Sauvegarder la carte manuellement

Si la sauvegarde automatique n'a pas eu lieu ou si vous souhaitez sauvegarder a un instant precis :

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/turtlebot3_ws/src/projet_robotique_i2/maps/map \
  --ros-args -p use_sim_time:=true
```

### 1.8 Verifier les topics principaux

```bash
# Carte SLAM
ros2 topic echo /map --once

# Scan lidar
ros2 topic echo /scan --once

# Odometrie
ros2 topic echo /odom --once

# Costmap global (utilise par explore_lite)
ros2 topic list | grep global_costmap
```

---

## Partie 2 — Depot sur GitHub via Git

### 2.1 Initialiser le depot local

```bash
cd ~/turtlebot3_ws/src/projet_robotique_i2
git init
git branch -M main
```

### 2.2 Configurer son identite Git (a faire une seule fois par machine)

```bash
git config --global user.name "Votre Nom"
git config --global user.email "votre.email@exemple.com"
```

### 2.3 Creer le fichier .gitignore

Certains fichiers generees ne doivent pas etre commites :

```bash
cat > .gitignore << 'EOF'
# ROS2 build artifacts
build/
install/
log/

# Cartes generees (optionnel, peut etre versionne si souhaite)
maps/*.pgm
maps/*.yaml

# Python cache
__pycache__/
*.pyc
*.pyo

# Fichiers systeme
.DS_Store
*.swp
*~
EOF
```

### 2.4 Ajouter les fichiers et faire le premier commit

```bash
git add .
git status          # Verifier ce qui sera commite
git commit -m "Initial commit : navigation autonome TurtleBot3 ROS2 Humble"
```

### 2.5 Creer le depot sur GitHub

Aller sur https://github.com, se connecter, puis :

1. Cliquer sur "New repository"
2. Nommer le depot : `projet_robotique_i2`
3. Laisser en Public ou mettre en Private selon besoin
4. Ne pas cocher "Initialize this repository" (le depot local existe deja)
5. Cliquer sur "Create repository"

### 2.6 Lier le depot local au depot distant et pousser

Copier l'URL fournie par GitHub (format HTTPS ou SSH), puis :

```bash
# Avec HTTPS
git remote add origin https://github.com/<votre-compte>/projet_robotique_i2.git

# Ou avec SSH (recommande si une cle SSH est configuree)
git remote add origin git@github.com:<votre-compte>/projet_robotique_i2.git

# Pousser
git push -u origin main
```

### 2.7 Workflow quotidien apres le premier push

Pour chaque modification ulterieure :

```bash
# Voir les fichiers modifies
git status

# Ajouter les modifications
git add config/nav2_params.yaml        # Un fichier specifique
git add .                              # Ou tous les fichiers modifies

# Commiter avec un message descriptif
git commit -m "Ajout du monde ferme 20x20 m pour exploration complete"

# Pousser vers GitHub
git push
```

### 2.8 Cloner le projet depuis GitHub sur une autre machine

```bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone https://github.com/<votre-compte>/projet_robotique_i2.git
cd ~/turtlebot3_ws
colcon build
source install/setup.bash
```

---

## Recapitulatif des Commandes Git Essentielles

| Commande | Action |
|---|---|
| `git init` | Initialise un depot local |
| `git status` | Affiche l'etat des fichiers |
| `git add .` | Prepare tous les fichiers pour le commit |
| `git commit -m "message"` | Enregistre un instantane |
| `git remote add origin <url>` | Lie le depot local a GitHub |
| `git push -u origin main` | Premier push vers GitHub |
| `git push` | Pushs suivants |
| `git pull` | Recupere les modifications distantes |
| `git log --oneline` | Affiche l'historique des commits |
| `git clone <url>` | Clone un depot existant |
