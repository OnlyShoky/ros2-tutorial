

## 🧭 Qu'est-ce que le SLAM ?

### Problème Fondamental
Les véhicules autonomes doivent répondre à deux questions simultanément :
1. **Cartographie** : "Quelle est la carte de mon environnement ?"
2. **Localisation** : "Où suis-je sur cette carte ?"

### Cadres de Référence dans ROS
- **`{odom}`** : Cadre fixe basé sur l'odométrie (encodeurs de roues). Continu et fluide, mais sujet à la dérive.
- **`{map}`** : Cadre fixe de la carte globale. Discrétisé, précis mais dépend de la qualité du SLAM.
- **`{base}`** : Position actuelle du robot (généralement `base_link` ou `base_footprint`).

Dans un monde idéal, `{odom}` et `{map}` seraient confondus grâce à une fusion parfaite des données LiDAR et odométriques.

### Algorithmes de SLAM Courants
- **Filtre de Kalman Étendu (EKF)** : Fusion probabiliste linéarisée
- **Filtre à particules** : Approche Monte Carlo pour espaces non-linéaires
- **Moindres carrés** : Minimisation d'erreur de bouclage
- **Optimisation de graphe de poses** : Méthode moderne dominante

## 🗺️ Carte d'Occupation (Occupancy Grid)

### Concept
L'environnement est divisé en cellules discrètes. Le SLAM calcule la probabilité qu'une cellule soit :
- **Occupée** (noire) : Obstacle détecté
- **Libre** (blanche) : Espace navigable
- **Inconnue** (gris) : Zone non explorée

### Architecture SLAM
#### **Frontend** (Traitement par trame)
- **Extraction de caractéristiques** : FAST, SIFT, ORB, SURF
- **Appariement et suivi** : Association des caractéristiques entre trames
- **Estimation de transformation** : Mouvement de la caméra/robot

#### **Backend** (Optimisation globale)
- Minimisation de l'erreur totale de la carte
- Optimisation par bouclage (loop closure)
- Génération de carte cohérente

## 📦 Installation de SLAM Toolbox

### Installation des Paquets
```bash
# Mettre à jour les dépôts
sudo apt-get update

# Installer SLAM Toolbox pour votre distribution ROS
# Remplacez ${ROS_DISTRO} par votre version (ex: humble)
sudo apt install ros-humble-slam-toolbox

# Alternative avec variable d'environnement
export ROS_DISTRO=humble
sudo apt install ros-${ROS_DISTRO}-slam-toolbox
```

### Vérification
```bash
# Vérifier l'installation
ros2 pkg list | grep slam_toolbox

# Voir les fichiers installés
ls /opt/ros/${ROS_DISTRO}/share/slam_toolbox/
```

## 🔧 Configuration LiDAR pour SLAM Toolbox

### Ajout du LiDAR dans le XACRO
```xml
<!-- Dans votre fichier cybertruck.urdf.xacro -->

<!-- Lien pour le LiDAR -->
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.1"/>
    </geometry>
    <material name="red">
      <color rgba="1.0 0.0 0.0 0.5"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.001" ixy="0.0" ixz="0.0"
             iyy="0.001" iyz="0.0"
             izz="0.001"/>
  </inertial>
</link>

<!-- Joint de fixation LiDAR -->
<!-- IMPORTANT : Rotation de π/2 autour de l'axe Z pour orientation correcte -->
<joint name="head_to_lidar_link" type="fixed">
  <origin xyz="0.1 0 0.15" rpy="0 0 ${pi/2}"/>
  <parent link="head"/>  <!-- ou 'body' selon votre robot -->
  <child link="lidar_link"/>
</joint>

<!-- Plugin Gazebo pour le LiDAR -->
<gazebo reference="lidar_link">
  <sensor name="lidar_sensor" type="ray">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1.0</resolution>
          <min_angle>-3.14159</min_angle>  <!-- -180° -->
          <max_angle>3.14159</max_angle>   <!-- +180° -->
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <!-- IMPORTANT : SLAM Toolbox nécessite LaserScan -->
    <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
      <topic_name>/lidar/out</topic_name>
    </plugin>
  </sensor>
</gazebo>
```

## 🏗️ Configuration du Monde Gazebo

### Fichier `walls.world` pour Tests SLAM
```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="slam_world">
    
    <!-- Environnement de base -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Mur 1 -->
    <model name="wall_1">
      <pose>2.0 0.0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>4.0 0.1 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>4.0 0.1 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Mur 2 -->
    <model name="wall_2">
      <pose>0.0 2.0 0.5 0 0 1.5708</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>4.0 0.1 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>4.0 0.1 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.8 0.2 1</ambient>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Obstacles supplémentaires pour tests -->
    <model name="cylinder_1">
      <pose>1.0 1.0 0.3 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>0.6</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.3</radius>
              <length>0.6</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.8 1</ambient>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Boîte de test -->
    <model name="box_1">
      <pose>-1.0 -1.0 0.3 0 0 0.7854</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.6 0.6 0.6</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.6 0.6 0.6</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.2 1</ambient>
          </material>
        </visual>
      </link>
    </model>
    
  </world>
</sdf>
```

### Lancement de Gazebo avec Monde Personnalisé
```python
# Dans votre fichier gazebo.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

gazebo_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('gazebo_ros'),
            'launch',
            'gazebo.launch.py'
        ])
    ]),
    launch_arguments={
        'world': PathJoinSubstitution([
            FindPackageShare('wheeled_robot'),
            'worlds',
            'walls.world'  # Votre monde personnalisé
        ]),
        'use_sim_time': 'true'  # Essentiel pour SLAM
    }.items()
)

# Tester le monde
def test_world():
    """Commande pour tester le monde avec obstacles"""
    import os
    os.system('gazebo ' + 
              PathJoinSubstitution([
                  FindPackageShare('wheeled_robot'),
                  'worlds',
                  'walls.world'
              ]))
```

## ⚙️ Configuration de SLAM Toolbox

### Copie des Fichiers de Paramètres
```bash
# Copier le fichier de paramètres par défaut
cp /opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml \
   ~/ros2_ws/src/wheeled_robot/config/
```

### Fichier `mapper_params_online_async.yaml` Complet
```yaml
slam_toolbox:
  ros__parameters:
    
    # Frames ROS (ADAPTER À VOTRE ROBOT)
    odom_frame: odom
    map_frame: map
    base_frame: body  # ou base_link selon votre robot
    scan_topic: /lidar/out
    
    # Mode de fonctionnement
    mode: "mapping"  # "mapping" ou "localization"
    debug_logging: false
    
    # Paramètres de carte
    resolution: 0.05  # Résolution de la carte en mètres/px
    max_laser_range: 8.0  # Portée maximale du LiDAR
    
    # Paramètres d'optimisation
    throttle_scans: 1
    transform_publish_period: 0.02  # 50 Hz
    map_update_interval: 5.0
    
    # Paramètres d'odométrie
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_time_interval: 0.5
    minimum_travel_distance: 0.1
    minimum_travel_heading: 0.05
    
    # Paramètres de scan matching
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    
    # Paramètres de bouclage
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45
    
    # Correspondance de scans
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
    
    # Paramètres de filtre
    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
    
    # Paramètres de réglage fin
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
    
    # Paramètres de carte d'occupation
    occupancy_grid:
      width: 4000  # Largeur en pixels
      height: 4000 # Hauteur en pixels
      origin_x: -20.0  # Origine X en mètres
      origin_y: -20.0  # Origine Y en mètres
      occupied_thresh: 0.65
      free_thresh: 0.25
      
    # Paramètres de sauvegarde/chargement
    map_file_name: ""  # Vide pour nouvelle carte
    map_start_pose: [0.0, 0.0, 0.0]  # x, y, yaw
    map_start_at_dock: false
```

## 🚀 Exécution de SLAM Toolbox

### Méthode 1 : Depuis le Terminal
```bash
# Construire le package
cd ~/ros2_ws
colcon build --packages-select wheeled_robot

# Charger l'environnement
source install/setup.bash

# Lancer SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=./src/wheeled_robot/config/mapper_params_online_async.yaml \
  use_sim_time:=true
```

### Méthode 2 : Depuis le Fichier Launch
```python
# Dans votre gazebo.launch.py
slam_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([
            FindPackageShare('slam_toolbox'),
            'launch',
            'online_async_launch.py'
        ])
    ]),
    launch_arguments={
        'slam_params_file': PathJoinSubstitution([
            FindPackageShare('wheeled_robot'),
            'config',
            'mapper_params_online_async.yaml'
        ]),
        'use_sim_time': 'true'
    }.items()
)
```

## 👁️ Configuration RViz pour SLAM

### Fichier `slam_config.rviz` Complet
```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
  - Class: rviz_common/Tool Properties
    Name: Tool Properties

Visualization Manager:
  Displays:
    
    # 1. Carte d'occupation
    - Class: rviz_default_plugins/Map
      Name: Occupancy Grid
      Enabled: true
      Topic: /map
      Color Scheme: map
      Alpha: 0.7
      
    # 2. Scan LiDAR
    - Class: rviz_default_plugins/LaserScan
      Name: Laser Scan
      Enabled: true
      Topic: /lidar/out
      Style: Points
      Size (m): 0.05
      Color Transformer: Intensity
      Autocompute Intensity Bounds: true
      
    # 3. Modèle du robot
    - Class: rviz_default_plugins/RobotModel
      Name: Robot Model
      Enabled: true
      Robot Description: robot_description
      Visual Enabled: true
      Collision Enabled: false
      
    # 4. Repères TF
    - Class: rviz_default_plugins/TF
      Name: TF Frames
      Enabled: true
      Marker Scale: 0.5
      
    # 5. Chemin estimé
    - Class: rviz_default_plugins/Path
      Name: Estimated Path
      Enabled: true
      Topic: /slam_toolbox/pose
      Color: 0; 255; 0
      Alpha: 1.0
      Buffer Length: 1000
      
    # 6. Nuage de points (optionnel)
    - Class: rviz_default_plugins/PointCloud2
      Name: PointCloud
      Enabled: false
      Topic: /laser_point_cloud
      Style: Points
      Size (Pixels): 2

  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Name: Current View
      Target Frame: map
      Distance: 10
      Focal Point:
        X: 0
        Y: 0
        Z: 0

  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map  # IMPORTANT : Toujours 'map' pour SLAM
    Frame Rate: 30
```

### Lancement de RViz
```bash
# Depuis le terminal
rviz2 -d ~/ros2_ws/src/wheeled_robot/config/slam_config.rviz

# Depuis le fichier launch
rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', PathJoinSubstitution([
        FindPackageShare('wheeled_robot'),
        'config',
        'slam_config.rviz'
    ])]
)
```

## ❌ Résolution d'Erreurs "No map received"

### Checklist de Dépannage
1. **Gazebo est-il en cours d'exécution ?**
   ```bash
   ps aux | grep gazebo
   ```

2. **Y a-t-il des objets dans Gazebo ?**
   - Le LiDAR a besoin d'obstacles pour détecter
   - Vérifiez avec `ros2 topic echo /lidar/out`

3. **SLAM Toolbox est-il lancé ?**
   ```bash
   ros2 node list | grep slam
   ```

4. **Cadre fixe correct dans RViz ?**
   - Doit être `map` (pas `odom` ou `base_link`)

5. **Topics correctement configurés ?**
   ```bash
   # Vérifier les topics publiés
   ros2 topic list | grep -E "(scan|map|lidar)"
   
   # Vérifier les données LiDAR
   ros2 topic echo /lidar/out --once | head -5
   
   # Vérifier la carte
   ros2 topic echo /map --once | head -5
   ```

### Ordre d'Exécution Correct
```bash
# Terminal 1: Gazebo avec monde
ros2 launch wheeled_robot gazebo.launch.py

# Terminal 2: SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=./config/mapper_params_online_async.yaml

# Terminal 3: RViz
rviz2 -d ./config/slam_config.rviz

# Terminal 4: Contrôle du robot
ros2 run wheeled_robot diff_drive_publisher
```

## 💾 Sauvegarde et Chargement de Cartes

### Fichiers Générés par SLAM Toolbox
- **`map.pgm`** : Image de la carte d'occupation (format PGM)
- **`map.yaml`** : Métadonnées de la carte (résolution, origine)
- **`map.data`** : Données brutes de cartographie
- **`map.posegraph`** : Graphe de poses pour l'optimisation

### Emplacement des Fichiers
```bash
# Par défaut, sauvegardé dans le répertoire courant
ls -la *.pgm *.yaml *.data *.posegraph

# Déplacer vers votre dossier config
mkdir -p ~/ros2_ws/src/wheeled_robot/config/maps/
mv map.* ~/ros2_ws/src/wheeled_robot/config/maps/
```

### Configuration pour Charger une Carte Existante
```yaml
# Dans mapper_params_online_async.yaml
slam_toolbox:
  ros__parameters:
    # ... autres paramètres ...
    
    # Chargement de carte
    mode: "localization"  # Passer en mode localisation
    map_file_name: "/home/votre_utilisateur/ros2_ws/src/wheeled_robot/config/maps/map"
    map_start_pose: [0.0, 0.0, 0.0]  # Position initiale
    map_start_at_dock: true  # Commencer à la position d'origine
    
    # Paramètres de localisation
    position_tracking_period: 1.0
    transform_timeout: 0.2
    minimum_travel_distance: 0.01
    minimum_travel_heading: 0.01
```

### Test de Chargement de Carte
```bash
# Construire et lancer
cd ~/ros2_ws
colcon build --packages-select wheeled_robot
source install/setup.bash

# Lancer avec carte pré-chargée
ros2 launch wheeled_robot gazebo.launch.py \
  slam_params_file:=./config/mapper_params_online_async.yaml \
  load_map:=true
```

## 📊 Commandes de Surveillance Utiles

### Vérification du Système SLAM
```bash
# Vérifier les transforms
ros2 run tf2_tools view_frames.py
evince frames.pdf

# Vérifier la qualité de la carte
ros2 topic hz /map
ros2 topic bw /lidar/out

# Vérifier les poses estimées
ros2 topic echo /slam_toolbox/pose

# Sauvegarder la carte manuellement
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  '{name: "/home/user/ros2_ws/src/wheeled_robot/config/maps/ma_carte"}'
```

### Services SLAM Toolbox Disponibles
```bash
# Liste des services
ros2 service list | grep slam

# Services utiles :
# /slam_toolbox/save_map
# /slam_toolbox/serialize_map
# /slam_toolbox/deserialize_map
# /slam_toolbox/pause_new_measurements
# /slam_toolbox/resume_new_measurements
# /slam_toolbox/clear_queue
# /slam_toolbox/clear_changes
```

Ce tutoriel complet vous permet d'implémenter un système SLAM fonctionnel avec ROS 2, SLAM Toolbox, Gazebo et RViz, incluant la cartographie initiale et la réutilisation de cartes existantes pour la localisation.