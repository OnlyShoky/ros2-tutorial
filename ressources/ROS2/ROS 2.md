
## 📸 Introduction à la Caméra dans Gazebo
Ce tutoriel montre comment intégrer un capteur caméra dans une simulation Gazebo et visualiser le flux vidéo dans RViz. Nous utiliserons la simulation Tesla Bot comme exemple concret.

## 🔧 Modèle de Caméra Pinhole

### Concept de Base
Le modèle pinhole (sténopé) simplifie la caméra à un point unique sans lentille. Les paramètres clés sont :
- **Champ de vision (FOV)** : Angle de la scène visible
- **Résolution** : Largeur × hauteur en pixels
- **Plans de découpage** : Distance min/max de visualisation

### Schéma du Modèle
```
      Plan Image
     ┌───────────┐
     │           │
     │  Largeur  │
     │           │
     └───────────┘
        /
       /
      / FOV
     /
    ● Centre Caméra
    │
    │ Hauteur
    ▼
```

## 📁 Fichier URDF du Capteur Caméra

### Structure Complète du Capteur
```xml
<gazebo reference="name_of_link">
  <sensor name="camera" type="camera">
    <!-- Pose relative au lien parent -->
    <pose>x y z roll pitch yaw</pose>
    
    <!-- Affichage dans Gazebo -->
    <visualize>true</visualize>
    
    <!-- Fréquence de mise à jour (Hz) -->
    <update_rate>30</update_rate>
    
    <camera>
      <!-- Champ de vision horizontal en radians -->
      <horizontal_fov>1.047</horizontal_fov> <!-- 60° -->
      
      <image>
        <!-- Format de couleur -->
        <format>R8G8B8</format>
        
        <!-- Résolution -->
        <width>640</width>
        <height>480</height>
      </image>
      
      <!-- Distances de découpage -->
      <clip>
        <near>0.1</near>  <!-- Plan proche (mètres) -->
        <far>100</far>    <!-- Plan lointain (mètres) -->
      </clip>
    </camera>
    
    <!-- Plugin ROS pour Gazebo -->
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <!-- Frame de référence -->
      <frame_name>camera_link</frame_name>
      
      <!-- Nom du topic ROS -->
      <topic_name>camera/image_raw</topic_name>
      
      <!-- Nom du topic des infos caméra -->
      <camera_info_topic>camera/camera_info</camera_info_topic>
    </plugin>
  </sensor>
</gazebo>
```

## 🎯 Positions de Caméra Typiques

### Exemples de Configurations
```xml
<!-- Pose 0 : Position initiale -->
<pose>0 0 0 0 0 0</pose>

<!-- Pose 1 : Déplacée vers le haut et tournée -->
<pose>0 0 1.5 0 0 1.57</pose>  <!-- 1.5m haut, rotation 90° Z -->

<!-- Pose 2 : Position latérale -->
<pose>0 1.0 1.0 0 0.78 0</pose>  <!-- Décalée Y et Z, inclinaison -->
```

## 🚀 Lancer la Simulation

### Terminal 1 : Démarrer Gazebo
```bash
cd ~/ros_ws
colcon build --packages-select gazebo_tutorial
source install/setup.bash
ros2 launch gazebo_tutorial gazebo.launch.py
```

### Terminal 2 : Contrôler les Joints
```bash
cd ~/ros_ws
source install/setup.bash
ros2 run gazebo_tutorial joint_publisher
```

### Installation du Package Requis
```bash
# Installer le publisher d'états de joints
sudo apt install ros-${ROS_DISTRO}-joint-state-publisher
```

## 👁️ Visualiser dans RViz

### Terminal 3 : Lancer RViz avec Configuration
```bash
cd ~/ros_ws
source install/setup.bash
rviz2 -d install/gazebo_tutorial/share/gazebo_tutorial/config/teslabot_config.rviz
```

### Fichier de Configuration RViz Minimal
```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views

Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Robot Description: robot_description
      
    - Class: rviz_default_plugins/Camera
      Name: Camera
      Enabled: true
      Image Topic: /camera/image_raw
      Transport: raw
      
    - Class: rviz_default_plugins/Image
      Name: Image
      Enabled: true
      Image Topic: /camera/image_raw
      Transport: raw
      Width: 640
      Height: 480
      
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Name: Orbit View
```

## 🔍 Vérification du Flux Caméra

### Lister les Topics Actifs
```bash
ros2 topic list | grep camera
```

### Visualiser les Images dans le Terminal
```bash
# Voir les messages de la caméra
ros2 topic echo /camera/image_raw --once | head -20

# Voir les infos de calibration
ros2 topic echo /camera/camera_info
```

### Tester avec RQT Image View
```bash
# Alternative à RViz pour visualiser simplement
ros2 run rqt_image_view rqt_image_view
```

## ⚙️ Paramètres Avancés du Plugin Caméra

### Configuration Complète du Plugin
```xml
<plugin name="camera_controller" filename="libgazebo_ros_camera.so">
  <!-- Identifiants -->
  <frame_name>camera_optical_frame</frame_name>
  <camera_name>camera</camera_name>
  
  <!-- Topics ROS -->
  <topic_name>camera/image_raw</topic_name>
  <camera_info_topic>camera/camera_info</camera_info_topic>
  
  <!-- Espaces de noms -->
  <frame_name>camera_link</frame_name>
  <robot_namespace>/teslabot</robot_namespace>
  
  <!-- Paramètres de publication -->
  <update_rate>30.0</update_rate>
  <hack_baseline>0.07</hack_baseline>
  
  <!-- Format et compression -->
  <image_format>RGB8</image_format>
  <distortion_k1>0.0</distortion_k1>
  <distortion_k2>0.0</distortion_k2>
  <distortion_k3>0.0</distortion_k3>
  <distortion_t1>0.0</distortion_t1>
  <distortion_t2>0.0</distortion_t2>
</plugin>
```

## 🛠️ Dépannage Courant

### Problème : Pas d'image dans RViz
```bash
# Vérifier que la caméra publie
ros2 topic hz /camera/image_raw

# Vérifier le type de message
ros2 topic info /camera/image_raw

# Regénérer les messages
cd ~/ros_ws
colcon build --packages-select gazebo_tutorial
```

### Problème : Caméra non visible dans Gazebo
1. Vérifier que `<visualize>true</visualize>` est présent
2. Augmenter la taille du frustum dans Gazebo
3. Vérifier les collisions avec d'autres objets

### Problème : Mauvais frame_id
```bash
# Vérifier le frame_id publié
ros2 topic echo /camera/image_raw header.frame_id

# Lister les frames disponibles
ros2 run tf2_tools view_frames.py
```

## 📊 Bonnes Pratiques

### Pour les Simulations Réalistes
- **Fréquence** : 30 Hz pour la vidéo, 10-15 Hz pour la perception
- **Résolution** : 640×480 pour l'équilibre performance/qualité
- **FOV** : 60-90° pour une vue naturelle

### Pour l'Intégration RViz
- **Frame convention** : Utiliser `_optical_frame` pour les caméras
- **Namespace** : Préfixer les topics (`/robot_name/camera/...`)
- **Calibration** : Inclure toujours un topic `camera_info`

## 🎯 Exemple Complet : POV Tesla Bot

### Configuration Finale URDF
```xml
<gazebo reference="head_link">
  <sensor name="head_camera" type="camera">
    <pose>0.1 0 0.15 0 -0.2 0</pose>
    <visualize>true</visualize>
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.396</horizontal_fov> <!-- 80° -->
      <image>
        <format>R8G8B8</format>
        <width>800</width>
        <height>600</height>
      </image>
      <clip>
        <near>0.05</near>
        <far>50</far>
      </clip>
    </camera>
    <plugin name="head_camera_controller" 
            filename="libgazebo_ros_camera.so">
      <frame_name>head_camera_optical_frame</frame_name>
      <topic_name>teslabot/camera/image_raw</topic_name>
    </plugin>
  </sensor>
</gazebo>
```

### Commande de Lancement Complète
```bash
# Terminal 1 : Simulation
ros2 launch gazebo_tutorial gazebo.launch.py use_camera:=true

# Terminal 2 : Contrôle
ros2 run gazebo_tutorial joint_publisher --ros-args -p camera_enabled:=true

# Terminal 3 : Visualisation
rviz2 -d $(ros2 pkg prefix gazebo_tutorial)/share/gazebo_tutorial/config/teslabot_camera.rviz
```

Ce tutoriel vous permet d'intégrer une caméra fonctionnelle dans votre simulation Gazebo et de visualiser le flux en temps réel dans RViz, essentiel pour le développement d'algorithmes de vision par ordinateur et de perception.