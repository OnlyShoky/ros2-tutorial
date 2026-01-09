# Tutoriel : 

## 📷 Qu'est-ce qu'une Caméra de Profondeur ?

### Principe de Fonctionnement
Une **caméra de profondeur** (ou caméra RGB-D) utilise deux capteurs infrarouges avec un projecteur IR pour déterminer la **disparité** entre les images gauche et droite. Par triangulation stéréoscopique, elle calcule la distance des objets.

### Technologies Principales
1. **Stéréoscopie Active** (Intel RealSense, Stereolabs ZED) : Projecteur IR + deux capteurs
2. **Time-of-Flight** (Microsoft Kinect v2) : Mesure directe du temps de vol
3. **Structured Light** (Apple Face ID, Kinect v1) : Motif projeté déformé par les surfaces

### Modèle Mathématique
```
        B
       /|
      / |
     /  |
    /   |
   /    | Z (profondeur)
  /     |
 /      |
/_______|
A   f   C

Z = (B × f) / d

Où :
Z = Distance (profondeur)
B = Baseline (distance entre caméras)
f = Distance focale
d = Disparité (différence de pixels)
```

## 📁 Configuration URDF de la Caméra de Profondeur

### Structure des Joints et Liens
```xml
<!-- Lien principal de la caméra -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.08 0.03"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
</link>

<!-- Lien du frame optique (convention ROS) -->
<link name="camera_frame_link"/>

<!-- Joint entre la tête et la caméra -->
<joint name="head_to_camera_link" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0 0 0.25" rpy="0 0 ${pi/2}"/>
</joint>

<!-- Joint pour l'orientation optique correcte -->
<joint name="camera_link_to_camera_frame_link" type="fixed">
  <parent link="camera_link"/>
  <child link="camera_frame_link"/>
  <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
</joint>
```

## 🔧 Plugin Gazebo pour Caméra de Profondeur

### Configuration Complète du Capteur
```xml
<gazebo reference="camera_link">
  <sensor name="depth_camera" type="depth">
    
    <!-- Pose dans le lien camera_link -->
    <pose>0 0 0 0 0 0</pose>
    
    <!-- Visualisation dans Gazebo -->
    <visualize>true</visualize>
    
    <!-- Fréquence de mise à jour -->
    <update_rate>30</update_rate>
    
    <!-- Configuration de la caméra -->
    <camera>
      
      <!-- Champ de vision horizontal en radians -->
      <horizontal_fov>1.047</horizontal_fov> <!-- 60° -->
      
      <!-- Format d'image -->
      <image>
        <format>R8G8B8</format>
        <width>640</width>
        <height>480</height>
      </image>
      
      <!-- Distances de découpage -->
      <clip>
        <near>0.01</near>    <!-- 1 cm minimum -->
        <far>5.0</far>       <!-- 5 mètres maximum -->
      </clip>
      
      <!-- Profondeur : paramètres spécifiques -->
      <depth_camera>
        <output>depth_points</output>
      </depth_camera>
      
    </camera>
    
    <!-- Plugin ROS pour Gazebo -->
    <plugin name="depth_camera_plugin" filename="libgazebo_ros_camera.so">
      
      <!-- Frame de sortie (convention optique) -->
      <frame_name>camera_frame_link</frame_name>
      
      <!-- Topics de sortie -->
      <image_topic_name>/camera/color/image_raw</image_topic_name>
      <camera_info_topic_name>/camera/color/camera_info</camera_info_topic_name>
      <depth_image_topic_name>/camera/depth/image_raw</depth_image_topic_name>
      <depth_image_camera_info_topic_name>/camera/depth/camera_info</depth_image_camera_info_topic_name>
      <point_cloud_topic_name>/camera/depth/points</point_cloud_topic_name>
      
      <!-- Nom de la caméra -->
      <camera_name>camera</camera_name>
      
      <!-- Taux de mise à jour -->
      <update_rate>30.0</update_rate>
      
      <!-- Format d'image -->
      <image_format>RGB8</image_format>
      
      <!-- Paramètres de distorsion -->
      <distortion_k1>0.0</distortion_k1>
      <distortion_k2>0.0</distortion_k2>
      <distortion_k3>0.0</distortion_k3>
      <distortion_t1>0.0</distortion_t1>
      <distortion_t2>0.0</distortion_t2>
      
      <!-- Baseline pour la stéréoscopie -->
      <baseline>0.075</baseline>
      
    </plugin>
    
  </sensor>
</gazebo>
```

## 🧭 Orientation des Frames de Caméra

### Convention des Coordonnées
**Sans transformation** (camera_link) :
- **X** : Direction de la caméra (vers l'avant)
- **Y** : Vers la gauche
- **Z** : Vers le haut

**Avec transformation** (camera_frame_link - convention optique ROS) :
- **Z** : Direction de la caméra (vers l'avant)
- **X** : Vers la droite
- **Y** : Vers le bas

### Transformation Nécessaire
```xml
<!-- Rotation de 90° autour de X, puis -90° autour de Z -->
<origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>

Équivalent à :
1. Rotation de -90° autour de l'axe X
2. Rotation de -90° autour de l'axe Z
```

### Pourquoi cette Transformation ?
1. **Gazebo** utilise la convention X-avant, Y-gauche, Z-haut
2. **ROS/OpenCV** utilise la convention Z-avant, X-droite, Y-bas
3. Sans correction, le nuage de points apparaîtrait tourné de 90°

## 🚀 Simulation Gazebo de la Caméra de Profondeur

### Lancer la Simulation
```bash
# Terminal 1 - Construire et lancer Gazebo
cd ~/ros_ws
colcon build --packages-select gazebo_tutorial
source install/setup.bash
ros2 launch gazebo_tutorial gazebo.launch.py
```

### Ajouter une Sphère de Test
Dans Gazebo :
1. Menu **Insert** → **Sphere**
2. Positionner à `(1.0, 0.0, 0.5)`
3. Redimensionner à rayon `0.3`
4. Couleur rouge pour meilleure visibilité

### Commande pour Ajouter un Objet
```bash
# Alternative en ligne de commande
ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity '{
  name: "test_sphere",
  xml: "<?xml version=\"1.0\"?><sdf version=\"1.7\"><model name=\"test_sphere\"><pose>1.0 0.0 0.5 0 0 0</pose><link name=\"link\"><visual name=\"visual\"><geometry><sphere><radius>0.3</radius></sphere></geometry><material><ambient>1 0 0 1</ambient></material></visual><collision name=\"collision\"><geometry><sphere><radius>0.3</radius></sphere></geometry></collision></link></model></sdf>"
}'
```

## 👁️ Simulation RViz avec Carte de Profondeur et Nuage de Points

### Lancer RViz avec Configuration
```bash
# Terminal 2 - Lancer RViz
source ~/ros_ws/install/setup.bash
rviz2
```

### Configuration Complète RViz

#### Étape 1 : Ajouter le Nuage de Points
1. **Add** → **By topic** tab
2. Chercher `/camera/depth/points`
3. Sélectionner **PointCloud2**
4. Configurer :
   - **Topic** : `/camera/depth/points`
   - **Style** : Points
   - **Size** : 0.01
   - **Color Transformer** : AxisColor
   - **Axis** : Z

#### Étape 2 : Ajouter l'Image de Profondeur
1. **Add** → **Image**
2. Configurer :
   - **Image Topic** : `/camera/depth/image_raw`
   - **Transport** : raw
   - **Width** : 640
   - **Height** : 480

#### Étape 3 : Ajouter l'Image Couleur (Optionnel)
1. **Add** → **Image**
2. Configurer :
   - **Image Topic** : `/camera/color/image_raw`
   - **Transport** : raw

#### Étape 4 : Ajouter le Modèle du Robot
1. **Add** → **RobotModel**
2. Configurer :
   - **Robot Description** : `robot_description`
   - **Description Topic** : `/robot_description`

### Fichier de Configuration RViz Complet
```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views

Visualization Manager:
  Displays:
    # Nuage de points de profondeur
    - Class: rviz_default_plugins/PointCloud2
      Name: Depth PointCloud
      Enabled: true
      Topic: /camera/depth/points
      Style: Points
      Size (Pixels): 3
      Alpha: 1.0
      Color Transformer: AxisColor
      Axis: Z
      Position Transformer: XYZ
      Fixed Frame: camera_frame_link
      
    # Image de profondeur
    - Class: rviz_default_plugins/Image
      Name: Depth Image
      Enabled: true
      Topic: /camera/depth/image_raw
      Transport: raw
      Width: 640
      Height: 480
      Queue Size: 2
      
    # Image couleur
    - Class: rviz_default_plugins/Image
      Name: Color Image
      Enabled: true
      Topic: /camera/color/image_raw
      Transport: raw
      Width: 320
      Height: 240
      
    # Modèle du robot
    - Class: rviz_default_plugins/RobotModel
      Name: TeslaBot
      Enabled: true
      Robot Description: robot_description
      Robot Description Topic: /robot_description
      Visual Enabled: true
      
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Name: Current View
      Target Frame: camera_frame_link
      Distance: 5.0
```

## 🔧 Vérification et Test

### Vérifier les Topics Actifs
```bash
# Lister tous les topics de la caméra
ros2 topic list | grep camera

# Voir les infos des topics
ros2 topic info /camera/depth/points
ros2 topic info /camera/depth/image_raw
ros2 topic info /camera/color/image_raw

# Voir un échantillon des données
ros2 topic echo /camera/depth/points --once | head -20
```

### Tester avec RQT
```bash
# Visualiser l'image de profondeur
ros2 run rqt_image_view rqt_image_view --topic /camera/depth/image_raw

# Visualiser l'image couleur
ros2 run rqt_image_view rqt_image_view --topic /camera/color/image_raw

# Voir toutes les images simultanément
ros2 run rqt_gui rqt_gui
```

## ⚙️ Configuration Avancée

### Caméra de Profondeur Réaliste (Type RealSense D435)
```xml
<gazebo reference="camera_link">
  <sensor name="realsense_d435" type="depth">
    <pose>0 0 0 0 0 0</pose>
    <visualize>false</visualize>
    <update_rate>30</update_rate>
    <camera>
      <horizontal_fov>1.50098</horizontal_fov> <!-- 86° -->
      <image>
        <format>R8G8B8</format>
        <width>1280</width>
        <height>720</height>
      </image>
      <clip>
        <near>0.11</near>
        <far>10.0</far>
      </clip>
      <depth_camera>
        <output>depth_points</output>
      </depth_camera>
    </camera>
    <plugin name="realsense_plugin" filename="libgazebo_ros_camera.so">
      <frame_name>camera_frame_link</frame_name>
      <camera_name>camera</camera_name>
      <image_topic_name>/camera/color/image_raw</image_topic_name>
      <depth_image_topic_name>/camera/aligned_depth_to_color/image_raw</depth_image_topic_name>
      <point_cloud_topic_name>/camera/depth/color/points</point_cloud_topic_name>
      <update_rate>30.0</update_rate>
      <image_format>RGB8</image_format>
      <baseline>0.05</baseline>
      <distortion_k1>0.0</distortion_k1>
      <distortion_k2>0.0</distortion_k2>
      <distortion_k3>0.0</distortion_k3>
      <distortion_t1>0.0</distortion_t1>
      <distortion_t2>0.0</distortion_t2>
    </plugin>
  </sensor>
</gazebo>
```

## 🛠️ Dépannage Courant

### Problème : Nuage de Points Invisible
```bash
# 1. Vérifier la transformation
ros2 run tf2_tools view_frames.py
ros2 topic echo /tf_static

# 2. Vérifier les données publiées
ros2 topic hz /camera/depth/points

# 3. Vérifier la portée de la caméra
# Augmenter <far> dans l'URDF si nécessaire
```

### Problème : Image de Profondeur Noire
```xml
<!-- Ajuster les plans de découpage -->
<clip>
  <near>0.1</near>    <!-- Réduire si trop proche -->
  <far>10.0</far>     <!-- Augmenter si trop loin -->
</clip>

<!-- Vérifier l'éclairage dans Gazebo -->
```

### Problème : Orientation Incorrecte
```bash
# Vérifier les frames
ros2 topic echo /camera/depth/points header.frame_id

# S'assurer que la transformation est appliquée
# Dans RViz, Fixed Frame doit être camera_frame_link
```

## 📊 Bonnes Pratiques

### Pour des Simulations Réalistes
- **Portée** : 0.1-10m pour intérieur, 0.5-50m pour extérieur
- **Résolution** : 640×480 pour l'équilibre performance/précision
- **Fréquence** : 15-30 Hz selon l'application

### Pour la Performance
- **Désactiver la visualisation** dans Gazebo : `<visualize>false</visualize>`
- **Réduire la résolution** pour les tests
- **Utiliser moins de points** dans le nuage

### Pour l'Intégration
- **Toujours inclure** la transformation optique
- **Namespace cohérent** : `/robot_name/camera/...`
- **Documenter** les paramètres dans l'URDF

## 🎯 Résumé des Commandes Essentielles

```bash
# Lancer la simulation
ros2 launch gazebo_tutorial gazebo.launch.py

# Vérifier les données
ros2 topic echo /camera/depth/points --once
ros2 topic echo /camera/depth/image_raw --once

# Visualiser dans RViz
rviz2 -d $(ros2 pkg prefix gazebo_tutorial)/share/gazebo_tutorial/config/depth_camera.rviz

# Tester avec RQT
ros2 run rqt_image_view rqt_image_view
```

Ce tutoriel vous permet d'intégrer une caméra de profondeur fonctionnelle dans vos simulations Gazebo et de visualiser les données de profondeur, couleur et nuage de points en temps réel dans RViz, essentiel pour le développement d'algorithmes de perception 3D et de vision par ordinateur.