# Tutoriel Complet : Robot RRBot avec Gazebo + ROS 2 Control

## Guide Détaillé en Français avec Explications

---

## 📚 Introduction

Ce tutoriel vous guide pas à pas pour créer un **robot simple à 2 joints (RRBot)** dans Gazebo, contrôlé par ROS 2 Control.

### Ce que vous allez apprendre :

- ✅ Créer un robot URDF fonctionnel
- ✅ Configurer Gazebo avec ROS 2
- ✅ Utiliser ROS 2 Control pour contrôler les joints
- ✅ Envoyer des trajectoires au robot
- ✅ Visualiser dans RVIZ

### Architecture du Robot RRBot :

```
world (frame fixe)
  └── base_link (base fixe du robot)
      └── link1 (premier segment, joint1)
          └── link2 (deuxième segment, joint2)
```

---

## 0. Prérequis : Installation et Nettoyage

### 0.1 Nettoyer l'ancien workspace (si nécessaire)

```bash
# Supprimer l'ancien package problématique
cd ~/ros2_ws
rm -rf src/gazebo_tutorial build install log

# Sourcer l'environnement ROS 2
source /opt/ros/humble/setup.bash
```

**💡 Explication :** On nettoie tout pour repartir sur des bases saines.

---

### 0.2 Installer tous les packages nécessaires

```bash
# Réparer dpkg si nécessaire
sudo dpkg --configure -a

# Mettre à jour la liste des packages
sudo apt update

# Installer Gazebo et tous les plugins ROS 2
sudo apt install -y \
  gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-effort-controllers \
  ros-humble-position-controllers \
  ros-humble-velocity-controllers \
  ros-humble-robot-state-publisher \
  ros-humble-xacro
```

**💡 Explication des packages :**

- `gazebo` : Le simulateur physique
- `gazebo-ros-pkgs` : Plugins pour connecter Gazebo à ROS 2
- `gazebo-ros2-control` : Plugin pour utiliser ROS 2 Control dans Gazebo
- `ros2-control` : Framework pour contrôler les robots
- `ros2-controllers` : Contrôleurs prédéfinis (position, vitesse, effort)
- `joint-state-broadcaster` : Publie l'état des joints sur `/joint_states`
- `joint-trajectory-controller` : Contrôleur pour suivre des trajectoires
- `robot-state-publisher` : Publie les transformations TF du robot
- `xacro` : Permet d'utiliser des macros dans les fichiers URDF

---

## 1. Création du Package ROS 2

```bash
# Aller dans le dossier src du workspace
cd ~/ros2_ws/src

# Créer un nouveau package avec ament_cmake
ros2 pkg create rrbot_gazebo \
  --build-type ament_cmake \
  --description "Robot RRBot simple avec Gazebo et ROS 2 Control"

# Entrer dans le package
cd rrbot_gazebo

# Créer la structure de dossiers
mkdir -p launch config urdf worlds rviz scripts
```

**💡 Structure du package :**

- `launch/` : Fichiers de lancement (.launch.py)
- `config/` : Fichiers de configuration YAML
- `urdf/` : Description du robot (URDF/Xacro)
- `worlds/` : Mondes Gazebo (.world)
- `rviz/` : Configurations RVIZ (.rviz)
- `scripts/` : Scripts Python pour contrôler le robot

---

## 2. Configuration du Package

### 2.1 CMakeLists.txt

```bash
cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(rrbot_gazebo)

# Options de compilation strictes
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Trouver ament_cmake (système de build de ROS 2)
find_package(ament_cmake REQUIRED)

# Installer tous les dossiers dans le workspace
install(
  DIRECTORY 
    launch      # Fichiers de lancement
    config      # Configurations YAML
    urdf        # Description du robot
    worlds      # Mondes Gazebo
    rviz        # Configs RVIZ
  DESTINATION share/${PROJECT_NAME}
)

# Installer les scripts Python et les rendre exécutables
install(
  PROGRAMS
    scripts/test_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Finaliser le package
ament_package()
EOF
```

**💡 Explication :**

- `install()` copie les fichiers dans le dossier `install/` lors de la compilation
- `DESTINATION share/${PROJECT_NAME}` : chemin standard pour les ressources ROS 2
- `PROGRAMS` : rend les scripts exécutables automatiquement

---

### 2.2 package.xml

```bash
cat > package.xml << 'EOF'
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rrbot_gazebo</name>
  <version>1.0.0</version>
  <description>Robot RRBot simple avec Gazebo et ROS 2 Control</description>
  <maintainer email="votre_email@example.com">Votre Nom</maintainer>
  <license>Apache-2.0</license>

  <!-- Dépendance de build : ament_cmake -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Dépendances d'exécution : nécessaires pour faire tourner le package -->
  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>gazebo_ros2_control</exec_depend>
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>joint_state_broadcaster</exec_depend>
  <exec_depend>joint_trajectory_controller</exec_depend>
  <exec_depend>effort_controllers</exec_depend>
  <exec_depend>position_controllers</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF
```

**💡 Explication :**

- `<exec_depend>` : packages nécessaires à l'exécution
- Ces dépendances seront vérifiées par ROS 2 avant de lancer le package

---

## 3. Description du Robot (URDF)

### 3.1 Fichier URDF Principal

```bash
cat > urdf/rrbot.urdf.xacro << 'EOF'
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="rrbot">

  <!-- ========================================== -->
  <!-- PROPRIÉTÉS ET CONSTANTES -->
  <!-- ========================================== -->
  
  <xacro:property name="PI" value="3.14159265359"/>
  
  <!-- Propriétés des liens du robot -->
  <xacro:property name="link_mass" value="0.5"/>      <!-- Masse en kg -->
  <xacro:property name="link_length" value="1.0"/>    <!-- Longueur en m -->
  <xacro:property name="link_radius" value="0.05"/>   <!-- Rayon en m -->

  <!-- ========================================== -->
  <!-- DÉFINITION DES COULEURS -->
  <!-- ========================================== -->
  
  <material name="blue">
    <color rgba="0 0 0.8 1"/>  <!-- Rouge Vert Bleu Alpha -->
  </material>
  
  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>
  
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1"/>
  </material>

  <!-- ========================================== -->
  <!-- WORLD LINK (Frame de référence fixe) -->
  <!-- ========================================== -->
  
  <link name="world"/>

  <!-- ========================================== -->
  <!-- BASE LINK (Base du robot) -->
  <!-- ========================================== -->
  
  <link name="base_link">
    <!-- Partie visuelle (ce qu'on voit) -->
    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>  <!-- Position: x y z, Orientation: roll pitch yaw -->
      <geometry>
        <box size="0.1 0.1 1.0"/>  <!-- Boîte de 10cm x 10cm x 1m -->
      </geometry>
      <material name="grey"/>
    </visual>
    
    <!-- Partie collision (pour la physique) -->
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 1.0"/>
      </geometry>
    </collision>
    
    <!-- Propriétés inertielles (pour la physique) -->
    <inertial>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <mass value="1.0"/>  <!-- Masse en kg -->
      <!-- Matrice d'inertie : résistance à la rotation -->
      <inertia ixx="0.083" ixy="0.0" ixz="0.0" 
               iyy="0.083" iyz="0.0" 
               izz="0.0017"/>
    </inertial>
  </link>

  <!-- Joint fixe entre world et base_link -->
  <joint name="world_to_base" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>

  <!-- ========================================== -->
  <!-- LINK 1 (Premier segment du bras) -->
  <!-- ========================================== -->
  
  <link name="link1">
    <visual>
      <origin xyz="0 0 ${link_length/2}" rpy="0 0 0"/>
      <geometry>
        <!-- Cylindre : rayon et longueur -->
        <cylinder radius="${link_radius}" length="${link_length}"/>
      </geometry>
      <material name="blue"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 ${link_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${link_radius}" length="${link_length}"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 ${link_length/2}" rpy="0 0 0"/>
      <mass value="${link_mass}"/>
      <inertia ixx="0.042" ixy="0.0" ixz="0.0" 
               iyy="0.042" iyz="0.0" 
               izz="0.00063"/>
    </inertial>
  </link>

  <!-- JOINT 1 (Articulation rotative) -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 1.0" rpy="0 0 0"/>  <!-- Position sur la base -->
    <axis xyz="0 1 0"/>  <!-- Axe de rotation : Y (rotation latérale) -->
    <!-- Limites : angle min/max, effort max, vitesse max -->
    <limit lower="-${PI/2}" upper="${PI/2}" effort="100.0" velocity="1.0"/>
    <dynamics damping="0.7"/>  <!-- Amortissement pour stabilité -->
  </joint>

  <!-- ========================================== -->
  <!-- LINK 2 (Deuxième segment du bras) -->
  <!-- ========================================== -->
  
  <link name="link2">
    <visual>
      <origin xyz="0 0 ${link_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${link_radius}" length="${link_length}"/>
      </geometry>
      <material name="white"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 ${link_length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${link_radius}" length="${link_length}"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 ${link_length/2}" rpy="0 0 0"/>
      <mass value="${link_mass}"/>
      <inertia ixx="0.042" ixy="0.0" ixz="0.0" 
               iyy="0.042" iyz="0.0" 
               izz="0.00063"/>
    </inertial>
  </link>

  <!-- JOINT 2 (Deuxième articulation) -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 ${link_length}" rpy="0 0 0"/>  <!-- Au bout de link1 -->
    <axis xyz="0 1 0"/>  <!-- Même axe de rotation que joint1 -->
    <limit lower="-${PI/2}" upper="${PI/2}" effort="100.0" velocity="1.0"/>
    <dynamics damping="0.7"/>
  </joint>

  <!-- ========================================== -->
  <!-- CONFIGURATION GAZEBO -->
  <!-- ========================================== -->
  
  <!-- Définir les couleurs dans Gazebo (différent de URDF) -->
  <gazebo reference="base_link">
    <material>Gazebo/Grey</material>
  </gazebo>
  
  <gazebo reference="link1">
    <material>Gazebo/Blue</material>
  </gazebo>
  
  <gazebo reference="link2">
    <material>Gazebo/White</material>
  </gazebo>

  <!-- ========================================== -->
  <!-- ROS 2 CONTROL CONFIGURATION -->
  <!-- ========================================== -->
  
  <!-- Définir le système de contrôle -->
  <ros2_control name="GazeboSystem" type="system">
    <hardware>
      <!-- Plugin qui connecte ROS 2 Control à Gazebo -->
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    
    <!-- Configuration du Joint 1 -->
    <joint name="joint1">
      <!-- Interface de commande : on envoie des positions -->
      <command_interface name="position">
        <param name="min">-1.57</param>  <!-- -90° -->
        <param name="max">1.57</param>   <!-- +90° -->
      </command_interface>
      <!-- Interfaces d'état : on lit position, vitesse, effort -->
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
    
    <!-- Configuration du Joint 2 -->
    <joint name="joint2">
      <command_interface name="position">
        <param name="min">-1.57</param>
        <param name="max">1.57</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>

  <!-- Plugin Gazebo ROS 2 Control -->
  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <!-- Chemin vers le fichier de configuration des contrôleurs -->
      <parameters>$(find rrbot_gazebo)/config/rrbot_controllers.yaml</parameters>
    </plugin>
  </gazebo>

</robot>
EOF
```

**💡 Explications importantes :**

1. **Structure URDF** :
    
    - `<link>` : un corps rigide (pièce du robot)
    - `<joint>` : une articulation entre deux links
    - Chaque link a 3 parties : visual (rendu), collision (physique), inertial (masse/inertie)
2. **Types de joints** :
    
    - `fixed` : joint fixe (pas de mouvement)
    - `revolute` : rotation avec limites
    - `continuous` : rotation sans limites
    - `prismatic` : translation (glissement)
3. **ROS 2 Control** :
    
    - `command_interface` : ce qu'on envoie au robot (position, vitesse, effort)
    - `state_interface` : ce qu'on lit du robot

---

## 4. Configuration des Contrôleurs

```bash
cat > config/rrbot_controllers.yaml << 'EOF'
# Configuration du Controller Manager
controller_manager:
  ros__parameters:
    update_rate: 100  # Fréquence de mise à jour en Hz

    # Liste des contrôleurs à charger
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

# Configuration du Joint Trajectory Controller
joint_trajectory_controller:
  ros__parameters:
    # Joints à contrôler
    joints:
      - joint1
      - joint2
    
    # Type de commandes acceptées
    command_interfaces:
      - position  # On envoie des positions cibles
    
    # Types d'états publiés
    state_interfaces:
      - position  # Position actuelle
      - velocity  # Vitesse actuelle
    
    # Fréquences de publication
    state_publish_rate: 50.0        # Hz
    action_monitor_rate: 20.0       # Hz
    
    # Options
    allow_partial_joints_goal: false  # Doit contrôler TOUS les joints
    
    # Contraintes de trajectoire
    constraints:
      stopped_velocity_tolerance: 0.01  # Vitesse considérée comme arrêt
      goal_time: 0.0  # Pas de timeout pour atteindre l'objectif
EOF
```

**💡 Explication des contrôleurs :**

1. **joint_state_broadcaster** :
    
    - Publie l'état des joints sur `/joint_states`
    - Nécessaire pour RVIZ et robot_state_publisher
    - Toujours actif
2. **joint_trajectory_controller** :
    
    - Accepte des trajectoires (suite de positions avec timing)
    - Interpole entre les points pour un mouvement fluide
    - Publie sur `/joint_trajectory_controller/joint_trajectory`

---

## 5. Monde Gazebo

```bash
cat > worlds/empty.world << 'EOF'
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    
    <!-- Ajouter le soleil (lumière) -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Ajouter le sol -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Configuration du moteur physique -->
    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>  <!-- Fréquence simulation -->
      <max_step_size>0.001</max_step_size>  <!-- Pas de temps : 1ms -->
      <real_time_factor>1</real_time_factor>  <!-- Vitesse normale (1x) -->
    </physics>
    
  </world>
</sdf>
EOF
```

**💡 Explication :**

- `<physics>` : définit comment la simulation physique fonctionne
- `real_time_update_rate` : nombre de fois par seconde que Gazebo calcule la physique
- `max_step_size` : précision de la simulation (plus petit = plus précis mais plus lent)

---

## 6. Fichier de Lancement Principal

```bash
cat > launch/rrbot_gazebo.launch.py << 'EOF'
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # ========================================
    # CHEMINS DES FICHIERS
    # ========================================
    
    pkg_path = get_package_share_directory('rrbot_gazebo')
    
    urdf_file = os.path.join(pkg_path, 'urdf', 'rrbot.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'empty.world')
    
    # ========================================
    # GÉNÉRATION DE LA DESCRIPTION DU ROBOT
    # ========================================
    
    # Convertir le fichier Xacro en URDF
    robot_description = Command(['xacro ', urdf_file])
    
    # ========================================
    # ROBOT STATE PUBLISHER
    # ========================================
    # Publie les transformations TF du robot
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True  # Utiliser le temps de Gazebo
        }]
    )
    
    # ========================================
    # GAZEBO
    # ========================================
    # Lancer le simulateur Gazebo
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'false'
        }.items()
    )
    
    # ========================================
    # SPAWN ROBOT DANS GAZEBO
    # ========================================
    # Injecter le robot dans la simulation
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',  # Lire depuis ce topic
            '-entity', 'rrbot'  # Nom du robot dans Gazebo
        ],
        output='screen'
    )
    
    # ========================================
    # CONTRÔLEURS ROS 2 CONTROL
    # ========================================
    
    # 1. Joint State Broadcaster (publie /joint_states)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    # 2. Joint Trajectory Controller (accepte les trajectoires)
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )
    
    # ========================================
    # SÉQUENCEMENT DES ACTIONS
    # ========================================
    # Attendre que le robot soit spawné avant de lancer les contrôleurs
    
    # Lancer joint_state_broadcaster après le spawn
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    
    # Lancer joint_trajectory_controller après joint_state_broadcaster
    delay_joint_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )
    
    # ========================================
    # RETOURNER LA DESCRIPTION DE LANCEMENT
    # ========================================
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        delay_joint_state_broadcaster,
        delay_joint_trajectory_controller
    ])
EOF

chmod +x launch/rrbot_gazebo.launch.py
```

**💡 Explication du fichier de lancement :**

1. **Ordre d'exécution** :
    
    - Gazebo démarre
    - Robot State Publisher publie la description
    - Le robot est injecté dans Gazebo
    - Les contrôleurs sont lancés (dans l'ordre)
2. **RegisterEventHandler** :
    
    - Permet d'attendre qu'une action se termine avant d'en lancer une autre
    - Important pour éviter les erreurs de synchronisation

---

## 7. Script de Test du Contrôleur

```bash
cat > scripts/test_controller.py << 'EOF'
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class RRBotTestController(Node):
    """
    Script de test qui envoie des trajectoires automatiques au robot.
    """
    
    def __init__(self):
        super().__init__('rrbot_test_controller')
        
        # Créer un publisher pour envoyer des trajectoires
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Timer pour envoyer la première trajectoire après 2 secondes
        self.timer = self.create_timer(2.0, self.send_trajectory)
        self.trajectory_sent = False
        
        self.get_logger().info('RRBot Test Controller démarré')
        self.get_logger().info('Le robot va bouger dans 2 secondes...')
    
    def send_trajectory(self):
        """Envoie une trajectoire complète au robot."""
        
        if not self.trajectory_sent:
            # Créer le message de trajectoire
            msg = JointTrajectory()
            msg.joint_names = ['joint1', 'joint2']
            
            # ========================================
            # POINT 1 : Mouvement vers la droite
            # ========================================
            point1 = JointTrajectoryPoint()
            point1.positions = [0.5, 0.5]  # Position des joints en radians
            point1.time_from_start = Duration(sec=2)  # Atteindre en 2 secondes
            
            # ========================================
            # POINT 2 : Mouvement vers la gauche
            # ========================================
            point2 = JointTrajectoryPoint()
            point2.positions = [-0.5, -0.5]
            point2.time_from_start = Duration(sec=4)  # Atteindre en 4 secondes
            
            # ========================================
            # POINT 3 : Retour à la position initiale
            # ========================================
            point3 = JointTrajectoryPoint()
            point3.positions = [0.0, 0.0]
            point3.time_from_start = Duration(sec=6)  # Atteindre en 6 secondes
            
            # Ajouter tous les points à la trajectoire
            msg.points = [point1, point2, point3]
            
            # Publier la trajectoire
            self.publisher.publish(msg)
            self.get_logger().info('✅ Trajectoire envoyée !')
            self.get_logger().info('   Point 1: [0.5, 0.5] à t=2s')
            self.get_logger().info('   Point 2: [-0.5, -0.5] à t=4s')
            self.get_logger().info('   Point 3: [0.0, 0.0] à t=6s')
            
            self.trajectory_sent = True
            
            # Répéter la trajectoire toutes les 7 secondes
            self.timer = self.create_timer(7.0, self.repeat_trajectory)
    
    def repeat_trajectory(self):
        """Répète la trajectoire en boucle."""
        self.get_logger().info('🔄 Répétition de la trajectoire...')
        self.trajectory_sent = False
        self.send_trajectory()

def main(args=None):
    rclpy.init(args=args)
    controller = RRBotTestController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

chmod +x scripts/test_controller.py
```

**💡 Explication du script :**

1. **JointTrajectory** :
    
    - Contient une liste de joints et une liste de points
    - Chaque point a des positions et un timestamp
2. **time_from_start** :
    
    - Temps depuis le début de la trajectoire
    - Le contrôleur interpole entre les points
3. **Interpolation** :
    
    - Si on va de 0 à 1 en 2 secondes, à t=1s on sera à 0.5
    - Mouvement fluide et continu

---

## 8. Configuration RVIZ

```bash
cat > rviz/rrbot.rviz << 'EOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views

Visualization Manager:
  Displays:
    # Grille de référence
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      
    # Modèle 3D du robot
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Visual Enabled: true
      Collision Enabled: false
      
    # Frames de transformation
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Frame Timeout: 15.0
      Show Axes: true
      Show Names: true
      
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 5.0
      Focal Point:
        X: 0.0
        Y: 0.0
        Z: 1.0
      Pitch: 0.5
      Yaw: 0.8
EOF
```

**💡 Configuration RVIZ :**
- `RobotModel` lit `/robot_description` pour afficher le robot
- `TF` montre les frames et leurs relations
- `Grid` donne une référence spatiale

---

## 9. Compilation et Test

### 9.1 Compiler le package

```bash
cd ~/ros2_ws
colcon build --packages-select rrbot_gazebo --symlink-install
source install/setup.bash
```

**💡 À propos de la compilation :**
- `--packages-select` : compile uniquement ce package (plus rapide)
- `--symlink-install` : crée des liens symboliques au lieu de copies
- `source install/setup.bash` : active le package dans le terminal

### 9.2 Lancer la simulation

```bash
# Terminal 1: Lancer Gazebo avec le robot
ros2 launch rrbot_gazebo rrbot_gazebo.launch.py
```

### 9.3 Vérifier que tout fonctionne

```bash
# Terminal 2: Vérifier les contrôleurs
ros2 control list_controllers

# Devrait afficher:
# joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
# joint_trajectory_controller[joint_trajectory_controller/JointTrajectoryController] active

# Vérifier les topics
ros2 topic list

# Vérifier les états des joints
ros2 topic echo /joint_states
```

### 9.4 Tester le mouvement du robot

```bash
# Terminal 3: Lancer le script de test
ros2 run rrbot_gazebo test_controller.py

# Le robot devrait bouger automatiquement !
```

**💡 Ce que vous devriez voir :**
- Dans Gazebo : le robot avec 2 segments bleu et blanc bouge
- Dans la console : logs de trajectoire envoyée
- Mouvement : droite (2s) → gauche (4s) → centre (6s) → répétition

### 9.5 Contrôle manuel (optionnel)

```bash
# Envoyer une commande manuelle
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint1', 'joint2'],
  points: [
    {
      positions: [0.7, -0.7],
      time_from_start: {sec: 2, nanosec: 0}
    }
  ]
}"
```

**💡 Commande manuelle :**
- `--once` : publie une seule fois
- Format JSON pour spécifier la trajectoire
- Utile pour tester des positions spécifiques

---

## 10. Lancement avec RVIZ

```bash
cat > launch/rrbot_with_rviz.launch.py << 'EOF'
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_path = get_package_share_directory('rrbot_gazebo')
    rviz_config = os.path.join(pkg_path, 'rviz', 'rrbot.rviz')
    
    # Inclure le lancement Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rrbot_gazebo'),
                'launch',
                'rrbot_gazebo.launch.py'
            ])
        ])
    )
    
    # RVIZ
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_launch,
        rviz
    ])
EOF

chmod +x launch/rrbot_with_rviz.launch.py
```

### Lancer avec RVIZ:

```bash
ros2 launch rrbot_gazebo rrbot_with_rviz.launch.py
```

**💡 RVIZ + Gazebo :**
- Visualisation temps réel du robot
- Peut voir les frames TF, les collisions, etc.
- Utile pour debugger la simulation

---

## 11. Commandes de Vérification

```bash
# Vérifier que le package est bien installé
ros2 pkg list | grep rrbot_gazebo

# Lister les contrôleurs
ros2 control list_controllers

# Vérifier les hardware interfaces
ros2 control list_hardware_interfaces

# Voir les états des contrôleurs
ros2 control list_controller_types

# Monitorer les joints en temps réel
ros2 topic hz /joint_states

# Voir les transformations TF
ros2 run tf2_tools view_frames
evince frames.pdf  # Ouvrir le PDF généré
```

**💡 Debugging :**
Si quelque chose ne marche pas :
1. Vérifier `ros2 control list_controllers` - sont-ils `active` ?
2. Vérifier `ros2 topic echo /joint_states` - les données arrivent-elles ?
3. Regarder les logs dans les terminaux de lancement

---

## 12. Résumé des Commandes

```bash
# 1. Compiler
cd ~/ros2_ws
colcon build --packages-select rrbot_gazebo --symlink-install
source install/setup.bash

# 2. Lancer la simulation
ros2 launch rrbot_gazebo rrbot_gazebo.launch.py

# 3. Dans un autre terminal: Tester le robot
ros2 run rrbot_gazebo test_controller.py

# 4. Ou lancer avec RVIZ
ros2 launch rrbot_gazebo rrbot_with_rviz.launch.py
```