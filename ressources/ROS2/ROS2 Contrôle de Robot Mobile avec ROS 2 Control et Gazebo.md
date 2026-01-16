# Tutoriel : 

## 📦 Installation des Dépendances

### Paquets ROS 2 Control et Gazebo
```bash
# Mettre à jour les dépôts
sudo apt-get update

# Installer les paquets essentiels
sudo apt install ros-${ROS_DISTRO}-gazebo-ros-pkgs \
                 ros-${ROS_DISTRO}-ros2-control \
                 ros-${ROS_DISTRO}-ros2-controllers \
                 ros-${ROS_DISTRO}-gazebo-ros2-control

# Remplacer ${ROS_DISTRO} par votre version (ex: humble)
sudo apt install ros-humble-gazebo-ros-pkgs \
                 ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-gazebo-ros2-control
```

### Vérification de l'Installation
```bash
# Vérifier que les paquets sont installés
ros2 pkg list | grep -E "(control|gazebo)"

# Tester Gazebo
gazebo --version

# Tester ROS 2 Control
ros2 control list_controllers
```

## 🗂️ Structure du Package Robot Mobile

### Création du Package
```bash
# Créer le package avec la structure CMake
ros2 pkg create --build-type ament_cmake wheeled_robot

# Structure finale du package
wheeled_robot/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── diff_drive_controller.yaml
├── launch/
│   └── gazebo.launch.py
├── urdf/
│   └── cybertruck.urdf.xacro
├── meshes/
│   ├── body.stl
│   └── wheel.stl
├── src/
│   └── diff_drive_publisher.cpp
└── include/
    └── wheeled_robot/
```

## 🛠️ Configuration de CMakeLists.txt

### Fichier CMakeLists.txt Complet
```cmake
cmake_minimum_required(VERSION 3.8)
project(wheeled_robot)

# Définir les standards C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Trouver les dépendances
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)

# Définir les includes
include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

# Créer l'exécutable du publisher
add_executable(diff_drive_publisher 
  src/diff_drive_publisher.cpp
)

# Lier les dépendances
ament_target_dependencies(diff_drive_publisher
  rclcpp
  geometry_msgs
)

# Installer les exécutables
install(TARGETS
  diff_drive_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Installer les fichiers launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Installer les fichiers URDF
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# Installer les fichiers de configuration
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Installer les maillages
install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}
)

# Dépendances ament
ament_export_dependencies(
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  urdf
  gazebo_ros
  controller_manager
  controller_interface
  hardware_interface
)

# Générer le package
ament_package()
```

## 📐 Cadres de Coordonnées du Robot Mobile

### Vue de Dessus (Top View)
```
          AVANT
        ▲ (y)
        │
        │   Roue AV_Droite
        │     ╔═══╗
        │     ║   ║
        │     ╚═══╝
GAUCHE ◄───●─────●─────► DROITE (x)
        │  Centre  │
        │ du Robot │
        │     ╔═══╗
        │     ║   ║
        │     ╚═══╝
        │   Roue AR_Gauche
        │
        ▼ ARRIÈRE

Dimensions :
- Longueur totale : 1.146 m
- Largeur totale : 0.4175 m
- Empattement : 0.85 m
- Voie : 0.38 m
```

### Vue de Côté (Side View)
```
              ▲ (z)
              │
              │    ╔════════════╗
              │    ║  Fenêtre   ║
              │    ╚════════════╝
              │    ╔════════════╗
              │    ║   Corps    ║
              │    ╚════════════╝
              │      ╔═══╗
              │      ║   ║  Roue
              │      ╚═══╝
──────────────┼─────────────────────► (x)
              │
              │ Hauteur du châssis : 0.15 m
              │ Diamètre des roues : 0.25 m
              ▼
```

## 📄 Fichier URDF/XACRO pour Robot Mobile

### Fichier `cybertruck.urdf.xacro` Complet
```xml
<?xml version="1.0"?>
<robot name="cybertruck" xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- Constantes du robot -->
  <xacro:property name="body_length" value="1.146" />
  <xacro:property name="body_width" value="0.4175" />
  <xacro:property name="body_height" value="0.3" />
  <xacro:property name="wheel_radius" value="0.125" />
  <xacro:property name="wheel_width" value="0.05" />
  <xacro:property name="wheelbase" value="0.85" />  <!-- Distance entre essieux -->
  <xacro:property name="track_width" value="0.38" /> <!-- Distance entre roues -->

  <!-- Macro pour une roue -->
  <xacro:macro name="wheel" params="prefix parent flip">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black">
          <color rgba="0.1 0.1 0.1 1.0"/>
        </material>
      </visual>
      
      <!-- IMPORTANT: Tag collision nécessaire pour les interactions physiques -->
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      
      <inertial>
        <mass value="2.0"/>
        <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                 iyy="0.1" iyz="0.0"
                 izz="0.1"/>
      </inertial>
    </link>
    
    <joint name="${parent}_to_${prefix}_wheel" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      
      <!-- IMPORTANT: Inverser l'axe pour les roues droites/gauches -->
      <axis xyz="${flip} 0 0"/>
      
      <origin xyz="${wheelbase/2 if 'front' in prefix else -wheelbase/2} 
                   ${track_width/2 if 'right' in prefix else -track_width/2} 
                   -${body_height/2 + wheel_radius}" 
              rpy="0 ${pi/2} 0"/>
    </joint>
  </xacro:macro>

  <!-- Lien Corps -->
  <link name="body">
    <visual>
      <geometry>
        <box size="${body_length} ${body_width} ${body_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${body_length} ${body_width} ${body_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="50.0"/>
      <inertia ixx="5.0" ixy="0.0" ixz="0.0"
               iyy="5.0" iyz="0.0"
               izz="5.0"/>
    </inertial>
  </link>

  <!-- Lien Fenêtre -->
  <link name="window">
    <visual>
      <geometry>
        <box size="${body_length*0.9} ${body_width*0.9} ${body_height*0.3}"/>
      </geometry>
      <material name="transparent">
        <color rgba="0.7 0.7 1.0 0.5"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${body_length*0.9} ${body_width*0.9} ${body_height*0.3}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.5" ixy="0.0" ixz="0.0"
               iyy="0.5" iyz="0.0"
               izz="0.5"/>
    </inertial>
  </link>

  <!-- Joint Corps → Fenêtre -->
  <joint name="body_to_window" type="fixed">
    <parent link="body"/>
    <child link="window"/>
    <origin xyz="0 0 ${body_height/2}" rpy="0 0 0"/>
  </joint>

  <!-- Créer les 4 roues -->
  <xacro:wheel prefix="front_left" parent="body" flip="1"/>
  <xacro:wheel prefix="front_right" parent="body" flip="-1"/>
  <xacro:wheel prefix="back_left" parent="body" flip="1"/>
  <xacro:wheel prefix="back_right" parent="body" flip="-1"/>

  <!-- Transmissions pour ROS 2 Control -->
  <transmission name="front_left_wheel_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="body_to_front_left_wheel">
      <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="front_left_wheel_motor">
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="front_right_wheel_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="body_to_front_right_wheel">
      <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="front_right_wheel_motor">
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="back_left_wheel_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="body_to_back_left_wheel">
      <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="back_left_wheel_motor">
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <transmission name="back_right_wheel_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="body_to_back_right_wheel">
      <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="back_right_wheel_motor">
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <!-- Plugin Gazebo -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros2_control.so">
      <robot_namespace>/cybertruck</robot_namespace>
      <robot_param>robot_description</robot_param>
      <robot_param_node>robot_state_publisher</robot_param_node>
    </plugin>
  </gazebo>

</robot>
```

## 🚀 Fichier Launch pour Gazebo et ROS 2 Control

### Fichier `gazebo.launch.py` Complet
```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    
    # Chemins des fichiers
    pkg_path = FindPackageShare('wheeled_robot').find('wheeled_robot')
    urdf_file = PathJoinSubstitution([pkg_path, 'urdf', 'cybertruck.urdf.xacro'])
    world_file = PathJoinSubstitution([pkg_path, 'worlds', 'empty.world'])
    rviz_config = PathJoinSubstitution([pkg_path, 'config', 'cybertruck.rviz'])
    
    # 1. Publisher d'état du robot
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
                FindExecutable('xacro'), ' ', urdf_file
            ]),
            'use_sim_time': True
        }]
    )
    
    # 2. Spawn du robot dans Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'cybertruck',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2',
            '-Y', '0.0'
        ],
        output='screen'
    )
    
    # 3. Lancement de Gazebo
    gazebo_process = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # 4. Publisher d'état des joints
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    # 5. Manager de contrôleurs ROS 2
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[PathJoinSubstitution([pkg_path, 'config', 'diff_drive_controller.yaml'])],
        output='screen'
    )
    
    # 6. Chargement du diffuseur d'état des joints
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    
    # 7. Chargement du contrôleur différentiel
    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'diff_drive_controller'],
        output='screen'
    )
    
    # 8. Lancement de PlotJuggler (optionnel)
    run_plotjuggler = ExecuteProcess(
        cmd=['ros2', 'run', 'plotjuggler', 'plotjuggler'],
        output='screen'
    )
    
    # 9. Lancement de RViz (optionnel)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    # Événement : Charger les contrôleurs après le spawn
    load_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster, load_diff_drive_controller]
        )
    )
    
    return LaunchDescription([
        gazebo_process,
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity,
        controller_manager_node,
        load_controllers,
        # run_plotjuggler,  # Décommenter si nécessaire
        # rviz_node,        # Décommenter si nécessaire
    ])
```

## ⚙️ Configuration YAML du Contrôleur Différentiel

### Fichier `diff_drive_controller.yaml` Complet
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    
    # Liste des contrôleurs disponibles
    controller_list:
      - name: joint_state_broadcaster
        type: joint_state_broadcaster/JointStateBroadcaster
        
      - name: diff_drive_controller
        type: diff_drive_controller/DiffDriveController

diff_drive_controller:
  ros__parameters:
    # Paramètres de publication
    left_wheel_names: ["body_to_front_left_wheel", "body_to_back_left_wheel"]
    right_wheel_names: ["body_to_front_right_wheel", "body_to_back_right_wheel"]
    
    # Dimensions du robot
    wheel_separation: 0.38     # Distance entre les roues (m)
    wheel_radius: 0.125        # Rayon des roues (m)
    
    # Paramètres de commande
    wheel_separation_multiplier: 1.0
    left_wheel_radius_multiplier: 1.0
    right_wheel_radius_multiplier: 1.0
    
    # PID pour le contrôle de vitesse
    linear.x.velocity.pid.i: 10.0
    linear.x.velocity.pid.d: 0.0
    linear.x.velocity.pid.p: 1.0
    linear.x.velocity.pid.i_clamp_max: 100.0
    linear.x.velocity.pid.i_clamp_min: -100.0
    
    angular.z.velocity.pid.i: 10.0
    angular.z.velocity.pid.d: 0.0
    angular.z.velocity.pid.p: 1.0
    angular.z.velocity.pid.i_clamp_max: 100.0
    angular.z.velocity.pid.i_clamp_min: -100.0
    
    # Limites de commande
    linear.x.max_velocity: 1.0      # m/s
    angular.z.max_velocity: 2.0     # rad/s
    
    linear.x.min_velocity: -1.0     # m/s
    angular.z.min_velocity: -2.0    # rad/s
    
    linear.x.max_acceleration: 0.5  # m/s²
    angular.z.max_acceleration: 1.0 # rad/s²
    
    # Topics
    odom_frame_id: "odom"
    base_frame_id: "base_link"
    pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    
    # Fréquences de publication
    publish_rate: 50.0       # Hz
    cmd_vel_timeout: 1.0     # secondes
    
    # Enable/disable features
    enable_odom_tf: true
    open_loop: false
    use_stamped_vel: false
    
    # Topics de publication
    velocity_rolling_window_size: 10
    publish_cmd: false
    
joint_state_broadcaster:
  ros__parameters:
    # Paramètres du diffuseur d'état des joints
    rate: 50  # Hz
```

## 🎮 Publisher de Commandes Différentielles

### Fichier `diff_drive_publisher.cpp` Complet
```cpp
#include <memory>
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class DiffDrivePublisher : public rclcpp::Node
{
public:
  DiffDrivePublisher()
  : Node("diff_drive_publisher"), count_(0)
  {
    // Créer le publisher pour les commandes Twist
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/diff_drive_controller/cmd_vel_unstamped", 10);
    
    // Timer pour publier régulièrement
    timer_ = this->create_wall_timer(
      100ms, std::bind(&DiffDrivePublisher::timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "Diff Drive Publisher démarré");
  }

private:
  void timer_callback()
  {
    auto message = geometry_msgs::msg::Twist();
    
    // Différentes commandes selon le compteur
    if (count_ < 20) {
      // Avancer tout droit pendant 2 secondes
      message.linear.x = 0.5;
      message.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Commande: Avancer [lin.x=%.2f]", message.linear.x);
    }
    else if (count_ < 40) {
      // Tourner à gauche pendant 2 secondes
      message.linear.x = 0.1;
      message.angular.z = 0.5;
      RCLCPP_INFO(this->get_logger(), "Commande: Tourner gauche [ang.z=%.2f]", message.angular.z);
    }
    else if (count_ < 60) {
      // Avancer tout droit
      message.linear.x = 0.5;
      message.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Commande: Avancer [lin.x=%.2f]", message.linear.x);
    }
    else if (count_ < 80) {
      // Tourner à droite pendant 2 secondes
      message.linear.x = 0.1;
      message.angular.z = -0.5;
      RCLCPP_INFO(this->get_logger(), "Commande: Tourner droite [ang.z=%.2f]", message.angular.z);
    }
    else if (count_ < 100) {
      // Reculer pendant 2 secondes
      message.linear.x = -0.3;
      message.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Commande: Reculer [lin.x=%.2f]", message.linear.x);
    }
    else {
      // S'arrêter
      message.linear.x = 0.0;
      message.angular.z = 0.0;
      if (count_ == 100) {
        RCLCPP_INFO(this->get_logger(), "Commande: Arrêt");
      }
    }
    
    publisher_->publish(message);
    count_++;
    
    // Réinitialiser après un cycle complet
    if (count_ > 120) {
      count_ = 0;
    }
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffDrivePublisher>());
  rclcpp::shutdown();
  return 0;
}
```

## 📊 Visualisation des Données avec PlotJuggler

### Configuration PlotJuggler pour le Robot Mobile
1. **Lancer PlotJuggler** après le démarrage de la simulation
```bash
ros2 run plotjuggler plotjuggler
```

2. **Configurer les Streams ROS 2** :
   - Dans PlotJuggler : **Streaming** → **ROS 2 Topic Subscriber**
   - Domain ID : `0`
   - Topics à ajouter :
     - `/joint_states` (états des joints)
     - `/diff_drive_controller/odom` (odométrie)
     - `/diff_drive_controller/cmd_vel` (commandes)

3. **Créer un Layout pour l'Analyse** :
```xml
<PlotJugglerLayout>
  <windows>
    <!-- Fenêtre 1: Vitesses des joints -->
    <window title="Vitesses des Roues">
      <plot title="Vitesses">
        <curve topic="/joint_states" field="velocity[0]" label="Roue AV Gauche"/>
        <curve topic="/joint_states" field="velocity[1]" label="Roue AV Droite"/>
        <curve topic="/joint_states" field="velocity[2]" label="Roue AR Gauche"/>
        <curve topic="/joint_states" field="velocity[3]" label="Roue AR Droite"/>
      </plot>
    </window>
    
    <!-- Fenêtre 2: Commandes vs Performance -->
    <window title="Commandes vs Odometrie">
      <plot title="Vitesse Linéaire">
        <curve topic="/diff_drive_controller/odom" field="twist.twist.linear.x" label="Vitesse Réelle"/>
        <curve topic="/diff_drive_controller/cmd_vel" field="linear.x" label="Commande"/>
      </plot>
      <plot title="Vitesse Angulaire">
        <curve topic="/diff_drive_controller/odom" field="twist.twist.angular.z" label="Vitesse Réelle"/>
        <curve topic="/diff_drive_controller/cmd_vel" field="angular.z" label="Commande"/>
      </plot>
    </window>
    
    <!-- Fenêtre 3: Position et Orientation -->
    <window title="Position du Robot">
      <plot title="Position X">
        <curve topic="/diff_drive_controller/odom" field="pose.pose.position.x" label="X"/>
      </plot>
      <plot title="Position Y">
        <curve topic="/diff_drive_controller/odom" field="pose.pose.position.y" label="Y"/>
      </plot>
      <plot title="Orientation (Yaw)">
        <curve topic="/diff_drive_controller/odom" field="pose.pose.orientation.z" label="Quaternion Z"/>
      </plot>
    </window>
  </windows>
</PlotJuggilerLayout>
```

## 🚀 Lancer la Simulation Complète

### Commandes Terminal
```bash
# Terminal 1: Construire le package
cd ~/ros2_ws
colcon build --packages-select wheeled_robot

# Terminal 2: Lancer la simulation
source ~/ros2_ws/install/setup.bash
ros2 launch wheeled_robot gazebo.launch.py

# Terminal 3: Lancer le publisher de commandes
source ~/ros2_ws/install/setup.bash
ros2 run wheeled_robot diff_drive_publisher

# Terminal 4: Visualiser avec PlotJuggler (optionnel)
ros2 run plotjuggler plotjuggler -l /chemin/vers/layout.xml
```

### Vérification du Système
```bash
# Vérifier les contrôleurs actifs
ros2 control list_controllers

# Vérifier les états des joints
ros2 topic echo /joint_states

# Vérifier l'odométrie
ros2 topic echo /diff_drive_controller/odom

# Tester manuellement avec téléop
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

## 🛠️ Dépannage Courant

### Problème : Robot tombe à travers le sol
```xml
<!-- S'assurer que chaque lien de roue a un tag collision -->
<link name="wheel">
  <collision>
    <geometry>
      <cylinder radius="0.125" length="0.05"/>
    </geometry>
  </collision>
</link>
```

### Problème : Roues tournent dans le mauvais sens
```xml
<!-- Inverser l'axe pour les roues droites -->
<axis xyz="-1 0 0"/>  <!-- Pour roue droite -->
<axis xyz="1 0 0"/>   <!-- Pour roue gauche -->
```

### Problème : Contrôleurs non chargés
```bash
# Vérifier le manager de contrôleurs
ros2 control list_controllers

# Charger manuellement les contrôleurs
ros2 control load_controller joint_state_broadcaster
ros2 control load_controller diff_drive_controller

# Activer les contrôleurs
ros2 control set_controller_state joint_state_broadcaster active
ros2 control set_controller_state diff_drive_controller active
```

Ce tutoriel complet vous permet de créer, simuler et contrôler un robot mobile différentiel avec ROS 2 Control dans Gazebo, avec visualisation avancée des données via PlotJuggler.