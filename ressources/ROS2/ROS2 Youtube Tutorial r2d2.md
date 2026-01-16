
# Tutoriel Complet R2D2 - Simulation de Trajectoire dans RVIZ

## 1. Structure du Package R2D2

### 1.1 Création du package
```bash
# Créer le package avec toutes les dépendances nécessaires
cd ~/ros2_ws/src
ros2 pkg create r2d2_robot \
  --build-type ament_python \
  --dependencies rclpy geometry_msgs tf2_ros tf2_msgs sensor_msgs nav_msgs \
  --description "Package R2D2 avec simulation de trajectoire dans RVIZ"

# Structure finale du package
cd r2d2_robot
mkdir -p launch urdf rviz scripts config
```

### 1.2 Structure complète des fichiers
```
r2d2_robot/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── r2d2_robot
├── launch/
│   ├── display_r2d2.launch.py      # Lanceur principal
│   └── trajectory.launch.py        # Lanceur avec trajectoire
├── urdf/
│   ├── r2d2.urdf.xacro             # Modèle R2D2 complet
│   ├── common/
│   │   ├── materials.xacro         # Matériaux et couleurs
│   │   └── constants.xacro         # Constantes R2D2
│   └── components/
│       ├── body.xacro              # Corps principal
│       ├── head.xacro              # Tête rotative
│       └── wheels.xacro            # Roues et moteurs
├── rviz/
│   └── r2d2_config.rviz            # Configuration RVIZ
├── scripts/
│   ├── state_publisher.py          # Publication d'état des joints
│   └── trajectory_publisher.py     # Publication de trajectoire
├── config/
│   └── trajectory.yaml             # Points de trajectoire
└── test/
    └── test_trajectory.py          # Tests unitaires
```

---

## 2. Fichier URDF/Xacro de R2D2

### 2.1 Fichier principal `r2d2.urdf.xacro`

```xml
<?xml version="1.0"?>
<robot name="r2d2" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ============================================ -->
  <!-- INCLUSIONS -->
  <!-- ============================================ -->
  
  <xacro:include filename="$(find r2d2_robot)/urdf/common/materials.xacro" />
  <xacro:include filename="$(find r2d2_robot)/urdf/common/constants.xacro" />
  <xacro:include filename="$(find r2d2_robot)/urdf/components/body.xacro" />
  <xacro:include filename="$(find r2d2_robot)/urdf/components/head.xacro" />
  <xacro:include filename="$(find r2d2_robot)/urdf/components/wheels.xacro" />
  
  <!-- ============================================ -->
  <!-- ARGUMENTS -->
  <!-- ============================================ -->
  
  <xacro:arg name="color_body" default="blue_metal" />
  <xacro:arg name="color_head" default="white" />
  <xacro:arg name="color_wheels" default="black" />
  <xacro:arg name="use_fixed_caster" default="false" />
  
  <!-- ============================================ -->
  <!-- PROPRIÉTÉS GÉOMÉTRIQUES R2D2 -->
  <!-- ============================================ -->
  
  <xacro:property name="body_radius" value="0.2" />
  <xacro:property name="body_height" value="0.5" />
  <xacro:property name="body_mass" value="5.0" />
  
  <xacro:property name="head_radius" value="0.15" />
  <xacro:property name="head_height" value="0.2" />
  <xacro:property name="head_mass" value="1.0" />
  
  <xacro:property name="wheel_radius" value="0.08" />
  <xacro:property name="wheel_length" value="0.03" />
  <xacro:property name="wheel_mass" value="0.3" />
  
  <!-- Positions des roues (x, y, z) relatives au corps -->
  <xacro:property name="wheel_positions" value="${{
    'left': [0.0, 0.15, -body_height/2 + wheel_radius],
    'right': [0.0, -0.15, -body_height/2 + wheel_radius]
  }}" />
  
  <!-- ============================================ -->
  <!-- INSTANCIATION DU ROBOT -->
  <!-- ============================================ -->
  
  <!-- Corps principal -->
  <xacro:r2d2_body 
    radius="${body_radius}" 
    height="${body_height}" 
    mass="${body_mass}"
    color="$(arg color_body)" />
  
  <!-- Tête -->
  <xacro:r2d2_head 
    radius="${head_radius}" 
    height="${head_height}" 
    mass="${head_mass}"
    parent="body_link"
    xyz="0 0 ${body_height/2}"
    color="$(arg color_head)" />
  
  <!-- Roues motrices -->
  <xacro:differential_wheels 
    parent="body_link"
    radius="${wheel_radius}"
    length="${wheel_length}"
    mass="${wheel_mass}"
    positions="${wheel_positions}"
    color="$(arg color_wheels)" />
  
  <!-- Roulette avant (caster) -->
  <xacro:if value="$(arg use_fixed_caster)">
    <xacro:caster_wheel 
      parent="body_link"
      xyz="${body_radius*0.7} 0 ${-body_height/2 + 0.02}"
      radius="0.03"
      color="gray" />
  </xacro:if>
  
  <!-- Capteurs (optionnels) -->
  <xacro:property name="has_sensors" value="true" />
  <xacro:if value="${has_sensors}">
    <!-- Capteur avant -->
    <link name="front_sensor">
      <visual>
        <geometry><box size="0.03 0.03 0.03" /></geometry>
        <material name="red" />
      </visual>
    </link>
    <joint name="front_sensor_joint" type="fixed">
      <parent link="body_link" />
      <child link="front_sensor" />
      <origin xyz="${body_radius} 0 0.1" rpy="0 0 0" />
    </joint>
  </xacro:if>

</robot>
```

### 2.2 Composant `wheels.xacro`

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Macro pour une roue différentielle -->
  <xacro:macro name="differential_wheel" params="name parent radius length mass xyz axis">
    
    <link name="${name}">
      <visual>
        <origin rpy="${pi/2} 0 0" />
        <geometry>
          <cylinder radius="${radius}" length="${length}" />
        </geometry>
        <material name="black" />
      </visual>
      <collision>
        <origin rpy="${pi/2} 0 0" />
        <geometry>
          <cylinder radius="${radius}" length="${length}" />
        </geometry>
      </collision>
      <inertial>
        <mass value="${mass}" />
        <origin xyz="0 0 0" />
        <inertia 
          ixx="${mass * (3*radius*radius + length*length) / 12}"
          iyy="${mass * (3*radius*radius + length*length) / 12}"
          izz="${mass * radius * radius / 2}"
          ixy="0" ixz="0" iyz="0" />
      </inertial>
    </link>
    
    <joint name="${name}_joint" type="continuous">
      <parent link="${parent}" />
      <child link="${name}" />
      <origin xyz="${xyz}" rpy="0 0 0" />
      <axis xyz="${axis}" />
      <dynamics damping="0.1" />
      <limit effort="50.0" velocity="10.0" />
    </joint>
    
  </xacro:macro>
  
  <!-- Macro pour deux roues différentielles -->
  <xacro:macro name="differential_wheels" params="parent radius length mass positions color">
    
    <!-- Roue gauche -->
    <xacro:differential_wheel 
      name="left_wheel"
      parent="${parent}"
      radius="${radius}"
      length="${length}"
      mass="${mass}"
      xyz="${positions['left'][0]} ${positions['left'][1]} ${positions['left'][2]}"
      axis="0 1 0" />
    
    <!-- Roue droite -->
    <xacro:differential_wheel 
      name="right_wheel"
      parent="${parent}"
      radius="${radius}"
      length="${length}"
      mass="${mass}"
      xyz="${positions['right'][0]} ${positions['right'][1]} ${positions['right'][2]}"
      axis="0 1 0" />
    
  </xacro:macro>
  
  <!-- Macro pour une roulette (caster) -->
  <xacro:macro name="caster_wheel" params="parent xyz radius color">
    
    <link name="caster_wheel">
      <visual>
        <geometry><sphere radius="${radius}" /></geometry>
        <material name="${color}" />
      </visual>
    </link>
    
    <joint name="caster_joint" type="fixed">
      <parent link="${parent}" />
      <child link="caster_wheel" />
      <origin xyz="${xyz}" rpy="0 0 0" />
    </joint>
    
  </xacro:macro>

</robot>
```

### 2.3 Composant `head.xacro`

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Macro pour la tête de R2D2 -->
  <xacro:macro name="r2d2_head" params="radius height mass parent xyz color">
    
    <link name="head_link">
      <visual>
        <geometry>
          <cylinder radius="${radius}" length="${height}" />
        </geometry>
        <material name="${color}" />
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${radius}" length="${height}" />
        </geometry>
      </collision>
      <inertial>
        <mass value="${mass}" />
        <origin xyz="0 0 0" />
        <inertia 
          ixx="${mass * (3*radius*radius + height*height) / 12}"
          iyy="${mass * (3*radius*radius + height*height) / 12}"
          izz="${mass * radius * radius / 2}"
          ixy="0" ixz="0" iyz="0" />
      </inertial>
    </link>
    
    <!-- Joint pour faire tourner la tête -->
    <joint name="head_joint" type="revolute">
      <parent link="${parent}" />
      <child link="head_link" />
      <origin xyz="${xyz}" rpy="0 0 0" />
      <axis xyz="0 0 1" />
      <limit lower="-3.14" upper="3.14" effort="10.0" velocity="1.0" />
      <dynamics damping="0.05" />
    </joint>
    
    <!-- Dôme sur la tête -->
    <link name="head_dome">
      <visual>
        <geometry><sphere radius="${radius*0.9}" /></geometry>
        <material name="blue_metal" />
      </visual>
    </link>
    
    <joint name="dome_joint" type="fixed">
      <parent link="head_link" />
      <child link="head_dome" />
      <origin xyz="0 0 ${height/2}" rpy="0 0 0" />
    </joint>

  </xacro:macro>

</robot>
```

---

## 3. Fichier de Lancement (`display_r2d2.launch.py`)

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    
    # ============================================
    # ARGUMENTS DE LANCEMENT
    # ============================================
    
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    use_state_publisher = DeclareLaunchArgument(
        'use_state_publisher',
        default_value='true',
        description='Use custom state publisher for trajectory'
    )
    
    color_body = DeclareLaunchArgument(
        'color_body',
        default_value='blue_metal',
        description='Color of R2D2 body'
    )
    
    # ============================================
    # CHEMINS DES FICHIERS
    # ============================================
    
    pkg_path = FindPackageShare('r2d2_robot')
    
    urdf_file = PathJoinSubstitution([
        pkg_path, 'urdf', 'r2d2.urdf.xacro'
    ])
    
    rviz_config = PathJoinSubstitution([
        pkg_path, 'rviz', 'r2d2_config.rviz'
    ])
    
    # ============================================
    # COMMANDE XACRO
    # ============================================
    
    robot_description = Command([
        'xacro ', urdf_file,
        ' color_body:=', LaunchConfiguration('color_body'),
        ' use_fixed_caster:=true'
    ])
    
    # ============================================
    # NŒUDS
    # ============================================
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'publish_frequency': 50.0  # 50 Hz
        }]
    )
    
    # Custom State Publisher (pour la trajectoire)
    state_publisher = Node(
        package='r2d2_robot',
        executable='state_publisher',
        name='state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'trajectory_type': 'circle',  # 'circle', 'square', 'figure8'
            'trajectory_radius': 1.0,
            'linear_velocity': 0.2,
            'angular_velocity': 0.5
        }]
    ) if LaunchConfiguration('use_state_publisher') else None
    
    # Joint State Publisher GUI (pour contrôle manuel)
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=LaunchConfiguration('use_state_publisher').equals('false')
    )
    
    # RVIZ2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    # ============================================
    # ASSEMBLAGE DU LAUNCH DESCRIPTION
    # ============================================
    
    nodes = [
        robot_state_publisher,
        rviz
    ]
    
    if state_publisher:
        nodes.append(state_publisher)
    
    if joint_state_publisher:
        nodes.append(joint_state_publisher)
    
    return LaunchDescription([
        use_sim_time,
        use_state_publisher,
        color_body,
        *nodes
    ])
```

---

## 4. Fichier de Publication d'État (`state_publisher.py`)

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import numpy as np

class R2D2StatePublisher(Node):
    """
    Nœud de publication d'état pour R2D2.
    Publie les états des joints et les transformations TF.
    Génère une trajectoire circulaire.
    """
    
    def __init__(self):
        super().__init__('state_publisher')
        
        # ============================================
        # PARAMÈTRES
        # ============================================
        
        self.declare_parameter('trajectory_type', 'circle')
        self.declare_parameter('trajectory_radius', 1.0)
        self.declare_parameter('linear_velocity', 0.2)
        self.declare_parameter('angular_velocity', 0.5)
        self.declare_parameter('publish_rate', 50.0)
        
        self.trajectory_type = self.get_parameter('trajectory_type').value
        self.trajectory_radius = self.get_parameter('trajectory_radius').value
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.angular_velocity = self.get_parameter('angular_velocity').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # ============================================
        # VARIABLES D'ÉTAT
        # ============================================
        
        self.joint_state = JointState()
        self.joint_state.name = [
            'left_wheel_joint',
            'right_wheel_joint',
            'head_joint'
        ]
        
        # Position initiale du robot
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0  # Orientation en radians
        
        # Angles des joints
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        self.head_angle = 0.0
        
        # Temps
        self.last_time = self.get_clock().now()
        self.start_time = self.last_time
        
        # ============================================
        # PUBLISHERS/SUBSCRIBERS
        # ============================================
        
        # Publisher pour les états des joints
        self.joint_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        # Transform Broadcaster pour TF
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # ============================================
        # TIMER
        # ============================================
        
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.update_callback)
        
        self.get_logger().info(
            f'R2D2 State Publisher démarré avec trajectoire: {self.trajectory_type}'
        )
    
    def update_callback(self):
        """Callback principal du timer."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        # Mettre à jour le temps
        self.last_time = current_time
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
        
        # ============================================
        # CALCUL DE LA TRAJECTOIRE
        # ============================================
        
        # Vitesses linéaire et angulaire basées sur la trajectoire
        v, omega = self.calculate_trajectory(elapsed_time)
        
        # Mettre à jour la position du robot
        self.update_robot_position(v, omega, dt)
        
        # Mettre à jour les angles des roues (cinématique différentielle)
        wheelbase = 0.3  # Distance entre les roues en mètres
        wheel_radius = 0.08
        
        # Vitesses des roues (modèle différentiel)
        left_wheel_velocity = (v - omega * wheelbase / 2) / wheel_radius
        right_wheel_velocity = (v + omega * wheelbase / 2) / wheel_radius
        
        self.left_wheel_angle += left_wheel_velocity * dt
        self.right_wheel_angle += right_wheel_velocity * dt
        
        # Mouvement de la tête (balayage)
        self.head_angle = math.sin(elapsed_time * 0.5) * 1.5
        
        # ============================================
        # PUBLICATION DES MESSAGES
        # ============================================
        
        # Publication des états des joints
        self.publish_joint_state(current_time)
        
        # Publication de la transformation TF
        self.publish_transform(current_time)
        
        # Log occasionnel
        if int(elapsed_time * 10) % 10 == 0:
            self.get_logger().info(
                f'Position: ({self.robot_x:.2f}, {self.robot_y:.2f}), '
                f'Angle: {math.degrees(self.robot_theta):.1f}°'
            )
    
    def calculate_trajectory(self, t):
        """Calcule la vitesse linéaire et angulaire basée sur la trajectoire."""
        
        if self.trajectory_type == 'circle':
            # Mouvement circulaire
            v = self.linear_velocity
            omega = self.angular_velocity
        
        elif self.trajectory_type == 'square':
            # Mouvement carré: 4 côtés droits, 4 virages à 90°
            cycle_time = 16.0  # Temps pour compléter un carré
            side_time = cycle_time / 4
            
            segment = int(t / side_time) % 4
            segment_time = t % side_time
            
            if segment_time < side_time - 1.0:
                # Aller tout droit
                v = self.linear_velocity
                omega = 0.0
            else:
                # Tourner à 90°
                v = 0.0
                omega = math.pi / 2  # 90°/sec
        
        elif self.trajectory_type == 'figure8':
            # Figure 8 (lemniscate)
            scale = 0.5
            v = self.linear_velocity * (1 + 0.3 * math.sin(t * 0.5))
            omega = self.angular_velocity * math.sin(t * 0.25)
        
        else:
            # Par défaut: arrêt
            v = 0.0
            omega = 0.0
        
        return v, omega
    
    def update_robot_position(self, v, omega, dt):
        """Met à jour la position du robot en fonction des vitesses."""
        # Modèle de déplacement différentiel
        if abs(omega) < 0.001:
            # Mouvement rectiligne
            self.robot_x += v * math.cos(self.robot_theta) * dt
            self.robot_y += v * math.sin(self.robot_theta) * dt
        else:
            # Mouvement curviligne
            radius = v / omega
            icc_x = self.robot_x - radius * math.sin(self.robot_theta)
            icc_y = self.robot_y + radius * math.cos(self.robot_theta)
            
            self.robot_theta += omega * dt
            
            self.robot_x = icc_x + radius * math.sin(self.robot_theta)
            self.robot_y = icc_y - radius * math.cos(self.robot_theta)
        
        # Normaliser l'angle entre -pi et pi
        self.robot_theta = math.atan2(math.sin(self.robot_theta), 
                                      math.cos(self.robot_theta))
    
    def publish_joint_state(self, time):
        """Publie l'état des joints."""
        self.joint_state.header.stamp = time.to_msg()
        self.joint_state.position = [
            self.left_wheel_angle,
            self.right_wheel_angle,
            self.head_angle
        ]
        
        # Vitesses (optionnelles)
        self.joint_state.velocity = [0.0, 0.0, 0.0]
        
        self.joint_pub.publish(self.joint_state)
    
    def publish_transform(self, time):
        """Publie la transformation TF du robot."""
        t = TransformStamped()
        
        t.header.stamp = time.to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        
        # Position
        t.transform.translation.x = self.robot_x
        t.transform.translation.y = self.robot_y
        t.transform.translation.z = 0.0
        
        # Orientation (quaternion depuis l'angle yaw)
        cy = math.cos(self.robot_theta * 0.5)
        sy = math.sin(self.robot_theta * 0.5)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = R2D2StatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 5. Configuration RVIZ (`r2d2_config.rviz`)

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
  - Class: rviz_common/Time
    Name: Time

Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Robot Description: robot_description
      Visual Enabled: true
      Collision Enabled: false
      Alpha: 1
      Update Interval: 0
    
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Frame Timeout: 15
      Marker Scale: 0.5
      Show Names: true
      Show Arrows: true
      Show Axes: true
    
    - Class: rviz_default_plugins/Axes
      Name: Axes
      Enabled: true
      Length: 0.3
      Radius: 0.01
      Reference Frame: world
    
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Cell Size: 0.5
      Color: 128; 128; 128
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: world
      Style: Lines
    
    - Class: rviz_default_plugins/Path
      Name: TrajectoryPath
      Enabled: true
      Color: 255; 0; 0
      Alpha: 0.8
      Buffer Length: 100
      Line Style: Lines
      Line Width: 0.02
      Offset: 0; 0; 0.01
      Pose Style: None
      Reference Frame: world

  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: world
    Frame Rate: 30

  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Name: Current View
      Distance: 3.0
      Focal Point:
        X: 0
        Y: 0
        Z: 0.3
      Yaw: 45
      Pitch: 30
    Saved: ~
```

---

## 6. Fichier `setup.py`

```python
from setuptools import setup
import os
from glob import glob

package_name = 'r2d2_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), 
         glob('urdf/*.urdf.xacro')),
        (os.path.join('share', package_name, 'urdf', 'common'), 
         glob('urdf/common/*.xacro')),
        (os.path.join('share', package_name, 'urdf', 'components'), 
         glob('urdf/components/*.xacro')),
        (os.path.join('share', package_name, 'rviz'), 
         glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), 
         glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='votre_nom',
    maintainer_email='votre_email@example.com',
    description='Package R2D2 avec simulation de trajectoire dans RVIZ',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = r2d2_robot.state_publisher:main',
            'trajectory_publisher = r2d2_robot.trajectory_publisher:main',
        ],
    },
)
```

---

## 7. Fichier `package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>r2d2_robot</name>
  <version>0.0.0</version>
  <description>Package R2D2 avec simulation de trajectoire dans RVIZ</description>
  <maintainer email="votre_email@example.com">Votre Nom</maintainer>
  <license>Apache License 2.0</license>

  <!-- Dépendances exécution -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_msgs</depend>
  
  <!-- Dépendances ROS 2 -->
  <depend>robot_state_publisher</depend>
  <depend>joint_state_publisher</depend>
  <depend>rviz2</depend>
  <depend>xacro</depend>
  <depend>launch</depend>
  <depend>launch_ros</depend>
  
  <!-- Dépendances construction -->
  <buildtool_depend>ament_python</buildtool_depend>
  
  <!-- Dépendances test -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

---

## 8. Construction et Exécution

### 8.1 Construire le package
```bash
# Construire le package
cd ~/ros2_ws
colcon build --packages-select r2d2_robot

# Sourcer l'installation
source install/setup.bash
```

### 8.2 Lancer la simulation
```bash
# Lancer R2D2 avec trajectoire circulaire
ros2 launch r2d2_robot display_r2d2.launch.py

# Lancer avec trajectoire personnalisée
ros2 launch r2d2_robot display_r2d2.launch.py \
  trajectory_type:="figure8" \
  linear_velocity:=0.3 \
  angular_velocity:=0.4
```

### 8.3 Vérifier les topics
```bash
# Voir les topics actifs
ros2 topic list

# Voir les états des joints
ros2 topic echo /joint_states

# Voir les transformations TF
ros2 run tf2_tools view_frames
```

### 8.4 Contrôler manuellement (sans trajectoire automatique)
```bash
# Lancer sans le state publisher
ros2 launch r2d2_robot display_r2d2.launch.py use_state_publisher:=false

# Utiliser l'interface graphique pour contrôler les joints
# Une fenêtre joint_state_publisher_gui apparaîtra
```

---

## 9. Résumé RVIZ et Visualisation

### 9.1 Éléments visibles dans RVIZ
1. **RobotModel** : Modèle 3D de R2D2 avec couleurs définies
2. **TF Frames** : Repères de transformation (world, base_link, head_link, etc.)
3. **Grid** : Grille de référence au sol
4. **Axes** : Axes de coordonnées
5. **TrajectoryPath** : Chemin suivi par le robot (ligne rouge)

### 9.2 Interactions possibles
- **Orbit View** : Cliquer-droit + déplacer pour tourner autour du robot
- **Pan View** : Cliquer-molette + déplacer pour se déplacer
- **Zoom** : Molette de la souris
- **Reset View** : Bouton "Reset" dans RVIZ

### 9.3 Dépannage RVIZ
```bash
# Si RVIZ ne montre rien :
# 1. Vérifier que Fixed Frame = "world"
# 2. Cliquer sur "Reset" dans RVIZ
# 3. Vérifier que robot_state_publisher tourne :
ros2 topic echo /robot_description --once

# Si les transformations sont incorrectes :
ros2 run tf2_ros tf2_monitor
```

---

## 10. Extensions Possibles

### 10.1 Ajouter un contrôleur de trajectoire avancé
```python
# scripts/trajectory_publisher.py
class TrajectoryController:
    """
    Contrôleur de trajectoire avancé avec waypoints.
    """
    def __init__(self):
        self.waypoints = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (1.0, 1.0, math.pi/2),
            (0.0, 1.0, math.pi),
            (0.0, 0.0, -math.pi/2)
        ]
        self.current_waypoint = 0
```

### 10.2 Intégration avec Gazebo
```python
# launch/gazebo_simulation.launch.py
from launch.actions import ExecuteProcess

gazebo = ExecuteProcess(
    cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
    output='screen'
)
```

### 10.3 Ajouter des capteurs simulés
```xml
<!-- Dans le fichier URDF -->
<xacro:macro name="laser_sensor" params="name parent">
  <link name="${name}">
    <visual>
      <geometry><cylinder radius="0.02" length="0.05" /></geometry>
    </visual>
  </link>
  <joint name="${name}_joint" type="fixed">
    <parent link="${parent}" />
    <child link="${name}" />
    <origin xyz="0.15 0 0.2" rpy="0 0 0" />
  </joint>
</xacro:macro>
```

---

## 11. Résumé des Commandes

```bash
# Construction
cd ~/ros2_ws
colcon build --packages-select r2d2_robot
source install/setup.bash

# Lancement
ros2 launch r2d2_robot display_r2d2.launch.py

# Vérification
ros2 topic list
ros2 topic echo /joint_states
ros2 run tf2_tools view_frames

# Trajectoires disponibles
ros2 launch r2d2_robot display_r2d2.launch.py trajectory_type:=circle
ros2 launch r2d2_robot display_r2d2.launch.py trajectory_type:=square
ros2 launch r2d2_robot display_r2d2.launch.py trajectory_type:=figure8
```

Votre robot R2D2 devrait maintenant se déplacer en cercle (ou selon la trajectoire choisie) dans RVIZ, avec la tête qui tourne et les roues qui tournent de manière cohérente !