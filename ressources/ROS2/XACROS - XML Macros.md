

# Xacro : Simplifier les Fichiers URDF

## 1. Introduction à Xacro

**Xacro** (XML Macros) est un langage de macro XML qui vous permet d'écrire des fichiers URDF **plus courts, plus lisibles et plus maintenables**.

**Pourquoi utiliser Xacro ?**
- **Éviter la répétition** : Définir une fois, utiliser partout
- **Gestion des constantes** : Modifier une valeur à un seul endroit
- **Réutilisabilité** : Créer des composants réutilisables (roues, bras, etc.)
- **Inclusion de fichiers** : Séparer votre robot en modules logiques
- **Calculs mathématiques** : Effectuer des calculs simples dans le XML

**Extension des fichiers** : `.urdf.xacro` (recommandé) ou `.xacro`

---

## 2. Commandes Xacro

### 2.1 Générer un URDF à partir d'un Xacro

```bash
# Méthode 1 : Utiliser ros2 run (recommandée)
ros2 run xacro xacro mon_robot.urdf.xacro > mon_robot.urdf

# Méthode 2 : Avec arguments
ros2 run xacro xacro mon_robot.urdf.xacro \
  prefix:="robot1_" \
  color:="red" \
  > mon_robot.urdf

# Méthode 3 : En ligne dans un lanceur (sans fichier intermédiaire)
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(ros2 run xacro xacro mon_robot.urdf.xacro)"
```

### 2.2 Vérifier la sortie Xacro

```bash
# Vérifier le XML généré
ros2 run xacro xacro mon_robot.urdf.xacro | check_urdf

# Afficher le résultat XML
ros2 run xacro xacro mon_robot.urdf.xacro | less

# Compter le nombre de lignes (pour voir la réduction)
wc -l mon_robot.urdf.xacro    # Xacro compact
wc -l mon_robot.urdf          # URDF expansé (beaucoup plus long)
```

---

## 3. Robot State Publisher avec Xacro

### 3.1 Dans un lanceur ROS 2

```python
# launch/display_xacro.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    pkg_path = get_package_share_directory('mon_robot_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'mon_robot.urdf.xacro')
    
    # Utiliser Command pour exécuter xacro
    robot_description = Command(['xacro ', xacro_file])
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(
                robot_description, value_type=str
            ),
            'use_sim_time': False
        }]
    )
    
    # ... autres nœuds (rviz, joint_state_publisher)
```

### 3.2 En ligne de commande

```bash
# Lancer robot_state_publisher directement avec xacro
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="$(ros2 run xacro xacro $(ros2 pkg prefix mon_robot_description)/share/mon_robot_description/urdf/mon_robot.urdf.xacro)"
```

---

## 4. Lanceur URDF avec Xacro

### 4.1 Lanceur complet avec paramètres

```python
# launch/robot.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Arguments de lancement
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='mon_robot',
        description='Name of the robot'
    )
    
    color = DeclareLaunchArgument(
        'color',
        default_value='blue',
        description='Color of the robot (blue/red/green)'
    )
    
    # Chemin vers le fichier xacro
    pkg_path = get_package_share_directory('mon_robot_description')
    xacro_path = os.path.join(pkg_path, 'urdf', 'mon_robot.urdf.xacro')
    
    # Commande xacro avec paramètres
    robot_description = Command([
        'xacro ', xacro_path,
        ' robot_name:=', LaunchConfiguration('robot_name'),
        ' color:=', LaunchConfiguration('color'),
        ' use_sim_time:=', LaunchConfiguration('use_sim_time')
    ])
    
    # Nœud robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    return LaunchDescription([
        use_sim_time,
        robot_name,
        color,
        robot_state_publisher,
        # ... autres nœuds
    ])
```

**Utilisation :**
```bash
# Lancement avec paramètres par défaut
ros2 launch mon_robot_description robot.launch.py

# Lancement avec paramètres personnalisés
ros2 launch mon_robot_description robot.launch.py \
  robot_name:="robot_rouge" \
  color:="red" \
  use_sim_time:="true"
```

---

## 5. Propriétés Xacro (Xacro Property)

### 5.1 Déclaration et utilisation de propriétés

```xml
<?xml version="1.0"?>
<robot name="mon_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ============================================ -->
  <!-- 1. PROPRIÉTÉS SIMPLES (constantes) -->
  <!-- ============================================ -->
  
  <!-- Déclaration -->
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_length" value="0.05" />
  <xacro:property name="wheel_mass" value="0.5" />
  
  <!-- Utilisation avec ${} -->
  <link name="wheel">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="${wheel_length}" />
      </geometry>
    </visual>
    <inertial>
      <mass value="${wheel_mass}" />
    </inertial>
  </link>
  
  <!-- ============================================ -->
  <!-- 2. PROPRIÉTÉS DÉRIVÉES (calculs) -->
  <!-- ============================================ -->
  
  <xacro:property name="chassis_length" value="0.4" />
  <xacro:property name="chassis_width" value="0.3" />
  <xacro:property name="chassis_height" value="0.1" />
  
  <!-- Calculs mathématiques -->
  <xacro:property name="chassis_volume" value="${chassis_length * chassis_width * chassis_height}" />
  <xacro:property name="chassis_mass" value="${chassis_volume * 1000}" /> <!-- densité 1000 kg/m³ -->
  
  <!-- Expressions mathématiques disponibles :
       +  : addition
       -  : soustraction
       *  : multiplication
       /  : division
       ** : puissance
       %  : modulo
       sin, cos, tan, asin, acos, atan, sinh, cosh, tanh
       exp, log, log10, sqrt, fabs, ceil, floor
       pi, e (constantes)
  -->
  
  <!-- ============================================ -->
  <!-- 3. PROPRIÉTÉS CONDITIONNELLES -->
  <!-- ============================================ -->
  
  <!-- Basé sur un argument -->
  <xacro:arg name="has_camera" default="true" />
  
  <!-- Propriété conditionnelle -->
  <xacro:property name="camera_mass">
    <xacro:if value="$(arg has_camera)">0.2</xacro:if>
    <xacro:else>0.0</xacro:else>
  </xacro:property>
  
  <!-- ============================================ -->
  <!-- 4. PROPRIÉTÉS COMPLEXES (tableaux, chaînes) -->
  <!-- ============================================ -->
  
  <!-- Tableau de positions des roues -->
  <xacro:property name="wheel_positions" value="${[0.15, -0.15, 0.12, -0.12]}" />
  
  <!-- Chaînes de caractères -->
  <xacro:property name="robot_prefix" value="robot_" />
  <xacro:property name="side_names" value="${['left', 'right', 'front', 'rear']}" />
  
  <!-- Utilisation avec index -->
  <link name="${robot_prefix}${side_names[0]}_wheel">
    <!-- Roue gauche -->
  </link>
  
</robot>
```

---

## 6. Macros Xacro (Xacro Macro)

### 6.1 Macro simple sans paramètres

```xml
<!-- Macro pour un lien de roue standard -->
<xacro:macro name="wheel_link">
  <link name="wheel">
    <visual>
      <geometry><cylinder radius="0.1" length="0.05" /></geometry>
      <material name="black" />
    </visual>
    <inertial>
      <mass value="0.5" />
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.0005" />
    </inertial>
  </link>
</xacro:macro>

<!-- Utilisation -->
<xacro:wheel_link />
```

### 6.2 Macro avec paramètres

```xml
<!-- Macro paramétrée pour une roue -->
<xacro:macro name="wheel" params="name radius length mass color">
  
  <link name="${name}_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="${pi/2} 0 0" />
      <geometry>
        <cylinder radius="${radius}" length="${length}" />
      </geometry>
      <material name="${color}" />
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="${pi/2} 0 0" />
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
  
</xacro:macro>

<!-- Utilisation avec paramètres -->
<xacro:wheel 
  name="front_left" 
  radius="0.1" 
  length="0.05" 
  mass="0.5" 
  color="black" />
```

### 6.3 Macro avec paramètres par défaut

```xml
<!-- Macro avec valeurs par défaut -->
<xacro:macro name="wheel_default" 
             params="name 
                     radius:=0.1 
                     length:=0.05 
                     mass:=0.5 
                     color:=black">
  
  <!-- Même contenu que l'exemple précédent -->
  
</xacro:macro>

<!-- Utilisations diverses -->
<xacro:wheel_default name="left" />  <!-- Utilise tous les défauts -->
<xacro:wheel_default name="right" radius="0.12" mass="0.6" />  <!-- Surcharge partielle -->
```

### 6.4 Macro pour un joint de roue

```xml
<!-- Macro pour un joint de roue -->
<xacro:macro name="wheel_joint" params="parent child xyz axis">
  
  <joint name="${child}_joint" type="continuous">
    <parent link="${parent}" />
    <child link="${child}" />
    <origin xyz="${xyz}" rpy="0 0 0" />
    <axis xyz="${axis}" />
    <dynamics damping="0.1" />
    <limit effort="100" velocity="100" />
  </joint>
  
</xacro:macro>

<!-- Utilisation -->
<xacro:wheel_joint 
  parent="chassis" 
  child="front_left_wheel" 
  xyz="0.15 0.15 -0.025" 
  axis="0 1 0" />
```

---

## 7. URDF avec Xacros dans un Seul Fichier

### 7.1 Exemple complet d'un robot mobile

```xml
<?xml version="1.0"?>
<robot name="mobile_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ============================================ -->
  <!-- 1. PROPRIÉTÉS GLOBALES -->
  <!-- ============================================ -->
  
  <xacro:property name="chassis_length" value="0.4" />
  <xacro:property name="chassis_width" value="0.3" />
  <xacro:property name="chassis_height" value="0.1" />
  <xacro:property name="chassis_mass" value="2.0" />
  
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_length" value="0.05" />
  <xacro:property name="wheel_mass" value="0.5" />
  
  <!-- Positions des roues (x, y, z) -->
  <xacro:property name="wheel_positions" value="${{
    'front_left': [chassis_length/2, chassis_width/2, -wheel_radius],
    'front_right': [chassis_length/2, -chassis_width/2, -wheel_radius],
    'rear_left': [-chassis_length/2, chassis_width/2, -wheel_radius],
    'rear_right': [-chassis_length/2, -chassis_width/2, -wheel_radius]
  }}" />
  
  <!-- ============================================ -->
  <!-- 2. MACROS -->
  <!-- ============================================ -->
  
  <!-- Macro pour le chassis -->
  <xacro:macro name="chassis" params="name color">
    <link name="${name}">
      <visual>
        <geometry>
          <box size="${chassis_length} ${chassis_width} ${chassis_height}" />
        </geometry>
        <material name="${color}" />
      </visual>
      <collision>
        <geometry>
          <box size="${chassis_length} ${chassis_width} ${chassis_height}" />
        </geometry>
      </collision>
      <inertial>
        <mass value="${chassis_mass}" />
        <inertia 
          ixx="${chassis_mass/12 * (chassis_width*chassis_width + chassis_height*chassis_height)}"
          iyy="${chassis_mass/12 * (chassis_length*chassis_length + chassis_height*chassis_height)}"
          izz="${chassis_mass/12 * (chassis_length*chassis_length + chassis_width*chassis_width)}"
          ixy="0" ixz="0" iyz="0" />
      </inertial>
    </link>
  </xacro:macro>
  
  <!-- Macro pour une roue -->
  <xacro:macro name="wheel" params="name">
    <link name="${name}_wheel">
      <visual>
        <origin rpy="${pi/2} 0 0" />
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_length}" />
        </geometry>
        <material name="black" />
      </visual>
      <inertial>
        <mass value="${wheel_mass}" />
        <inertia 
          ixx="${wheel_mass * (3*wheel_radius*wheel_radius + wheel_length*wheel_length) / 12}"
          iyy="${wheel_mass * (3*wheel_radius*wheel_radius + wheel_length*wheel_length) / 12}"
          izz="${wheel_mass * wheel_radius * wheel_radius / 2}"
          ixy="0" ixz="0" iyz="0" />
      </inertial>
    </link>
  </xacro:macro>
  
  <!-- Macro pour un joint de roue -->
  <xacro:macro name="wheel_joint" params="name">
    <joint name="${name}_joint" type="continuous">
      <parent link="chassis" />
      <child link="${name}_wheel" />
      <origin xyz="${wheel_positions[name][0]} ${wheel_positions[name][1]} ${wheel_positions[name][2]}" />
      <axis xyz="0 1 0" />
      <limit effort="100" velocity="100" />
    </joint>
  </xacro:macro>
  
  <!-- ============================================ -->
  <!-- 3. INSTANCIATION DU MODÈLE -->
  <!-- ============================================ -->
  
  <!-- Chassis -->
  <xacro:chassis name="chassis" color="blue" />
  
  <!-- Roues -->
  <xacro:wheel name="front_left" />
  <xacro:wheel name="front_right" />
  <xacro:wheel name="rear_left" />
  <xacro:wheel name="rear_right" />
  
  <!-- Joints -->
  <xacro:wheel_joint name="front_left" />
  <xacro:wheel_joint name="front_right" />
  <xacro:wheel_joint name="rear_left" />
  <xacro:wheel_joint name="rear_right" />
  
  <!-- ============================================ -->
  <!-- 4. ÉLÉMENTS SUPPLÉMENTAIRES CONDITIONNELS -->
  <!-- ============================================ -->
  
  <xacro:arg name="has_camera" default="true" />
  
  <xacro:if value="$(arg has_camera)">
    <!-- Macro pour une caméra -->
    <xacro:macro name="camera" params="name xyz rpy">
      <link name="${name}_camera">
        <visual>
          <geometry><box size="0.05 0.05 0.05" /></geometry>
          <material name="white" />
        </visual>
      </link>
      <joint name="${name}_camera_joint" type="fixed">
        <parent link="chassis" />
        <child link="${name}_camera" />
        <origin xyz="${xyz}" rpy="${rpy}" />
      </joint>
    </xacro:macro>
    
    <xacro:camera name="front" xyz="0.2 0 0.1" rpy="0 0 0" />
  </xacro:if>

</robot>
```

---

## 8. URDF avec Xacros dans des Fichiers Séparés

### 8.1 Structure de projet recommandée

```
mon_robot_description/
├── urdf/
│   ├── mobile_robot.urdf.xacro          # Fichier principal
│   ├── common/                          # Définitions communes
│   │   ├── materials.xacro              # Matériaux
│   │   ├── constants.xacro              # Constantes
│   │   └── utils.xacro                  # Utilitaires
│   ├── components/                      # Composants réutilisables
│   │   ├── chassis.xacro                # Châssis
│   │   ├── wheel.xacro                  # Roues
│   │   ├── sensor_camera.xacro          # Capteurs
│   │   └── arm.xacro                    # Bras robotique
│   └── robots/                          # Robots complets
│       ├── diff_drive_robot.xacro       # Robot à différentiel
│       └── arm_robot.xacro              # Robot à bras
└── launch/
    └── display_robot.launch.py
```

### 8.2 Fichiers de base

**`urdf/common/materials.xacro` :**
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Couleurs de base -->
  <material name="blue">
    <color rgba="0.1 0.1 0.8 1.0" />
  </material>
  
  <material name="red">
    <color rgba="0.8 0.1 0.1 1.0" />
  </material>
  
  <material name="green">
    <color rgba="0.1 0.8 0.1 1.0" />
  </material>
  
  <material name="black">
    <color rgba="0.1 0.1 0.1 1.0" />
  </material>
  
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0" />
  </material>
  
  <!-- Métaux -->
  <material name="steel">
    <color rgba="0.7 0.7 0.7 1.0" />
  </material>
  
  <material name="aluminum">
    <color rgba="0.9 0.9 0.9 1.0" />
  </material>

</robot>
```

**`urdf/common/constants.xacro` :**
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Constantes mathématiques -->
  <xacro:property name="PI" value="3.1415926535897931" />
  <xacro:property name="DEG_TO_RAD" value="${PI/180}" />
  <xacro:property name="RAD_TO_DEG" value="${180/PI}" />
  
  <!-- Densités (kg/m³) -->
  <xacro:property name="DENSITY_STEEL" value="7850" />
  <xacro:property name="DENSITY_ALUMINUM" value="2700" />
  <xacro:property name="DENSITY_PLASTIC" value="1400" />
  
  <!-- Couleurs standard -->
  <xacro:property name="COLOR_CHASSIS" value="blue" />
  <xacro:property name="COLOR_WHEEL" value="black" />
  <xacro:property name="COLOR_SENSOR" value="white" />

</robot>
```

**`urdf/components/wheel.xacro` :**
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Inclure les dépendances -->
  <xacro:include filename="$(find mon_robot_description)/urdf/common/materials.xacro" />
  <xacro:include filename="$(find mon_robot_description)/urdf/common/constants.xacro" />
  
  <!-- Macro de roue complète -->
  <xacro:macro name="wheel" params="name radius length mass 
                                    color:=${COLOR_WHEEL}
                                    parent:=chassis
                                    xyz:='0 0 0'
                                    axis:='0 1 0'">
    
    <!-- Lien de la roue -->
    <link name="${name}_wheel">
      <visual>
        <origin rpy="${PI/2} 0 0" />
        <geometry>
          <cylinder radius="${radius}" length="${length}" />
        </geometry>
        <material name="${color}" />
      </visual>
      
      <collision>
        <origin rpy="${PI/2} 0 0" />
        <geometry>
          <cylinder radius="${radius}" length="${length}" />
        </geometry>
      </collision>
      
      <inertial>
        <mass value="${mass}" />
        <inertia 
          ixx="${mass * (3*radius*radius + length*length) / 12}"
          iyy="${mass * (3*radius*radius + length*length) / 12}"
          izz="${mass * radius * radius / 2}"
          ixy="0" ixz="0" iyz="0" />
      </inertial>
    </link>
    
    <!-- Joint de la roue -->
    <joint name="${name}_joint" type="continuous">
      <parent link="${parent}" />
      <child link="${name}_wheel" />
      <origin xyz="${xyz}" />
      <axis xyz="${axis}" />
      <dynamics damping="0.1" />
      <limit effort="100" velocity="100" />
    </joint>
    
  </xacro:macro>
  
  <!-- Macro pour 4 roues de voiture -->
  <xacro:macro name="four_wheels" params="wheel_radius wheel_length wheel_mass
                                          wheelbase track">
    
    <!-- Calcul des positions -->
    <xacro:property name="front_x" value="${wheelbase/2}" />
    <xacro:property name="rear_x" value="${-wheelbase/2}" />
    <xacro:property name="left_y" value="${track/2}" />
    <xacro:property name="right_y" value="${-track/2}" />
    <xacro:property name="wheel_z" value="${-wheel_radius}" />
    
    <!-- Roues avant -->
    <xacro:wheel name="front_left" 
                 radius="${wheel_radius}" 
                 length="${wheel_length}" 
                 mass="${wheel_mass}"
                 xyz="${front_x} ${left_y} ${wheel_z}" />
    
    <xacro:wheel name="front_right" 
                 radius="${wheel_radius}" 
                 length="${wheel_length}" 
                 mass="${wheel_mass}"
                 xyz="${front_x} ${right_y} ${wheel_z}" />
    
    <!-- Roues arrière -->
    <xacro:wheel name="rear_left" 
                 radius="${wheel_radius}" 
                 length="${wheel_length}" 
                 mass="${wheel_mass}"
                 xyz="${rear_x} ${left_y} ${wheel_z}" />
    
    <xacro:wheel name="rear_right" 
                 radius="${wheel_radius}" 
                 length="${wheel_length}" 
                 mass="${wheel_mass}"
                 xyz="${rear_x} ${right_y} ${wheel_z}" />
    
  </xacro:macro>

</robot>
```

### 8.3 Fichier principal

**`urdf/mobile_robot.urdf.xacro` :**
```xml
<?xml version="1.0"?>
<robot name="mobile_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ============================================ -->
  <!-- INCLUSIONS -->
  <!-- ============================================ -->
  
  <!-- Inclure les fichiers communs -->
  <xacro:include filename="$(find mon_robot_description)/urdf/common/materials.xacro" />
  <xacro:include filename="$(find mon_robot_description)/urdf/common/constants.xacro" />
  
  <!-- Inclure les composants -->
  <xacro:include filename="$(find mon_robot_description)/urdf/components/chassis.xacro" />
  <xacro:include filename="$(find mon_robot_description)/urdf/components/wheel.xacro" />
  <xacro:include filename="$(find mon_robot_description)/urdf/components/sensor_camera.xacro" />
  
  <!-- ============================================ -->
  <!-- ARGUMENTS DU FICHIER -->
  <!-- ============================================ -->
  
  <xacro:arg name="robot_name" default="mobile_robot" />
  <xacro:arg name="color" default="blue" />
  <xacro:arg name="has_camera" default="true" />
  <xacro:arg name="has_lidar" default="false" />
  
  <!-- ============================================ -->
  <!-- PROPRIÉTÉS SPÉCIFIQUES AU ROBOT -->
  <!-- ============================================ -->
  
  <!-- Dimensions -->
  <xacro:property name="wheelbase" value="0.3" />    <!-- Distance entre essieux -->
  <xacro:property name="track" value="0.25" />       <!-- Largeur entre roues -->
  <xacro:property name="chassis_height" value="0.15" />
  
  <xacro:property name="wheel_radius" value="0.075" />
  <xacro:property name="wheel_length" value="0.04" />
  <xacro:property name="wheel_mass" value="0.4" />
  
  <!-- ============================================ -->
  <!-- INSTANCIATION DU ROBOT -->
  <!-- ============================================ -->
  
  <!-- Châssis -->
  <xacro:chassis 
    name="chassis"
    length="${wheelbase + wheel_radius*2}"
    width="${track + wheel_length}"
    height="${chassis_height}"
    mass="2.5"
    color="$(arg color)" />
  
  <!-- 4 roues -->
  <xacro:four_wheels 
    wheel_radius="${wheel_radius}"
    wheel_length="${wheel_length}"
    wheel_mass="${wheel_mass}"
    wheelbase="${wheelbase}"
    track="${track}" />
  
  <!-- Capteurs conditionnels -->
  <xacro:if value="$(arg has_camera)">
    <xacro:camera 
      name="front_camera"
      parent="chassis"
      xyz="0.15 0 0.1"
      rpy="0 0 0" />
  </xacro:if>
  
  <xacro:if value="$(arg has_lidar)">
    <xacro:lidar 
      name="top_lidar"
      parent="chassis"
      xyz="0 0 0.2"
      rpy="0 0 0" />
  </xacro:if>

</robot>
```

### 8.4 Comment inclure des fichiers

```xml
<!-- Syntaxes d'inclusion valides -->

<!-- 1. Chemin absolu (peu portable) -->
<xacro:include filename="/home/user/ros2_ws/src/mon_robot_description/urdf/components/wheel.xacro" />

<!-- 2. Avec $(find package) (ROS 1 style, fonctionne en ROS 2 avec xacro) -->
<xacro:include filename="$(find mon_robot_description)/urdf/components/wheel.xacro" />

<!-- 3. Chemin relatif depuis le package (recommandé) -->
<xacro:include filename="$(dirname)/../common/materials.xacro" />

<!-- 4. Avec substitution de variables -->
<xacro:property name="components_dir" value="$(find mon_robot_description)/urdf/components" />
<xacro:include filename="${components_dir}/wheel.xacro" />
```

---

## 9. Compilation et Utilisation

### 9.1 Construire le package

```bash
# Assurez-vous que votre package.xml a la dépendance xacro
<depend>xacro</depend>

# Construire
cd ~/ros2_ws
colcon build --packages-select mon_robot_description
source install/setup.bash
```

### 9.2 Visualiser le robot

```bash
# Méthode 1 : Avec lanceur
ros2 launch mon_robot_description display_robot.launch.py \
  robot_name:="mon_robot" \
  color:="red" \
  has_camera:=true

# Méthode 2 : En ligne de commande
ros2 run xacro xacro \
  src/mon_robot_description/urdf/mobile_robot.urdf.xacro \
  robot_name:=test color:=green has_camera:=false \
  > test.urdf

# Vérifier
check_urdf test.urdf

# Visualiser
ros2 launch urdf_tutorial display.launch.py model:=test.urdf
```

### 9.3 Tester avec des paramètres différents

```bash
# Générer plusieurs versions
ros2 run xacro xacro mobile_robot.urdf.xacro \
  color:="blue" has_camera:=true > robot_avec_camera.urdf

ros2 run xacro xacro mobile_robot.urdf.xacro \
  color:="red" has_camera:=false has_lidar:=true > robot_avec_lidar.urdf

# Comparer les fichiers
diff robot_avec_camera.urdf robot_avec_lidar.urdf | head -20
```

---

## 10. Bonnes Pratiques et Conseils

### 10.1 Organisation des fichiers
- **Séparer par fonctionnalité** : Un fichier par composant
- **Hiérarchie claire** : common/, components/, robots/
- **Noms descriptifs** : `differential_wheel.xacro`, `camera_mount.xacro`
- **Documentation** : Commenter les macros complexes

### 10.2 Paramétrage
- **Utiliser des arguments** pour la personnalisation
- **Valeurs par défaut** sensibles
- **Validation** : Ajouter des vérifications si possible
- **Unités cohérentes** : Toujours en mètres, kilogrammes, radians

### 10.3 Performance
- **Éviter les inclusions circulaires**
- **Minimiser les calculs complexes** dans les macros
- **Utiliser des propriétés** pour les valeurs réutilisées
- **Préférer les chemins relatifs** pour la portabilité

### 10.4 Débogage
```bash
# Activer le mode verbeux de xacro
ros2 run xacro xacro --inorder mobile_robot.urdf.xacro

# Voir l'arbre des inclusions
ros2 run xacro xacro --includes mobile_robot.urdf.xacro

# Générer avec des commentaires de débogage
ros2 run xacro xacro --check-order mobile_robot.urdf.xacro
```

---

## 11. Exemple Avancé : Bras Robotique Modulaire

```xml
<!-- urdf/components/arm.xacro -->
<xacro:macro name="robot_arm" params="name 
                                      num_joints:=3
                                      link_length:=0.2
                                      joint_limits:='-1.57 1.57'">
  
  <!-- Propriétés calculées -->
  <xacro:property name="link_mass" value="0.5" />
  <xacro:property name="joint_mass" value="0.2" />
  
  <!-- Premier lien (base) -->
  <link name="${name}_base">
    <visual>
      <geometry><cylinder radius="0.05" length="0.1" /></geometry>
      <material name="steel" />
    </visual>
  </link>
  
  <!-- Joints et liens supplémentaires -->
  <xacro:property name="prev_link" value="${name}_base" />
  
  <xacro:foreach item="index" range="1, ${num_joints}">
    <xacro:property name="link_name" value="${name}_link${index}" />
    <xacro:property name="joint_name" value="${name}_joint${index}" />
    
    <!-- Lien -->
    <link name="${link_name}">
      <visual>
        <geometry><box size="${link_length} 0.02 0.02" /></geometry>
        <material name="aluminum" />
      </visual>
    </link>
    
    <!-- Joint -->
    <joint name="${joint_name}" type="revolute">
      <parent link="${prev_link}" />
      <child link="${link_name}" />
      <origin xyz="${link_length/2} 0 0" rpy="0 0 0" />
      <axis xyz="0 0 1" />
      <limit lower="${joint_limits.split()[0]}" 
             upper="${joint_limits.split()[1]}" 
             effort="10" velocity="2" />
    </joint>
    
    <!-- Mettre à jour pour la prochaine itération -->
    <xacro:property name="prev_link" value="${link_name}" />
  </xacro:foreach>
  
</xacro:macro>
```

**Utilisation :**
```xml
<xacro:include filename="$(find mon_robot_description)/urdf/components/arm.xacro" />
<xacro:robot_arm name="left_arm" num_joints="4" link_length="0.15" />
<xacro:robot_arm name="right_arm" num_joints="3" joint_limits="-2.0 2.0" />
```

---

## 12. Résumé des Avantages Xacro

| **Avantage** | **Exemple URDF** | **Exemple Xacro** | **Gain** |
|-------------|-----------------|------------------|----------|
| **Réutilisation** | Copier-coller 4 roues | Macro `wheel` utilisée 4× | -75% de code |
| **Maintenance** | Changer 4 valeurs de rayon | Changer 1 propriété | 4× plus rapide |
| **Personnalisation** | Fichiers séparés | Arguments de macro | Flexible |
| **Lisibilité** | Code répétitif | Structure claire | +50% de compréhension |
| **Testabilité** | Multiples fichiers | Paramètres variables | Facile à valider |

**Commande finale pour visualiser :**
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch mon_robot_description display_robot.launch.py
```

Votre système Xacro est maintenant prêt ! Vous pouvez créer des robots complexes avec une fraction du code nécessaire en URDF pur.
```