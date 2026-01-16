
---

# 1. Qu'est-ce qu'un fichier URDF ? Structure Générale

Un fichier **URDF (Unified Robot Description Format)** est un document **XML** qui sert de "carte d'identité physique" pour votre robot. Il décrit **chaque composant rigide** (les liens) et **comment ils sont connectés** (les joints), permettant aux outils ROS de comprendre la structure de votre robot.

**Structure XML de base :**
```xml
<?xml version="1.0"?>
<!-- La balise racine <robot> contient TOUTE la description -->
<robot name="nom_unique_de_mon_robot">
  
  <!-- SECTION 1: Tous les liens (parties rigides) -->
  <link name="base_link">...</link>
  <link name="roue_gauche">...</link>
  <link name="roue_droite">...</link>
  
  <!-- SECTION 2: Tous les joints (connexions entre liens) -->
  <joint name="joint_roue_gauche" type="continuous">...</joint>
  <joint name="joint_roue_droite" type="continuous">...</joint>
  
</robot>
```

**Pourquoi cette structure est importante :** Le fichier URDF est lu séquentiellement. Vous **devez** définir un lien avant de l'utiliser dans un joint. C'est pourquoi on liste généralement tous les `<link>` d'abord, puis tous les `<joint>`.

---

# 2. Structure Générale d'un Lien (Link)

Un **lien** représente **un segment rigide et indivisible** de votre robot. Chaque lien peut avoir trois types de propriétés, chacune servant un but différent dans la simulation et la visualisation.

```xml
<link name="nom_unique_du_lien">
  
  <!-- 1. Aspect VISUEL (pour RVIZ) - Optionnel mais recommandé -->
  <visual> ... </visual>
  
  <!-- 2. Propriétés de COLLISION (pour Gazebo) - Optionnel -->
  <collision> ... </collision>
  
  <!-- 3. Propriétés d'INERTIE (physique) - Requis pour Gazebo -->
  <inertial> ... </inertial>
  
</link>
```

**Exemple concret :** Pour une roue de robot, le `<link>` contiendrait :
- Un `<visual>` cylindrique gris (pour voir la roue dans RVIZ)
- Un `<collision>` cylindrique identique (pour les collisions dans Gazebo)
- Un `<inertial>` avec sa masse et son inertie (pour la physique réaliste)

---

# 3. Élément VISUEL (`<visual>`)

La section `<visual>` définit **uniquement l'apparence** dans RVIZ. Elle n'affecte **pas** la physique.

## 3.1 Structure Complète

```xml
<visual>
  <!-- 1. Position/orientation de la géométrie -->
  <origin rpy="0 0 0" xyz="0 0 0"/>
  
  <!-- 2. Forme géométrique (UN SEUL autorisé) -->
  <geometry>
    <!-- BOÎTE -->
    <box size="longueur largeur hauteur"/>
    
    <!-- CYLINDRE -->
    <cylinder radius="0.1" length="0.2"/>
    
    <!-- SPHÈRE -->
    <sphere radius="0.15"/>
    
    <!-- MAILLAGE 3D (fichier externe) -->
    <mesh filename="package://mon_package/meshes/roue.stl"/>
  </geometry>
  
  <!-- 3. Matériau/couleur -->
  <material name="mon_materiau">
    <color rgba="rouge vert bleu alpha"/>
  </material>
</visual>
```

## 3.2 Types de Géométries avec Exemples

### Boîte (`<box>`)
```xml
<geometry>
  <!-- taille en mètres: X Y Z -->
  <box size="0.3 0.2 0.1"/>
</geometry>
```
**Cas d'usage :** Châssis de robot, boîtier électronique, capteur rectangulaire.

### Cylindre (`<cylinder>`)
```xml
<geometry>
  <!-- radius=rayon, length=hauteur -->
  <cylinder radius="0.05" length="0.3"/>
</geometry>
```
**Cas d'usage :** Roues, bras cylindriques, tubes structurels.

### Sphère (`<sphere>`)
```xml
<geometry>
  <sphere radius="0.1"/>
</geometry>
```
**Cas d'usage :** Joints sphériques, extrémités arrondies, balles.

### Maillage (`<mesh>`)
```xml
<geometry>
  <!-- Chemin relatif au package ROS -->
  <mesh filename="package://robot_description/meshes/bras_complexe.dae"/>
</geometry>
```
**Formats supportés :** `.dae` (Collada), `.stl` (STereoLithography)
**Cas d'usage :** Pièces complexes, robots réalistes, objets organiques.

## 3.3 Origine (`<origin>`) - rpy et xyz

L'`<origin>` positionne **le repère de la géométrie** par rapport au **repère du lien**.

```xml
<!-- Translation PUIS rotation -->
<origin 
  xyz="déplacement_x déplacement_y déplacement_z"
  rpy="rotation_roll rotation_pitch rotation_yaw"/>
```

**Exemples concrets :**
```xml
<!-- Centré, pas de rotation -->
<origin xyz="0 0 0" rpy="0 0 0"/>

<!-- Décalé de 10cm vers l'avant, 5cm vers le haut -->
<origin xyz="0.1 0 0.05" rpy="0 0 0"/>

<!-- Tourné de 90° autour de l'axe Z (yaw) -->
<origin xyz="0 0 0" rpy="0 0 1.5708"/>
```

**IMPORTANT :** L'ordre des rotations est **toujours** Roll (X) → Pitch (Y) → Yaw (Z).

## 3.4 Matériau (`<material>`)

Définit la couleur ou la texture. Peut être **réutilisé** entre plusieurs liens.

```xml
<!-- Définition avec nom -->
<material name="rouge_vif">
  <!-- RGBA: 0-1 pour chaque composante -->
  <color rgba="1.0 0.0 0.0 1.0"/>
</material>

<!-- Définition anonyme (juste pour ce lien) -->
<material>
  <color rgba="0.0 0.8 0.0 1.0"/>
</material>
```

**Couleurs prédéfinies utiles :**
```xml
<!-- Rouge -->
<color rgba="0.8 0.0 0.0 1.0"/>

<!-- Vert -->
<color rgba="0.0 0.8 0.0 1.0"/>

<!-- Bleu -->
<color rgba="0.0 0.0 0.8 1.0"/>

<!-- Gris métallique -->
<color rgba="0.7 0.7 0.7 1.0"/>

<!-- Noir -->
<color rgba="0.1 0.1 0.1 1.0"/>
```

---

# 4. Élément COLLISION (`<collision>`)

La section `<collision>` définit la géométrie utilisée pour **la détection de collisions** dans les simulateurs physiques comme Gazebo.

```xml
<collision>
  <!-- Même structure que <visual> -->
  <origin rpy="0 0 0" xyz="0 0 0"/>
  <geometry>
    <box size="0.3 0.2 0.1"/>
  </geometry>
  <!-- PAS de <material> ici ! -->
</collision>
```

**DIFFÉRENCES CRUCIALES avec `<visual>` :**
1. **Pas de couleur** : `<material>` n'est pas autorisé
2. **Géométrie souvent simplifiée** : Pour des calculs plus rapides
3. **Optionnel** : Si absent, Gazebo utilise la géométrie `<visual>`

**Exemple d'optimisation :**
```xml
<!-- Pour une roue complexe -->
<visual>
  <mesh filename="package://.../roue_detaillee.dae"/>
</visual>

<collision>
  <!-- Version simplifiée pour la physique -->
  <cylinder radius="0.1" length="0.05"/>
</collision>
```

---

# 5. Élément INERTIAL (`<inertial>`)

**NÉCESSAIRE pour Gazebo.** Définit les propriétés physiques du lien.

## 5.1 Structure Complète

```xml
<inertial>
  <!-- 1. Masse en kilogrammes -->
  <mass value="1.5"/>
  
  <!-- 2. Centre de masse RELATIF au repère du lien -->
  <origin xyz="0 0 0.05"/>
  
  <!-- 3. Tenseur d'inertie -->
  <inertia 
    ixx="0.01" ixy="0"    ixz="0"
            iyy="0.01" iyz="0"
                    izz="0.01"/>
</inertial>
```

## 5.2 Calcul Pratique des Valeurs d'Inertie

### Formules pour formes simples :
```xml
<!-- Pour une BOÎTE de dimensions (lx, ly, lz) et masse m -->
ixx = m/12 * (ly² + lz²)
iyy = m/12 * (lx² + lz²)
izz = m/12 * (lx² + ly²)

<!-- Pour un CYLINDRE de rayon r, longueur L et masse m -->
ixx = m/12 * (3*r² + L²)
iyy = ixx  (symétrie)
izz = m/2 * r²

<!-- Pour une SPHÈRE de rayon r et masse m -->
ixx = iyy = izz = 2/5 * m * r²
```

### Exemple concret pour une boîte :
```xml
<link name="chassis">
  <visual>
    <geometry><box size="0.4 0.3 0.1"/></geometry>
  </visual>
  
  <inertial>
    <mass value="2.0"/>
    <origin xyz="0 0 0"/>  <!-- Centré -->
    <inertia 
      ixx="2.0/12 * (0.3² + 0.1²) = 0.0167"
      iyy="2.0/12 * (0.4² + 0.1²) = 0.0283"
      izz="2.0/12 * (0.4² + 0.3²) = 0.0417"
      ixy="0" ixz="0" iyz="0"/>
  </inertial>
</link>
```

**CONSEIL :** Si vous êtes pressé, utilisez des valeurs **petites mais non nulles** (ex: 0.001) pour éviter les instabilités dans Gazebo.

---

# 6. Exemple Complet d'un Lien

```xml
<link name="roue_avant_gauche">
  
  <!-- APPARENCE dans RVIZ -->
  <visual>
    <origin xyz="0 0 0" rpy="1.5708 0 0"/>  <!-- Couché sur le côté -->
    <geometry>
      <cylinder radius="0.1" length="0.05"/>
    </geometry>
    <material name="gris_metal">
      <color rgba="0.6 0.6 0.6 1.0"/>
    </material>
  </visual>
  
  <!-- COLLISIONS dans Gazebo -->
  <collision>
    <origin xyz="0 0 0" rpy="1.5708 0 0"/>
    <geometry>
      <cylinder radius="0.1" length="0.05"/>
    </geometry>
  </collision>
  
  <!-- PROPRIÉTÉS PHYSIQUES -->
  <inertial>
    <mass value="0.5"/>  <!-- 500g -->
    <origin xyz="0 0 0"/>
    <inertia 
      ixx="0.003125"  <!-- m/12*(3r²+L²) pour cylindre -->
      iyy="0.003125"
      izz="0.0025"    <!-- m/2*r² -->
      ixy="0" ixz="0" iyz="0"/>
  </inertial>
  
</link>
```

---

# 7. Types de Joints (Joints)

Un **joint** définit le **type de mouvement possible** entre deux liens.

## 7.1 Les 6 Types de Joints URDF

### 1. FIXED (`type="fixed"`)
```xml
<!-- Aucun mouvement - comme une soudure -->
<joint name="soudure" type="fixed">
  <parent link="base"/>
  <child link="capteur"/>
</joint>
```
**Degrés de liberté :** 0  
**Cas d'usage :** Capteurs fixés, caméras montées, pièces soudées.

### 2. REVOLUTE (`type="revolute"`)
```xml
<!-- Rotation LIMITÉE autour d'un axe -->
<joint name="coude" type="revolute">
  <axis xyz="0 1 0"/>  <!-- Rotation autour de Y -->
  <limit lower="-1.57" upper="1.57"/>  <!-- -90° à +90° -->
</joint>
```
**Degrés de liberté :** 1 (rotation)  
**Cas d'usage :** Articulations de bras, portes, leviers.

### 3. CONTINUOUS (`type="continuous"`)
```xml
<!-- Rotation SANS LIMITES -->
<joint name="roue" type="continuous">
  <axis xyz="0 0 1"/>  <!-- Rotation autour de Z -->
  <!-- PAS de <limit> pour lower/upper ! -->
</joint>
```
**Degrés de liberté :** 1 (rotation illimitée)  
**Cas d'usage :** Roues, moteurs rotatifs, tourelles.

### 4. PRISMATIC (`type="prismatic"`)
```xml
<!-- Translation LIMITÉE le long d'un axe -->
<joint name="piston" type="prismatic">
  <axis xyz="1 0 0"/>  <!-- Translation selon X -->
  <limit lower="0" upper="0.5"/>  <!-- Course de 50cm -->
</joint>
```
**Degrés de liberté :** 1 (translation)  
**Cas d'usage :** Pneus hydrauliques, tiroirs, guides linéaires.

### 5. PLANAR (`type="planar"`)
```xml
<!-- Mouvement dans un PLAN -->
<joint name="plateau" type="planar">
  <axis xyz="1 0 0"/>  <!-- Le plan normal -->
</joint>
```
**Degrés de liberté :** 3 (2 translations + 1 rotation dans le plan)  
**Cas d'usage :** Plateaux mobiles, pièces glissantes.

### 6. FLOATING (`type="floating"`)
```xml
<!-- Mouvement COMPLET -->
<joint name="flottant" type="floating">
  <!-- Pas d'<axis> ni <limit> -->
</joint>
```
**Degrés de liberté :** 6 (3 rotations + 3 translations)  
**Cas d'usage :** Objets non attachés, robots volants.

---

# 8. Lien Parent et Enfant (`<parent>` et `<child>`)

La relation **parent→enfant** définit la hiérarchie et le sens des transformations.

```xml
<joint name="mon_joint" type="revolute">
  <!-- Le repère de RÉFÉRENCE -->
  <parent link="lien_parent"/>
  
  <!-- Le repère qui BOUGE par rapport au parent -->
  <child link="lien_enfant"/>
</joint>
```

**RÈGLE D'OR :** Le repère de l'**enfant** est défini **par rapport au repère du parent** via l'`<origin>` du joint.

**Exemple d'une roue attachée à un châssis :**
```xml
<!-- Le châssis ne bouge pas par rapport à lui-même -->
<parent link="chassis"/>

<!-- La roue bouge par rapport au châssis -->
<child link="roue_gauche"/>

<!-- L'origine positionne le centre de la roue
     à 20cm à gauche et 10cm au-dessus du châssis -->
<origin xyz="-0.2 0 0.1" rpy="0 0 0"/>
```

---

# 9. Origine (`<origin>`) et Axe (`<axis>`) du Joint

## 9.1 Origine du Joint
Positionne **le repère de l'enfant** dans **le repère du parent**.

```xml
<origin 
  xyz="x y z"    <!-- Position de l'enfant par rapport au parent -->
  rpy="r p y"/>  <!-- Orientation de l'enfant par rapport au parent -->
```

**Exemple : Bras attaché avec décalage**
```xml
<!-- Bras attaché 30cm vers l'avant, tourné de 90° vers le bas -->
<origin xyz="0.3 0 0" rpy="0 -1.5708 0"/>
```

## 9.2 Axe du Joint
Définit **la direction du mouvement** pour les joints mobiles.

```xml
<axis xyz="x y z"/>
```

**Vecteurs normalisés courants :**
```xml
<!-- Rotation autour de X -->
<axis xyz="1 0 0"/>

<!-- Rotation autour de Y -->
<axis xyz="0 1 0"/>

<!-- Rotation autour de Z -->
<axis xyz="0 0 1"/>

<!-- Direction diagonale -->
<axis xyz="0.707 0.707 0"/>
```

---

# 10. Dynamique (`<dynamics>`)

Contrôle le **comportement physique** du joint dans les simulateurs.

```xml
<dynamics
  damping="valeur"   <!-- Amortissement visqueux -->
  friction="valeur"/> <!-- Frottement statique -->
```

**Valeurs typiques :**
```xml
<!-- Joint très fluide (moteur dans l'air) -->
<dynamics damping="0.01" friction="0.0"/>

<!-- Joint amorti (amortisseur) -->
<dynamics damping="0.5" friction="0.0"/>

<!-- Joint avec friction (vieux mécanisme) -->
<dynamics damping="0.1" friction="0.3"/>
```

**Unités :**
- `damping` : N·m·s/rad (rotational) ou N·s/m (prismatic)
- `friction` : N·m (rotational) ou N (prismatic)

---

# 11. Limites (`<limit>`)

Définit les **contraintes physiques** du mouvement.

## Pour les joints REVOLUTE :
```xml
<limit
  lower="-3.14"    <!-- Angle MINIMUM en radians -->
  upper="3.14"     <!-- Angle MAXIMUM en radians -->
  effort="10.0"    <!-- Couple MAXIMUM en N·m -->
  velocity="2.0"/> <!-- Vitesse MAXIMUM en rad/s -->
```

## Pour les joints PRISMATIC :
```xml
<limit
  lower="0"        <!-- Position MINIMUM en mètres -->
  upper="0.5"      <!-- Position MAXIMUM en mètres -->
  effort="50.0"    <!-- Force MAXIMUM en Newtons -->
  velocity="0.1"/> <!-- Vitesse MAXIMUM en m/s -->
```

## Pour les joints CONTINUOUS :
```xml
<!-- Pas de lower/upper, mais effort/velocity existent -->
<limit
  effort="5.0"     <!-- Couple MAX -->
  velocity="10.0"/> <!-- Vitesse MAX -->
```

**Valeurs réalistes :**
```xml
<!-- Articulation de bras humain -->
<limit lower="-1.57" upper="1.57" effort="20.0" velocity="3.14"/>

<!-- Piston hydraulique -->
<limit lower="0" upper="1.0" effort="100.0" velocity="0.5"/>

<!-- Moteur de roue -->
<limit effort="2.0" velocity="31.4"/>  <!-- 300 RPM -->
```

---

# 12. Exemple Complet d'un Joint

```xml
<joint name="shoulder_pitch" type="revolute">
  
  <!-- HIÉRARCHIE -->
  <parent link="torso"/>
  <child link="upper_arm"/>
  
  <!-- POSITION/ORIENTATION -->
  <origin xyz="0.1 0 0.3" rpy="0 0 0"/>
  
  <!-- DIRECTION DU MOUVEMENT -->
  <axis xyz="0 1 0"/>  <!-- Rotation autour de Y (pitch) -->
  
  <!-- COMPORTEMENT DYNAMIQUE -->
  <dynamics damping="0.2" friction="0.05"/>
  
  <!-- CONTRAINTES PHYSIQUES -->
  <limit 
    lower="-1.5708"  <!-- -90° -->
    upper="2.3562"   <!-- +135° -->
    effort="15.0"    <!-- 15 N·m -->
    velocity="3.1416"/><!-- 180°/s -->
    
</joint>
```

**Interprétation :** Ce joint représente une épaule robotique qui :
1. Connecte le torse (parent) au bras supérieur (enfant)
2. Est positionnée 10cm vers l'avant et 30cm vers le haut du torse
3. Tourne autour de l'axe Y (mouvement de haussement d'épaule)
4. A un léger amortissement et frottement
5. Peut bouger de -90° à +135° avec un couple max de 15N·m

---





# Créer un Package URDF de A à Z

## 1. Structure du Projet pour un Package URDF

Quand vous partez d'un projet vierge, voici la structure **complète** à créer :

```
~/ros2_ws/src/
└── mon_robot_description/           # Votre package URDF
    ├── package.xml                  # Métadonnées et dépendances
    ├── CMakeLists.txt              # Configuration de compilation
    ├── urdf/                       # Dossier pour les fichiers URDF
    │   └── mon_robot.urdf          # Votre fichier URDF principal
    ├── launch/                     # Fichiers de lancement
    │   └── display_urdf.launch.py  # Lanceur pour visualiser
    ├── rviz/                       # Configurations RVIZ
    │   └── view_robot.rviz         # Configuration de vue
    └── meshes/                     # (Optionnel) Maillages 3D
        └── chassis.stl             # Exemple de maillage
```

## 2. Création du Package

```bash
# Depuis votre espace de travail ROS 2
cd ~/ros2_ws/src

# Créer le package - VERSION COMPLÈTE avec toutes les dépendances
ros2 pkg create mon_robot_description \
  --build-type ament_cmake \
  --dependencies rclcpp urdf launch_ros rviz2 gazebo_ros \
  --description "Description URDF de mon robot"

# Aller dans le package
cd mon_robot_description
```

## 3. Configuration du `package.xml`

Voici le fichier **complet** avec toutes les dépendances nécessaires :

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>mon_robot_description</name>
  <version>0.0.0</version>
  <description>Description URDF complète de mon robot</description>
  <maintainer email="vous@example.com">Votre Nom</maintainer>
  <license>Apache-2.0</license>

  <!-- DÉPENDANCES EXÉCUTION (obligatoires) -->
  <depend>rclcpp</depend>
  <depend>urdf</depend>          <!-- Parser URDF -->
  <depend>rviz2</depend>         <!-- Visualisation -->
  <depend>robot_state_publisher</depend>  <!-- Publication TF -->
  <depend>joint_state_publisher</depend>  <!-- Contrôle joints -->
  <depend>gazebo_ros</depend>    <!-- Simulation Gazebo -->
  <depend>xacro</depend>         <!-- Macros XML -->
  
  <!-- DÉPENDANCES CONSTRUCTION -->
  <build_depend>ament_cmake</build_depend>
  
  <!-- DÉPENDANCES TEST -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## 4. Configuration du `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(mon_robot_description)

# Trouver les dépendances
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(rviz2 REQUIRED)

# Installer les répertoires
install(
  DIRECTORY urdf launch rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

# Installer les fichiers de lancement
install(
  PROGRAMS
  launch/display_urdf.launch.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

## 5. Fichier URDF Complet

Créez `urdf/mon_robot.urdf` :

```xml
<?xml version="1.0"?>
<robot name="mon_robot_simple" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ============================================ -->
  <!-- 1. MATÉRIAUX (couleurs réutilisables) -->
  <!-- ============================================ -->
  <material name="bleu">
    <color rgba="0.1 0.1 0.8 1.0"/>
  </material>
  <material name="rouge">
    <color rgba="0.8 0.1 0.1 1.0"/>
  </material>
  <material name="noir">
    <color rgba="0.1 0.1 0.1 1.0"/>
  </material>

  <!-- ============================================ -->
  <!-- 2. LIENS (parties rigides du robot) -->
  <!-- ============================================ -->

  <!-- 2.1 BASE (châssis principal) -->
  <link name="base_link">
    <!-- Apparence visuelle -->
    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
      <material name="bleu"/>
    </visual>
    
    <!-- Collision (identique au visuel ici) -->
    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.4 0.3 0.1"/>
      </geometry>
    </collision>
    
    <!-- Propriétés inertielles (ESSENTIEL pour Gazebo) -->
    <inertial>
      <mass value="2.5"/>  <!-- 2.5 kg -->
      <origin xyz="0 0 0.05"/>
      <inertia 
        ixx="0.0208" ixy="0" ixz="0"
        iyy="0.0167" iyz="0"
        izz="0.0417"/>
    </inertial>
  </link>

  <!-- 2.2 ROUE GAUCHE -->
  <link name="left_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>  <!-- Rotation pour mettre à plat -->
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <material name="noir"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0"/>
      <inertia 
        ixx="0.0004" ixy="0" ixz="0"
        iyy="0.0004" iyz="0"
        izz="0.000375"/>
    </inertial>
  </link>

  <!-- 2.3 ROUE DROITE (identique à la gauche) -->
  <link name="right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
      <material name="noir"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="0.02"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0 0 0"/>
      <inertia 
        ixx="0.0004" ixy="0" ixz="0"
        iyy="0.0004" iyz="0"
        izz="0.000375"/>
    </inertial>
  </link>

  <!-- ============================================ -->
  <!-- 3. JOINTS (connexions entre liens) -->
  <!-- ============================================ -->

  <!-- 3.1 JOINT ROUE GAUCHE (rotation continue) -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0.0 0.15 -0.025"/>  <!-- 15cm à gauche, 2.5cm sous la base -->
    <axis xyz="0 1 0"/>  <!-- Rotation autour de Y -->
    <dynamics damping="0.1"/>
    <limit effort="100" velocity="100"/>
  </joint>

  <!-- 3.2 JOINT ROUE DROITE (rotation continue) -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0.0 -0.15 -0.025"/>  <!-- 15cm à droite, 2.5cm sous la base -->
    <axis xyz="0 1 0"/>  <!-- Rotation autour de Y -->
    <dynamics damping="0.1"/>
    <limit effort="100" velocity="100"/>
  </joint>

</robot>
```

## 6. Fichier de Lancement (`display_urdf.launch.py`)

Créez `launch/display_urdf.launch.py` :

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # Chemin vers le package et le fichier URDF
    pkg_path = get_package_share_directory('mon_robot_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'mon_robot.urdf')
    
    # Lire le fichier URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    # Nœud 1: Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }]
    )
    
    # Nœud 2: Joint State Publisher (contrôle interactif des joints)
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # Nœud 3: RVIZ2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_path, 'rviz', 'view_robot.rviz')]
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node
    ])
```

## 7. Configuration RVIZ (`view_robot.rviz`)

Créez `rviz/view_robot.rviz` :

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
      Description: Visualisation du modèle URDF
      Enabled: true
      Robot Description: robot_description
      Visual Enabled: true
      Collision Enabled: false
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Show Names: true
      Show Arrows: true
      Marker Scale: 0.5
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Name: Current View
      Target Frame: base_link
```

## 8. Construction et Test

### 8.1 Construire le package
```bash
# Depuis la racine de votre espace de travail
cd ~/ros2_ws

# Construire uniquement votre package
colcon build --packages-select mon_robot_description

# Sourcer l'installation
source install/setup.bash
```

### 8.2 Tester manuellement (sans lanceur)
```bash
# Terminal 1: Robot State Publisher
source ~/ros2_ws/install/setup.bash
ros2 run robot_state_publisher robot_state_publisher \
  ~/ros2_ws/src/mon_robot_description/urdf/mon_robot.urdf

# Terminal 2: Joint State Publisher
source ~/ros2_ws/install/setup.bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui

# Terminal 3: RVIZ2
source ~/ros2_ws/install/setup.bash
rviz2
```

**Dans RVIZ :**
1. Cliquez sur "Add" en bas à gauche
2. Sélectionnez "RobotModel"
3. Dans "Robot Description", entrez "robot_description"
4. Vous devriez voir votre robot bleu avec deux roues noires

### 8.3 Utiliser le lanceur (méthode recommandée)
```bash
# Une seule commande lance tout
source ~/ros2_ws/install/setup.bash
ros2 launch mon_robot_description display_urdf.launch.py
```

## 9. Vérification et Dépannage

### 9.1 Vérifier la syntaxe URDF
```bash
# Installer le package urdfdom-tools si nécessaire
sudo apt install liburdfdom-tools

# Vérifier votre fichier URDF
check_urdf ~/ros2_ws/src/mon_robot_description/urdf/mon_robot.urdf
```

**Résultat attendu :**
```
robot name is: mon_robot_simple
---------- Successfully Parsed XML ---------------
root Link: base_link has 2 child(ren)
    child(1):  left_wheel
    child(2):  right_wheel
```

### 9.2 Vérifier les transformations (TF)
```bash
# Voir les frames disponibles
ros2 run tf2_tools view_frames.py
# Génère un fichier frames.pdf avec l'arborescence TF

# Voir les transformations en temps réel
ros2 run tf2_ros tf2_monitor
```

### 9.3 Problèmes courants et solutions

**Problème :** "Could not find package..."
```bash
# Assurez-vous d'avoir sourcé
source ~/ros2_ws/install/setup.bash
# Ou pour un terminal frais
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

**Problème :** RVIZ ne montre rien
1. Vérifiez que `robot_state_publisher` tourne : `ros2 topic list | grep robot_description`
2. Dans RVIZ, vérifiez que "Fixed Frame" est défini sur `base_link`
3. Cliquez sur "Reset" dans RVIZ

**Problème :** Erreurs de compilation
```bash
# Nettoyer et reconstruire
cd ~/ros2_ws
rm -rf build install log
colcon build --packages-select mon_robot_description
```

## 10. Prochaines Étapes

### 10.1 Ajouter un contrôleur de mouvement
```python
# Exemple de nœud pour faire tourner les roues
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 10.2 Simuler dans Gazebo
Créez `launch/gazebo.launch.py` :
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        )
    ])
```

### 10.3 Utiliser Xacro pour simplifier
```xml
<!-- Au lieu de répéter pour chaque roue -->
<xacro:macro name="wheel" params="side">
  <link name="${side}_wheel">...</link>
  <joint name="${side}_wheel_joint">...</joint>
</xacro:macro>

<xacro:wheel side="left"/>
<xacro:wheel side="right"/>
```

## Résumé des Commandes Essentielles

```bash
# 1. Construire
colcon build --packages-select mon_robot_description

# 2. Sourcer
source install/setup.bash

# 3. Lancer
ros2 launch mon_robot_description display_urdf.launch.py

# 4. Vérifier
check_urdf src/mon_robot_description/urdf/mon_robot.urdf

# 5. Voir les topics
ros2 topic list
ros2 topic echo /robot_description
```

Votre projet URDF est maintenant **complètement fonctionnel** ! Vous pouvez :
- Visualiser le robot dans RVIZ
- Contrôler les joints avec l'interface graphique
- Étendre avec plus de liens/joints
- Intégrer à Gazebo pour la simulation physique
```