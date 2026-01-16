
# Créer son propre package ROS 2

## Structure et création d'un package

Un package est le conteneur de votre code ROS 2. Il doit contenir au minimum :
- `CMakeLists.txt` : Instructions de compilation (pour C++).
- `package.xml` : Métadonnées (nom, version, dépendances).
- Code source (dans `src/` pour C++, `my_package/` pour Python).

**Pourquoi créer son package ?** Pour organiser, compiler et distribuer son propre code dans l'écosystème ROS 2.

### Prérequis et création

**0. Prérequis** (si ce n'est pas déjà fait) :
```bash
sudo apt install python3-colcon-common-extensions
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

1.  **Créer un nouveau package** depuis `~/ros2_ws/src` :
    ```bash
    # Syntaxe générale (C++) :
    ros2 pkg create --build-type ament_cmake <package_name>

    # Exemple concret :
    ros2 pkg create --build-type ament_cmake my_package
    ```

---
## Inspection de la structure générée

2.  **Explorer les fichiers créés automatiquement** :
    ```bash
    cd ~/ros2_ws/src/my_package
    ls -la
    ```
    **Structure typique :**
    - `include/my_package/` : Dossier pour les fichiers d'en-tête (headers C++).
    - `src/` : Contient le fichier source `my_node.cpp` (un exemple de nœud "Hello World").
    - `CMakeLists.txt` : Configure la construction avec `ament_cmake`.
    - `package.xml` : Définit le nom, la version, les auteurs et les dépendances du package.

---
## Construction et exécution

3.  **Construire uniquement votre nouveau package** (depuis `~/ros2_ws`) :
    ```bash
    colcon build --packages-select my_package
    ```
    *L'option `--packages-select` cible spécifiquement votre package pour une construction rapide.*

4.  **Activer l'environnement de votre espace de travail** (overlay) :
    ```bash
    source install/local_setup.bash
    ```

5.  **Exécuter le nœud exemple** généré automatiquement :
    ```bash
    ros2 run my_package my_node
    ```
    **Résultat attendu :** Le terminal devrait afficher un message de type `"Hello World my_package package"`.

**Important :** Votre package personnalisé (`my_package`) est maintenant fonctionnel et prioritaire sur tout package système du même nom. Vous pouvez le modifier, le reconstruire et l'exécuter.



---
## Publisher et Subscriber en C++ - Package complet

## Création et configuration complète du package

### 0. Préparation
Installer une extension pour la coloration syntaxique des fichiers CMakeLists.txt dans votre éditeur.

### 1. Création du package
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake cpp_pubsub
```

### 2. Vérification des dépendances
Avant de continuer, vérifiez et installez les dépendances manquantes :
```bash
# Depuis ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
```
**Important :** `rosdep` installe automatiquement les dépendances système requises listées dans `package.xml`.

### 3. Fichiers source C++
Créez ces deux fichiers dans `~/ros2_ws/src/cpp_pubsub/src/` :

**`publisher_member_function.cpp` :**
```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

**`subscriber_member_function.cpp` :**
```cpp
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

### 4. Modification du `CMakeLists.txt`
Ouvrez `~/ros2_ws/src/cpp_pubsub/CMakeLists.txt` et ajoutez/modifiez :

```cmake
# Trouver les dépendances REQUISES
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Créer l'exécutable du publisher
add_executable(talker src/publisher_member_function.cpp)
target_include_directories(talker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(talker
  rclcpp
  std_msgs)

# Créer l'exécutable du subscriber
add_executable(listener src/subscriber_member_function.cpp)
target_include_directories(listener PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(listener
  rclcpp
  std_msgs)

# Installer les exécutables
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
```

### 5. Modification du `package.xml`
Ajoutez ces dépendances dans `~/ros2_ws/src/cpp_pubsub/package.xml` :
```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

### 6. Construction et exécution
```bash
cd ~/ros2_ws
colcon build --packages-select cpp_pubsub
source install/local_setup.bash

# Terminal 1 : Publisher
ros2 run cpp_pubsub talker

# Terminal 2 : Subscriber
ros2 run cpp_pubsub listener
```

**Concepts ROS 2/C++ :**
- **Héritage** : Les classes `MinimalPublisher` et `MinimalSubscriber` héritent de `rclcpp::Node`.
- **Smart Pointers** : `SharedPtr` pour la gestion automatique de la mémoire des objets ROS.
- **Callbacks** : Mécanisme de fonction de rappel pour les timers et les messages.
- **Dépendances** : `rclcpp` (core ROS 2 C++) et `std_msgs` (messages standards).

```md
---
title: Publisher et Subscriber en Python
---

## Création et configuration du package Python

### 1. Création du package (type différent)
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python py_pubsub
```

### 2. Structure des fichiers Python
Créez ces fichiers dans `~/ros2_ws/src/py_pubsub/py_pubsub/` :

**`publisher_member_function.py` :**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # secondes
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**`subscriber_member_function.py` :**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prévenir les avertissements de variable inutilisée

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    
    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        pass
    
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. Modification du `setup.py`
Ouvrez `~/ros2_ws/src/py_pubsub/setup.py` et modifiez :

```python
from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='votre_nom',
    maintainer_email='votre_email@example.com',
    description='Exemple de publisher/subscriber Python',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = py_pubsub.publisher_member_function:main',
            'listener = py_pubsub.subscriber_member_function:main',
        ],
    },
)
```

### 4. Modification du `package.xml`
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>py_pubsub</name>
  <version>0.0.0</version>
  <description>Exemple de publisher/subscriber Python</description>
  <maintainer email="votre_email@example.com">Votre Nom</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 5. Vérification des dépendances
```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
```

### 6. Construction et exécution
```bash
cd ~/ros2_ws
colcon build --packages-select py_pubsub
source install/local_setup.bash

# Terminal 1 : Publisher
ros2 run py_pubsub talker

# Terminal 2 : Subscriber
ros2 run py_pubsub listener
```

**Différences clés avec C++ :**
- **Build type** : `ament_python` au lieu de `ament_cmake`
- **Setup** : `setup.py` au lieu de `CMakeLists.txt`
- **Entry points** : Configuration dans `setup.py` pour créer les commandes exécutables
- **Syntaxe** : Pas de point-virgules, gestion mémoire automatique (pas de smart pointers)
- **Import** : `import rclpy` au lieu de `#include "rclcpp/rclcpp.hpp"`

**Points communs :**
- Même architecture de classes (héritage de `Node`)
- Même pattern de callbacks
- Mêmes concepts ROS 2 (topics, messages, logging)