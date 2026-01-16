
---
https://www.youtube.com/watch?v=HJAE5Pk8Nyw
---

## Installation et exécution

Cette section couvre l'installation d'un package, la localisation de ses composants et le lancement des noeuds exécutables.

**Pourquoi faire ?** Pour comprendre comment installer, trouver et démarrer les programmes (noeuds) qui composent une application ROS 2.

### Commandes clés
1.  **Installer un package** :
    ```bash
    sudo apt update
    sudo apt install ros-<distribution>-<package_name>
    # Exemple :
    sudo apt install ros-humble-turtlesim
    ```

2.  **Lister les packages installés** :
    ```bash
    ros2 pkg list
    ```

3.  **Lister les exécutables (noeuds)** d'un package :
    ```bash
    ros2 pkg executables <package_name>
    # Exemple :
    ros2 pkg executables turtlesim
    ```

### Exécuter un noeud
La commande pour lancer un programme est fondamentale :
```bash
ros2 run <package_name> <executable_name>
# Exemple : Lance le simulateur de tortue
ros2 run turtlesim turtlesim_node
```

**Important** : Chaque exécutable correspond généralement à un **noeud** dans le graphe de calcul ROS 2. On utilise `ros2 run` pour les démarrer individuellement.

### Localisation physique
Les packages sont installés dans le répertoire système de ROS (ex : `/opt/ros/humble/`). Leurs fichiers ressources (interfaces, lanceurs) se trouvent principalement dans `share/`.
```bash
cd /opt/ros/humble/
# Chercher où est installé 'turtlesim'
```

---
# Commandes des noeuds (Nodes) ROS 2


## Concepts et commandes de base

Un **noeud** est un processus exécutable qui fait partie d'un graphe de calcul ROS 2. Il effectue une tâche spécifique (par exemple, contrôler un robot, lire un capteur).

**Pourquoi les utiliser ?** Pour structurer un système robotique en modules logiciels indépendants et communicants.

### Commandes essentielles

1.  **Afficher l'aide** pour toutes les commandes liées aux noeuds :
    ```bash
    ros2 node -h
    ```

2.  **Lister tous les noeuds actuellement en cours d'exécution** dans le système ROS 2 :
    ```bash
    ros2 node list
    # Initialement, cette liste est vide.
    ```

3.  **Démarrer un noeud** (exemple avec le simulateur de tortue) :
    ```bash
    ros2 run turtlesim turtlesim_node
    ```

4.  **Vérifier que le noeud est bien actif** (la liste doit maintenant contenir `/turtlesim`) :
    ```bash
    ros2 node list
    ```

5.  **Obtenir des informations détaillées** sur un noeud spécifique, y compris ses publications (topics), abonnements, services et actions :
    ```bash
    ros2 node info <node_name>
    # Exemple concret :
    ros2 node info /turtlesim
    ```

---
## Les topics ROS (Canaux de communication)

Un **topic** est un canal nommé par lequel les noeuds échangent des **messages**. Il suit un modèle de communication **publication/abonnement (pub/sub)**.

**Pourquoi les utiliser ?** Pour transmettre un flux continu de données (comme la vitesse, l'image d'une caméra, les données d'un laser) d'un ou plusieurs nœuds émetteurs (*publishers*) vers un ou plusieurs nœuds récepteurs (*subscribers*).

### Travail pratique avec les topics

1.  **Préparer l'environnement** (dans deux terminaux séparés) :
    ```bash
    # Terminal 1 : Démarrer le simulateur
    ros2 run turtlesim turtlesim_node
    # Terminal 2 : Démarrer la télécommande au clavier
    ros2 run turtlesim turtle_teleop_key
    ```
    *Bouger la tortue avec les touches fléchées pour établir une connexion active.*

2.  **Découvrir les topics disponibles** :
    ```bash
    # Lister simplement les noms
    ros2 topic list
    # Lister les noms avec leur type de message associé (très utile)
    ros2 topic list -t
    ```

3.  **Visualiser le graphe de communication** entre les noeuds et les topics :
    ```bash
    rqt_graph
    ```
    *Dans la fenêtre de `rqt_graph`, décochez "Hide debug" et "Hide leaf topics" pour voir tous les éléments.*

4.  **Écouter (lire) les messages publiés sur un topic** en temps réel :
    ```bash
    ros2 topic echo <topic_name>
    # Exemple - affiche les commandes de vitesse envoyées à la tortue :
    ros2 topic echo /turtle1/cmd_vel
    ```

5.  **Obtenir des informations sur un topic spécifique** (nombre de publishers/subscribers) :
    ```bash
    ros2 topic info <topic_name>
    ros2 topic info /turtle1/cmd_vel
    ```

6.  **Inspecter la structure (le format) d'un type de message** :
    ```bash
    ros2 interface show <message_type>
    # Exemple - voir les champs 'linear' et 'angular' du message Twist :
    ros2 interface show geometry_msgs/msg/Twist
    ```

7.  **Publier manuellement un message sur un topic** (une seule fois) :
    ```bash
    ros2 topic pub --once <topic_name> <message_type> '<values>'
    # Exemple - commande pour faire avancer et tourner la tortue une fois :
    ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
    ```

8.  **Publier des messages de manière continue à une fréquence fixe** (par exemple, 1 message par seconde) :
    ```bash
    ros2 topic pub --rate 1 <topic_name> <message_type> '<values>'
    # Exemple - publie une commande de rotation continue à 1 Hz :
    ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
    ```

9.  **Écouter un autre type de données** (par exemple, la pose de la tortue) :
    ```bash
    ros2 topic echo /turtle1/pose
    ```

10. **Mesurer la fréquence de publication** d'un topic (en messages par seconde) :
    ```bash
    ros2 topic hz <topic_name>
    # Exemple - vérifier à quelle fréquence la pose est mise à jour :
    ros2 topic hz /turtle1/pose
    ```


---

## Concept du modèle client-serveur

Un **service** est un mécanisme de communication synchrone entre les noeuds, basé sur un modèle **client-serveur**.

**Pourquoi les utiliser ?** Pour réaliser des interactions ponctuelles qui nécessitent une réponse immédiate. Contrairement aux topics (flux continu), le client envoie une **requête** et attend une **réponse** du serveur. Exemples : activer/désactiver un composant, demander un calcul spécifique, déclencher une action unique.

### Commandes fondamentales

1.  **Lancer l'environnement de démonstration** :
    ```bash
    # Terminal 1 : Serveur (le simulateur)
    ros2 run turtlesim turtlesim_node
    # Terminal 2 : Client de test (télécommande)
    ros2 run turtlesim turtle_teleop_key
    ```

2.  **Lister tous les services actuellement disponibles** dans le système :
    ```bash
    ros2 service list
    ```

---
## Inspection des services

3.  **Connaître le type d'un service** (le format de sa requête et de sa réponse) :
    ```bash
    ros2 service type <service_name>
    # Exemple - service pour effacer l'arrière-plan du simulateur :
    ros2 service type /clear
    ```

4.  **Trouver tous les services d'un type donné** :
    ```bash
    ros2 service find <service_type>
    # Exemple - trouver tous les services utilisant le type vide (sans arguments) :
    ros2 service find std_srvs/srv/Empty
    ```

5.  **Afficher l'interface (le schéma) complète d'un type de service** :
    ```bash
    ros2 interface show <service_type>
    # Exemple - voir la structure pour faire apparaître une nouvelle tortue :
    ros2 interface show turtlesim/srv/Spawn
    ```
    *Important : L'affichage montre deux parties séparées par `---` : la **requête** (ce que le client envoie) et la **réponse** (ce que le serveur renvoie).*

---
## Appeler un service (Client)

6.  **Appeler un service** pour exécuter une action.
    Syntaxe générale :
    ```bash
    ros2 service call <service_name> <service_type> '<arguments>'
    ```

    **Exemple 1 : Effacer le dessin de la tortue** (service sans arguments) :
    ```bash
    ros2 service call /clear std_srvs/srv/Empty
    ```

    **Exemple 2 : Faire apparaître une nouvelle tortue** (service avec arguments) :
    ```bash
    ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.5, y: 9.0, theta: 1.57, name: 'nouvelle_tortue'}"
    ```
    *`x`, `y` : position initiale. `theta` : orientation (1.57 rad ≈ 90°). `name` : nom unique du nouveau noeud.*

---
title: Les paramètres (Parameters) ROS 2
---

## Concept de configuration dynamique

Un **paramètre** est une valeur de configuration interne à un nœud qui peut être lue et modifiée à l'exécution.

**Pourquoi les utiliser ?** Pour ajuster le comportement d'un nœud sans le redémarrer. Par exemple : changer une couleur, une vitesse limite, un mode de fonctionnement, ou un seuil de détection.

### Commandes de base

1.  **Démarrer les nœuds de référence** :
    ```bash
    # Terminal 1 : Nœud avec des paramètres modifiables
    ros2 run turtlesim turtlesim_node
    # Terminal 2 : Télécommande (pour interagir)
    ros2 run turtlesim turtle_teleop_key
    ```

2.  **Lister tous les paramètres disponibles** pour un nœud (ou tous les nœuds) :
    ```bash
    # Lister tous les paramètres de tous les nœuds actifs
    ros2 param list
    ```

---
## Lecture et modification des paramètres

3.  **Lire la valeur actuelle d'un paramètre spécifique** :
    ```bash
    ros2 param get <node_name> <parameter_name>
    # Exemple - obtenir la composante verte du fond d'écran :
    ros2 param get /turtlesim background_g
    ```

4.  **Modifier la valeur d'un paramètre à chaud** :
    ```bash
    ros2 param set <node_name> <parameter_name> <value>
    # Exemple - changer le fond en vert vif :
    ros2 param set /turtlesim background_g 255
    ```
    *Important : Le changement est instantané et persiste jusqu'à la fermeture du nœud.*

---
## Sauvegarde et chargement de configurations

5.  **Afficher (dumper) tous les paramètres d'un nœud** avec leurs valeurs actuelles :
    ```bash
    ros2 param dump <node_name>
    # Exemple - voir la configuration complète du simulateur :
    ros2 param dump /turtlesim
    ```

6.  **Sauvegarder cette configuration dans un fichier YAML** pour une réutilisation ultérieure :
    ```bash
    ros2 param dump /turtlesim > turtlesim_params.yaml
    ```

7.  **Charger une configuration à partir d'un fichier YAML** sur un nœud déjà en cours d'exécution :
    ```bash
    ros2 param load <node_name> <parameter_file>
    # Exemple :
    ros2 param load /turtlesim turtlesim_params.yaml
    ```

8.  **Charger une configuration directement au démarrage d'un nœud** (méthode recommandée) :
    ```bash
    ros2 run <package_name> <executable_name> --ros-args --params-file <fichier.yaml>
    # Exemple :
    ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim_params.yaml
    ```
    *C'est la meilleure pratique pour initialiser un nœud avec une configuration spécifique.*

---
title: Les actions (Actions) ROS 2
---

## Concept des actions à longue durée

Une **action** est un mécanisme de communication pour les tâches longues et pouvant être annulées. Elle suit un modèle **client-serveur** avec trois parties : un **objectif** (goal), un **retour d'état** (feedback), et un **résultat** (result).

**Pourquoi les utiliser ?** Pour les opérations qui prennent du temps (ex : navigation vers un point, manipulation d'objet), où le client doit pouvoir suivre la progression (feedback) et annuler si nécessaire. C'est une combinaison des avantages des services (requête/réponse) et des topics (flux continu).

### Démonstration avec la télécommande

1.  **Démarrer les nœuds** :
    ```bash
    # Terminal 1 : Serveur d'actions (le simulateur)
    ros2 run turtlesim turtlesim_node
    # Terminal 2 : Client d'actions (la télécommande avancée)
    ros2 run turtlesim turtle_teleop_key
    ```

2.  **Tester l'action de rotation** dans la fenêtre `turtle_teleop_key` :
    - Appuyez sur `G` : la tortue tourne de 180°. Observez le statut qui s'affiche à la fin.
    - Appuyez sur `G` puis sur `F` avant la fin : annulez l'action en cours.
    - Appuyez sur `G` et regardez l'orientation de la tortue pendant la rotation (feedback visuel).

---
## Inspection des actions disponibles

3.  **Vérifier les connexions d'actions** d'un nœud (client ou serveur) :
    ```bash
    ros2 node info /teleop_turtle
    ```
    *Regardez la section "Action Clients" en bas de la sortie.*

4.  **Lister toutes les actions actives** dans le système :
    ```bash
    ros2 action list
    # Lister avec leur type associé (-t pour "type") :
    ros2 action list -t
    ```

5.  **Obtenir des informations sur une action spécifique** (nombre de clients/serveurs) :
    ```bash
    ros2 action info <action_name>
    # Exemple - action de rotation absolue :
    ros2 action info /turtle1/rotate_absolute
    ```

---
## Structure et utilisation des actions

6.  **Afficher l'interface complète d'un type d'action** :
    ```bash
    ros2 interface show turtlesim/action/RotateAbsolute
    ```
    *Important : L'affichage montre trois sections séparées par `---` :*
    1.  **Goal** : ce que le client demande (ex : `float32 theta`).
    2.  **Result** : ce que le serveur renvoie à la fin (ex : `float32 delta`).
    3.  **Feedback** : les mises à jour envoyées pendant l'exécution (ex : `float32 remaining`).

7.  **Envoyer un objectif (goal) à une action** depuis la ligne de commande :
    ```bash
    ros2 action send_goal <action_name> <action_type> '<values>'
    # Exemple - demander une rotation de 90 degrés (1.57 radians) :
    ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
    ```
    *Pour voir le **feedback** en temps réel, ajoutez l'option `--feedback` à la commande.*

---
title: Utiliser un espace de travail overlay avec Colcon
---

## Sourcer et tester son overlay

Après avoir construit vos packages avec `colcon build`, vous devez activer cet environnement dans votre terminal pour qu'il prenne le pas sur l'installation système de ROS.

### Activation de l'overlay

7.  **Sourcer le fichier de configuration local** depuis le répertoire `install/` de votre espace de travail :
    ```bash
    # Depuis ~/ros2_ws
    source install/local_setup.bash
    ```
    **Pourquoi `local_setup.bash` ?** Il configure l'environnement **uniquement** pour les packages de votre espace de travail courant, en priorité sur ceux du système.

---
## Vérification et modification du package

8.  **Exécuter le nœud depuis votre version construite** (overlay) :
    ```bash
    ros2 run turtlesim turtlesim_node
    ```
    La fenêtre du simulateur devrait s'ouvrir normalement. Cela prouve que votre overlay fonctionne.

9.  **Modifier le code source pour personnaliser** et prouver que vous utilisez bien votre version, et non celle du système.
    - Ouvrez le fichier source principal du simulateur (`turtle_frame.cpp`) :
        ```bash
        # Depuis ~/ros2_ws
        code src/ros_tutorials/turtlesim/src/turtle_frame.cpp
        ```
    - **Allez à la ligne ~52** et modifiez le titre de la fenêtre :
        ```cpp
        // Remplacer :
        setWindowTitle(QString::fromStdString("TurtleSim"));
        // Par exemple :
        setWindowTitle(QString::fromStdString("KevinTurtleSim"));
        ```

---
## Reconstruire et voir les changements

10. **Reconstruire uniquement le package modifié** (`turtlesim`) :
    ```bash
    # Depuis ~/ros2_ws
    colcon build --symlink-install --packages-select turtlesim
    ```
    L'option `--packages-select` accélère la construction en ne traitant que le package spécifié.

11. **Redémarrer le simulateur** (fermez l'ancienne fenêtre) :
    ```bash
    ros2 run turtlesim turtlesim_node
    ```
    La fenêtre devrait maintenant afficher votre titre personnalisé (**"KevinTurtleSim"**). Cela confirme que vous travaillez bien avec votre propre version construite localement.

**Important :** Avec `--symlink-install`, les modifications dans `src/` sont immédiatement reflétées après une reconstruction. Sans cela, il faudrait une recopie complète des fichiers.