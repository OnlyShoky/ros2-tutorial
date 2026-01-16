
## 📌 1. Introduction à PlotJuggler
PlotJuggler est un **visualiseur de données série temporelle** avancé et open-source. Dans l'écosystème ROS, il est particulièrement utile pour :
- **Analyser les messages ROS** en temps réel (états de joints, données de capteurs, commandes)
- **Déboguer des algorithmes** en visualisant plusieurs signaux simultanément
- **Comparer des données** entre différentes expériences/exécutions

## 📦 2. Installation de PlotJuggler

### Méthode recommandée : Installation via les paquets ROS
```bash
# Remplace ${ROS_DISTRO} par votre distribution (humble, iron, etc.)
sudo apt install ros-humble-plotjuggler
sudo apt install ros-humble-plotjuggler-ros
```

### Vérification de l'installation
```bash
# Vérifiez que les paquets sont bien installés
ros2 pkg list | grep plotjuggler

# Vérifiez la version installée
apt show ros-humble-plotjuggler
```

## 🚀 3. Exécution de PlotJuggler

### Méthode 1 : Lancer depuis le terminal (démarrage rapide)
```bash
# Méthode la plus simple pour tester
ros2 run plotjuggler plotjuggler
```

### Méthode 2 : Intégration dans un fichier launch (recommandé pour des projets)
```python
# Dans votre fichier launch Python (.launch.py)
from launch.actions import ExecuteProcess

# Définir l'exécution de PlotJuggler
run_plotjuggler = ExecuteProcess(
    cmd=['ros2', 'run', 'plotjuggler', 'plotjuggler'],
    output='screen',
    shell=True
)

# Ajouter à votre LaunchDescription
def generate_launch_description():
    return LaunchDescription([
        run_plotjuggler,
        # ... autres nœuds
    ])
```

## 🤖 4. Démonstration Pratique : Visualisation des Trajectoires de Joints

### Étape 1 : Lancer la Simulation Tesla Bot
```bash
# Terminal 1 - Lancer Gazebo avec la simulation
cd ~/ros2_ws
source install/setup.bash
ros2 launch gazebo_tutorial gazebo.launch.py
```

### Étape 2 : Lancer le Publisher de Joints
```bash
# Terminal 2 - Lancer le nœud qui publie les états de joints
cd ~/ros2_ws
source install/setup.bash
ros2 run gazebo_tutorial joint_publisher
```

### Étape 3 : Lancer PlotJuggler et Configurer la Visualisation
```bash
# Terminal 3 - Lancer PlotJuggler
ros2 run plotjuggler plotjuggler
```

## 📊 5. Guide d'Utilisation de PlotJuggler

### Connexion aux Données ROS
1. Dans PlotJuggler, cliquez sur **"Streaming"** dans la barre d'outils
2. Sélectionnez **"ROS 2 Topic Subscriber"**
3. Configurez les paramètres :
   - **Domain ID** : Généralement 0 (par défaut dans ROS 2)
   - **Topics** : Sélectionnez les topics à visualiser (ex: `/joint_states`)

### Tracer un Signal
1. **Glissez-déposez** un topic depuis la liste à gauche vers la zone de graphique
2. **Sélectionnez les champs** spécifiques à tracer (ex: `position[0]` pour le premier joint)
3. **Ajustez l'échelle** avec la molette de la souris ou les contrôles de zoom

### Exemple de Configuration pour les Joints de Coude
```xml
<!-- Exemple de layout XML pour les joints de coude -->
<PlotJugglerLayout>
  <curves>
    <curve topic="/joint_states" field="position[2]" label="Elbow_Left"/>
    <curve topic="/joint_states" field="position[3]" label="Elbow_Right"/>
  </curves>
  <time_axis range="10.0"/> <!-- Affiche les 10 dernières secondes -->
</PlotJuggilerLayout>
```

## 💾 6. Sauvegarde et Chargement des Configurations

### Sauvegarder une Disposition
1. Arrangez vos graphiques comme souhaité
2. **File → Save Layout As...**
3. Choisissez un nom (ex: `elbow_trajectories.xml`)
4. Le fichier contient toute la configuration : topics, courbes, échelles, couleurs

### Charger une Disposition Existante
```bash
# Méthode 1 : Depuis l'interface graphique
# File → Load Layout...

# Méthode 2 : En ligne de commande (utile pour l'automatisation)
ros2 run plotjuggler plotjuggler -l /chemin/vers/elbow_trajectories.xml
```

### Format des Fichiers de Layout
PlotJuggler supporte deux formats :
- **XML** : Format par défaut, lisible et éditable
- **JSON** : Alternative pour l'intégration avec d'autres outils

## 🛠️ 7. Fonctionnalités Avancées

### Filtrage et Transformations de Données
1. **Filtres mathématiques** : Appliquez des transformations (dérivée, intégrale, moyenne mobile)
2. **Transformations statistiques** : Calcul de min/max/moyenne sur une fenêtre glissante
3. **Annotations** : Ajoutez des marqueurs sur des événements spécifiques

### Scripting avec Lua
```lua
-- Exemple de script Lua pour un traitement personnalisé
function onNewData(data)
    -- Calculer la vitesse à partir de la position
    local velocity = (data.position - previous_position) / dt
    previous_position = data.position
    return {velocity = velocity}
end
```

## 🔧 8. Dépannage Courant

### Problème : PlotJuggler ne voit pas les topics ROS
```bash
# Solution 1 : Vérifier que ROS_DOMAIN_ID est cohérent
echo $ROS_DOMAIN_ID

# Solution 2 : Redémarrer le démon ROS
ros2 daemon stop
ros2 daemon start

# Solution 3 : Vérifier la connexion réseau
ros2 topic list
```

### Problème : Données qui n'apparaissent pas
1. Vérifiez que le **nœud source publie effectivement** (`ros2 topic echo /joint_states`)
2. Confirmez que **PlotJuggler est abonné** au bon topic
3. Vérifiez les **paramètres QoS** (fiabilité, durée)

## 📈 9. Bonnes Pratiques

### Pour la Visualisation en Temps Réel
- **Limitez le nombre de courbes** visibles simultanément (max 5-6 pour une bonne lisibilité)
- **Utilisez des couleurs distinctes** pour chaque signal
- **Ajustez la fréquence d'échantillonnage** selon vos besoins

### Pour l'Analyse Post-Process
- **Enregistrez les données brutes** avec `ros2 bag`
- **Utilisez des layouts prédéfinis** pour des analyses reproductibles
- **Exportez les données** vers CSV/JSON pour analyse externe

## 🎯 10. Cas d'Usage : Analyse des Trajectoires de Joints

### Configuration Optimale pour les Joints
```bash
# Commandes pour une analyse complète
ros2 run plotjuggler plotjuggler --layout ~/layouts/joint_analysis.xml \
  --subscribe /joint_states \
  --subscribe /joint_commands \
  --buffer_size 10000
```

### Métriques à Surveiller
1. **Précision du suivi** : Position désirée vs position réelle
2. **Vélocité et accélération** : Dérivées de la position
3. **Efforts/forces** : Données de couple si disponibles
4. **Latence** : Délai entre commande et exécution

---

## 📝 Résumé des Commandes Essentielles

```bash
# Installation
sudo apt install ros-humble-plotjuggler ros-humble-plotjuggler-ros

# Lancement de base
ros2 run plotjuggler plotjuggler

# Lancement avec layout prédéfini
ros2 run plotjuggler plotjuggler -l /chemin/vers/layout.xml

# Vérification des topics disponibles
ros2 topic list
```

Ce tutoriel vous permet de maîtriser PlotJuggler pour visualiser et analyser efficacement les données de vos systèmes robotiques ROS 2. La capacité à sauvegarder et recharger des layouts est particulièrement utile pour des analyses reproductibles et des comparaisons entre différentes expériences.