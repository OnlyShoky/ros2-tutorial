# Module 12: Sensors & Camera 📷

Robots need to see! This module covers adding **camera sensors** to your robot for vision capabilities.

---

## 🎯 What You Will Learn

- Add a camera sensor to your robot URDF
- Configure Gazebo camera plugins
- View camera images in RViz2
- Subscribe to camera topics in Python
- Understand common sensor types in ROS 2

---

## 📸 Camera Sensor Overview

A camera in ROS 2 publishes:

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Raw image data |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Calibration data |

---

## 1️⃣ Add Camera to URDF

### Create Camera Link

Add to `urdf/sensors.xacro`:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Camera Properties -->
  <xacro:property name="camera_size" value="0.03"/>
  <xacro:property name="camera_mass" value="0.1"/>

  <!-- Camera Link -->
  <link name="camera_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${camera_size} ${camera_size * 1.5} ${camera_size}"/>
      </geometry>
      <material name="black"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${camera_size} ${camera_size * 1.5} ${camera_size}"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0"/>
      <mass value="${camera_mass}"/>
      <inertia ixx="0.00001" ixy="0" ixz="0" 
               iyy="0.00001" iyz="0" 
               izz="0.00001"/>
    </inertial>
  </link>

  <!-- Camera Optical Frame (standard convention) -->
  <link name="camera_optical_frame"/>

  <!-- Attach Camera to End Effector -->
  <joint name="camera_joint" type="fixed">
    <parent link="end_effector_link"/>
    <child link="camera_link"/>
    <origin xyz="0 0 0.02" rpy="0 ${pi/2} 0"/>
  </joint>

  <!-- Optical Frame Joint (rotated for image convention) -->
  <joint name="camera_optical_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_optical_frame"/>
    <origin xyz="0 0 0" rpy="${-pi/2} 0 ${-pi/2}"/>
  </joint>

  <!-- Gazebo Camera Material -->
  <gazebo reference="camera_link">
    <material>Gazebo/Black</material>
  </gazebo>

</robot>
```

---

## 2️⃣ Configure Gazebo Camera Plugin

Add to `urdf/sensors.xacro`:

```xml
  <!-- Gazebo Camera Sensor -->
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      
      <!-- Update rate in Hz -->
      <update_rate>30.0</update_rate>
      
      <!-- Show camera frustum in Gazebo -->
      <visualize>true</visualize>
      
      <camera name="end_effector_camera">
        <!-- Image resolution -->
        <horizontal_fov>1.39</horizontal_fov>  <!-- ~80 degrees -->
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        
        <!-- Clipping planes -->
        <clip>
          <near>0.02</near>
          <far>10.0</far>
        </clip>
        
        <!-- Noise (optional, for realism) -->
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      
      <!-- ROS 2 Plugin -->
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/arm</namespace>
          <remapping>~/image_raw:=camera/image_raw</remapping>
          <remapping>~/camera_info:=camera/camera_info</remapping>
        </ros>
        <camera_name>end_effector_camera</camera_name>
        <frame_name>camera_optical_frame</frame_name>
      </plugin>
      
    </sensor>
  </gazebo>
```

---

## 3️⃣ Include Sensors in Main File

Update `simple_arm_gazebo.urdf.xacro`:

```xml
<!-- Add near other includes -->
<xacro:include filename="$(find simple_arm_description)/urdf/sensors.xacro"/>
```

---

## 4️⃣ Build and Launch

```bash
cd ~/ros2_ws
colcon build --packages-select simple_arm_description
source install/setup.bash
ros2 launch simple_arm_description gazebo.launch.py
```

---

## 5️⃣ Verify Camera Topics

```bash
# List camera topics
ros2 topic list | grep camera

# Expected:
# /arm/camera/camera_info
# /arm/camera/image_raw

# Check image publish rate
ros2 topic hz /arm/camera/image_raw

# View camera info
ros2 topic echo /arm/camera/camera_info --once
```

---

## 6️⃣ View Camera in RViz2

1. Open RViz2: `rviz2`
2. Click **Add** → **By topic** → `/arm/camera/image_raw` → **Image**
3. Or add **Camera** display for 3D overlay

### RViz2 Camera Configuration

| Setting | Value |
|---------|-------|
| Image Topic | `/arm/camera/image_raw` |
| Transport | raw |

---

## 7️⃣ Subscribe to Camera in Python

Create `simple_arm_control/camera_subscriber.py`:

```python
#!/usr/bin/env python3
"""
Camera Image Subscriber
Receives and processes images from robot arm camera
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraSubscriber(Node):
    
    def __init__(self):
        super().__init__('camera_subscriber')
        
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            '/arm/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.get_logger().info('Camera Subscriber started!')
        self.get_logger().info('Listening to /arm/camera/image_raw')

    def image_callback(self, msg):
        """Process received camera image."""
        
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Get image dimensions
            height, width = cv_image.shape[:2]
            self.get_logger().info(f'Received image: {width}x{height}')
            
            # Optional: Display image (requires display)
            cv2.imshow('Robot Arm Camera', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Install Dependencies

```bash
sudo apt install ros-jazzy-cv-bridge python3-opencv -y
```

### Add Entry Point

In `setup.py`:

```python
'camera_subscriber = simple_arm_control.camera_subscriber:main',
```

---

## 8️⃣ Other Common Sensors

### LiDAR Sensor

```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14</min_angle>
          <max_angle>3.14</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Sensor

```xml
<gazebo reference="imu_link">
  <sensor name="imu" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <remapping>~/out:=imu/data</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

---

## 🚨 Common Troubleshooting

### No Image Published

**Problem:** Camera topic exists but no messages
**Solution:**
- Check Gazebo is running and not paused
- Verify plugin is loaded: look for "camera_controller" in Gazebo logs

### Black Image

**Problem:** Image shows all black
**Solution:**
- Check camera is facing the scene
- Adjust clipping planes (near/far)
- Verify lighting in world file

### cv_bridge Import Error

**Problem:** `No module named 'cv_bridge'`
**Solution:**
```bash
sudo apt install ros-jazzy-cv-bridge -y
```

---

## 📁 Module File Structure

After completing this module:

```
~/ros2_ws/
└── src/
    ├── simple_arm_description/
    │   └── urdf/
    │       ├── sensors.xacro               # NEW
    │       └── simple_arm_gazebo.urdf.xacro # Updated
    │
    └── simple_arm_control/
        └── simple_arm_control/
            └── camera_subscriber.py        # NEW
```

---

## ✅ Summary

In this module, you learned:

- Add **camera sensors** to URDF with proper optical frame
- Configure **Gazebo camera plugins** for image publishing
- View camera images in **RViz2**
- **Subscribe to images** in Python using cv_bridge
- Other sensor types: **LiDAR** and **IMU**

**Next:** In Module 13, we'll master **RViz2** for visualization and debugging - the final module!
