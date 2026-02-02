# Module 06: Actions - Long Running Tasks 🎬

In Module 05, you used **Services** to open a gripper—a quick, instant operation.
But what if you want to **move the robot arm** to a new position? That takes time (e.g., 5 seconds).

If you use a Service, your program will **freeze** for 5 seconds waiting for the response. You won't know if it's working or stuck.
That's why we use **Actions**!

---

## 🎯 What You Will Learn

- Understand the **Action** communication pattern (Goal, Feedback, Result)
- Create custom **Action Definitions** (`.action` files)
- Build an **Action Server** that provides feedback during execution
- Build an **Action Client** that can cancel goals
- Implement a **Move Arm** action for our project

---

## 🎬 Services vs Actions

| Feature | Services | Actions |
|---------|----------|---------|
| **Duration** | Quick / Instant | Long-running |
| **Feedback** | None (Wait...) | Yes (Progress updates) |
| **Cancelable** | No | Yes (Can stop mid-task) |
| **Logic** | "Do this now" | "Start this, keep me updated" |

```
ACTION (Complex Interaction):
┌────────────┐     Goal      ┌────────────┐
│            │ ────────────► │            │
│            │   Feedback    │            │
│  Client    │ ◄──────────── │   Server   │
│            │    Result     │            │
│            │ ◄──────────── │            │
└────────────┘               └────────────┘
```

---

## 1️⃣ Define a Custom Action

We need an action to move the arm. It should:
1. **Goal:** Target position (0-100%)
2. **Result:** Final message ("Arrived")
3. **Feedback:** Current position (progress)

### Create the Action File

1. Create the directory:
```bash
mkdir -p ~/ros2_ws/src/simple_arm_interfaces/action
```

2. Create `action/MoveArm.action`:
```bash
cat > ~/ros2_ws/src/simple_arm_interfaces/action/MoveArm.action << 'EOF'
# Goal
float32 target_position   # 0.0 to 1.0 (0% to 100%)
---
# Result
bool success
string message
---
# Feedback
float32 current_position
EOF
```

### Configure CMakeLists.txt

Edit `~/ros2_ws/src/simple_arm_interfaces/CMakeLists.txt`:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GripperControl.srv"
  "action/MoveArm.action"   # Add this line!
)
```

### Configure package.xml

Add the action dependency to `~/ros2_ws/src/simple_arm_interfaces/package.xml`:

```xml
<member_of_group>rosidl_interface_packages</member_of_group>
```
*(Already added in Module 05, just verify!)*

### Build the Interface

```bash
cd ~/ros2_ws
colcon build --packages-select simple_arm_interfaces
source install/setup.bash
```

---

## 2️⃣ Create the Create Action Server

The **Action Server** executes the long-running task. It will simulate moving the arm by counting up to the target.

Create `~/ros2_ws/src/simple_arm_control/simple_arm_control/arm_action_server.py`:

```python
#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from simple_arm_interfaces.action import MoveArm

class ArmActionServer(Node):
    def __init__(self):
        super().__init__('arm_action_server')
        
        self._action_server = ActionServer(
            self,
            MoveArm,
            'move_arm',
            self.execute_callback
        )
        self.get_logger().info('Arm Action Server Ready...')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        # Get target from goal
        target = goal_handle.request.target_position
        feedback_msg = MoveArm.Feedback()
        
        # Simulate movement (loop 0 -> target)
        current_pos = 0.0
        step = 0.1
        
        while current_pos < target:
            # check if cancel requested (optional advanced step)
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return MoveArm.Result(success=False, message="Canceled")

            # Update position
            current_pos += step
            feedback_msg.current_position = current_pos
            
            # Publish feedback
            self.get_logger().info(f'Feedback: {current_pos:.1f}')
            goal_handle.publish_feedback(feedback_msg)
            
            # Simulate hardware delay
            time.sleep(0.5)

        # Success!
        goal_handle.succeed()
        
        result = MoveArm.Result()
        result.success = True
        result.message = "Arm arrived at target!"
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ArmActionServer()
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

## 3️⃣ Create the Action Client

The **Action Client** sends the goal and listens for updates.

Create `~/ros2_ws/src/simple_arm_control/simple_arm_control/arm_action_client.py`:

```python
#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from simple_arm_interfaces.action import MoveArm

class ArmActionClient(Node):
    def __init__(self):
        super().__init__('arm_action_client')
        self._action_client = ActionClient(self, MoveArm, 'move_arm')

    def send_goal(self, target):
        goal_msg = MoveArm.Goal()
        goal_msg.target_position = float(target)

        self.get_logger().info('Waiting for server...')
        self._action_client.wait_for_server()

        self.get_logger().info(f'Sending goal: move to {target}...')
        
        # Send goal and register callback for response
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.message}')
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        pos = feedback_msg.feedback.current_position
        self.get_logger().info(f'>> Received Feedback: {pos:.1f}')

def main(args=None):
    rclpy.init(args=args)
    action_client = ArmActionClient()
    
    target = sys.argv[1] if len(sys.argv) > 1 else "1.0"
    action_client.send_goal(target)
    
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

---

## 4️⃣ Register Nodes & Build

Update `setup.py` entry points in `simple_arm_control`:

```python
    entry_points={
        'console_scripts': [
            'gripper_server = simple_arm_control.gripper_server:main',
            'gripper_client = simple_arm_control.gripper_client:main',
            'arm_action_server = simple_arm_control.arm_action_server:main',
            'arm_action_client = simple_arm_control.arm_action_client:main',
        ],
    },
```

Build:

```bash
cd ~/ros2_ws
colcon build --packages-select simple_arm_control
source install/setup.bash
```

---

## 5️⃣ Test the Action

**Terminal 1:** Start Server
```bash
ros2 run simple_arm_control arm_action_server
```

**Terminal 2:** Start Client
```bash
source ~/ros2_ws/install/setup.bash
ros2 run simple_arm_control arm_action_client 2.5
```

Observation:
- Client sends "2.5"
- Server accepts
- You see `>> Received Feedback: 0.1` ... `0.2` ... updates every 0.5s
- Finally `Result: Arm arrived at target!`

**Terminal 3:** Test Cancellation (Advanced)
Run the client, then press `Ctrl+C` on the CLIENT. The Action continues on the server (unless we implement sophisticated cancel logic). Or try sending a new goal!

---

## 6️⃣ Action CLI Tools

You can send goals from the terminal too!

```bash
ros2 action send_goal /move_arm simple_arm_interfaces/action/MoveArm "{target_position: 1.5}" --feedback
```

- `send_goal`: Send a goal
- `--feedback`: Print feedback to screen

---

## ✅ Summary

In this module, you:
- Defined a custom **Action** (`MoveArm.action`) with Goal, Result, and Feedback
- Built an **Action Server** that publishes feedback while running
- Built an **Action Client** that handles asynchronous updates
- Learned why **Actions** are superior for long tasks compared to Services

**Next:** In Module 07, we'll stop running nodes manually in separate terminals and learn to use **Launch Files** to start the whole system at once!
