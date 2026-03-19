# 🐳 Lite3 Robot Docker Workspace

This guide explains how to run the Lite3 quadruped robot simulation using the provided Docker environment.

## 🚀 Quick Start (Host Machine)

**1. Allow GUI connections:**
```bash
xhost +local:docker
```

**2. Build and Start the Container:**
```bash
docker compose build
docker compose up -d
```

## 🎮 Running the Simulation (4 Terminals)

You need **4 terminals** to fully operate the simulation. Enter the container in each terminal using:
`docker compose exec quadruped_worker bash`

### Terminal 1: Gazebo Environment
Start the physical world simulation:
```bash
source devel/setup.bash
roslaunch qr_gazebo gazebo_startup.launch wname:=earth
```

### Terminal 2: Spawn Lite3 Robot
Spawn the robot model into the world:
```bash
source devel/setup.bash
roslaunch qr_gazebo model_spawn.launch rname:=lite3 use_xacro:=true
```

### Terminal 3: Simulation Controller (MPC/WBC)
Start the brain/logic of the robot:
```bash
source devel/setup.bash
rosrun examples example_lite3_sim
```
*Wait until you see `start control loop....`*

### Terminal 4: Keyboard Teleop
Control the robot with your keyboard:
```bash
source devel/setup.bash
rosrun examples example_keyboard
```

---

## ⌨️ Control Keys

| Key | Action |
| :--- | :--- |
| **U** | **Stand Up / Sit Down** (Toggle) |
| **K** | Switch Control Mode (Stand ↔ Trot) |
| **J** | Change Gait (Trot ↔ Walk) |
| **W, A, S, D** | Move Forward, Left, Backward, Right |
| **Q, E** | Rotate Left, Right |
| **L** | Stop Movement |

## 🛠 Troubleshooting
- **Black Screen / No Gazebo**: Ensure `xhost +local:docker` was run on the host.
- **Robot Falls**: Use `Ctrl+C` in Terminal 3 to restart the controller.
- **No Effect**: Ensure Terminal 3 (Controller) is running and says `Joy Received`.
