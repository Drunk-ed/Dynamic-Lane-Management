
# Dynamic Lane Management

A simulation-based Dynamic Lane Management system using a cloverleaf highway environment, SUMO traffic simulation, movable lane-divider concepts, and autonomous TurtleBot3-based divider robots.

The current implementation focuses on the autonomous divider navigation layer using an adapted Dynamic Window Approach (DWA) local planner in ROS 2 Humble.

---

## Project Overview

The objective of this project is to demonstrate dynamic lane-management concepts in a simulated highway environment.

The system combines:

- Modified cloverleaf/highway environment in Gazebo
- SUMO-based traffic simulation and vehicle synchronization
- Movable lane-divider elements
- TurtleBot3-based autonomous divider robots
- DWA-based local navigation
- Namespace-based multi-robot operation

The current navigation architecture is intentionally kept simple and uses:

```text
Current Robot Position
        +
Goal Position
        +
2D LiDAR
        ↓
      DWA
        ↓
   cmd_vel
        ↓
   TurtleBot3
````

The navigation layer does not depend on a pre-built map or AMCL-based localization.

---

## Current Status

### Completed

* Highway/cloverleaf environment adapted for the project
* Gazebo simulation environment integrated
* SUMO traffic simulation integrated with Gazebo
* Coordinate transformation and vehicle-position synchronization implemented
* Multiple vehicle types incorporated
* Lane-changing and overtaking behaviour implemented
* Movable lane-divider concept implemented
* Lane closure and lane-evacuation mechanisms implemented
* Lane-state indicators implemented
* TurtleBot3 Burger integrated as the autonomous divider platform
* Existing ROS 2 Humble DWA planner adapted for the project
* DWA topic interfaces modified to support ROS 2 namespaces
* Goal-based local navigation verified
* `divider_01` and `divider_02` configured with independent DWA planner instances
* Both divider robots independently verified for local navigation

### Current Limitation

The available computing hardware limits the practical simulation load when multiple robots, LiDAR sensors, Gazebo physics, and traffic simulation are running simultaneously.

The current implementation therefore focuses on the verified two-robot configuration rather than large-scale multi-robot simulation.

Higher-level coordination between traffic conditions, lane-management decisions, and autonomous divider goals is the next integration stage.

---

## Repository Structure

```text
Dynamic-Lane-Management/
│
├── custom_dwa_planner_cpp/
│   ├── include/
│   ├── src/
│   ├── CMakeLists.txt
│   └── package.xml
│
├── lane_bringup/
│   ├── launch/
│   ├── CMakeLists.txt
│   └── package.xml
│
├── simulation/
│   ├── worlds/
│   │   └── highway_only.world
│   │
│   └── models/
│       └── cloverleaf_interchange/
│           ├── materials/
│           │   └── textures/
│           ├── meshes/
│           ├── model.config
│           └── model.sdf
│
└── README.md
```

---

## Software Requirements

* Ubuntu
* ROS 2 Humble
* Gazebo
* TurtleBot3 packages
* C++ compiler
* `colcon`

Set the TurtleBot3 model before launching the robot:

```bash
export TURTLEBOT3_MODEL=burger
```

---

## Building the Workspace

Clone the repository:

```bash
git clone https://github.com/Drunk-ed/Dynamic-Lane-Management.git
cd Dynamic-Lane-Management
```

Build the ROS 2 packages:

```bash
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

---

## Cloverleaf Simulation

The project uses the modified cloverleaf highway environment stored under:

```text
simulation/worlds/highway_only.world
```

The cloverleaf model is stored under:

```text
simulation/models/cloverleaf_interchange/
```

The world uses the following model resource:

```text
model://cloverleaf_interchange/meshes/cloverleaf.obj
```

The required model mesh, material, texture, `model.sdf`, and `model.config` files are included with the project.

---

## TurtleBot3 Divider Robot

A divider robot is spawned using the `spawn_divider.launch.py` launch file.

Example:

```bash
export TURTLEBOT3_MODEL=burger

ros2 launch lane_bringup spawn_divider.launch.py \
    entity:=divider_01 \
    x:=0.0 \
    y:=0.0
```

The robot operates under the namespace:

```text
/divider_01
```

Typical interfaces are:

```text
/divider_01/odom
/divider_01/scan
/divider_01/cmd_vel
```

---

## DWA Local Planner

The project uses an existing ROS 2 Humble DWA implementation as the local-navigation foundation.

The original implementation was adapted to use relative topic names:

```text
/odom    → odom
/scan    → scan
/cmd_vel → cmd_vel
```

This allows the same planner to run independently under different robot namespaces.

### Divider 1

Run:

```bash
ros2 run custom_dwa_planner_cpp dwa_planner \
    --ros-args \
    -r __ns:=/divider_01 \
    -p goal_x:=4.0 \
    -p goal_y:=0.0
```

### Divider 2

Run:

```bash
ros2 run custom_dwa_planner_cpp dwa_planner \
    --ros-args \
    -r __ns:=/divider_02 \
    -p goal_x:=4.0 \
    -p goal_y:=0.0
```

Each planner instance operates on the corresponding robot namespace.

---

## Multi-Robot Operation

The current system has been verified with two independent divider robots:

```text
/divider_01
/divider_02
```

Each robot has its own:

```text
odom
scan
cmd_vel
```

interfaces and its own DWA planner instance.

The namespace-based architecture prevents topic conflicts between the robots and allows the same DWA implementation to be reused for multiple divider robots.

A common launch configuration and simultaneous goal assignment can be added as the project develops further.

---

## Navigation Data Flow

For each divider robot:

```text
Odometry
    +
2D LiDAR
    +
Goal Coordinates
    ↓
Dynamic Window Approach
    ↓
Velocity Command
    ↓
TurtleBot3
```

For `divider_01`:

```text
/divider_01/odom
/divider_01/scan
        ↓
   DWA Planner
        ↓
/divider_01/cmd_vel
```

For `divider_02`:

```text
/divider_02/odom
/divider_02/scan
        ↓
   DWA Planner
        ↓
/divider_02/cmd_vel
```

---

## Simulation Performance

The complete simulation combines:

* Gazebo physics
* Cloverleaf highway environment
* SUMO traffic
* Vehicle synchronization
* LiDAR sensors
* TurtleBot3 robots
* Multiple DWA planner instances

Increasing the number of simultaneously simulated robots increases the computational load.

The current hardware was sufficient for configuring and independently testing two divider robots, but larger-scale simultaneous testing was limited by available computational resources.

The current milestone therefore focuses on a verified two-robot configuration.

---

## DWA Attribution

The DWA local planner used in this project is adapted from:

**Ashwin Sivakumar**

**DWA Local Planner in ROS 2 Humble**

Original repository:

[https://github.com/ashwinsivakumar-18/DWA-Local-Planner-in-ROS2-Humble](https://github.com/ashwinsivakumar-18/DWA-Local-Planner-in-ROS2-Humble)

The original implementation was adapted for:

* ROS 2 namespace-based operation
* Multiple TurtleBot3 divider robots
* Relative topic interfaces
* Project-specific goal-based navigation
* Integration with the Dynamic Lane Management simulation

The original license and attribution should be retained with the adapted code.

---

## Cloverleaf Model Attribution

The cloverleaf interchange model included in the simulation contains the original model attribution provided in its `model.config`.

The included `model.config` file is retained with the model.

---

## Project Status

The current milestone establishes:

```text
Cloverleaf Simulation
        ↓
Traffic Simulation
        ↓
Movable Divider
        ↓
TurtleBot3 Divider
        ↓
DWA Local Navigation
        ↓
Two Independent Divider Robots
```

The next development stage is the integration of higher-level coordination between traffic conditions, lane-management decisions, and autonomous divider movement.


