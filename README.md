
# Autonomous-Navigation-of-a-Mobile-Robot

## Section 1: Mapping

As you've learned in the first 2 chapters of the course, the first thing you need in order to navigate autonomously with a robot is a map of the environment.

### Goal

Map the environment around the robot and save the map for localization and navigation.

---

### 🧭 Steps

1. **Visualize the TF Frame Tree**
   - Ensure that the TurtleBot3 is publishing correct TF data.

2. **Create the Package**
   - Create a package called `project_mapping`.

3. **Launch Cartographer Node**
   - Create a launch file for the `cartographer_node`.
   - Set correct parameters for `slam_gmapping`.

4. **Create the Map**
   - Launch the node using the launch file.
   - Use keyboard teleop to drive the robot:
     ```bash
     ros2 run teleop_twist_keyboard teleop_twist_keyboard
     ```

   > ⚠️ Terminal with teleop must be in focus.

   - Launch `rviz2` and configure mapping display.

   > **Tip:** If mapping is poor, increase `TRAJECTORY_BUILDER_2D.max_range` to 3.5 in `.lua` config.

5. **Save the Map**
   - Create a `maps/` folder in your package and save the `.pgm` and `.yaml` files there.

6. **Map Server Launch File**
   - Create a launch file that provides the map via `nav2_map_server`.

---

## Section 2: Localization

### Goal

Use AMCL to localize the robot within the map.

---

### 🔧 Steps

1. **Create the Package**
   - Create `project_localization`.

2. **Create AMCL Launch File**
   - Include the map server from Section 1.
   - Add parameters needed for AMCL configuration.

3. **Run and Verify**
   - Launch your node.
   - Open `rviz2` and confirm localization via particle cloud.

   > ✔️ Dispersed → Grouped particles = Good localization

4. **Create Spots Table**
   - Define 3 tagged spots based on `/amcl_pose` topic.
   - Save to `spots.yaml`.

5. **Create Spot Recorder Service**

### 🧩 Spot Recorder Requirements

- Node name: `spot_recorder`
- Service: `/save_spot`
- Takes a string (label) as input
- Saves current pose into `spots.txt` upon receiving `"end"` as label

---

### ✍️ Service Definition

```srv
# request
string label
---
# response
bool navigation_successfull
string message
```

---

### 💻 Service Usage

- Move robot to each location with teleop.
- Use:
  ```bash
  ros2 service call /record_spot "label: corner1"
  ```
- Repeat for each spot.
- Call with `"end"` to save all data.

---

## Section 3: Path Planning System

### Goal

Use Nav2 to plan and execute autonomous navigation.

---

### 🛠 Steps

1. **Create the Package**
   - Name: `project_path_planning`

2. **Create Launch File**
   - Include:
     - `controller_server`
     - `planner_server`
     - `behavior_server`
     - `bt_navigator`

3. **Add Parameters**
   - Create `project_params.yaml`
   - Configure all 4 Nav2 nodes

4. **Run and Test**
   - Use `rviz2` to send goals and verify motion

---

## Section 4: Interacting with Nav2

### Goal

Create programs that send commands to Nav2.

---

### 🚩 Steps

1. **Create Spot Recorder Script**

- Subscribe to `/initialpose` from `rviz2`
- Save location with label to YAML file

Spots to register:
- `corner1`
- `corner2`
- `pedestrian`

> Start only `map_server` and `rviz2`  
> Use "2D Pose Estimate" to mark spots

---

2. **Format the Parameter File**

Transform your raw YAML into proper ROS2 format:

```yaml
move_to_spot:
  ros__parameters:
    spot_name: "corner1"
```

---

3. **Create a Navigation Script**

- Read spot from `spot-list.yaml`
- Use `NavigateToPose` action to send goal

---

### 🏁 Run the Script

```bash
ros2 run nav2_project go_to_pose --ros-args --params-file spot-list.yaml -p spot_name:=corner1
```

---

> ✅ Once working in simulation, test on the Real Robot Lab.
