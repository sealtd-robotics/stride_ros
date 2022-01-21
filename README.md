# STRIDE

Table of Content
---
I. What is Stride ROS?
II. How to run?
III. Directory Structure

---
I. What is Stride ROS?
----

II. How to run?
----

For developers:

Step 1: Lauch All Nodes
```bash
roslaunch top_level top_level.launch 
```

Step 2: Start/Stop Robot
- Headless/no gui: ssh and overwrite `custom_script/script1.py` 
- Publishes `Empty` message to `/gui/start_path_following_clicked` to simulate path following (ref: `overseer/overseer.py`)
- Previous step will take care of this: Publish value 2 for `Int32` to `/overseer/state` to simulate the __AUTO__ command (ref: `robot_commander/robot_commander.py` `overseer/overseer.py`)
- If robot goes to end of path, Jetson will put a __STOP__ automatically to the path following.
- If for active __STOP__, publish `Empty` to `/gui/stop_clicked`.

Step 3: Scripting Rules



III. Directory Structure
----
```
.
├── bash
└── stride_ws
    ├── custom_script
    ├── gps_debug_log
    ├── path
    └── src
        ├── advanced_navigation_driver
        ├── can_interface
        ├── drive_mechanism
        ├── external_interface
        ├── joystick
        ├── networking
        ├── overseer
        ├── path_follower
        ├── robot_commander
        └── top_level
```