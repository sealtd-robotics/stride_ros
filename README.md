# STRIDE

## Table of Content

I. Build Workspace
II. What to run?
III. Collect data
IV. Directory Structure

---

## I. Build Workspace

```
cd ~
git clone https://github.com/sealtd-robotics/stride_ros.git
cd ~/stride_ros/

# if first time cloning the folder
git submodule update --init --recursive

## OR if anything goes wrong and the git submodule command above does not work
rm -rf stride_ws/src/oxford_gps_decoder
rm -rf stride_ws/src/sbg_ros_driver
git submodule init
git submodule update

## Install dependencies, should need to do this only once on the same machine
./bash/dep_install.sh

## Build workspace
./bash/build_ws.sh
```

If submodule is updated, pull latest changes in the submodules:

```
cd ~/stride_ros
git submodule update --recursive
```

For developer, pull the latest commit of the submodule into project

```
cd ~/stride_ros
git submodule update --remote --merge
git add stride_ros/<submodule_name>
git commit -m "content of your commit"
git push
```

## II. How to run on target robot?

1. The manual approach

   Spin up the webserver first to get access to gui:

   ```
   python3 web_server_for_gui.py
   ```

   Then the web gui can be accessed with `ip_address:3000` from browser of pc connect to the network. `ip_address` is the address of machine runs the web server.

   Then, launch ALL nodes

   ```bash
   roslaunch top_level top_level.launch
   ```

2. The Auto approach

   Enable the systemd services to take care of the steps above. So everytime the robot is booted up, it will automatically spin up webserver, set up can bus and roslaunch top level file within target robot.

   ```
   sudo ./system/enable_webserver.sh
   sudo ./system/enable_services.sh
   ```

   To disable all services and remove systemd services

   ```
   sudo ./system/disable_webserver.sh
   sudo ./system/disable_services.sh
   ```

## III. Collect Data

When test is trigged with state AUTO (value 2) in `overseer/state` topic, `data_record_node` starts recording until the state is out of AUTO.

Other way to manually starts recording is by publish
`rostopic pub /cmd/record std_msgs/Bool true -1`

To stop manual recording:
`rostopic pub /cmd/record std_msgs/Bool false -1`

Manual record has higher priority than state mode previously. Once `/cmd/record` is published `true`, no other mode can turn it off until `/cmd/record` is published `false`. **Use carefully and don't forget to turn it off**

Data is saved in `stride_ws/test_log/data.csv`. Make sure to download the data at each recording (either by ssh copy or using our web gui download feature), or it will be overwritten in the next recording.

## IV. Directory Structure

```
.
├── bash                                # Include scripts to help build project and dependencies
├── example_scripts                     # Include some example scripts to help user getting started to setup robot test
├── stride_gui_build                    # Binary build of our web gui
├── stride_ws                           # Main ROS workspace
│   ├── custom_script                   # Where the test script is stored
│   ├── gps_debug_log
│   ├── params                          # Location to parameters and config files
│   ├── path                            # Location to test path files
│   ├── src
│   │   ├── can_interface               # Can interface interact with low level hardware
│   │   ├── descender                   # Safety on slope
│   │   ├── drive_mechanism             # Control
│   │   ├── external_interface          # Interface with external vehicle
│   │   ├── joystick                    
│   │   ├── networking                  # Everything networking
│   │   ├── overseer                    # Top level state machine manager
│   │   ├── oxford_gps_decoder          # Python driver to read Oxts GPS through CANbus and Ethernet
│   │   ├── path_follower               # Path following algos
│   │   ├── robot_commander             # Process test setup and execution
│   │   ├── sbg_ros_driver              # submodule to SBG GPS driver
│   │   ├── shared_tools                # misc
│   │   └── top_level                   # top level launch files
│   └── test_log                        
└── system                              # Systemd files
```
