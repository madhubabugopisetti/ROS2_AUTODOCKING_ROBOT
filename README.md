# ROS2_AUTODOCKING_ROBOT

# GOAL: Autonomous Docking Navigation

### KILL TERMINALS
```
pkill -f ros2
pkill -f gazebo
pkill -f gz
pkill -f rviz
clear
```

### BUILD
```
cd ~/ros2_autodocking_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source ~/ros2_autodocking_ws/install/setup.bash
```
### CLONING
```
cd
mkdir -p ros2_autodocking_ws
cd ~/ros2_autodocking_ws
git clone https://github.com/madhubabugopisetti/ROS2_AUTODESK_DIFF_ROBOT
mv src to ros2_autodocking_ws
```

## GOAL 1: Setting up repo for auto_docking
## STEP 1:
```
rename gazebo.launch.py to gazebo_slam.launch.py(move slam.launch.py code to this)
```
## STEP 2: Delete these from our ros2_autodesk_ws
```
gazebo.rviz
localization_params.yaml
localization.rviz
slam_toolbox.yaml
localization.launch.py
rviz.launch.py
sim_rviz.launch.py
slam.launch.py
world.launch.py
```

**Verify**:
- [BUILD](#build)
- **Terminal 1**: ```ros2 launch robot_description gazebo_slam.launch.py```
- **Terminal 2**: 
```
ros2 lifecycle set /slam_toolbox configure 
ros2 lifecycle set /slam_toolbox activate
```
```
Check TF tree : ros2 run tf2_tools view_frames
map
└── odom
    └── base_link
        └── lidar_1
```
- **Terminal 2**: 
```
ros2 launch nav2_bringup navigation_launch.py \
use_sim_time:=true \
params_file:=src/robot_description/config/navigation_params.yaml
```
- **Terminal 3**: ```rviz2 -d src/robot_description/config/display.rviz```
```
Add Map, select topic as /robot_description
In File, save
```
- Select 2D Goal Pose - robot should move<br/>

## GOAL 2: Pose-based Docking

## STEP 1: Cloning navigation repo
```
cd ~/ros2_autodocking_ws/src
git clone https://github.com/ros-navigation/navigation2.git -b jazzy
```

```
cd ~/ros2_autodocking_ws/src
move nav2_docking to src
remove navigation2 folder
```
```
cd ~/ros2_autodocking_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```
```
cd ~/ros2_autodocking_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```
### Verify: ros2 pkg list | grep docking <br/>


## STEP 2: 
- Create docks.yaml in config and add these
```
docks:
  home_dock:
    type: simple_charging_dock
    frame: map
    pose: [1.0, 0.0, 0.0]

```
- Add these into navigation.launch.py
```
Node(
	package='nav2_behaviors',
	executable='behavior_server',
	output='screen',
	parameters=[params]
),
Node(
    package='opennav_docking',
    executable='opennav_docking',
    output='screen',
    parameters=[
		params,
		{'dock_database': '/home/madhu/ros2_docking_ws/src/robot_description/config/docks.yaml'},
		{'use_sim_time': True}
	]
),
Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_nav',
    output='screen',
    parameters=[{
		'use_sim_time': True,
		'autostart': True,
		'node_names': [
			'map_server',
			'amcl',
			'controller_server',
			'planner_server',
			'bt_navigator',
			'behavior_server'
		]
    }]
),
```

## STEP 3: 
- Add these into navigation_params.yaml
```
amcl:
  ros__parameters:
    use_sim_time: true
    base_frame_id: base_link
    odom_frame_id: odom
    global_frame_id: map
    scan_topic: scan
```
- Update these in navigation_params.yaml
```
collision_monitor:
	ros__parameters:
    	enabled: false
docking_server:
	ros__parameters:
		fixed_frame: "map"
		navigate_to_staging_pose: false
		staging_x_offset: 0.0
		use_external_detection_pose: false

		external_detection_timeout: 1.0
		external_detection_translation_x: -0.18
		external_detection_translation_y: 0.0
		external_detection_rotation_roll: -1.57
		external_detection_rotation_pitch: -1.57
		external_detection_rotation_yaw: 0.0
		filter_coef: 0.1

		docks: ['home_dock']
		home_dock:
		type: 'simple_charging_dock'
		frame: map
		pose: [1.0, 0.0, 0.0]
```

## STEP 4: 
- [KILL](#kill-terminals)
- [BUILD](#build)
- **Terminal 1**:```ros2 launch robot_description gazebo_slam.launch.py```
- **Terminal 2**: ```
ros2 lifecycle set /slam_toolbox configure 
ros2 lifecycle set /slam_toolbox activate```
- **Terminal 2**: ```ros2 launch robot_description navigation.launch.py```
- **Terminal 3**: ```ros2 lifecycle set /docking_server configure ros2 lifecycle set /docking_server activate```
- **Terminal 3**: ```rviz2 -d src/robot_description/config/display.rviz```
- **Terminal 4**: ```
ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot "{dock_id: 'home_dock', navigate_to_staging_pose: false}"```
<br/>

## GOAL 3: Navigate to HOME_DOCK

## STEP 1: Add an object into world and render
- Create a folder models
- Create another folder inside it home_dock with model.config, model.sdf files
- Add models to CmakeLists.txt
- Import model.sdf in world.sdf
- Add Node and SetEnvironmentVariable in gazebo_slam.launch.py<br/>
**IMPORTANT**: to make model available for that workspace run ```export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/install/robot_description/share/robot_description/models``` <br/>
**VERIFY**: ros2 launch robot_description gazebo_slam.launch.py

## STEP 2: Navigate it
- [KILL](#kill-terminals)
- [BUILD](#build)
- **Terminal 1**:```ros2 launch robot_description gazebo_slam.launch.py```
- **Terminal 2**: ```
ros2 lifecycle set /slam_toolbox configure 
ros2 lifecycle set /slam_toolbox activate```
- **Terminal 2**: ```ros2 launch robot_description navigation.launch.py```
- **Terminal 3**: ```ros2 lifecycle set /docking_server configure ros2 lifecycle set /docking_server activate```
- **Terminal 3**: ```rviz2 -d src/robot_description/config/display.rviz```
- **Terminal 4**: ros2 action send_goal /dock_robot nav2_msgs/action/DockRobot "{dock_id: 'home_dock'}"
