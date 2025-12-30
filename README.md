# ROS2_AUTODOCKING_ROBOT

# BUILD
```
cd ~/ros2_autodocking_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```
## CLONING
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

## GOAL 2: Move robot to auto pose

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