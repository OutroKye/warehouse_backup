# warehouse_backup

Build the workspace
```
colcon build --allow-overriding turtlebot3_cartographer turtlebot3_description turtlebot3_navigation2 nav2_amcl slam_toolbox 
```

Launch the Gazebo world
```
ros2 launch launch_pkg gazebo_world.launch.py
```

Launch Rviz
```
ros2 launch launch_pkg navigation2.launch.py use_sim_time:=True 
```