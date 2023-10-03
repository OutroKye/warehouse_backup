import subprocess

# for complex commands, with many args, use string + `shell=True`:
cmd_str = "ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap '{map_url: /home/keyu/map2.yaml}'"
subprocess.run(cmd_str, shell=True)

# ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml
# ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap '{map_url: /home/keyu/map2.yaml}'
# ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'turtlebot3_burger'}"
# ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap "{request: {}}"
# ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap "{request: {}}"