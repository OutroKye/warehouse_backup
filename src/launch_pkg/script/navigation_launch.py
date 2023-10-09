import time  # Time library
import os
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from rclpy.node import Node
from rclpy.clock import Clock

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult # Helper module
from launch_ros.substitutions import FindPackageShare
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
import subprocess

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initialpose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

class DeleteEntityAsync(Node):
    def __init__(self):
        super().__init__('delete_entity_async')
        self.cli = self.create_client(DeleteEntity, '/delete_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = DeleteEntity.Request()

    def send_request(self, name):
        self.req.name = name
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

# class SpawnEntityAsync(Node):
#     def __init__(self):
#         super().__init__('add_entity_async')
#         self.cli = self.create_client(SpawnEntity, '/spawn_entity')
#         while not self.cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('service not available, waiting again...')
#         self.req = SpawnEntity.Request()

#     def send_request(self, name):
#         self.req.name = "turtlebot3_waffle"
#         self.req.xml = ""
#         self.req.robot_namespace = ""
#         self.req.initial_pose = ""
#         self.req.reference_frame = ""

#         self.future = self.cli.call_async(self.req)
#         rclpy.spin_until_future_complete(self, self.future)
#         return self.future.result()


rclpy.init()
package_name = 'turtlebot3_navigation2'
pkg_share = FindPackageShare(package=package_name).find(package_name)
# map_dir = FindPackageShare("turtlebot3_navigation2")

# Launch the ROS 2 Navigation Stack
navigator = BasicNavigator()
map_file_path_stage1 = 'maps/map.yaml'
map_file_path_stage2 = 'maps/map2.yaml'

first_floor_map = os.path.join(pkg_share, map_file_path_stage1)
second_floor_map = os.path.join(pkg_share, map_file_path_stage2)

#Otherwise, you can use navigator function
initial_pose = PoseStamped()
initial_pose.header.frame_id = 'map'
initial_pose.header.stamp = navigator.get_clock().now().to_msg()
initial_pose.pose.position.x = 0.0
initial_pose.pose.position.y = 0.0
initial_pose.pose.position.z = 0.0
initial_pose.pose.orientation.x = 0.0
initial_pose.pose.orientation.y = 0.0
initial_pose.pose.orientation.z = 0.0
initial_pose.pose.orientation.w = 1.0
navigator.setInitialPose(initial_pose)

# Wait for navigation to fully activate. Use this line if autostart is set to true.
navigator.waitUntilNav2Active()

def moveTo(goal, name):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()

    goal_pose.pose.position.x = float(goal[0])
    goal_pose.pose.position.y = float(goal[1])
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    done = False
    print('[Robot]   Going to ' + name)
    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():  
      feedback = navigator.getFeedback()
      if feedback:
        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
          navigator.cancelTask()

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('[Robot]   Arrived to ' + name)
        done = True
    elif result == TaskResult.FAILED:
        print('[Robot]   Could not get to ' + name)
    else:
        print('Goal has an invalid return status!')
        
    return done

def waypoints(goals):
    goals_pose = []
    
    for i in range(len(goals)):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(goals[i][0])
        goal_pose.pose.position.y = float(goals[i][1])
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        goals_pose.append(goal_pose)
    
    print(goals_pose)
    return goals_pose


def main():
    # # Every Coord is repreasented as x_pos, y_pos, x_quat, y_quat, z_quat, w_quat 
    first_floor_goals = ["1F_goal", "1F_lift"]
    second_floor_goals = ["2F_lift", "2F_goal"]
    # first_floor_goals_coordinates = [[3.0, -7.5, 0.0], [-3.7, 9.0, 0.0]]
    # second_floor_goals_coordinates = [[-3.7, 9.0, 10.0], [3.0, 4.0, 10.0]]
    # first_floor_goals_coordinates = [[-3.0, 7.5], [3.7, -9.0]]
    first_floor_goals_coordinates = [[-4.5, 6.0], [1.8, -9.5]]
    second_floor_goals_coordinates = [[1.8, -9.5], [-5, 7.0]]

    i = 0
    have_task = 0
    package1_picked = 0
    package1_delivered = 0

    to_elevator = 0
    done = False 

    current_floor = 1
    current_goal = first_floor_goals[1]
    print('[Robot]   Okay, I heard I have a task')

    ## There are two way to generate trajectory, one is a hard coding 
    initialize_flag = False
    while not done:
        if have_task == 0: 
            current_floor = 1
            current_goal = first_floor_goals[0]
            navigator.clearAllCostmaps()
            navigator.changeMap('/home/keyu/warehouse_ws/src/turtlebot3_navigation2/map/map.yaml')
            isDone = moveTo(first_floor_goals_coordinates[0], current_goal)
            isDone = 1
            if isDone:
                have_task = 1
                current_goal = first_floor_goals[1]
                moveTo(first_floor_goals_coordinates[1], current_goal)
                current_floor = 1
            else:
                print("[Robot]   Trying again")

        elif have_task == 1:
            delete_model_client = DeleteEntityAsync()
            response = delete_model_client.send_request("turtlebot3_burger") 
            # delete model for next stage
            # launch = subprocess.call(["ros2 launch launch_pkg respawn.launch.py"], shell = False)
            # subprocess.run(["ros2 launch launch_pkg respawn.launch.py"], shell=True)
            print("Respawn the robot...")
            subprocess.Popen(["ros2", "launch", "launch_pkg", "respawn.launch.py"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=False)
            time.sleep(10)
            print("Respawn successed.")            
            print ("[Robot] Move to Next Stage")

            if initialize_flag is False:
                navigator.clearAllCostmaps()
                navigator.changeMap('/home/keyu/warehouse_ws/src/turtlebot3_navigation2/map/map2.yaml')

                initial_pose = PoseStamped()
                initial_pose.header.frame_id = 'map'
                initial_pose.header.stamp = navigator.get_clock().now().to_msg()
                initial_pose.pose.position.x = 1.8
                initial_pose.pose.position.y = -9.5
                initial_pose.pose.position.z = 0.0
                initial_pose.pose.orientation.x = 0.0
                initial_pose.pose.orientation.y = 0.0
                initial_pose.pose.orientation.z = 0.0
                initial_pose.pose.orientation.w = 1.0
                navigator.setInitialPose(initial_pose)
                navigator.clearAllCostmaps()

                initialize_flag = True

            current_goal = second_floor_goals[1]
            isDone = moveTo(second_floor_goals_coordinates[1], current_goal)
            current_floor = 2
            have_task = 2
            # if isDone:
            #     current_goal = stage2_goals[1]
            #     moveTo(stage2_goals_coordinates[1], current_goal)
            #     print('[Robot]   I have a package, need to go to deliver it...')
            #     have_task = 2
        elif have_task == 2:
            print('[Robot]   Everythins is Okay for me')
            done = True

    # Shut down the ROS 2 Navigation Stack
    navigator.lifecycleShutdown()
    exit(0) 

if __name__ == '__main__':
  main()