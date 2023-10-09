import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the launch directory
    aws_small_warehouse_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')

    robot_name_in_model = 'turtlebot3_burger'
    urdf_file_path = 'urdf/turtlebot3_burger.urdf'
    sdf_model_path = 'models/turtlebot3_burger/model.sdf'
    nav2_params_path = 'param/burger.yaml'

    # Pose where we want to spawn the robot
    spawn_x_val = '1.8'
    spawn_y_val = '-9.5'
    spawn_z_val = '10.5'
    spawn_yaw_val = '0.0'
    timeout = '30.0'

    # Set the path to different files and folders.  
    default_urdf_model_path = os.path.join(get_package_share_directory('turtlebot3_description'), urdf_file_path)
    sdf_model_path = os.path.join(get_package_share_directory('turtlebot3_gazebo'), sdf_model_path)
    nav2_params_path = os.path.join(get_package_share_directory('turtlebot3_navigation2'), nav2_params_path)
    gazebo_ros = get_package_share_directory('gazebo_ros')
    # launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    # Launch configuration variables specific to simulation
    namespace = LaunchConfiguration('namespace')
    sdf_model = LaunchConfiguration('sdf_model')
    urdf_model = LaunchConfiguration('urdf_model')  
    use_namespace = LaunchConfiguration('use_namespace')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')  
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')

    remappings = [('/tf', 'tf'),
                    ('/tf_static', 'tf_static')]
    
    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        name='use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file',
        default_value=nav2_params_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model', 
        default_value=default_urdf_model_path, 
        description='Absolute path to robot urdf file')
    
    declare_sdf_model_path_cmd = DeclareLaunchArgument(
        name='sdf_model', 
        default_value=sdf_model_path, 
        description='Absolute path to robot sdf file')    
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(aws_small_warehouse_dir, 'worlds', 'warehouse.world'),
        description='Full path to world model file to load'
    )

    # # Specify the actions
    # start_gazebo_server_cmd = ExecuteProcess(
    #     cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world],
    #     cwd=[aws_small_warehouse_dir],
    #     output='screen'
    # )

    # start_gazebo_client_cmd = ExecuteProcess(
    #     condition=IfCondition(PythonExpression(['not ', headless])),
    #     cmd=['gzclient'],
    #     cwd=[aws_small_warehouse_dir],
    #     output='screen'
    # )

    # Specify the actions
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py'))
    )

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression(['not ', headless]))
    )

    # Launch the robot
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model,                   
                    '-file', sdf_model,
                    '-timeout', timeout,
                    '-x', spawn_x_val,
                    '-y', spawn_y_val,
                    '-z', spawn_z_val,
                    '-Y', spawn_yaw_val],
        output='screen')
    
    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time, 
        'robot_description': Command(['xacro ', urdf_model])}],
        remappings=remappings,
        arguments=[default_urdf_model_path])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_sdf_model_path_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_simulator_cmd)
    # ld.add_action(declare_world_cmd)

    # Add any conditioned actions
    # ld.add_action(start_gazebo_server_cmd)
    # ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(spawn_entity_cmd)


    return ld