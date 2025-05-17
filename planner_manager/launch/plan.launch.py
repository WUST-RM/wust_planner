import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription 
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler

def generate_launch_description():

    rviz_config_file = LaunchConfiguration("rviz_config")

    config_file_path = os.path.join(
        get_package_share_directory('planner_manager'),
        'config',
        'planner_manager.yaml'
    )
    planner_manager_node = Node(
        package='planner_manager',
        executable='planner_manager_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config_file_path]
    )
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(get_package_share_directory('planner_manager'), "rviz", "rv.rviz"),
        description="Full path to the RViz config file to use",
    )

    # Launch rviz
    start_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        #namespace=namespace,
        arguments=["-d", rviz_config_file],
        output="screen",
        # remappings=[
        #     ("/tf", "tf"),
        #     ("/tf_static", "tf_static"),
        # ],
    )
    return LaunchDescription([
                     
                              planner_manager_node,
                              declare_rviz_config_file_cmd,
                              start_rviz_cmd,
                              
                          
                              ]
                              )
