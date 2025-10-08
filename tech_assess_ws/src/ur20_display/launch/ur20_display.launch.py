from launch import LaunchDescription 
from launch_ros.actions import Node #to run the nodes
from launch.actions import DeclareLaunchArgument # to declare an argument for the urdf
import os
from  ament_index_python.packages import get_package_share_directory # to get the share directory of the robot description package
from launch_ros.parameter_descriptions import ParameterValue #to get the value of the robot_model parameter
from launch.substitutions import Command , LaunchConfiguration



def generate_launch_description():


    robot_model = DeclareLaunchArgument(
        name="model",
        default_value= os.path.join(get_package_share_directory("ur_description"),"urdf","ur.urdf.xacro"), 
        description="Absolute path to the URDF"
    )
    
    robot_description = ParameterValue(Command(["xacro"," ", LaunchConfiguration("model"), " ",
            "safety_limits:=",
            "true",
            " ",
            "safety_pos_margin:=",
            "0.15",
            " ",
            "safety_k_position:=",
            "20",
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            "ur20",
            " ",
            "tf_prefix:=",
            '""',])) #to convert to urdf amd choose the UR20

    robot_state_publisher = Node (
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    custom_state_publisher = Node (
        package="ur20_display",
        executable="state_publisher",
        name="state_publisher",
        parameters= [os.path.join(get_package_share_directory("ur20_display"),"config","joint_angles.yaml")]
    )

    joint_state_publisher_gui = Node (
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        
    )

    joint_traj_node = Node (
        package = "joint_traj_graph",
        executable="joint_traj",
        name="joint_traj",


    )

    rviz_node = Node (
        package="rviz2",
        executable="rviz2",
        name="rviz2", 
        output = "screen",
        arguments= ["-d",os.path.join(get_package_share_directory("ur20_display"),"rviz","display2.rviz")]


    )

    return LaunchDescription([robot_model,robot_state_publisher,custom_state_publisher,rviz_node,joint_traj_node])