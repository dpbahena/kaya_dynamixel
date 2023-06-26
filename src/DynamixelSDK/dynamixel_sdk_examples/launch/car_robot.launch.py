import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription, Substitution
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution

#
#  ros2 launch dynamixel_sdk_examples car_robot.launch.py  maxtime:=1 accel:=1023 torque:=1023
#
#  @ testing with joystick / from a different computer in same network 
#
#  ros2 launch teleop_twist_joy teleop-launch.py config_filepath:="/home/dario/foxy_ws/config/xbox.yaml" 
#
#  @ testing with joystick from same computer/container. Make sure to have config file  xbox.yaml in same folder you launch the following command:
#
#  ros2 launch teleop_twist_joy teleop-launch.py config_filepath:="xbox.yaml"  
#



def generate_launch_description():
    ld = LaunchDescription()

    # ! -------- MOTOR CONTROLLER ----------------->
    launch_motor_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('dynamixel_sdk_examples'),
                'launch/motorcontroller.launch.py'
            )
        )
    )

    # ! -------- RELATIVE SPEEDS ---------------->
    launch_relative_speeds = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('dynamixel_sdk_examples'),
                'launch/dynamixelcontroller.launch.py'
            )
        )
    )

    # ! -------- bmi160 ---------------->
    launch_bmi160 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('dynamixel_sdk_examples'),
                'launch/bmi160.launch.py'
            )
        )
    )

    ld.add_action(launch_motor_controller) 
    ld.add_action(launch_relative_speeds)
    ld.add_action(launch_bmi160)
 
    
   
    return ld
