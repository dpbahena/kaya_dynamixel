from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

#
# ros2 launch dynamixel_sdk_examples motorcontroller.launch.py  maxtime:=6 accel:=1023 torque:=1023
#


def generate_launch_description():
    
    ld = LaunchDescription()
    # args that can be set from the command line or a default will be used
    maxtime_arg = DeclareLaunchArgument(
        "maxtime", default_value=TextSubstitution(text="3")
    )
    
    accel_arg = DeclareLaunchArgument(
        "accel", default_value=TextSubstitution(text="10")
    )

    torque_arg = DeclareLaunchArgument(
        "torque", default_value=TextSubstitution(text="512")
    )
           
    motor_controller = Node(
        package='dynamixel_sdk_examples',
        executable='motorcontroller',
        #output='screen',
        parameters=[{
            "maxtime": LaunchConfiguration('maxtime'),
            "accel": LaunchConfiguration('accel'),
            "torque": LaunchConfiguration('torque')
        }]
    )
   
    ld.add_action(maxtime_arg)
    ld.add_action(accel_arg)
    ld.add_action(torque_arg)
    ld.add_action(motor_controller)
    return ld
