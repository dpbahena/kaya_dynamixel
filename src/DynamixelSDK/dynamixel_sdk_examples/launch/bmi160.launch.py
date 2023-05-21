from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    

    #
    #  ros2 launch dynamixel_sdk_examples bmi160.launch.py
    #

    ld = LaunchDescription()  
              
    bmi160 = Node(
        package='dynamixel_sdk_examples',
        executable='bmi160',
        
        
    )
    

   
    
    ld.add_action(bmi160)

    return ld
