from launch import LaunchDescription
from launch_ros.actions import Node
#from launch.actions import DeclareLaunchArgument
#from launch.substitutions import TextSubstitution
#from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    

    #
    #  ros2 launch dynamixel_sdk_examples dynamixelcontroller.launch.py
    #

    ld = LaunchDescription()  
              
    dynamixel_control = Node(
        package='dynamixel_sdk_examples',
        executable='dynamixelcontroller',
        #output='screen'
        
    )
    

   
    
    ld.add_action(dynamixel_control)

    return ld
