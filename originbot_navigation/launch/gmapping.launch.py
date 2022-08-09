from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
        
    gmapping_node=Node(
    	package='slam_gmapping',
    	node_executable='slam_gmapping', 
    	output='screen',
    	parameters=[{'use_sim_time':use_sim_time}])
    	
    ld = LaunchDescription()
    ld.add_action(gmapping_node)
    
    return ld
