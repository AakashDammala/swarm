import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the workspace root and build directory
    package_dir = get_package_share_directory('swarm_view')
    # Navigate from install/swarm_view/share/swarm_view to build/swarm_view
    build_dir = os.path.join(package_dir, '../../../../build/swarm_view')
    test_executable = os.path.join(build_dir, 'robot_topics_test')
    
    # Normalize the path
    test_executable = os.path.normpath(test_executable)
    
    # Run the robot topics test
    test_process = ExecuteProcess(
        cmd=[test_executable],
        output='screen'
    )
    
    return LaunchDescription([test_process])
