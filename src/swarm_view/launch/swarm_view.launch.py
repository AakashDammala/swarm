import os
import yaml
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    # Declare launch arguments
    run_test_arg = launch.actions.DeclareLaunchArgument(
        'run_test',
        default_value='false',
        description='Run robot topics test'
    )
    
    package_dir = get_package_share_directory('swarm_view')
    config_path = os.path.join(package_dir, 'config', 'swarm_config.yaml')
    urdf_path = os.path.join(package_dir, 'resource', 'epuck_webots.urdf')
    
    resource_dir = os.path.join(package_dir, 'resource')
    if 'PYTHONPATH' in os.environ:
        os.environ['PYTHONPATH'] = resource_dir + os.pathsep + os.environ['PYTHONPATH']
    else:
        os.environ['PYTHONPATH'] = resource_dir

    # Path to the generated world
    generated_world = '/tmp/generated_swarm.wbt'
    
    # 1. Read Config
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    # 2. Generate World File with EXTERNPROTO headers
    print(f"Generating world file at: {generated_world}")
    
    with open(generated_world, 'w') as f_out:
        # --- HEADER SECTION (Critical for Webots R2023b+) ---
        f_out.write("#VRML_SIM R2023b utf8\n")
        f_out.write("EXTERNPROTO \"https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackground.proto\"\n")
        f_out.write("EXTERNPROTO \"https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto\"\n")
        f_out.write("EXTERNPROTO \"https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/floors/protos/RectangleArena.proto\"\n")
        f_out.write("EXTERNPROTO \"https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/robots/gctronic/e-puck/protos/E-puck.proto\"\n\n")
        
        # --- WORLD SETUP ---
        f_out.write("WorldInfo {\n")
        f_out.write("  basicTimeStep 16\n") # 16ms step is standard for E-puck
        f_out.write("}\n")
        f_out.write("Viewpoint { orientation -0.5773 0.5773 0.5773 2.0944 position 0 0 10 }\n")
        f_out.write("TexturedBackground {}\n")
        f_out.write("TexturedBackgroundLight {}\n")
        f_out.write("RectangleArena { floorSize 10 10 }\n\n")
        
        # --- ROBOTS ---
        for robot in config['robots']:
            f_out.write(f"E-puck {{\n")
            f_out.write(f"  translation {robot['x']} {robot['y']} 0\n")
            f_out.write(f"  name \"{robot['name']}\"\n")
            f_out.write(f"  controller \"<extern>\"\n") 
            f_out.write(f"  turretSlot [\n")
            f_out.write(f"    Lidar {{\n")
            f_out.write(f"      translation 0 0 0.05\n")
            f_out.write(f"      horizontalResolution 360\n")
            f_out.write(f"      fieldOfView 6.28\n")
            f_out.write(f"      numberOfLayers 1\n")
            f_out.write(f"      minRange 0.05\n")
            f_out.write(f"      maxRange 2.0\n")
            f_out.write(f"      type \"fixed\"\n")
            f_out.write(f"    }}\n")
            f_out.write(f"  ]\n")
            f_out.write(f"}}\n\n")

    # 3. Setup Webots
    webots = WebotsLauncher(
        world=generated_world,
        ros2_supervisor=True
    )

    # 4. Create Driver Nodes
    launch_nodes = []
    launch_nodes.append(webots)
    launch_nodes.append(webots._supervisor)

    for robot in config['robots']:
        name = robot['name']
        driver = WebotsController(
            robot_name=name,
            parameters=[{'robot_description': urdf_path}],
            namespace=name
        )
        launch_nodes.append(driver)

    return LaunchDescription(launch_nodes)
