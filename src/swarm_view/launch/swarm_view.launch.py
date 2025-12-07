import os
import yaml
import math
import random
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
    
    num_rooms = config.get('num_rooms', 10)
    
    # Read world configuration
    world_config = config.get('world_setup', {})
    gen_config = config.get('generation_params', {})
    waste_config = config.get('waste_params', {})
    robot_config = config.get('robot_params', {})
    
    arena_width = world_config.get('arena_width', 10)
    basic_time_step = world_config.get('basic_time_step', 16)
    cast_shadows = world_config.get('cast_shadows', False)
    floor_color = world_config.get('floor_color', [0.8, 0.6, 0.4])
    floor_color_str = f"{floor_color[0]} {floor_color[1]} {floor_color[2]}"
    shadows_str = "TRUE" if cast_shadows else "FALSE"
    
    room_size = gen_config.get('room_size', 1.0)
    wall_thick = gen_config.get('wall_thick', 0.05)
    wall_height = gen_config.get('wall_height', 0.2)
    col_spacing = gen_config.get('col_spacing', 3.0)
    row_spacing = gen_config.get('row_spacing', 2.0)
    
    # Waste Params
    num_waste = waste_config.get('num_waste_per_room', 5)
    waste_size = waste_config.get('waste_block_size', 0.1)
    seed = waste_config.get('random_seed', -1)
    
    # Robot Params
    cam_w = robot_config.get('camera_width', 64)
    cam_h = robot_config.get('camera_height', 64)
    
    if seed >= 0:
        random.seed(seed)
    
    # 2. Generate World File with EXTERNPROTO headers
    rows = math.ceil(num_rooms / 2)
    print(f"Generating world file at: {generated_world} for {num_rooms} rooms")
    
    with open(generated_world, 'w') as f_out:
        # --- HEADER SECTION (Critical for Webots R2023b+) ---
        f_out.write("#VRML_SIM R2023b utf8\n")
        f_out.write("EXTERNPROTO \"https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackground.proto\"\n")
        f_out.write("EXTERNPROTO \"https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto\"\n")
        f_out.write("EXTERNPROTO \"https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/floors/protos/RectangleArena.proto\"\n")
        f_out.write("EXTERNPROTO \"https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/robots/gctronic/e-puck/protos/E-puck.proto\"\n\n")
        
        # --- WORLD SETUP ---
        arena_height = max(10, rows * 2 + 2)
        # arena_width is now from config
        
        f_out.write("WorldInfo {\n")
        f_out.write(f"  basicTimeStep {basic_time_step}\n") 
        f_out.write("}\n")
        f_out.write("Viewpoint { orientation -0.5773 0.5773 0.5773 2.0944 position 0 0 10 }\n")
        f_out.write("TexturedBackground { texture \"factory\" }\n")
        # f_out.write(f"TexturedBackgroundLight {{ castShadows {shadows_str} }}\n")
        f_out.write(f"DirectionalLight {{ direction 0 0 -1 castShadows FALSE intensity 1.0 color 1 1 1 }}\n") 
        f_out.write(f"RectangleArena {{ floorSize {arena_width} {arena_height} floorAppearance Appearance {{ material Material {{ diffuseColor {floor_color_str} }} }} }}\n\n")
        
        # --- GENERATION PARAMETERS ---
        # --- GENERATION PARAMETERS (now from config) ---
        # room_size, wall_thick, wall_height, col_spacing, row_spacing defined above
        
        # --- WALL MATERIAL ---
        # We will embed the appearance in each shape for simplicity, or could use DEF/USE
        
        for i in range(num_rooms):
            col = i % 2 # 0: Left, 1: Right
            row = i // 2
            
            # Room Center
            # Left col: x = -1.5, Right col: x = 1.5
            x_c = -1.5 if col == 0 else 1.5
            y_c = row * row_spacing - (rows * row_spacing / 2) + (row_spacing / 2) # Center vertically around 0? Or start at 0?
            # Let's start y at 0 and go up.
            # Center vertically around 0
            # Total height span is (rows-1) * row_spacing
            y_offset = ((rows - 1) * row_spacing) / 2
            y_c = (row * row_spacing) - y_offset
            
            robot_name = f"robot_{i+1}"
            
            f_out.write(f"# --- Room {i+1} ({'Left' if col==0 else 'Right'}) ---\n")
            
            # Identify Wall Positions relative to (x_c, y_c)
            # Top Wall
            f_out.write(f"Solid {{\n")
            f_out.write(f"  translation {x_c} {y_c + room_size/2 + wall_thick/2} {wall_height/2}\n")
            f_out.write(f"  children [\n")
            f_out.write(f"    Shape {{\n")
            f_out.write(f"      appearance Appearance {{ material Material {{ diffuseColor {floor_color_str} emissiveColor {floor_color_str}}} }}\n")
            f_out.write(f"      geometry Box {{ size {room_size + 2*wall_thick} {wall_thick} {wall_height} }}\n")
            f_out.write(f"    }}\n")
            f_out.write(f"  ]\n")
            f_out.write(f"  boundingObject Box {{ size {room_size + 2*wall_thick} {wall_thick} {wall_height} }}\n")
            f_out.write(f"}}\n")
            
            # Bottom Wall
            f_out.write(f"Solid {{\n")
            f_out.write(f"  translation {x_c} {y_c - room_size/2 - wall_thick/2} {wall_height/2}\n")
            f_out.write(f"  children [\n")
            f_out.write(f"    Shape {{\n")
            f_out.write(f"      appearance Appearance {{ material Material {{ diffuseColor {floor_color_str} emissiveColor {floor_color_str}}} }}\n")
            f_out.write(f"      geometry Box {{ size {room_size + 2*wall_thick} {wall_thick} {wall_height} }}\n")
            f_out.write(f"      }}\n")
            f_out.write(f"  ]\n")
            f_out.write(f"  boundingObject Box {{ size {room_size + 2*wall_thick} {wall_thick} {wall_height} }}\n")
            f_out.write(f"}}\n")
            
            # Inner Wall (Solid, no door)
            # If col 0 (Left), Inner is Right Wall (+x relative to center)
            # If col 1 (Right), Inner is Left Wall (-x relative to center)
            inner_x_offset = (room_size/2 + wall_thick/2) if col == 0 else -(room_size/2 + wall_thick/2)
            f_out.write(f"Solid {{\n")
            f_out.write(f"  translation {x_c + inner_x_offset} {y_c} {wall_height/2}\n")
            f_out.write(f"  children [\n")
            f_out.write(f"    Shape {{\n")
            f_out.write(f"      appearance Appearance {{ material Material {{ diffuseColor {floor_color_str} emissiveColor {floor_color_str}}} }}\n")
            f_out.write(f"      geometry Box {{ size {wall_thick} {room_size} {wall_height} }}\n")
            f_out.write(f"    }}\n")
            f_out.write(f"  ]\n")
            f_out.write(f"  boundingObject Box {{ size {wall_thick} {room_size} {wall_height} }}\n")
            f_out.write(f"}}\n")
            
            # Outer Wall (With Door)
            # If col 0 (Left), Outer is Left Wall (-x relative to center)
            # If col 1 (Right), Outer is Right Wall (+x relative to center)
            outer_x_offset = -(room_size/2 + wall_thick/2) if col == 0 else (room_size/2 + wall_thick/2)
            
            # Door Geometry: 2 segments with a gap in the middle
            door_gap = 0.4
            segment_len = (room_size - door_gap) / 2
            # Upper Segment
            f_out.write(f"Solid {{\n")
            f_out.write(f"  translation {x_c + outer_x_offset} {y_c + room_size/2 - segment_len/2} {wall_height/2}\n")
            f_out.write(f"  children [\n")
            f_out.write(f"    Shape {{\n")
            f_out.write(f"      appearance Appearance {{ material Material {{ diffuseColor {floor_color_str} emissiveColor {floor_color_str}}} }}\n")
            f_out.write(f"      geometry Box {{ size {wall_thick} {segment_len} {wall_height} }}\n")
            f_out.write(f"    }}\n")
            f_out.write(f"  ]\n")
            f_out.write(f"  boundingObject Box {{ size {wall_thick} {segment_len} {wall_height} }}\n")
            f_out.write(f"}}\n")
            
            # Lower Segment
            f_out.write(f"Solid {{\n")
            f_out.write(f"  translation {x_c + outer_x_offset} {y_c - room_size/2 + segment_len/2} {wall_height/2}\n")
            f_out.write(f"  children [\n")
            f_out.write(f"    Shape {{\n")
            f_out.write(f"      appearance Appearance {{ material Material {{ diffuseColor {floor_color_str} emissiveColor {floor_color_str}}} }}\n")
            f_out.write(f"      geometry Box {{ size {wall_thick} {segment_len} {wall_height} }}\n")
            f_out.write(f"    }}\n")
            f_out.write(f"  ]\n")
            f_out.write(f"  boundingObject Box {{ size {wall_thick} {segment_len} {wall_height} }}\n")
            f_out.write(f"}}\n")
            
            # --- RADIOACTIVE WASTE ---
            for w in range(num_waste):
                # Random position within room inner bounds
                # Room x-range: [x_c - room_size/2 + wall_thick, x_c + room_size/2 - wall_thick]
                # Room y-range: [y_c - room_size/2 + wall_thick, y_c + room_size/2 - wall_thick]
                # Margin for waste size: waste_size/2 + safe margin
                margin = wall_thick + waste_size/2 + 0.05
                spawn_limit = room_size/2 - margin
                
                # Ensure we don't spawn on top of the robot (at center)
                # Simple retry logic or keep-out zone
                valid_pos = False
                w_x, w_y = 0, 0
                while not valid_pos:
                    w_x = x_c + random.uniform(-spawn_limit, spawn_limit)
                    w_y = y_c + random.uniform(-spawn_limit, spawn_limit)
                    # Check distance to center (robot spawn is at x_c, y_c)
                    dist = math.sqrt((w_x - x_c)**2 + (w_y - y_c)**2)
                    if dist > 0.2: # Keep 20cm away from center
                        valid_pos = True

                f_out.write(f"Solid {{\n")
                f_out.write(f"  translation {w_x} {w_y} {waste_size/2}\n")
                f_out.write(f"  children [\n")
                f_out.write(f"    Shape {{\n")
                f_out.write(f"      appearance Appearance {{ material Material {{ diffuseColor 0 1 0 emissiveColor 0 1 0 }} }}\n")
                f_out.write(f"      geometry Box {{ size {waste_size} {waste_size} {waste_size} }}\n")
                f_out.write(f"    }}\n")
                f_out.write(f"  ]\n")
                f_out.write(f"  boundingObject Box {{ size {waste_size} {waste_size} {waste_size} }}\n")
                f_out.write(f"  physics Physics {{ mass 0.1 }}\n")
                f_out.write(f"}}\n")
            
            # --- ROBOT ---
            f_out.write(f"E-puck {{\n")
            f_out.write(f"  translation {x_c} {y_c} 0\n")
            
            # Rotation to face the door (Outer Wall)
            # Col 0 (Left Room): Door is at -X. Robot defaults to +X. Rotate pi.
            # Col 1 (Right Room): Door is at +X. Robot defaults to +X. Rotate 0.
            rot_angle = math.pi if col == 0 else 0
            f_out.write(f"  rotation 0 0 1 {rot_angle}\n")

            f_out.write(f"  name \"{robot_name}\"\n")
            f_out.write(f"  controller \"<extern>\"\n")
            f_out.write(f"  camera_width {cam_w}\n")
            f_out.write(f"  camera_height {cam_h}\n") 
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

    # Generate drivers for each robot based on num_rooms
    for i in range(num_rooms):
        name = f"robot_{i+1}"
        driver = WebotsController(
            robot_name=name,
            parameters=[{'robot_description': urdf_path}],
            namespace=name
        )
        launch_nodes.append(driver)

        # Calculate robot home position for static TF
        col = i % 2 # 0: Left, 1: Right
        row = i // 2
        
        # Calculate x_c (Center X)
        x_c = -1.5 if col == 0 else 1.5
        
        # Calculate y_c (Center Y)
        y_offset = ((rows - 1) * row_spacing) / 2
        y_c = (row * row_spacing) - y_offset

        # Calculate rotation (Yaw)
        rot_angle = math.pi if col == 0 else 0

        # Create Static Transform Publisher
        # Args: x y z yaw pitch roll parent_frame child_frame
        # robot_home_tf = Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=[str(x_c), str(y_c), '0', str(rot_angle), '0', '0', 'world', f'{name}_home'],
        #     output='screen'
        # )
        
        # ROS 2 static_transform_publisher arguments: --x --y --z --yaw --pitch --roll --frame-id --child-frame-id
        # OR positional: x y z yaw pitch roll parent child
        robot_home_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[str(x_c), str(y_c), '0', str(rot_angle), '0', '0', 'world', f'{name}_home'],
            name=f'static_tf_{name}_home'
        )
        launch_nodes.append(robot_home_tf)

    return LaunchDescription(launch_nodes)
