import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math

WHEEL_DISTANCE = 0.052
WHEEL_RADIUS = 0.0205

def axis_angle_to_quaternion(axis, angle):
    # Axis is a list of 3 elements [x, y, z]
    # Angle is in radians
    # q = [x, y, z, w]
    s = math.sin(angle / 2)
    x = axis[0] * s
    y = axis[1] * s
    z = axis[2] * s
    w = math.cos(angle / 2)
    return x, y, z, w

class EpuckDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        
        # 1. Initialize rclpy
        if not rclpy.ok():
            rclpy.init(args=None)

        # 2. Get the robot name (e.g., "robot_1")
        self.__robot_name = self.__robot.getName()

        # 3. Create the node with the CORRECT NAMESPACE
        # This ensures topics are /robot_1/cmd_vel, etc.
        self.__node = rclpy.create_node(
            'driver', 
            namespace=self.__robot_name
        )

        # 4. Get Motors
        self.__left_motor = self.__robot.getDevice('left wheel motor')
        self.__right_motor = self.__robot.getDevice('right wheel motor')
        self.__left_motor.setPosition(float('inf'))
        self.__left_motor.setVelocity(0)
        self.__right_motor.setPosition(float('inf'))
        self.__right_motor.setVelocity(0)
        
        self.__target_twist = Twist()
        
        # 5. Create Subscription
        # Since the node is in a namespace, 'cmd_vel' becomes '/robot_X/cmd_vel'
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        
        # 6. Initialize TF Broadcaster
        self.__tf_broadcaster = TransformBroadcaster(self.__node)
        
        # Check if we can access the self node (Supervisor privilege)
        if hasattr(self.__robot, 'getSelf'):
            self.__is_supervisor = True
            self.__node.get_logger().info(f"{self.__robot_name} has supervisor privileges, enabling Ground Truth TF publishing.")
            self.__self_node = self.__robot.getSelf()
        else:
            self.__is_supervisor = False
            self.__node.get_logger().warn(f"{self.__robot_name} does NOT have supervisor privileges (getSelf not found). Ground Truth TF will NOT be published.")
            self.__self_node = None

        self.__node.get_logger().info(f"Driver started for {self.__robot_name} in namespace /{self.__robot_name}")

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z
        
        command_motor_left = (forward_speed - angular_speed * (WHEEL_DISTANCE / 2)) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * (WHEEL_DISTANCE / 2)) / WHEEL_RADIUS
        
        self.__left_motor.setVelocity(command_motor_left)
        self.__right_motor.setVelocity(command_motor_right)
        
        # Publish Ground Truth TF
        if self.__is_supervisor and self.__self_node:
            position = self.__self_node.getPosition()
            # getOrientation returns a 3x3 matrix, but we can also use getField('rotation') if we want axis-angle
            # Or assume we want the rotation field.
            # Using getOrientation() (matrix) is safer for global orientation representation? 
            # But converting matrix to Quat in python without deps...
            # The 'rotation' field of the node is usually the easiest way if available.
            rotation_field = self.__self_node.getField('rotation')
            if rotation_field:
                rot_values = rotation_field.getSFRotation() # [x, y, z, angle]
                q_x, q_y, q_z, q_w = axis_angle_to_quaternion(rot_values[:3], rot_values[3])
                
                t = TransformStamped()
                t.header.stamp = self.__node.get_clock().now().to_msg()
                t.header.frame_id = 'world'
                t.child_frame_id = f'{self.__robot_name}/base_link'
                
                t.transform.translation.x = position[0]
                t.transform.translation.y = position[1]
                t.transform.translation.z = position[2]
                
                t.transform.rotation.x = q_x
                t.transform.rotation.y = q_y
                t.transform.rotation.z = q_z
                t.transform.rotation.w = q_w
                
                self.__tf_broadcaster.sendTransform(t)