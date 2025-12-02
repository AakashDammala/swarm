import rclpy
from geometry_msgs.msg import Twist

WHEEL_DISTANCE = 0.052
WHEEL_RADIUS = 0.0205

class EpuckDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        
        # 1. Initialize rclpy
        if not rclpy.ok():
            rclpy.init(args=None)

        # 2. Get the robot name (e.g., "robot_1")
        robot_name = self.__robot.getName()

        # 3. Create the node with the CORRECT NAMESPACE
        # This ensures topics are /robot_1/cmd_vel, etc.
        self.__node = rclpy.create_node(
            'driver', 
            namespace=robot_name
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
        
        self.__node.get_logger().info(f"Driver started for {robot_name} in namespace /{robot_name}")

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