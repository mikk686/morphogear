import rclpy
from rclpy.node import Node

from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandHome
from mavros_msgs.msg import State, HomePosition
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped


class FlightManager(Node):
    def __init__(self):
        super().__init__("flight_service")
        self.arm_srv = self.create_service(SetBool, 'arm_unity', self.arm_callback)
        self.takeoff_srv = self.create_service(SetBool, 'takeoff_unity', self.takeoff_callback)
        self.landing_srv = self.create_service(SetBool, 'land_unity', self.land_callback)

        self.arming = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.takingoff = self.create_client(CommandTOL, 'mavros/cmd/takeoff')
        self.landing = self.create_client(CommandTOL, 'mavros/cmd/land')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.set_home_client = self.create_client(CommandHome, '/mavros/cmd/set_home')
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 1)

        self.current_state = State()
        self.pose_local= PoseStamped()

        while not self.arming.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Mavros Arming Service...")
        while not self.takingoff.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Mavros TakingOff Service...")
        while not self.landing.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Mavros Landing Service...")
        while not self.set_mode_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Mavros Set Mode Service...")
        while not self.set_home_client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Mavros Set Home Service...")
            
        self.get_logger().info("Ready to Arm...")

        self.current_status = "Init"
        self.arm_req = CommandBool.Request()
        self.mode_req = SetMode.Request()
        self.takeoff_req = CommandTOL.Request()
        self.land_req = CommandTOL.Request()
        self.set_home_req = CommandHome.Request()