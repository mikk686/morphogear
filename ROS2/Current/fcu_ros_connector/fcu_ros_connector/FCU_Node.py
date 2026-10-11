import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped, Transform, Twist
from std_msgs.msg import String
from mavros_msgs.msg import State
from mavros_msgs.srv import StreamRate, CommandBool, SetMode, CommandTOL
from nav_msgs.msg import Path

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool



class FlightController(Node):

    def __init__(self):
        super().__init__('flight_controller')
        self.connected = False
        self.armed = False
        self.mode = "STABILIZE"
        self.path_ready = False
        self.pose_coord_local = np.zeros(3)
        self.track = None

        self.goal= np.zeros(3)
        self.state = "disarmed"
        self.vel_local=TwistStamped()
        self.ground_offset=0.

        qos_policy = rclpy.qos.QoSProfile(
                reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                depth=10)

        self.state_pub = self.create_publisher (String, '/state_info',1)
        self.vel_sub = self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local', self.vel_callback, qos_policy)

        # Subscribers
        self.state_sub = self.create_subscription(State, '/mavros/state', self.callback_state, qos_policy)
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/vision_pose/pose', self.drone_pose_callback, qos_policy)
        # self.scene_sub = self.create_subscription(String, '/director/scene', self.scene_callback, 10)
        # self.track_sub = self.create_subscription(Path, '/director/track', self.track_callback, 10)

        # Publishers
        self.setpoint_publisher_local = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', qos_policy)
        self.vel_publisher = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', qos_policy)
        #self.setpoint_path_pub = self.create_publisher(MultiDOFJointTrajectory, "/mavros/setpoint_trajectory/local", 1)

        # Clients
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.service_set_stream_rate = self.create_client(StreamRate, '/mavros/set_stream_rate')

        # Unity Services
        self.current_req = False
        self.arm_srv = self.create_service(SetBool, 'arm_unity', self.unity_arm_callback)
        self.takeoff_srv = self.create_service(SetBool, 'takeoff_unity', self.unity_takeoff_callback)
        self.landing_srv = self.create_service(SetBool, 'land_unity', self.unity_land_callback)

        self.timer = self.create_timer(0.5, self.state_observer)
        # Unity Subscribers
        self.unity_setpose_sub = self.create_subscription(PoseStamped, 'setpose_unity', self.unity_setpose_callback, qos_policy)

        while not self.service_set_stream_rate.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for ROS services')

        try:
            self.get_logger().warn("Setting stream rate to 50")
            req = StreamRate.Request()
            req.stream_id = 0
            req.message_rate = 50
            req.on_off = True
            future = self.service_set_stream_rate.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.get_logger().warn("Stream rate set successfully")
            else:
                self.get_logger().error("Failed to set stream rate")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

        # Wait until connected to drone
        while not self.connected:
            rclpy.spin_once(self, timeout_sec=0.1)



        self.get_logger().warn("Ready for flight")



    def state_observer(self):
        distance=self.goal-self.pose_coord_local
        vel=np.array([self.vel_local.twist.linear.x,self.vel_local.twist.linear.y,self.vel_local.twist.linear.z])
        if (self.state=="arming"):
            if (self.armed):
                self.state = 'arm done'
        elif (self.state == "taking off"):
            if (self.pose_coord_local[2]>0.75):
                self.state = 'takeoff done'
        elif (self.state == 'flying'):
            distance=self.goal-self.pose_coord_local
            if (np.linalg.norm(distance)<0.25):
                self.state='hover'
        elif (self.state == 'landing'):
            vel=np.array([self.vel_local.twist.linear.x,self.vel_local.twist.linear.y,self.vel_local.twist.linear.z])
            if (np.linalg.norm(vel)<0.03 and self.pose_coord_local[2]<0.7):
                self.state = 'land done'
        print (self.state,self.pose_coord_local, self.goal, np.linalg.norm(distance), np.linalg.norm(vel),  self.vel_local.twist.linear.x,self.vel_local.twist.linear.y,self.vel_local.twist.linear.z )
        
        self.publish_state()


    def publish_state(self):
        msg = String()
        msg.data=self.state
        self.state_pub.publish(msg)

    def vel_callback(self,msg):
        #print(msg)
        self.vel_local=msg
        

    def unity_setpose_callback(self, msg):
        '''
        Function to request flight to point from Unity.
        This function redirect PoseStamped message from Unity to Mavros
        '''
        self.state = "flying"
        msg.header.frame_id = "map"  #SHOULD BE THE SAME AS VICON
        
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        msg.pose.position.z = msg.pose.position.z + self.ground_offset #HEIGHT OF THE DRONE
        self.get_logger().warn(f"Publishing mavros set point: {msg}")
        self.setpoint_publisher_local.publish(msg)

    def unity_arm_callback(self, request, response):
        '''
        Function to request arm from Unity.
        This function triggers from Unity and request ARM to Mavros
        '''
        self.current_req = False
        self.state = "arming"

        self.arm()
        
        #rclpy.spin_once(self, timeout_sec=5.0)
        if self.current_req:
            response.success = True
            response.message = "MorphoGear is armed successfully"
        else:
            response.success = False
            response.message = "MorphoGear is not armed"
        self.current_req = False
        print("EXIT ARM response")
        return response

    def unity_takeoff_callback(self, request, response):
        '''
        Function to request take off from Unity.
        This function triggers from Unity and request TAKE OFF to Mavros
        '''
        self.current_req = False
        self.state = "taking off"
        self.takeoff()
        #rclpy.spin_once(self, timeout_sec=5.0)
        if self.current_req:
            response.success = True
            response.message = "MorphoGear is flying successfully"
        else:
            response.success = False
            response.message = "MorphoGear is not flying"
        self.current_req = False
        return response

    def unity_land_callback(self, request, response):
        '''
        Function to request landing from Unity.
        This function triggers from Unity and request TAKE OFF to Mavros
        '''
        self.current_req = False
        self.state = "landing"
        self.land()
        #rclpy.spin_once(self, timeout_sec=5.0)
        if self.current_req:
            response.success = True
            response.message = "MorphoGear is landed successfully"
        else:
            response.success = False
            response.message = "MorphoGear is not landed"
        self.current_req = False
        return response

    def track_callback(self, msg):
        self.track = []
        for point in msg.poses:
            self.track.append(np.array([self.pose_coord_local[0] + point.pose.position.x,
                                        self.pose_coord_local[1] + point.pose.position.y,
                                        point.pose.position.z]))
        self.track = np.array(self.track)

    def scene_callback(self, msg):
        scene = msg.data
        if scene == "blue takeoff":
            self.takeoff()
            #rclpy.spin_once(self, timeout_sec=5.0)
            self.takeoff_pose = self.pose_coord_local
        if scene == "blue fly trajectory":
            self.execute_trajectory()

    def execute_trajectory(self):
        #from tf_transformations import euler_from_quaternion, quaternion_from_euler
        import toppra as ta
        import toppra.constraint as constraint
        import toppra.algorithm as algo
        from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
        if self.track is not None:
            ss = np.linspace(0, 1, self.track.shape[0])
            path = ta.SplineInterpolator(ss, self.track)

            vlim = np.array([[-0.25, 0.25], [-0.25, 0.25], [-0.25, 0.25]])
            alim = np.array([[-4, 4], [-4, 4], [-4, 4]])
            pc_vel = constraint.JointVelocityConstraint(vlim)
            pc_acc = constraint.JointAccelerationConstraint(alim)

            instance = algo.TOPPRA([pc_vel, pc_acc], path)
            jnt_traj = instance.compute_trajectory(0, 0)

            t = np.linspace(0, jnt_traj.duration, 100)

            qs = jnt_traj(t)
            qds = jnt_traj(t, 1)
            qdds = jnt_traj(t, 2)

            traj = MultiDOFJointTrajectory()
            traj.header.stamp = self.get_clock().now().to_msg()
            traj.header.frame_id = '/vicon/world'
            current_time = 0
            for i in range(len(qs)):
                pose = MultiDOFJointTrajectoryPoint()
                transform = Transform()
                transform.translation.x = qs[i, 0]
                transform.translation.y = qs[i, 1]
                transform.translation.z = qs[i, 2]
                pose.transforms.append(transform)

                vel = Twist()
                vel.linear.x = qds[i, 0]
                vel.linear.y = qds[i, 1]
                vel.linear.z = qds[i, 2]

                acc = Twist()
                acc.linear.x = qdds[i, 0]
                acc.linear.y = qdds[i, 1]
                acc.linear.z = qdds[i, 2]

                traj.points.append(pose)
                pose.time_from_start = rclpy.duration.Duration(seconds=t[i] - current_time).to_msg()
                current_time = t[i]

            self.setpoint_path_pub.publish(traj)

    def callback_state(self, state):
        """
        Subscriber callback function from the "mavros/state" topic.
        Is used to save the connection and armed states of the drone.

        param state: a ros message that contains the state of the drone
        """
        self.connected = state.connected
        self.armed = state.armed

    def drone_pose_callback(self, msg):
        """
        The callback function for the mavros/local_position/pose topic. Updates the current drone
        position and the is_grounded status.

        param msg: A ROS message of type PoseStamped.
        """
        #print(msg)
        self.pose_local = msg
        self.pose_coord_local = np.array([self.pose_local.pose.position.x,
                                          self.pose_local.pose.position.y, self.pose_local.pose.position.z])
    def set_mode(self):
        """
        Sets the drone mode to "GUIDED" and arms the drone, enabling it to complete missions and tasks.
        """
        # Changes mode to "GUIDED"
        try:
            self.get_logger().warn("Sending mode: GUIDED")
            req = SetMode.Request()
            req.base_mode = 0
            req.custom_mode = 'GUIDED'
            future = self.set_mode_client.call_async(req)
            #rclpy.spin_until_future_complete(self, future)
            if future.result() is not None and future.result().mode_sent:
                self.get_logger().warn("Mode changed")
                self.mode = "GUIDED"
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")



    def arm(self):
        # Arms the drone
        self.ground_offset = self.pose_coord_local[2]
        self.set_mode()
        try:
            self.get_logger().warn("sending arm request")
            req = CommandBool.Request()
            req.value = True
            future = self.arming_client.call_async(req)
            #rclpy.spin_until_future_complete(self, future)
            if future.result() is not None and future.result().success:
                self.get_logger().warn("armed")
                self.current_req = True
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        print(" EXIT ARM function")

    def takeoff(self):
        """
        The takeoff strategy for the drone.
        """
        # Arm the drone
        if not self.armed:
            self.arm()
        # Takeoff
        try:
            self.get_logger().warn("sending takeoff request")
            req = CommandTOL.Request()
            req.altitude = 2.0
            future = self.takeoff_client.call_async(req)
            #rclpy.spin_until_future_complete(self, future)
            if future.result() is not None and future.result().success:
                self.get_logger().warn("takeoff successful")
                self.current_req = True
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        self.get_logger().warn("takeoff END")

    def land(self):
        """
        Landing strategy for the drone.
        Lands the drone by simply switching the mode to "LAND"
        """
        try:
            self.get_logger().warn("Sending mode")
            req = SetMode.Request()
            req.base_mode = 0
            req.custom_mode = 'LAND'
            future = self.set_mode_client.call_async(req)
            #rclpy.spin_until_future_complete(self, future)
            if future.result() is not None and future.result().mode_sent:
                self.get_logger().warn("Mode changed to landing")
                self.mode = "LAND"
                self.is_grounded = True
                self.current_req = True
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def shutdown(node):
    """
    Function is called on shutdown.
    """
    # Land the drone
    set_mode_client = node.create_client(SetMode, 'mavros/set_mode')
    try:
        node.get_logger().warn("Sending mode")
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = 'LAND'
        future = set_mode_client.call_async(req)
        #rclpy.spin_until_future_complete(node, future)
        if future.result() is not None and future.result().mode_sent:
            node.get_logger().warn("Mode changed to landing")
    except Exception as e:
        node.get_logger().error(f"Service call failed: {e}")
    node.get_logger().warn("Ground station node shutting down")

def main(args=None):

    rclpy.init(args=args)
    controller = FlightController()
    #rclpy.on_shutdown(lambda: shutdown(controller))
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()




# Example usage




if __name__ == "__main__":
    main()
