import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
import pyvicon_datastream as pv
from pyvicon_datastream import tools
import numpy as np
from scipy.spatial.transform import Rotation
import time

class VICON(Node):
    def __init__(self):
        super().__init__('vicon_connector')
        self.publisher_ = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
        timer_period = 0.025  # seconds
        self.timer = self.create_timer(timer_period, self.callback)
        self.i = 0
        VICON_TRACKER_IP = "192.168.50.73"
        self.OBJECT_NAME = "morphogear"
        vicon_client = pv.PyViconDatastream()
        ret = vicon_client.connect(VICON_TRACKER_IP)
        if ret != pv.Result.Success:
            print(f"Connection to {VICON_TRACKER_IP} failed")
        else:
            print(f"Connection to {VICON_TRACKER_IP} successful")
        self.mytracker = tools.ObjectTracker(VICON_TRACKER_IP)
        self.msg=PoseStamped()


    def callback(self):
        self.pub_callback()
        self.timer_callback()

    def pub_callback(self):
        self.publisher_.publish(self.msg)

                
    def timer_callback(self):
        #start_time = time.time()
        position = self.mytracker.get_position(self.OBJECT_NAME)
        #print(f"Position: {position}")
        try:
            tmp=position[2]
        except:
            print(position)
            return
        pose=tmp[0][2:5]
        rot=tmp[0][5:8]
        rot = Rotation.from_euler('xyz', rot, degrees=True).as_quat(scalar_first=False)
        #print(rot)
        #print(rot.as_euler('xyz', degrees=True))
        self.msg.pose.position.x = pose[1]/1000
        self.msg.pose.position.y = pose[0]/1000
        self.msg.pose.position.z = -pose[2]/1000
        self.msg.pose.orientation.x = rot[0]
        self.msg.pose.orientation.y = rot[1]
        self.msg.pose.orientation.z = rot[2]
        self.msg.pose.orientation.w = rot[3]
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.msg.header.frame_id = "map"
        
        #print(time.time() - start_time)



def main(args=None):
    rclpy.init(args=args)

    node = VICON()

    rclpy.spin(node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()






if __name__ == '__main__':
    main()

