# Send SET_GPS_GLOBAL_ORIGIN and SET_HOME_POSITION messages

import rclpy
from rclpy.node import Node
from pymavlink.dialects.v10 import ardupilotmega as MAV_APM
from mavros.mavlink import convert_to_rosmsg
from mavros_msgs.msg import Mavlink
import time

# Global position of the origin
lat = 32.67606 * 1e7   # Terni
lon = -117.15786 * 1e7   # Terni
alt = 163 * 1e3
# lat = 42.56335 * 1e7   # Terni
# lon = 12.64329 * 1e7   # Terni
# alt = 163 * 1e3


class fifo(object):
    """ A simple buffer """
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)

def send_message(msg, mav, pub):
    """
    Send a mavlink message
    """
    msg.pack(mav)
    rosmsg = convert_to_rosmsg(msg)
    pub.publish(rosmsg)

    print("sent message %s" % msg)

def set_global_origin(mav, pub):
    """
    Send a mavlink SET_GPS_GLOBAL_ORIGIN message, which allows us
    to use local position information without a GPS.
    """
    target_system = mav.srcSystem
    #target_system = 0   # 0 --> broadcast to everyone
    lattitude = lat
    longitude = lon
    altitude = alt
    msg = MAV_APM.MAVLink_set_gps_global_origin_message(
            int(target_system),
            int(lattitude), 
            int(longitude),
            int(altitude))

    send_message(msg, mav, pub)

def set_home_position(mav, pub):
    """
    Send a mavlink SET_HOME_POSITION message, which should allow
    us to use local position information without a GPS
    """
    target_system = mav.srcSystem
    #target_system = 0  # broadcast to everyone

    lattitude = lat
    longitude = lon
    altitude = alt
    
    x = 0
    y = 0
    z = 0
    q = [1, 0, 0, 0]   # w x y z

    approach_x = 0
    approach_y = 0
    approach_z = 1

    msg = MAV_APM.MAVLink_set_home_position_message(
            int(target_system),
            int(lattitude),
            int(longitude),
            int(altitude),
            x,
            y,
            z,
            q,
            approach_x,
            approach_y,
            approach_z)

    send_message(msg, mav, pub)

class OriginSetterNode(Node):

    def __init__(self):
        super().__init__('origin_setter')
        mavlink_pub = self.create_publisher(Mavlink, "/uas1/mavlink_sink", 20)
        f = fifo()
        mav = MAV_APM.MAVLink(f, srcSystem=1, srcComponent=1)
        # wait to initialize
        while mavlink_pub.get_subscription_count() <= 0:
            self.get_logger().info('SET ORIGIN: Waiting FCU...')
            pass
   
        # for _ in range(2):
        #     print("Here")
        time.sleep(1)
        self.get_logger().info('Publishing1')
        set_global_origin(mav, mavlink_pub)
        set_home_position(mav, mavlink_pub)


def main(args=None):
    rclpy.init(args=args)
    origin_setter = OriginSetterNode()
    origin_setter.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()