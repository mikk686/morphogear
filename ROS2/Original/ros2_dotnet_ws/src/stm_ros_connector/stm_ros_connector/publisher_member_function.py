#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import serial
import struct
import itertools
import datetime
import time
from std_msgs.msg import String
#from unity_robotics_demo_msgs.msg import Angles
from std_msgs.msg import Int8MultiArray
from custom_control_msg.msg import Torque 
#from custom_control_msg.msg import Wrist
from std_msgs.msg import Int16MultiArray
from custom_control_msg.msg import Speed 

from .p2p_rtu import P2P_RTU 
from threading import Lock


class STM(Node):
    def __init__(self, serial_port_name:str, baudrate=921600): 
        super().__init__("stm_node")
        self.anglesub=self.create_subscription(Int8MultiArray, "/angles_control",self.set_angles_callback,10)
        #self.wristsub=self.create_subscription(Wrist, "/wrist",self.set_wrist_callback,10)
        self.wristsub=self.create_subscription(Int16MultiArray, "/wrist",self.set_wrist,10)
        self.speedsub=self.create_subscription(Speed, "/speed",self.set_speed_callback,10)
        self.torquepub = self.create_publisher(Torque, "/torque", 10) ##
        self.set_speed_cmd = 0x03
        self.set_mx_angles = 0x01
        self.set_ax_angles = 0x02
        self.get_torque=0x04
        #self.timer = self.create_timer(0.05, self.torque_callback)

        self.mutex = Lock()
        self.p2p_rtu = P2P_RTU(serial_port_name, baudrate)
        self.p2p_rtu.open_com_port()
        self.calibration = [246, 181, 180,
                            228, 180, 146,# 268,
                            228, 177, 180,
                            238, 178, 187]
        self.initspeed = [int(120),int(120),int(120),
                          int(120),int(120),int(120),
                          int(120),int(120),int(120),
                          int(120),int(120),int(120),
                          int(0),int(0),int(0),int(0)]


    def torque_callback(self):
        msg = Torque()
        try:
            tmp = self.get_torques()
        except:
            return
        msg.lf_trq=tmp[0]
        msg.rf_trq=tmp[1]
        msg.rh_trq=tmp[2]
        msg.lh_trq=tmp[3]
        self.torquepub.publish(msg)
        self.get_logger().info('Torques on AX: "%s"' % msg)

    def get_torques(self)-> list:
        self.p2p_rtu.send_request(self.get_torque,bytearray(range(0)))
        error_code,data=self.p2p_rtu.receive_response(32)
        if (error_code):
            self.get_logger().info('Cant get Torque')
            raise NameError('Cant get Torques')
        MX1061=int.from_bytes(data[2:4],'little',signed=True)
        MX1062=int.from_bytes(data[8:10],'little',signed=True)
        MX1063=int.from_bytes(data[14:16],'little',signed=True)
        MX1064=int.from_bytes(data[20:22],'little',signed=True)
        torques=[MX1061,MX1062,MX1063,MX1064]
        #self.get_logger().info('get: "%s"' % torques)
        return torques

    def set_speed_init(self): ##
        time.sleep(0.5)
        self.get_logger().info('START RESET')
        self.p2p_rtu.send_request(0x06,bytearray(range(0))) #REBOOT
        time.sleep(0.5)
        self.get_logger().info('Publishing speed: "%s"' % self.initspeed)
        self.__send_write_request(self.set_speed_cmd, self.initspeed)
        time.sleep(0.5)
        self.p2p_rtu.send_request(0x05,bytearray(range(0)))  ##ENABLE TORQUE AFTER OBTAINING SPEED
        self.get_logger().info('Reboot+Speed+Enable Torque')

    def set_speed_callback(self, data): ##
        time.sleep(0.1)
        self.get_logger().info('START RESET')
        parsed_data = self.__parse_data_mx_ax(data)
        self.p2p_rtu.send_request(0x06,bytearray(range(0))) #REBOOT
        time.sleep(0.3)
        self.get_logger().info('Publishing speed: "%s"' % parsed_data)
        self.__send_write_request(self.set_speed_cmd, parsed_data)
        time.sleep(0.3)
        self.p2p_rtu.send_request(0x05,bytearray(range(0)))  ##ENABLE TORQUE AFTER OBTAINING SPEED
        self.get_logger().info('Reboot+Speed+Enable Torque')

    def __parse_data_mx(self, data)->list:
        data_split = [self.calibration[0] - int (data[0]), self.calibration[1] + int (data[4]), self.calibration[2] - int (data[8]),
                      self.calibration[3] - int (data[1]), self.calibration[4] + int (data[5]), self.calibration[5] - int (data[9]),
                      self.calibration[6] - int (data[2]), self.calibration[7] + int (data[6]), self.calibration[8] - int (data[10]),
                      self.calibration[9] - int (data[3]), self.calibration[10]+ int (data[7]), self.calibration[11]- int (data[11])]
        return data_split

    def __parse_data_mx_ax(self, data)->list: 
        parsed_data = [int (data.rf_shoulder_speed),int (data.rf_upperarm_speed),int (data.rf_forearm_speed),
                      int (data.rh_shoulder_speed),int (data.rh_upperarm_speed),int (data.rh_forearm_speed),
                      int (data.lh_shoulder_speed),int (data.lh_upperarm_speed),int (data.lh_forearm_speed),
                      int (data.lf_shoulder_speed),int (data.lf_upperarm_speed),int (data.lf_forearm_speed),
                      int (data.rf_wrist_speed),int (data.rh_wrist_speed), int (data.lh_wrist_speed), int (data.lf_wrist_speed)]
        return parsed_data


    def set_wrist_callback(self, data): 
        parsed_data = self.__parse_data_ax(data)
        self.get_logger().info('Publishing gripper: "%s"' % parsed_data)
        self.__send_write_request(self.set_ax_angles, parsed_data)

    def set_wrist(self, msg): 
        parsed_data = self.__parse_data_ax_msg(msg)
        self.get_logger().info('Publishing gripper: "%s"' % parsed_data)
        self.__send_write_request(self.set_ax_angles, parsed_data)

    def __parse_data_ax(self, data)->list: 
        parsed_data = [int (data.rf_wrist),int (data.lf_wrist), int (data.lh_wrist), int (data.rh_wrist)]
        return parsed_data


    def __parse_data_ax_msg(self, msg)->list: 
        parsed_data = [int (msg.data[0]),int (msg.data[3]), int (msg.data[2]), int (msg.data[1])]
        return parsed_data

    def set_angles_callback(self, data):
        parsed_data = self.__parse_data_mx(data.data)
        self.get_logger().info('Publishing angles: "%s"' % parsed_data)
        self.__send_write_request(self.set_mx_angles, parsed_data)
        error, x = self.p2p_rtu.receive_response(0)
        if (not error):
            print(x)
            #self.torque_callback()
        else:
            print(x)
            self.get_logger().info('Problems in recieving angles')

    def parse_data_mx(self, data)->list:
        data_split = [246 - int (data.rf_shoulder),181 + int (data.rf_upperarm),180 - int (data.rf_forearm),
                      228 - int (data.lf_shoulder),180 + int (data.lf_upperarm),268 - int (data.lf_forearm),
                      228 - int (data.lh_shoulder),177 + int (data.lh_upperarm),180 - int (data.lh_forearm),
                      238 - int (data.rh_shoulder),178 + int (data.rh_upperarm),187 - int (data.rh_forearm)]
        return data_split

    def __send_write_request(self, cmd: bytes, data: list)->bool: 
        self.mutex.acquire()
        request_data = bytearray()
        for value in data:
            request_data += int.to_bytes(value,2,'little')
        self.p2p_rtu.send_request(cmd, request_data)
        self.mutex.release()
        return True



    
def main(args=None):
    rclpy.init(args=args)
    serial_port = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
    stm = STM(serial_port)
    stm.set_speed_init()
    rclpy.spin(stm)
    stm.destroy_node()
    rclpy.shutdown()
