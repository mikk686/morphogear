import numpy as np
import serial
import array
import time
import binascii
import sys
import numpy

class P2P_RTU():
    def __init__(self, port_name: str, baud_rate: int)->None:
        self.port = serial.Serial()
        self.port_name = port_name
        self.baud_rate = baud_rate
    
    def open_com_port(self)->None:
        if self.port.isOpen()==True:
            self.port.close()
        self.port = serial.Serial(self.port_name, self.baud_rate, timeout=0.1, xonxoff = False, rtscts = False, dsrdtr = False)
        self.port.setRTS(False)
        self.port.setDTR(False) 

    def close_com_port(self)->None:
        if self.port.isOpen()==True:
            self.port.close()

    def __calc_crc(self, packet: bytearray)->int:
        crc = 0
        for ik in range(2,packet[2]+2):
            crc += packet[ik]
        crc = ~crc
        crc &= 0xFF
        return crc
    
    def __verify_response(self, response: bytearray, response_length: bytes)->int:
        if len(response)<response_length:
            return 1
        if (response[0]!=0xFF) or (response[1]!=0xFF):
            return 1
        if (response[2]+3)!=len(response):
            return 1
        crc = self.__calc_crc(response[:-1])
        if crc!=response[-1]:
            return 1
        return 0

    def send_request(self, cmd: bytes, data: bytearray)->None:
        request = bytearray([0xFF,0xFF,len(data)+2,cmd]) + data
        request += bytearray([self.__calc_crc(request)])
        self.port.reset_input_buffer()
        self.port.reset_output_buffer()
	#self.port.flushInput()
        self.port.write(request)
        
    def receive_response(self, data_length: bytes)->list:
        response = self.port.read(data_length+5)
        error = self.__verify_response(response, data_length+5)
        return error, response[4:-1]
