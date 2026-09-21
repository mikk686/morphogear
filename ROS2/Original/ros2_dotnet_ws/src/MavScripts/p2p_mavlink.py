from pymavlink import mavutil
import time
import numpy as np
import threading

class P2P_FCU():
    def __init__(self)->None:
    # Connect to the vehicle
        self.master = mavutil.mavlink_connection("/dev/ttyACM0",baud=921600)  # Adjust as needed
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print("Heartbeat received!")
        self.roll=0.  #rad
        self.pitch=0. #rad
        self.yaw=0.   #rad
        self.rollspeed=0.   #rad/s
        self.pitchspeed=0.  #rad/s
        self.yawspeed=0.    #rad/s

        self.data_collection = True


    def set_thrust(self, motor_index: int, throttle_percentage: int, duration: float):
            duration = np.clip(duration, 0., 1.)            #Safety switch
            throttle_percentage = np.clip(throttle_percentage, 0, 50)
            self.master.mav.command_long_send(
                                            self.master.target_system,
                                            self.master.target_component,
                                            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                                            0,  # Confirmation
                                            motor_index,  # Motor index
                                            mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT,  # Test type (use throttle percentage)
                                            throttle_percentage,  # Throttle value
                                            duration,  # Duration
                                            0, 0, 0  # Unused parameters
                                        )
            
    def request_message(self):
        # Set the message interval for SCALED_IMU3
        message_id = mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE
        interval = 100000  # Set interval to 0.1 second (in microseconds) for demonstration
        command_id = mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL

        print(f"Setting message interval for ATTITUDE to {interval} microseconds...")
        self.master.mav.command_long_send(
        self.master.target_system,
        self.master.target_component,
            command_id,
            0,  # Confirmation
            message_id,  # Message ID to set interval for
            interval,   # Desired interval in microseconds
            0,  # Param3 (unused)
            0,  # Param4 (unused)
            0,  # Param5 (unused)
            0,  # Param6 (unused)
            0   # Param7 (unused)
        )

        # Wait for an acknowledgement
        response = self.master.recv_match(type='COMMAND_ACK', blocking=True)
        if response and response.command == command_id and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Message interval set successfully.")
        else:
            print("Failed to set message interval.")
        
        while self.data_collection:
            # Wait for the SCALED_IMU3 message
            message = self.master.recv_match(type='ATTITUDE', blocking=False)
            if message:
                #print(message)

                self.roll=message.roll
                self.pitch=message.pitch
                self.yaw=message.yaw
                self.rollspeed=message.rollspeed
                self.pitchspeed=message.pitchspeed
                self.yawspeed=message.yawspeed
                # self.xacc = message.xacc    #Mg vector direction
                # self.yacc = message.yacc
                # self.zacc = message.zacc

                # self.xgyro = message.xgyro #angular velocity
                # self.ygyro = message.ygyro
                # self.zgyro = message.zgyro
          
    def start_data_collection(self):
        self.running=True
        # Start the data collection in a new thread
        thread = threading.Thread(target=self.request_message)
        thread.start()

    def stop_data_collection(self):
        message_id = mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE
        command_id = mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
        # Stop the data collection loop
        self.running = False
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            command_id,
            0,  # Confirmation
            message_id,  # Message ID to set interval for
            0,   # DEFAULT Desired interval in microseconds
            0,  # Param3 (unused)
            0,  # Param4 (unused)
            0,  # Param5 (unused)
            0,  # Param6 (unused)
            0   # Param7 (unused)
        )
        print("data collection is stopped")


#TEST FUNCTION

if __name__ == '__main__':
    p2p_protocol = P2P_FCU()
    p2p_protocol.start_data_collection()

    p2p_protocol.set_thrust(1,15,0.1)  #15 is minimum

    try:
        # The main thread can perform other tasks here
        while True:
            #p2p_protocol.set_thrust(1,15,0.01)
            # time.sleep(0.1)
            pass  # Replace with any other necessary functionality
    except KeyboardInterrupt:
        print("Stopping data collection...")
        p2p_protocol.stop_data_collection()