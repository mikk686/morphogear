from pymavlink import mavutil
import time

'''
GPT Generation
https://www.getmerlin.in/share/chat/38nB74MhAST
'''

# Connect to the vehicle
master = mavutil.mavlink_connection("/dev/ttyACM0",baud=57600)  # Adjust as needed
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received!")

'''
Unable to set PWM directly while FLight Controller is using it
'''
# # Set PWM values for motors (values range from 1000 to 2000)
# motor_pwm_values = [1500, 1500, 1500, 1500, 1500, 1500]  # Example for 4 motors

# # master.arducopter_arm()
# # time.sleep(1) 

# # while True:
# #     print("send cmd")
# #     for i in range(6):  # Assuming a quadcopter with 4 servos/motors
# #         master.mav.command_long_send(
# #             master.target_system,
# #             master.target_component,
# #             mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
# #             0,  # Confirmation
# #             i + 1,  # Servo channel number (AUX1 starts from 9 for Pixhawk)
# #             motor_pwm_values[i],  # PWM value
# #             0, 0, 0, 0, 0
# #         )


'''
Using function TestMotors Solution
'''

# Example: Perform a motor test (motor 1, spin at 50% speed for 2 seconds)
motor_index = 1  # Motor index (1 for PWM1, 2 for PWM2, etc.), starts at 1
throttle_percentage = 50  # Throttle in percentage (0-100)
duration = 0.2  # Duration in seconds


while True:
    time.sleep(0.5)
    motor_index=motor_index+1
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,  # Confirmation
        motor_index,  # Motor index
        mavutil.mavlink.MOTOR_TEST_THROTTLE_PERCENT,  # Test type (use throttle percentage)
        throttle_percentage,  # Throttle value
        duration,  # Duration
        0, 0, 0  # Unused parameters
    )
    motor_index=motor_index%6