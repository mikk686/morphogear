from pymavlink import mavutil

master = mavutil.mavlink_connection("/dev/ttyACM0",baud=921600)  # Adjust as needed
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received!")

command = mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN

# Send the command
master.mav.command_long_send(
    master.target_system,   # target system
    master.target_component, # target component
    command,                    # command
    0,                          # confirmation (0 for no confirmation)
    1,                          # param1 (not used)
    0,                          # param2 (not used)
    0,                          # param3 (not used)
    0,                          # param4 (not used)
    0,                          # param5 (not used)
    0,                          # param6 (not used)
    0                           # param7 (not used)
)

print("MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN command sent.")