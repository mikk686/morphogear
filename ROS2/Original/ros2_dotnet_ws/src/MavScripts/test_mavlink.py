"""
Example of how to read all the parameters from the Autopilot with pymavlink
"""

import time
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection("/dev/ttyACM0",baud=57600)
print(master)

# Wait for a heartbeat before sending commands
connection = master.wait_heartbeat()
print(connection)


while True:
    msg = master.recv_match(blocking=True)
    print(msg)
    time.sleep(1)
