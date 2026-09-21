from p2p_mavlink import P2P_FCU
import threading 
import time


class PIDController():
    def __init__(self)->None:
        self.p2p=P2P_FCU() 
        self.PID_pitch=PID(3.,0.,0.) #1
        self.PID_roll=PID(2.,0.,0.)
        self.PID_pitchspeed=PID(0.6,0.,0.)
        self.PID_rollspeed=PID(0.4,0.,0.)
        self.running=False

    def applyAttitudeControl(self):
        PIDPitchRateOut = self.PID_pitch.pid_output(-self.p2p.pitch , 0.1)
        PIDRollRateOut = self.PID_roll.pid_output(-self.p2p.roll, 0.1)
        self.applyAttitudeRateControl(PIDPitchRateOut,  PIDRollRateOut)


    def applyAttitudeRateControl(self, PIDPitchRateOut, PIDRollRateOut):
        attitudePitchError = PIDPitchRateOut - self.p2p.pitchspeed
        attitudeRollError = PIDRollRateOut - self.p2p.rollspeed
        PIDPitchRateOut = self.PID_pitchspeed.pid_output(attitudePitchError, 0.1)
        PIDRollRateOut = self.PID_rollspeed.pid_output(attitudeRollError, 0.1)

        # Calculate prop forces using arrays
        propForces = [
            (-PIDPitchRateOut + PIDRollRateOut),   # Front Right
            (PIDRollRateOut),                       # Right
            (PIDPitchRateOut + PIDRollRateOut),    # Back Right
            (PIDPitchRateOut - PIDRollRateOut),    # Back Left
            (-PIDRollRateOut),                      # Left
            (-PIDPitchRateOut - PIDRollRateOut)    # Front Left
        ]

        # Print the prop forces
        #print(*propForces)  # Unpack the list for printing
        self.send_Forces(propForces)

    def send_Forces(self, propForces):
        for i, force in enumerate(propForces):
            force_scaled = int(-force * 100)
            force_clamped = max(min(force_scaled, 50), 0)
            #print(f"Prop Force {i+1}: {force_clamped}")
            # Uncomment to send the forces to the thrust control
            self.p2p.set_thrust(i+1, force_clamped, 0.15)  #0.15

    def control_by_PID(self):
        while self.running:
            time.sleep(0.1)
            self.applyAttitudeControl()

    def start_PID_stabilize(self):
        self.running=True
        # Start the data collection in a new thread
        self.p2p.start_data_collection()
        thread = threading.Thread(target=self.control_by_PID)
        thread.start()

    def stop_PID_stabilize(self):
        self.p2p.stop_data_collection()
        self.running=False


class PID:
    def __init__(self, P, I, D):
        self.error_old = 0.0
        self.error_old_2 = 0.0
        self.error_sum = 0.0

        # PID parameters
        self.gain_P = P
        self.gain_I = I
        self.gain_D = D
        # Sometimes you have to limit the total sum of all errors used in the I
        self.error_sumMax = 2.0 #20

    def pid_output(self, error, time):
        # The output from PID
        output = 0.0
        output += self.gain_P * error # P

        self.error_sum += time * error 
        # Clamp the sum 
        self.error_sum = max(min(self.error_sum, self.error_sumMax), -self.error_sumMax)

        output += self.gain_I * self.error_sum # I

        d_dt_error = (error - self.error_old) / time # D # Assuming 'time' is the time since last update

        # Save the last errors
        self.error_old_2 = self.error_old
        self.error_old = error

        output += self.gain_D * d_dt_error

        return output



 

if __name__ == '__main__':
    pid=PIDController()
    pid.stop_PID_stabilize()
    pid.start_PID_stabilize()
    try:
        # The main thread can perform other tasks here
        while True:
            pass  # Replace with any other necessary functionality
    except KeyboardInterrupt:
        print("Stopping pid collection...")
        pid.stop_PID_stabilize()
