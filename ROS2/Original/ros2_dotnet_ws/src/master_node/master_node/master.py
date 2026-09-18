import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import signal
import os
import time
import psutil

class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')
        self.subscription = self.create_subscription(
            String,
            'switch_nodes_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.active_processes = {}

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        command = msg.data.split(";")
        if command[0] == 'start':
            self.start_node(command[1], command[2]) #package, node
        elif command[0] == 'stop':
            self.stop_node(command[1],command[2]) #package, node
        elif command[0] == 'launch':
            self.launch_node(command[1],command[2]) #package, launchfile
        elif command[0] == 'script':
            self.run_script(command[1],command[2]) #folder, script
        elif command[0] == "reinitialize":
            self.reinit_IMU_GYRO()

    def reinit_IMU_GYRO(self):
        self.get_logger().info(f'RESTART')
        script="restart.py"
        folder = "~/MavScripts/"
        folder = os.path.expanduser(folder)
        process = subprocess.Popen(['python3', os.path.join(folder, script)])
        #process = subprocess.Popen(['ros2', 'service', 'call', '/mavros/cmd/command', 'mavros_msgs/srv/CommandLong', "{broadcast: false, command: 241, confirmation: 0, param1: 1, param2: 0, param3: 0, param4: 0, param5: 4, param6: 0, param7: 0}"])

    def start_node(self, package_name, node_name):
        process_key = package_name + " " + node_name
        if process_key not in self.active_processes:
            self.get_logger().info(f'Starting node: {node_name} from package {package_name}')
            process = subprocess.Popen(['ros2', 'run', package_name, node_name]) #(f'ros2 run {package_name} {node_name}',shell=True)
            self.active_processes[process_key] = process
            time.sleep(5)
        else:
            self.get_logger().info(f'Node {node_name} from package{package_name} is already running')

    def stop_node(self, package_name, node_name):
        process_key = package_name + " " + node_name
        if process_key in self.active_processes:
            self.get_logger().info(f'Stopping node: {node_name} from package {package_name}')
            process = self.active_processes.pop(process_key)
            
            try:
                # Use psutil to find the process and forcefully terminate it
                parent_process = psutil.Process(process.pid)
                
                for child in parent_process.children(recursive=True):
                    print(f"Sending SIGINT to child process {child.pid}")
                    child.send_signal(signal.SIGINT)
                time.sleep(1)    


                if any(child.is_running() for child in parent_process.children(recursive=True)):
                    print("Child processes are still running, sending SIGTERM.")
                    for child in parent_process.children(recursive=True):
                        child.send_signal(signal.SIGTERM)
                    time.sleep(1)

                parent_process.send_signal(signal.SIGINT)
                time.sleep(1)

                if parent_process.is_running():
                    parent_process.send_signal(signal.SIGTERM)
                    time.sleep(1)

                if any(child.is_running() for child in parent_process.children(recursive=True)):
                    print(f"I will kill this buddy {node_name} with his ID {process.pid} and his children (c)OS")
                    for child in parent_process.children(recursive=True):
                        child.kill()
                    if parent_process.is_running():    
                        parent_process.kill()  # Kill the parent process (your node)
                    time.sleep(1)


            except psutil.NoSuchProcess:
                print(f"No such process with PID {process.pid}, already terminated.")

        else:
            self.get_logger().info(f'Node {node_name} from package {package_name} is not running')

    def launch_node(self, package_name, node_name):
        process_key = package_name + " " + node_name
        if process_key not in self.active_processes:
            self.get_logger().info(f'Starting node: {node_name} from package {package_name}')
            process = subprocess.Popen(['ros2', 'launch', package_name, node_name])
            self.active_processes[process_key] = process
        else:
            self.get_logger().info(f'Node {node_name} from package {package_name} is already running')
    
    def run_script(self, folder, script):
        process_key = folder + " " + script
        folder = os.path.expanduser(folder)
        if process_key not in self.active_processes:
            self.get_logger().info(f'Starting script: {script} from folder {folder}')
            process = subprocess.Popen(['python3', os.path.join(folder, script)])
            self.active_processes[process_key] = process
        else:
            self.get_logger().info(f'Script {script} from folder {folder} is already running')



def main(args=None):
    rclpy.init(args=args)
    master_node = MasterNode()
    rclpy.spin(master_node)
    master_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()