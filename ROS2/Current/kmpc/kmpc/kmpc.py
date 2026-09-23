#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time 
import numpy as np
import casadi as ca
import quaternion
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Empty, Bool, String
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int8MultiArray, Int32

class MPCNode(Node):
    def __init__(self):
        super().__init__("mpc_node")
        #-------------------------Subscribers
        self.vicon_point_sub = self.create_subscription(
            TransformStamped, '/vicon/morphogear', self.vicon_point_callback, 10)
        self.request_theta_sub = self.create_subscription(
            Empty, '/request_angles', self.request_theta_callback, 10)
        self.corner_point_sub = self.create_subscription(
            PoseArray, '/corner_points', self.point_callback, 10)
        self.stop_mpc_sub = self.create_subscription(
            Empty, '/stop_mpc', self.stop_mpc_callback, 10)

        #-------------------------Publishers
        self.gait1_publisher = self.create_publisher(String, "/morphogear_sudo_cmd", 1)
        self.request_theta_pub = self.create_publisher(Empty, "/request_angles", 1)
        self.theta_pub = self.create_publisher(Int8MultiArray, '/theta_angles', 1)
        self.trajectory_size_pub = self.create_publisher(Int32, '/trajectory_size', 1)

        #-------------------------Initialisers
        self.vicon_x = []
        self.vicon_y = []
        self.vicon_z = []
        self.object = False
        self.condition = False
        self.points_received = False
        self.mpc_enabled = True  # Flag to control MPC execution
        self.vicon_data = np.zeros(3)
        self.orientation = np.quaternion()
        self.yaw = 0
        self.current_point_index = 0  # Start from 0 to target first point
        self.point_tolerance = 15
        self.points = []
        self.trajectory_num_points = 200  # Default, configurable

    def vicon_point_callback(self, msg):
        self.vicon_data = np.array([
            msg.transform.translation.x, 
            msg.transform.translation.y, 
            msg.transform.translation.z
        ])
        self.orientation = np.quaternion(
            msg.transform.rotation.x, 
            msg.transform.rotation.y, 
            msg.transform.rotation.z,
            msg.transform.rotation.w
        )
        self.yaw = np.arctan2(
            2 * (self.orientation.x * self.orientation.w + self.orientation.y * self.orientation.z), 
            1 - 2 * (self.orientation.z**2 + self.orientation.w**2)
        )
        self.vicon_data = self.vicon_data * 100

        x = msg.transform.translation.x
        y = msg.transform.translation.y
        z = msg.transform.translation.z

        if self.condition == False:
            self.vicon_x.append(x)
            self.vicon_y.append(y)
            self.vicon_z.append(z)

    def point_callback(self, msg):
        self.x_cord = []
        self.y_cord = []
        self.z_cord = []
        self.points = []

        for point in msg.poses:
            x = point.position.x
            y = point.position.y
            z = point.position.z
            self.points.append(np.array([x, y, z]) * 100)
            self.x_cord.append(x)
            self.y_cord.append(y)
            if z == 0:
                z = z + 0.4
            self.z_cord.append(z)

        self.current_point_index = 0
        msg = Empty()
        self.request_theta_pub.publish(msg)
        self.points_received = True
        self.mpc_enabled = True
        print(f'Received {len(self.points)} corner points')

    def stop_mpc_callback(self, msg):
        print('----------------------------------------------MPC STOPPED----------------------------------------------')
        self.mpc_enabled = False
        self.condition = False
        self.object = False

    def request_theta_callback(self, event): 
        if not self.mpc_enabled:
            print('MPC is disabled - skipping execution')
            return

        print('----------------------------------------------Angles Requested----------------------------------------------')
        self.condition = False
        self.object = False
        #time.sleep(0.01)

        if self.points_received:
            self.MPC_activation_PHD()

    def MPC_activation_PHD(self):
        if not self.mpc_enabled:
            return

        vicon_point = self.vicon_data

        if len(self.points) == 0:
            print("-------Trajectory is empty-------")
            self.current_point_index = 0
            return

        if self.current_point_index >= len(self.points):
            print("-------ALL TARGETS REACHED---------")
            msg = String()
            msg.data = 'initial'
            self.gait1_publisher.publish(msg)
            self.current_point_index = 0
            self.points_received = False
            return

        target_point_global = self.points[self.current_point_index]
        distance_to_target = np.linalg.norm(target_point_global[0:2] - vicon_point[0:2])

        if distance_to_target < self.point_tolerance:
            print(f'---Point {self.current_point_index} Reached---')
            self.current_point_index += 1

            if self.current_point_index >= len(self.points):
                print("-------ALL TARGETS REACHED---------")
                msg = String()
                msg.data = 'initial'
                self.gait1_publisher.publish(msg)
                self.current_point_index = 0
                self.points_received = False
                return

            target_point_global = self.points[self.current_point_index]
            distance_to_target = np.linalg.norm(target_point_global - vicon_point)

        target_vector = target_point_global - vicon_point

        print(f'Yaw angle: {np.rad2deg(self.yaw):.2f} deg')

        rotMatrix = np.array([
            [np.cos(self.yaw + np.pi * 3/4), -np.sin(self.yaw + np.pi * 3/4)], 
            [np.sin(self.yaw + np.pi * 3/4),  np.cos(self.yaw + np.pi * 3/4)]
        ])

        target_point_local = np.dot(rotMatrix, np.transpose(target_vector[:2]))

        print(f'Vicon point: {vicon_point}')
        print(f'Target point {self.current_point_index}: {target_point_global}')
        print(f'Robot orientation (euler): {np.rad2deg(quaternion.as_euler_angles(self.orientation))}')
        print(f'Local target point: {target_point_local}')
        print(f'Distance to target: {distance_to_target:.2f} cm')

        if not self.object:
            self.KMPC(target_point_local[:2])

    def KMPC(self, target_point):
        N = 5

        theta = 45 * np.pi / 180
        lf = 15.5  
        lt = 27.5
        initialStandingAngle = 25
        robot_height = np.sin(initialStandingAngle * np.pi / 180) * lf + lt

        step_length_max = np.sqrt(((lf + lt)**2) - robot_height**2) * 0.85
        step_length_min = -step_length_max

        x = ca.SX.sym("x")
        y = ca.SX.sym("y")
        states = ca.vertcat(x, y)
        n_states = states.numel()

        a = ca.SX.sym('a')
        controls = ca.vertcat(a)
        n_controls = controls.numel()

        initial = [0, 0] 
        target = target_point

        RHS = ca.vertcat(
            a * ca.cos(theta), 
            a * ca.sin(theta)
        )
        f = ca.Function('f', [states, controls], [RHS])

        U = ca.SX.sym('U', n_controls, N)
        P = ca.SX.sym('P', n_states + n_states) 
        X = ca.SX.sym('X', n_states, (N + 1))
        X[:, 0] = P[:n_states]

        for i in range(N):
            st = X[:, i]
            con = U[:, i]
            f_value = f(st, con)
            f_value[-1] *= (-1)**i
            st_next = st + f_value
            X[:, i + 1] = st_next

        obj = 0
        g = []

        Qx = 3
        Q = ca.diagcat(Qx, Qx) 
        R1 = 0.2
        R = ca.diagcat(R1)

        for i in range(N):
            st = X[:, i]
            con = U[:, i]
            obj = obj + (st - P[n_states:2*n_states]).T @ Q @ (st - P[n_states:2*n_states]) + con.T @ R @ con        

        for i in range(N + 1):
            g.append(X[:, i])     

        opt_variables = ca.reshape(U, -1, 1)

        nlp_prob = {
            'f': obj,
            'x': opt_variables,
            'g': ca.vcat(g),
            'p': P
        }    

        solver_opts = {
            'ipopt': {
                'max_iter': 200,
                'print_level': 0,
                'acceptable_tol': 1e-8,
                'acceptable_obj_change_tol': 1e-6
            },
            'print_time': 0
        }

        solver = ca.nlpsol('solver', 'ipopt', nlp_prob, solver_opts)

        lbg = -10000
        ubg = 10000

        lbx = []
        ubx = []
        for _ in range(N):
            lbx.append(step_length_min * np.cos(theta))
            ubx.append(step_length_max * np.cos(theta))

        args = { 
            'lbg': lbg,  
            'ubg': ubg,
            'lbx': lbx,
            'ubx': ubx,
        }

        x0 = np.array(initial).reshape(-1, 1)
        u0 = np.zeros((N, n_controls))
        xs = np.array(target).reshape(-1, 1)

        def DM2Arr(dm):
            return np.array(dm.full())

        args['p'] = ca.vertcat(x0, xs)
        args['x0'] = ca.vertcat(ca.reshape(u0, n_controls * N, 1))

        sol = solver(
            x0=args['x0'],
            lbx=args['lbx'],
            ubx=args['ubx'],
            lbg=args['lbg'],
            ubg=args['ubg'],
            p=args['p']
        )

        u_sol = ca.reshape(sol['x'][:n_controls * (N)], n_controls, N)

        step_length1 = DM2Arr(u_sol[:, 0])[0][0]
        step_length2 = DM2Arr(u_sol[:, 1])[0][0]

        print(f'Step lengths: {step_length1:.2f}, {step_length2:.2f}')

        def trajectory_generator(step_length, num_points=200): 
            step_length_max = np.sqrt(((lf + lt)**2) - robot_height**2)
            theta = np.linspace(np.pi, 0, num_points // 2)
            xxx = step_length_max - step_length
            r = np.linspace(0, step_length, num_points // 2) 
            xstep = np.linspace(xxx, step_length_max * 0.9, num_points // 2)
            xspiral = -r * np.cos(theta).flatten() + step_length_max * 0.9
            ystep = -robot_height * np.ones_like(xstep)

            half_points = num_points // 4
            for i in range(half_points):
                ystep[i] *= 1 + i / 1000
                if i + half_points < len(ystep):
                    ystep[i + half_points] *= 1.05 - i / 1000

            for i in range(min(10, len(xspiral))):
                xspiral[-i - 1] = xspiral[-10] if len(xspiral) >= 10 else xspiral[-1]

            yspiral = (r * np.sin(theta) * 0.5).flatten() - robot_height
            trajectoryX = np.concatenate((xspiral[half_points:], xstep, xspiral[:half_points]))      
            trajectoryY = np.concatenate((yspiral[half_points:], ystep, yspiral[:half_points]))
            return trajectoryX, trajectoryY

        def Inverse_Kinematics(x, y):
            L1 = lf
            L2 = lt
            c2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
            c2 = np.clip(c2, -1, 1)
            s2 = (1 - c2**2)**0.5
            th2 = -np.arccos(c2)
            th1 = np.arctan2(y, x) + np.arctan2(L2 * s2, L1 + L2 * c2)
            return ([th1 / np.pi * 180, th2 / np.pi * 180])

        x_traj_1, y_traj_1 = trajectory_generator(abs(step_length1), self.trajectory_num_points)
        x_traj_2, y_traj_2 = trajectory_generator(abs(step_length2), self.trajectory_num_points)

        angles_1 = Inverse_Kinematics(x_traj_1, y_traj_1)
        angles_2 = Inverse_Kinematics(x_traj_2, y_traj_2)

        t1_1 = np.array(angles_2)[0, :].astype(int)
        t2_1 = np.array(angles_2)[1, :].astype(int)
        t1_2 = np.array(angles_1)[0, :].astype(int)
        t2_2 = np.array(angles_1)[1, :].astype(int)

        if step_length2 > 0:
            half_idx = self.trajectory_num_points // 2
            t1_BL = np.concatenate((t1_1[half_idx:], t1_1[:half_idx]))
            t2_BL = np.concatenate((t2_1[half_idx:], t2_1[:half_idx]))
            t1_FR = t1_BL[::-1]
            t2_FR = t2_BL[::-1]
        else:
            half_idx = self.trajectory_num_points // 2
            t1_FR = np.concatenate((t1_1[half_idx:], t1_1[:half_idx]))
            t2_FR = np.concatenate((t2_1[half_idx:], t2_1[:half_idx]))
            t1_BL = t1_FR[::-1]
            t2_BL = t2_FR[::-1]

        if step_length1 > 0:
            t1_BR = t1_2
            t2_BR = t2_2
            t1_FL = t1_BR[::-1]
            t2_FL = t2_BR[::-1]
        else:
            t1_FL = t1_2
            t2_FL = t2_2
            t1_BR = t1_FL[::-1]
            t2_BR = t2_FL[::-1]

        theta_msg = Int8MultiArray()
        theta_msg.data.extend(t1_FR.tolist()) 
        theta_msg.data.extend(t2_FR.tolist())
        theta_msg.data.extend(t1_BR.tolist())
        theta_msg.data.extend(t2_BR.tolist())
        theta_msg.data.extend(t1_BL.tolist())
        theta_msg.data.extend(t2_BL.tolist())
        theta_msg.data.extend(t1_FL.tolist())
        theta_msg.data.extend(t2_FL.tolist())

        size_msg = Int32()
        size_msg.data = self.trajectory_num_points
        self.trajectory_size_pub.publish(size_msg)

        self.theta_pub.publish(theta_msg)
        print(f'Angles Published - {self.trajectory_num_points} points per leg')

def main(args=None):
    rclpy.init(args=args)
    mpc = MPCNode()
    rclpy.spin(mpc)
    mpc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
