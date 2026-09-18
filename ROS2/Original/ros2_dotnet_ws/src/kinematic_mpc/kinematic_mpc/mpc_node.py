#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time 
#import rospy
#import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import casadi as ca
import quaternion
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Empty, Bool, String
from geometry_msgs.msg import TransformStamped
#from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int8MultiArray

class MPCNode(Node):
    def __init__(self):
        super().__init__("mpc_node")
        #-------------------------Subscribers
        self.vicon_point_sub =self.create_subscription( TransformStamped, '/vicon/morphogear',self.vicon_point_callback,10)
        self.request_theta_sub = self.create_subscription( Empty, '/request_angles', self.request_theta_callback,10)
        self.corner_point_sub = self.create_subscription( PoseArray, '/corner_points', self.point_callback,10)
        self.request_fly_point_sub = self.create_subscription( Empty, '/request_fly_point', self.request_fly_point_callback,10)  

        

        #-------------------------Publishers
        self.gait1_publisher = self.create_publisher(String, "/morphogear_sudo_cmd",1)
        self.request_theta_pub = self.create_publisher(Empty, "/request_angles",1)
        self.theta_pub =self.create_publisher( Int8MultiArray, '/theta_angles', 1)
        self.fly_point_msg_pub = self.create_publisher( PoseStamped, '/fly_point', 1)
        self.fly_msg_pub = self.create_publisher(Bool, '/ly_condition', 1)
        self.land_msg_pub = self.create_publisher( Bool, '/land_condition', 1)
        self.target_pub = self.create_publisher( Bool, '/target_reached', 1)

        #-------------------------Initialisers
        self.vicon_x = []
        self.vicon_y = []
        self.vicon_z = []
        self.object = False
        self.condition = False
        self.points_received = False
        self.vicon_data = np.zeros(3)
        self.orientation = np.quaternion()
        self.yaw=0
        self.index = 1
        self.point_tolerance = 15
        self.path_tolerance = 15
        self.corner_tolerance = 15
        self.count = 0
    
    # -------------------Function to publish standing initial angles
    def standing_angles_message(self):
        theta_msg = Int8MultiArray()
        standing_angles_t1 = [-30]*200
        standing_angles_t2 = [-60]*200
        t1_FL = standing_angles_t1; t1_BR = standing_angles_t1; t1_FR = standing_angles_t1; t1_BL = standing_angles_t1
        t2_FL = standing_angles_t2; t2_BR = standing_angles_t2; t2_FR = standing_angles_t2; t2_BL = standing_angles_t2 
        theta_msg.data.extend(t1_FR) 
        theta_msg.data.extend(t2_FR)
        theta_msg.data.extend(t1_BR)
        theta_msg.data.extend(t2_BR)
        theta_msg.data.extend(t1_BL)
        theta_msg.data.extend(t2_BL)
        theta_msg.data.extend(t1_FL)
        theta_msg.data.extend(t2_FL)
        self.theta_pub.publish(theta_msg)

    # -------------------Function for vicon callback to get vicon coordinates
    def vicon_point_callback(self,msg):
        self.vicon_data = np.array([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
        self.orientation = np.quaternion(msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z,msg.transform.rotation.w)
        self.yaw = np.arctan2(2 * (self.orientation.x * self.orientation.w + self.orientation.y * self.orientation.z), 1 - 2 * (self.orientation.z**2 + self.orientation.w**2))
        #self.yaw+=np.pi
        self.vicon_data = self.vicon_data*100 # conversion from meters to centimeters
        x = msg.transform.translation.x
        y =  msg.transform.translation.y
        z = msg.transform.translation.z
        if self.condition == False:
            self.vicon_x.append(x)
            self.vicon_y.append(y)
            self.vicon_z.append(z)

    def vicon_storing(self):
        vc = self.vicon_x
        vy = self.vicon_y
        vz = self.vicon_z
        return vc, vy, vz
    # -------------------Function for point callback to get corner points of trajectory
    def point_callback(self, msg):
        self.x_cord = []
        self.y_cord = []
        self.z_cord = []
        self.points = []
        for point in msg.poses:
            x = point.position.x
            y = point.position.y
            z =  point.position.z
            self.points.append(np.array([x,y,z])*100)
            self.x_cord.append(x)
            self.y_cord.append(y)
            if z == 0:
                z = z + 0.4
            self.z_cord.append(z)
        msg = Empty()
        self.request_theta_pub.publish(msg)
        self.points_received = True

    def path_storing(self):
        self.x_path = self.x_cord
        self.y_path = self.y_cord
        self.z_path = self.z_cord
        return self.x_path, self.y_path, self.z_cord
    # -------------------Function to publish success message when target reached
    def success_message(self):
        self.success_msg = Bool()
        self.success_msg.data = True
        self.target_pub.publish(self.success_msg)
        print('Success Message Published')
    # -------------------Function to publish flight point message when obstacle detected
    def flight_message(self, target_point):
        fly_msg = PoseStamped()
        fly_msg.pose.position.x = target_point[0]
        fly_msg.pose.position.y = target_point[1]
        fly_msg.pose.position.z = target_point[2]
        self.fly_point_msg_pub.publish(fly_msg)
        if self.index < len(self.points)-1:
            self.index = self.index + 1 
    # -------------------Function to decide whether to fly or land
    def request_fly_point_callback(self, event):
        print('----------------------------------------------Flight Point Requested----------------------------------------------')
        target_point = self.points[self.index]
        target_point = target_point/100
        print('next flight point: ', target_point)
        if target_point[2] > 0.4:
            self.flight_message(target_point)
        else:
            self.land_msg = Bool()
            self.land_msg.data = True
            self.land_msg_pub.publish(self.land_msg)
            self.flight_message(target_point)
    # -------------------Function for limb angles callback to publish theta angles 
    def request_theta_callback(self, event): 
        print('----------------------------------------------Angles Requested----------------------------------------------')
        self.condition = False
        self.object = False
        time.sleep(0.1)
        if self.points_received:
            self.MPC_activation_PHD()

#------------------------------PHD LEVEL OF IMPLEMENTATION------------------------------------------------

    def MPC_activation_PHD(self):
        vicon_point = self.vicon_data
        try:
            target_point = self.points[self.index]-vicon_point
            if np.linalg.norm(target_point) < self.point_tolerance:
                print('---Point Reached---')
                self.index +=1
        except:
            print("-------Trajectrory is empty-------")
            self.index=1
            return 
        print(np.rad2deg(self.yaw)) #+np.pi
        rotMatrix = np.array([[np.cos(self.yaw+np.pi*3/4), -np.sin(self.yaw+np.pi*3/4)], 
                                 [np.sin(self.yaw+np.pi*3/4),  np.cos(self.yaw+np.pi*3/4)]]) #-45 deg to change orientation to diagonal between legs
        target_point = np.dot(rotMatrix,np.transpose(target_point[:2]))

        print('vicon point: ', vicon_point)
        try:
            print('target point: ', self.points[self.index])
        except:
            print("-------TARGET REACHED---------")
            msg=String()
            msg.data = 'initial'
            self.gait1_publisher.publish(msg)
            self.index=1
            return 
        print ('self orientation', np.rad2deg(quaternion.as_euler_angles(self.orientation)))
        print('local target point: ', target_point)
        print('corner point index: ', self.index)


        print("DISTANCE ", np.linalg.norm(target_point))
        # if self.index != (len(self.points)):
        #     if np.linalg.norm(target_point) < self.point_tolerance:
        #         print('---Point Reached---')
        #         self.index +=1


        # if self.points[self.index][2]/100 > 0.5:
        #     self.object = True
        #     print('Obstacle Detected, Required to Fly')
        #     self.success_message()
        #     self.fly_msg = Bool()
        #     self.fly_msg.data = True
        #     self.fly_msg_pub.publish(self.fly_msg)

        if not self.object:
            self.KMPC(target_point[:2])

        # elif (target_point[2]/100 < 0.4):
        #     print('standing')
        #     self.standing_angles_message()

    def KMPC(self, target_point): #Consider LOCAL Coordinate system
        N = 5 # Prediction horizon 

        # -------------------MorphoGear data
        theta = 45*np.pi/180    # Angle between each leg
        lf = 15.5  
        lt = 27.5
        initialStandingAngle=25
        robot_height =  np.sin(initialStandingAngle * np.pi / 180) * lf + lt
        # -------------------Step length constraint
        step_length_max = np.sqrt(((lf + lt)**2) - robot_height**2)*0.85
        step_length_min = -step_length_max; #(np.sqrt(((lf + lt)**2) - robot_height**2)*0.9)
        # -------------------Defining states
        x = ca.SX.sym("x")
        y = ca.SX.sym("y")
        states = ca.vertcat(
                            x,
                            y
                            )
        n_states = states.numel()
        # -------------------Defining control action
        a = ca.SX.sym('a') #Step Length
        #th = ca.SX.sym('theta') #Limb Yaw

        controls = ca.vertcat(a)
        n_controls = controls.numel()
        # -------------------Initial State
        initial = [0,0] 
        # -------------------Target State
        target = target_point

        # -------------------Defining non-linear mapping function
        RHS = ca.vertcat(
                    a*ca.cos(theta), 
                    a*ca.sin(theta)
                    )
        f = ca.Function('f', [states, controls], [RHS])
        # -------------------Control at each step of prediction horizon
        U = ca.SX.sym('U', n_controls, N) # rows = number of control inputs & columns = N
        # -------------------Initial state at every time step with the last two as reference states 
        P = ca.SX.sym('P', n_states+n_states) 
        # -------------------Prediction at each step of prediction horizon
        X = ca.SX.sym('X', n_states, (N+1))  # rows = number of states & columns = N+1
        # -------------------Initializing states
        X[:,0] = P[:n_states]  
        # -------------------Filling up states over the prediciton horizon
        for i in range(N):
            st = X[:, i]
            con = U[:, i]
            f_value = f(st, con)
            f_value[-1]*=(-1)**i
            st_next = st + f_value
            X[:, i+1] = st_next

        obj = 0     # objective function
        g = []      # constraint vector (for state)
        # -------------------Weight matrix Q
        Qx = 3
        Q = ca.diagcat(Qx,Qx) 
        # -------------------Weight matrix R
        R1 = 0.2
        R = ca.diagcat(R1)
        # -------------------Cost function
        for i in range(N):
            st = X[:, i]
            con = U[:, i]
            obj = obj + (st-P[n_states:2*n_states]).T@Q@(st-P[n_states:2*n_states]) + con.T@R@con        
        # -------------------State constraints
        for i in range(N+1):
            g.append(X[:, i])     
        # -------------------Optimization variables
        opt_variables = ca.reshape(U, -1, 1)
        # -------------------Defining non-linear problem
        nlp_prob = {
                    'f': obj,
                    'x': opt_variables,
                    'g': ca.vcat(g),
                    'p': P
                    }    
        # -------------------Defining optimization tolerance
        solver_opts = {
                    'ipopt': {
                                'max_iter': 200,
                                'print_level': 0,
                                'acceptable_tol': 1e-8,
                                'acceptable_obj_change_tol': 1e-6
                                },
                    'print_time': 0
                    }
        # -------------------Defining solver
        solver = ca.nlpsol('solver', 'ipopt', nlp_prob, solver_opts)
        # -------------------Defining inequality state constraints
        lbg = -10000
        ubg = 10000
        # -------------------Defining input constraints
        lbx = []
        ubx = []
        for _ in range(N):
            lbx.append(step_length_min*np.cos(theta))
            ubx.append(step_length_max*np.cos(theta))

        # -------------------Defining constraints for MPC
        args = { 
                'lbg': lbg,  
                'ubg': ubg,
                'lbx': lbx,
                'ubx': ubx,
                }
        # -------------------Defining initial values
        x0 = np.array(initial).reshape(-1, 1)
        u0 = np.zeros((N,n_controls))
    
        # -------------------Defining final (reference) values
        xs = np.array(target).reshape(-1, 1)
    
        # -------------------Running MPC
        def DM2Arr(dm):
            return np.array(dm.full())
        
        args['p'] = ca.vertcat(
                                x0,    # current state
                                xs     # target state
                                )
        args['x0'] = ca.vertcat(ca.reshape(u0, n_controls*N, 1))

        sol = solver(
                    x0=args['x0'],
                    lbx=args['lbx'],
                    ubx=args['ubx'],
                    lbg=args['lbg'],
                    ubg=args['ubg'],
                    p=args['p']
                    )
        
        u_sol = ca.reshape(sol['x'][:n_controls*(N)], n_controls, N)

        step_length1 = DM2Arr(u_sol[:,0])[0][0]
        step_length2 = DM2Arr(u_sol[:,1])[0][0]

        print('steplength: ', step_length1, step_length2)

        L_UA = 15.5 #lf
        L_FA = 27.5 #lt
        L_Sh = 9.5

        def  calculateStepAngle( l_step):
            maxstep= (np.cos(initialStandingAngle/180*np.pi) * lf + L_Sh)
            l_step=np.clip(l_step,0, maxstep)
            stepAngle = int (np.asin(l_step / (2 * maxstep)) * 180/np.pi)
            return stepAngle

        if ((abs(step_length1) < step_length_max/3 and abs(step_length2) > step_length_max/2 ) 
            or (abs(step_length2) < step_length_max/3 and abs(step_length1) > step_length_max/2 )):
            msg=String()
            angle1 = calculateStepAngle(abs(step_length1))
            angle2 = calculateStepAngle(abs(step_length2))
            if (abs(step_length1)>abs(step_length2)):
                if step_length1<0:
                    msg.data="rightKMPC_"+ str(angle1)
                else:
                    msg.data="leftKMPC_"+ str(angle1)
            else:
                if step_length2>0:
                    msg.data="forwardKMPC_"+ str(angle2)
                else:
                    msg.data="backwardKMPC_"+ str(angle2)
            self.gait1_publisher.publish(msg)
            return
        
        def trajectory_generator(step_length): 
            print("IM HERE")
            step_length_max = np.sqrt(((lf + lt)**2) - robot_height**2)
            theta = np.linspace( np.pi, 0, 100), 
            xxx=step_length_max - step_length
            r = np.linspace(0, step_length, 100) 
            xstep =  np.linspace( xxx,step_length_max*0.9,100)
            xspiral= -r * np.cos(theta).flatten() + step_length_max*0.9
            ystep= -robot_height*np.ones_like(xstep)
            for i in range(50):
                 ystep[i]*=1+i/1000
                 ystep[i+50]*=1.05-i/1000
            for i in range(10):
                xspiral[-i-1]= xspiral[-10]
            yspiral=(r * np.sin(theta)*0.5).flatten() - robot_height
            trajectoryX = np.concatenate(( xspiral[50:],xstep, xspiral[:50]))      
            trajectoryY = np.concatenate((yspiral[50:], ystep, yspiral[:50]))
            return trajectoryX,trajectoryY

        def trajectory_generator_hind(step_length): 
            theta = np.linspace( np.pi, 0, 100), 
            xxx=step_length_max - step_length
            r = np.linspace(0, step_length, 100) 
            xstep =  np.linspace( xxx,step_length_max,100)
            xspiral= -r * np.cos(theta).flatten() + step_length_max
            ystep= -robot_height*np.ones_like(xstep)
            for i in range (50):
                ystep[i]*=1+i/1000
                ystep[i+50]*=1.05-i/1000
            yspiral=(r * np.sin(theta)*0.9).flatten() - robot_height
            trajectoryX = np.concatenate((xstep, xspiral))      
            trajectoryY = np.concatenate((ystep, yspiral))
            return trajectoryX,trajectoryY


        

        def trajectory_generator2(step_length): 
            theta = np.linspace( np.pi, 0, 100), 
            xxx=step_length_max - step_length
            r = np.linspace(0, xxx, 100) 
            xstep =  np.linspace( xxx,step_length_max,100)
            xspiral= -r * np.cos(theta).flatten() + step_length_max
            ystep= -robot_height*np.ones_like(xstep)
            # for i in range (100):
            #     ystep[i]*=1+i/1000
            # for i in range(50):
            #     ystep[i]*=1+i/1000
            #     ystep[i+50]*=1.05-i/1000

            yspiral=(r * np.sin(theta)).flatten() - robot_height
            trajectoryX = np.concatenate((xstep, xspiral))      
            trajectoryY = np.concatenate((ystep, yspiral))
            return trajectoryX,trajectoryY

        def trajectory_generator1(step_length):    
            theta = np.linspace(0, np.pi, 100),  
            r = np.linspace(0, step_length, 100) 
            xstep =  np.linspace(0, step_length, 100)
            xspiral= r * np.cos(theta).flatten() + step_length
            ystep=-robot_height*np.ones_like(xstep)
            yspiral=(r * np.sin(theta)*0.9).flatten() - robot_height
            trajectoryX = np.concatenate((xstep, xspiral))      
            trajectoryY = np.concatenate((ystep, yspiral))
            return trajectoryX,trajectoryY
             
        def Inverse_Kinematics (x,y):
            L1 = lf
            L2 = lt
            c2 = (x**2+y**2- L1**2-L2**2)/(2*L1*L2)
            c2=np.clip(c2,-1,1)
            s2 = (1-c2**2)**0.5
            th2 = -np.arccos(c2)
            th1 = np.arctan2(y,x) + np.arctan2(L2*s2,L1+L2*c2)
            return ([th1/np.pi*180,th2/np.pi*180])

        x_traj_1, y_traj_1=trajectory_generator(abs(step_length1))
        x_traj_2, y_traj_2=trajectory_generator(abs(step_length2))

        # x_traj_1_hind, y_traj_1_hind=trajectory_generator_hind(abs(step_length1))
        # x_traj_2_hind, y_traj_2_hind=trajectory_generator_hind(abs(step_length2))
                        
        # angles_1_hind=Inverse_Kinematics(x_traj_1_hind, y_traj_1_hind)
        # angles_2_hind=Inverse_Kinematics(x_traj_2_hind, y_traj_2_hind)
        
        angles_1=Inverse_Kinematics(x_traj_1, y_traj_1)
        angles_2=Inverse_Kinematics(x_traj_2, y_traj_2)

        t1_1=np.array(angles_2)[0,:].astype(int)
        t2_1=np.array(angles_2)[1,:].astype(int)
        t1_2=np.array(angles_1)[0,:].astype(int)
        t2_2=np.array(angles_1)[1,:].astype(int)


        # t1_1_hind=np.array(angles_2_hind)[0,:].astype(int)
        # t2_1_hind=np.array(angles_2_hind)[1,:].astype(int)
        # t1_2_hind=np.array(angles_1_hind)[0,:].astype(int)
        # t2_2_hind=np.array(angles_1_hind)[1,:].astype(int)

        num_start=70
        num=num_start+100


        if step_length2>0:
            t1_BL=np.concatenate((t1_1[100:],t1_1[:100]))
            t2_BL=np.concatenate((t2_1[100:],t2_1[:100]))
            t1_FR=t1_BL[::-1]
            t2_FR=t2_BL[::-1]
        else:
            t1_FR=np.concatenate((t1_1[100:],t1_1[:100]))
            t2_FR=np.concatenate((t2_1[100:],t2_1[:100]))
            t1_BL=t1_FR[::-1]
            t2_BL=t2_FR[::-1]

        if step_length1>0:
            t1_BR=t1_2
            t2_BR=t2_2
            t1_FL=t1_BR[::-1]
            t2_FL=t2_BR[::-1]
            #(t1_2_hind[100::],t1_2_hind[:100:])
            #(t2_2_hind[100::],t2_2_hind[:100:])
        else:
            t1_FL=t1_2
            t2_FL=t2_2
            t1_BR=t1_FL[::-1]
            t2_BR=t2_FL[::-1]
            
        # if step_length2>0:

        # if step_length2>0:
        #     t1_BL=t1_1#t1_1_hind
        #     t2_BL=t2_1#t2_1_hind
        #     t1_FR=np.concatenate((t1_1[100::-1],t1_1[200:100:-1]))
        #     t2_FR=np.concatenate((t2_1[100::-1],t2_1[200:100:-1]))
        # else:
        #     t1_BL=np.concatenate((t1_1[100::-1],t1_1[200:100:-1]))
        #     t2_BL=np.concatenate((t2_1[100::-1],t2_1[200:100:-1]))
        #     t1_FR=t1_1#t1_1_hind
        #     t2_FR=t2_1#t2_1_hind

        # if step_length1>0:
        #     t1_FL=t1_2[::-1]
        #     t2_FL=t2_2[::-1]
        #     t1_BR=np.concatenate((t1_2[100::],t1_2[:100:]))#(t1_2_hind[100::],t1_2_hind[:100:])
        #     t2_BR=np.concatenate((t2_2[100::],t2_2[:100:]))#(t2_2_hind[100::],t2_2_hind[:100:])
        # else:
        #     t1_BR=t1_2[::-1]
        #     t2_BR=t2_2[::-1]
        #     t1_FL=np.concatenate((t1_2[100::],t1_2[:100:]))#(t1_2_hind[100::],t1_2_hind[:100:])
        #     t2_FL=np.concatenate((t2_2[100::],t2_2[:100:]))#(t2_2_hind[100::],t2_2_hind[:100:])
        # if step_length2>0:
        #     t1_BL=t1_1
        #     t2_BL=t2_1
        #     t1_FR=np.concatenate((t1_1[100::-1],t1_1[200:100:-1]))
        #     t2_FR=np.concatenate((t2_1[100::-1],t2_1[200:100:-1]))
        # else:
        #     t1_BL=np.concatenate((t1_1[100::-1],t1_1[200:100:-1]))
        #     t2_BL=np.concatenate((t2_1[100::-1],t2_1[200:100:-1]))
        #     t1_FR=t1_1
        #     t2_FR=t2_1

        # if step_length1>0:
        #     t1_FL=t1_2[::-1]
        #     t2_FL=t2_2[::-1]
        #     t1_BR=np.concatenate((t1_2[100::],t1_2[:100:]))
        #     t2_BR=np.concatenate((t2_2[100::],t2_2[:100:]))
        # else:
        #     t1_BR=t1_2[::-1]
        #     t2_BR=t2_2[::-1]
        #     t1_FL=np.concatenate((t1_2[100::],t1_2[:100:]))
        #     t2_FL=np.concatenate((t2_2[100::],t2_2[:100:]))

        # -------------------Publishing limb angles 
        theta_msg = Int8MultiArray()
        theta_msg.data.extend(t1_FR) 
        theta_msg.data.extend(t2_FR)
        theta_msg.data.extend(t1_BR)
        theta_msg.data.extend(t2_BR)
        theta_msg.data.extend(t1_BL)
        theta_msg.data.extend(t2_BL)
        theta_msg.data.extend(t1_FL)
        theta_msg.data.extend(t2_FL)
        self.theta_pub.publish(theta_msg)
        print('Angles Published')


def main(args=None):
    rclpy.init(args=args)
    mpc = MPCNode()
    rclpy.spin(mpc)
    mpc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
