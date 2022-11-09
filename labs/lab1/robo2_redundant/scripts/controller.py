#!/usr/bin/env python3
#Robotics II - Project 1 - Team26 - Iliopoulos - Serlis

"""
Start ROS node to publish angles for the position control of the xArm7.
"""

# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
#Math imports
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import numpy as np
import time as t

# Arm parameters
# xArm7 kinematics class
from kinematics import xArm7_kinematics

# from tf.transformations import quaternion_matrix
# matrix = quaternion_matrix([1, 0, 0, 0])

class xArm7_controller():
    """Class to compute and publish joints positions"""
    def __init__(self,rate):

        # Init xArm7 kinematics handler
        self.kinematics = xArm7_kinematics()

        # joints' angular positions
        self.joint_angpos = [0, 0, 0, 0, 0, 0, 0]
        # joints' angular velocities
        self.joint_angvel = [0, 0, 0, 0, 0, 0, 0]
        # joints' states
        self.joint_states = JointState()
        # joints' transformation matrix wrt the robot's base frame
        self.A01 = self.kinematics.tf_A01(self.joint_angpos)
        self.A02 = self.kinematics.tf_A02(self.joint_angpos)
        self.A03 = self.kinematics.tf_A03(self.joint_angpos)
        self.A04 = self.kinematics.tf_A04(self.joint_angpos)
        self.A05 = self.kinematics.tf_A05(self.joint_angpos)
        self.A06 = self.kinematics.tf_A06(self.joint_angpos)
        self.A07 = self.kinematics.tf_A07(self.joint_angpos)
        # gazebo model's states
        self.model_states = ModelStates()

        # ROS SETUP
        # initialize subscribers for reading encoders and publishers for performing position control in the joint-space
        # Robot
        self.joint_states_sub = rospy.Subscriber('/xarm/joint_states', JointState, self.joint_states_callback, queue_size=1)
        self.joint1_pos_pub = rospy.Publisher('/xarm/joint1_position_controller/command', Float64, queue_size=1)
        self.joint2_pos_pub = rospy.Publisher('/xarm/joint2_position_controller/command', Float64, queue_size=1)
        self.joint3_pos_pub = rospy.Publisher('/xarm/joint3_position_controller/command', Float64, queue_size=1)
        self.joint4_pos_pub = rospy.Publisher('/xarm/joint4_position_controller/command', Float64, queue_size=1)
        self.joint5_pos_pub = rospy.Publisher('/xarm/joint5_position_controller/command', Float64, queue_size=1)
        self.joint6_pos_pub = rospy.Publisher('/xarm/joint6_position_controller/command', Float64, queue_size=1)
        self.joint7_pos_pub = rospy.Publisher('/xarm/joint7_position_controller/command', Float64, queue_size=1)
        # Obstacles
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)

        #Publishing rate
        self.period = 1.0/rate
        self.pub_rate = rospy.Rate(rate)

        self.publish()

    #SENSING CALLBACKS
    def joint_states_callback(self, msg):
        # ROS callback to get the joint_states

        self.joint_states = msg
        # (e.g. the angular position of joint 1 is stored in :: self.joint_states.position[0])

    def model_states_callback(self, msg):
        # ROS callback to get the gazebo's model_states

        self.model_states = msg
        # (e.g. #1 the position in y-axis of GREEN obstacle's center is stored in :: self.model_states.pose[1].position.y)
        # (e.g. #2 the position in y-axis of RED obstacle's center is stored in :: self.model_states.pose[2].position.y)

    def publish(self):

        # set configuration
        self.joint_angpos = [0, 0.75, 0, 1.5, 0, 0.75, 0]
        tmp_rate = rospy.Rate(1)
        tmp_rate.sleep()
        self.joint4_pos_pub.publish(self.joint_angpos[3])
        tmp_rate.sleep()
        self.joint2_pos_pub.publish(self.joint_angpos[1])
        self.joint6_pos_pub.publish(self.joint_angpos[5])
        tmp_rate.sleep()
        print("The system is ready to execute your algorithm...")

        rostime_now = rospy.get_rostime()
        time_now = rostime_now.to_nsec()

        #Posiiton of End-Effector after Initialisation
        A07 = self.kinematics.tf_A07(self.joint_angpos)
        pE = A07[0:3,3] #coordinates of (pA+pB)/2

        
       
        ###### Path Planning #######      
        
        #setting up time
        T = 2             
        Tmax = int(600*T + 1)
        t = [(i/600) for i in range(int(Tmax))]

        #coeffs
        a1 = (1.0666667/T)*np.ones(Tmax)
        a2 = (2.4888889/T**2)*np.ones(Tmax)
        a3 =  (- (20.622222)/T**3)*np.ones(Tmax)
        a4 = (28.4444444/T**4)*np.ones(Tmax)
        a5 =  (- (11.377778)/T**5)*np.ones(Tmax)
 
        #Interpolation - Polynomial
        xd = (0.6043)*np.ones(Tmax)
        yd = a1*t + a2*np.power(t,2) + a3*np.power(t,3) + a4*np.power(t,4) + a5*np.power(t,5)
        zd = (0.1508)*np.ones(Tmax)

        Vxd = np.zeros(Tmax)
        Vyd = a1*np.ones(Tmax) + 2*a2*t + 3*a3*np.power(t,2) + 4*a4*np.power(t,3)+5*a5*np.power(t,4)
        Vzd = np.zeros(Tmax)

        #initialize data extraction
        joint1 = []
        vjoint1 = []
        joint2 = []
        vjoint2 = []
        joint3 = []
        vjoint3 = []
        joint4 = []
        vjoint4 = []
        joint5 = []
        vjoint5 = []
        joint6 = []
        vjoint6 = []
        joint7 = []
        vjoint7 = []
        py = []
        vy = []
        error = []
        dist_q3_A = []
        dist_q3_B = []
        dist_q4_A = []
        dist_q4_B = []
        dist_q5_A = []
        dist_q5_B = []
        red_obst = []
        green_obst = []
        data = (joint1, vjoint1, joint2, vjoint2, joint3, vjoint3, joint4, vjoint4, joint5, vjoint5, joint6, vjoint6, joint7, vjoint7, py, vy, dist_q3_A, dist_q3_B, dist_q4_A, dist_q4_B, dist_q5_A, dist_q5_B, error, red_obst, green_obst)
        
        counter = 0     
        
        #Here starts the periodic movement
        while not rospy.is_shutdown():
  
            for tk in range(int(Tmax)):
                rostime_now = rospy.get_rostime()
                time_now = rostime_now.to_nsec()

                # Compute each transformation matrix wrt the base frame from joints' angular positions
                self.A01 = self.kinematics.tf_A01(self.joint_angpos)
                self.A02 = self.kinematics.tf_A02(self.joint_angpos)
                self.A03 = self.kinematics.tf_A03(self.joint_angpos)
                self.A04 = self.kinematics.tf_A04(self.joint_angpos)
                self.A05 = self.kinematics.tf_A05(self.joint_angpos)
                self.A06 = self.kinematics.tf_A06(self.joint_angpos)
                self.A07 = self.kinematics.tf_A07(self.joint_angpos)

                # Compute jacobian matrix
                J = self.kinematics.compute_jacobian(self.joint_angpos)
                # pseudoinverse jacobian
                pinvJ = pinv(J)

                ############Task 1##############
                
                # desired end-effector velocity
                vd = np.matrix([[Vxd[tk]],[Vyd.item(tk)],[Vzd[tk]]])
                # desired end-effector position
                pd = np.matrix([[xd[tk]], [yd.item(tk)], [zd[tk]]])

                # real position of end-effector
                pE_real = self.A07[:-1, 3]

                K1 = 150 # control gain

                qdot1 = pinvJ @ (vd + K1*(pd - pE_real)) #Velocity from 1st task

                ############Task_2##############

                green_obst = self.model_states.pose[1].position.y
                red_obst = self.model_states.pose[2].position.y

                yo = (green_obst + red_obst)/2 # middle of the two obstacles
            
                # Criteria Functions to Optimize (Minimize): Distance of Joints from Middle of Obstacles
                Kc = 100
                V1 = (1/2) * Kc * ((self.A03[1,3] - yo) ** 2) #distance from q3
                V2 = (1/2) * Kc * ((self.A04[1,3] - yo) ** 2) #distance from q4
                V3 = (1/2) * Kc * ((self.A05[1,3] - yo) ** 2) #distance from q5

                # Gradient of V1, V2 --> reference joint velocities of 2nd Task

                # Fetch some useful data...
                l2 = self.kinematics.l2
                l3 = self.kinematics.l3
                l4 = self.kinematics.l4
                th1 = self.kinematics.theta1
                th2 = self.kinematics.theta2
                q1 = self.joint_angpos[0]
                q2 = self.joint_angpos[1]
                q3 = self.joint_angpos[2]
                q4 = self.joint_angpos[3]
                c1 = np.cos(q1)
                c2 = np.cos(q2)
                c3 = np.cos(q3)
                c4 = np.cos(q4)
                s1 = np.sin(q1)
                s2 = np.sin(q2)
                s3 = np.sin(q3)
                s4 = np.sin(q4)
 
                #Distance from q3
 
                cost1 = np.zeros((7,1))
                cost1[0] = -Kc * (self.A03[1,3] - yo) * l2*c1*s2
                cost1[1] = -Kc * (self.A03[1,3] - yo) * l2*c2*s1 

                # Distance from joint q4
                cost2 = np.zeros((7,1))
                cost2[0] = -Kc * (self.A04[1,3] - yo) * (l2*c1*s2 - l3*(s1*s3 - c1*c2*c3))
                cost2[1] = -Kc * (self.A04[1,3] - yo) * (l2*c2*s1 - l3*c3*s1*s2)
                cost2[2] = -Kc * (self.A04[1,3] - yo) * (l3*(c1*c3 - c2*s1*s3))

                #Distance from q5
                dev0 = l2*np.sin(q2)*np.cos(q1) - l3*np.sin(q1)*np.sin(q3) + l3*np.cos(q1)*np.cos(q2)*np.cos(q3) - l4*np.sin(q1)*np.sin(q3)*np.sin(q4 + th1) - l4*np.sin(q2)*np.cos(q1)*np.cos(q4 + th1) +     l4*np.sin(q4 +th1)*np.cos(q1)*np.cos(q2)*np.cos(q3)
                dev1 = (l2*np.cos(q2) - l3*np.sin(q2)*np.cos(q3) - l4*np.sin(q2)*np.sin(q4 + th1)*np.cos(q3) - l4*np.cos(q2)*np.cos(q4 + th1))*np.sin(q1)
                dev2 = -(l3 + l4*np.sin(q4 + th1))*(np.sin(q1)*np.sin(q3)*np.cos(q2) - np.cos(q1)*np.cos(q3))
                dev3 = l4*(np.sin(q1)*np.sin(q2)*np.sin(q4 + th1) + np.sin(q1)*np.cos(q2)*np.cos(q3)*np.cos(q4 + th1) + np.sin(q3)*np.cos(q1)*np.cos(q4 + th1))
                cost3 = np.zeros((7,1))
                cost3[0] = -Kc * (self.A05[1,3] - yo)*dev0
                cost3[1] = -Kc * (self.A05[1,3] - yo)*dev1
                cost3[2] = -Kc * (self.A05[1,3] - yo)*dev2
                cost3[3] = -Kc * (self.A05[1,3] - yo)*dev3

                K2 = [15, 30, 15] # some gains
                total_cost = K2[0]*cost1 + K2[1]*cost2 +K2[2]*cost3

                qdot2 = (np.eye(7) - np.dot(pinvJ, J)) @ total_cost #Velocity from 2nd task

                for i in range(7):
                       self.joint_angvel[i] = qdot1[i,0] + qdot2[i,0] #Velocity from both tasks
                
                ############################################################################ 

                #Setting up time
                time_prev = time_now
                rostime_now = rospy.get_rostime()
                time_now = rostime_now.to_nsec()
                dt = (time_now - time_prev)/1e9
                
                #Integration
                self.joint_angpos = np.add( self.joint_angpos, [index * dt for index in self.joint_angvel] )

                #Angular Positions
                self.joint1_pos_pub.publish(self.joint_angpos[0])
                self.joint2_pos_pub.publish(self.joint_angpos[1])
                self.joint3_pos_pub.publish(self.joint_angpos[2])
                self.joint4_pos_pub.publish(self.joint_angpos[3])
                self.joint5_pos_pub.publish(self.joint_angpos[4])
                self.joint6_pos_pub.publish(self.joint_angpos[5])
                self.joint7_pos_pub.publish(self.joint_angpos[6])

                #Saving data fror poltting
                data[0].append(self.joint_angpos[0])
                data[1].append(self.joint_angvel[0])
                data[2].append(self.joint_angpos[1])
                data[3].append(self.joint_angvel[1])
                data[4].append(self.joint_angpos[2])
                data[5].append(self.joint_angvel[2])
                data[6].append(self.joint_angpos[3])
                data[7].append(self.joint_angvel[3])
                data[8].append(self.joint_angpos[4])
                data[9].append(self.joint_angvel[4])
                data[10].append(self.joint_angpos[5])
                data[11].append(self.joint_angvel[5])
                data[12].append(self.joint_angpos[6])
                data[13].append(self.joint_angvel[6])
                data[14].append(pd[1])
                data[15].append(vd[1])
                data[16].append(self.A03[1,3] - green_obst)
                data[17].append(self.A03[1,3] - red_obst)
                data[18].append(self.A04[1,3] - green_obst)
                data[19].append(self.A04[1,3] - red_obst)
                data[20].append(self.A05[1,3] - green_obst)
                data[21].append(self.A05[1,3] - red_obst)
                data[22].append(pd[1] - pE_real[1])
                data[23].append(green_obst)
                data[24].append(red_obst)
                
                counter=counter+1

        #^^^The simulation has stopped now^^^

        #Opening files for data extraction
        print('opening files to export data...')
        joint1_data = open('joint1.txt','w')
        vjoint1_data = open('vjoint1.txt','w')
        joint2_data = open('joint2.txt','w')
        vjoint2_data = open('vjoint2.txt','w')
        joint3_data = open('joint3.txt','w')
        vjoint3_data = open('vjoint3.txt','w')
        joint4_data = open('joint4.txt','w')
        vjoint4_data = open('vjoint4.txt','w')
        joint5_data = open('joint5.txt','w')
        vjoint5_data = open('vjoint5.txt','w')
        joint6_data = open('joint6.txt','w')
        vjoint6_data = open('vjoint6.txt','w')
        joint7_data = open('joint7.txt','w')
        vjoint7_data = open('vjoint7.txt','w')
        pdy_data = open('pdy.txt','w')
        vdy_data = open('vdy.txt','w')
        dist_q3_A_data = open('dist_q3_A','w')
        dist_q3_B_data = open('dist_q3_B','w')
        dist_q4_A_data = open('dist_q4_A','w')
        dist_q4_B_data = open('dist_q4_B','w')
        dist_q5_A_data = open('dist_q5_A','w')
        dist_q5_B_data = open('dist_q5_B','w')
        error_data = open('error.txt','w')
        green_obst = open('green_obst.txt', 'w')
        red_obst = open('red_obst.txt', 'w')
        print('writing data...')

        for j in range(len(data[0])):            
            joint1_data.write(str(data[0][j]) + ' ')
            vjoint1_data.write(str(data[1][j]) + ' ')
            joint2_data.write(str(data[2][j]) + ' ')
            vjoint2_data.write(str(data[3][j]) + ' ')
            joint3_data.write(str(data[4][j]) + ' ')
            vjoint3_data.write(str(data[5][j]) + ' ')
            joint4_data.write(str(data[6][j]) + ' ')
            vjoint4_data.write(str(data[7][j]) + ' ')
            joint5_data.write(str(data[8][j]) + ' ')
            vjoint5_data.write(str(data[9][j]) + ' ')
            joint6_data.write(str(data[10][j]) + ' ')
            vjoint6_data.write(str(data[11][j]) + ' ')
            joint7_data.write(str(data[12][j]) + ' ')
            vjoint7_data.write(str(data[13][j]) + ' ')
            pdy_data.write(str(data[14][j]) + ' ')
            vdy_data.write(str(data[15][j]) + ' ')
            dist_q3_A_data.write(str(data[16][j]) + ' ')
            dist_q3_B_data.write(str(data[17][j]) + ' ')
            dist_q4_A_data.write(str(data[18][j]) + ' ')
            dist_q4_B_data.write(str(data[19][j]) + ' ')
            dist_q5_A_data.write(str(data[20][j]) + ' ')
            dist_q5_B_data.write(str(data[21][j]) + ' ')
            error_data.write(str(data[22][j]) + ' ') 
            green_obst.write(str(data[23][j]) + ' ')
            red_obst.write(str(data[24][j]) + ' ')            
        print('...finished writing data')

        print('closing files...')
        joint1_data.close()
        vjoint1_data.close()
        joint2_data.close()
        vjoint2_data.close()
        joint3_data.close()
        vjoint3_data.close()
        joint4_data.close()
        vjoint4_data.close()
        joint5_data.close()
        vjoint5_data.close()
        joint6_data.close()
        vjoint6_data.close()
        joint7_data.close()
        vjoint7_data.close()
        pdy_data.close()
        vdy_data.close()
        dist_q3_A_data.close()
        dist_q3_B_data.close()
        dist_q4_A_data.close()
        dist_q4_B_data.close()
        dist_q5_A_data.close()
        dist_q5_B_data.close()
        error_data.close()
        green_obst.close()
        red_obst.close()
        print('finished getting data')
    
    def turn_off(self):
        pass

def controller_py():
    # Starts a new node
    rospy.init_node('controller_node', anonymous = True)
    # Reading parameters set in launch file
    rate = rospy.get_param("/rate")

    controller = xArm7_controller(rate)
    rospy.on_shutdown(controller.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        controller_py()
    except rospy.ROSInterruptException:
        pass