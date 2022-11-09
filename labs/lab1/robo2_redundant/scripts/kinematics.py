#!/usr/bin/env python3
#Robotics II - Project 1 - Team26 - Iliopoulos - Serlis

"""
Compute state space kinematic matrices for xArm7 robot arm (5 links, 7 joints)
"""

import numpy as np

class xArm7_kinematics():
    def __init__(self):

        self.l1 = 0.267
        self.l2 = 0.293
        self.l3 = 0.0525
        self.l4 = 0.3512
        self.l5 = 0.1232

        self.theta1 = 0.2225 #(rad) (=12.75deg)
        self.theta2 = 0.6646 #(rad) (=38.08deg)

        pass

    def compute_jacobian(self, r_joints_array):

        #setting up the variables
        th1 = self.theta1
        th2 = self.theta2

        l1 = self.l1
        l2 = self.l2
        l3 = self.l3
        l4 = self.l4
        l5 = self.l5
        
        #joint angles
        q1 = r_joints_array[0]
        q2 = r_joints_array[1]
        q3 = r_joints_array[2]
        q4 = r_joints_array[3]
        q5 = r_joints_array[4]
        q6 = r_joints_array[5]
        q7 = r_joints_array[6]

        #inserting the elements of the Jacobian after calculating them in seperate python code

        J_11 = -l2*np.sin(q1)*np.sin(q2) - l3*np.sin(q1)*np.cos(q2)*np.cos(q3) - l3*np.sin(q3)*np.cos(q1) + l4*np.sin(q1)*np.sin(q2)*np.cos(q4 + th1) - l4*np.sin(q1)*np.sin(q4 + th1)*np.cos(q2)*np.cos(q3) - l4*np.sin(q3)*np.sin(q4 + th1)*np.cos(q1) + l5*np.sin(q1)*np.sin(q2)*np.sin(q4)*np.sin(q6 - th2)*np.cos(q5) + l5*np.sin(q1)*np.sin(q2)*np.cos(q4)*np.cos(q6 - th2) + l5*np.sin(q1)*np.sin(q3)*np.sin(q5)*np.sin(q6 - th2)*np.cos(q2) - l5*np.sin(q1)*np.sin(q4)*np.cos(q2)*np.cos(q3)*np.cos(q6 - th2) + l5*np.sin(q1)*np.sin(q6 - th2)*np.cos(q2)*np.cos(q3)*np.cos(q4)*np.cos(q5) - l5*np.sin(q3)*np.sin(q4)*np.cos(q1)*np.cos(q6 - th2) + l5*np.sin(q3)*np.sin(q6 - th2)*np.cos(q1)*np.cos(q4)*np.cos(q5) - l5*np.sin(q5)*np.sin(q6 - th2)*np.cos(q1)*np.cos(q3)
        J_12 = (l2*np.cos(q2) - l3*np.sin(q2)*np.cos(q3) - l4*np.sin(q2)*np.sin(q4 + th1)*np.cos(q3) - l4*np.cos(q2)*np.cos(q4 + th1) + l5*np.sin(q2)*np.sin(q3)*np.sin(q5)*np.sin(q6 - th2) - l5*np.sin(q2)*np.sin(q4)*np.cos(q3)*np.cos(q6 - th2) + l5*np.sin(q2)*np.sin(q6 - th2)*np.cos(q3)*np.cos(q4)*np.cos(q5) - l5*np.sin(q4)*np.sin(q6 - th2)*np.cos(q2)*np.cos(q5) - l5*np.cos(q2)*np.cos(q4)*np.cos(q6 - th2))*np.cos(q1)
        J_13 = -l3*np.sin(q1)*np.cos(q3) - l3*np.sin(q3)*np.cos(q1)*np.cos(q2) - l4*np.sin(q1)*np.sin(q4 + th1)*np.cos(q3) - l4*np.sin(q3)*np.sin(q4 + th1)*np.cos(q1)*np.cos(q2) + l5*np.sin(q1)*np.sin(q3)*np.sin(q5)*np.sin(q6 - th2) - l5*np.sin(q1)*np.sin(q4)*np.cos(q3)*np.cos(q6 - th2) + l5*np.sin(q1)*np.sin(q6 - th2)*np.cos(q3)*np.cos(q4)*np.cos(q5) - l5*np.sin(q3)*np.sin(q4)*np.cos(q1)*np.cos(q2)*np.cos(q6 - th2) + l5*np.sin(q3)*np.sin(q6 - th2)*np.cos(q1)*np.cos(q2)*np.cos(q4)*np.cos(q5) - l5*np.sin(q5)*np.sin(q6 - th2)*np.cos(q1)*np.cos(q2)*np.cos(q3)
        J_14 = -l4*np.sin(q1)*np.sin(q3)*np.cos(q4 + th1) + l4*np.sin(q2)*np.sin(q4 + th1)*np.cos(q1) + l4*np.cos(q1)*np.cos(q2)*np.cos(q3)*np.cos(q4 + th1) - l5*np.sin(q1)*np.sin(q3)*np.sin(q4)*np.sin(q6 - th2)*np.cos(q5) - l5*np.sin(q1)*np.sin(q3)*np.cos(q4)*np.cos(q6 - th2) + l5*np.sin(q2)*np.sin(q4)*np.cos(q1)*np.cos(q6 - th2) - l5*np.sin(q2)*np.sin(q6 - th2)*np.cos(q1)*np.cos(q4)*np.cos(q5) + l5*np.sin(q4)*np.sin(q6 - th2)*np.cos(q1)*np.cos(q2)*np.cos(q3)*np.cos(q5) + l5*np.cos(q1)*np.cos(q2)*np.cos(q3)*np.cos(q4)*np.cos(q6 - th2)
        J_15 = l5*(-np.sin(q1)*np.sin(q3)*np.sin(q5)*np.cos(q4) - np.sin(q1)*np.cos(q3)*np.cos(q5) + np.sin(q2)*np.sin(q4)*np.sin(q5)*np.cos(q1) - np.sin(q3)*np.cos(q1)*np.cos(q2)*np.cos(q5) + np.sin(q5)*np.cos(q1)*np.cos(q2)*np.cos(q3)*np.cos(q4))*np.sin(q6 - th2)
        J_16 = l5*(np.sin(q1)*np.sin(q3)*np.sin(q4)*np.sin(q6 - th2) + np.sin(q1)*np.sin(q3)*np.cos(q4)*np.cos(q5)*np.cos(q6 - th2) - np.sin(q1)*np.sin(q5)*np.cos(q3)*np.cos(q6 - th2) - np.sin(q2)*np.sin(q4)*np.cos(q1)*np.cos(q5)*np.cos(q6 - th2) + np.sin(q2)*np.sin(q6 - th2)*np.cos(q1)*np.cos(q4) - np.sin(q3)*np.sin(q5)*np.cos(q1)*np.cos(q2)*np.cos(q6 - th2) - np.sin(q4)*np.sin(q6 - th2)*np.cos(q1)*np.cos(q2)*np.cos(q3) - np.cos(q1)*np.cos(q2)*np.cos(q3)*np.cos(q4)*np.cos(q5)*np.cos(q6 - th2))
        J_17 = 0

        J_21 = l2*np.sin(q2)*np.cos(q1) - l3*np.sin(q1)*np.sin(q3) + l3*np.cos(q1)*np.cos(q2)*np.cos(q3) - l4*np.sin(q1)*np.sin(q3)*np.sin(q4 + th1) - l4*np.sin(q2)*np.cos(q1)*np.cos(q4 + th1) + l4*np.sin(q4 + th1)*np.cos(q1)*np.cos(q2)*np.cos(q3) - l5*np.sin(q1)*np.sin(q3)*np.sin(q4)*np.cos(q6 - th2) + l5*np.sin(q1)*np.sin(q3)*np.sin(q6 - th2)*np.cos(q4)*np.cos(q5) - l5*np.sin(q1)*np.sin(q5)*np.sin(q6 - th2)*np.cos(q3) - l5*np.sin(q2)*np.sin(q4)*np.sin(q6 - th2)*np.cos(q1)*np.cos(q5) - l5*np.sin(q2)*np.cos(q1)*np.cos(q4)*np.cos(q6 - th2) - l5*np.sin(q3)*np.sin(q5)*np.sin(q6 - th2)*np.cos(q1)*np.cos(q2) + l5*np.sin(q4)*np.cos(q1)*np.cos(q2)*np.cos(q3)*np.cos(q6 - th2) - l5*np.sin(q6 - th2)*np.cos(q1)*np.cos(q2)*np.cos(q3)*np.cos(q4)*np.cos(q5)
        J_22 = (l2*np.cos(q2) - l3*np.sin(q2)*np.cos(q3) - l4*np.sin(q2)*np.sin(q4 + th1)*np.cos(q3) - l4*np.cos(q2)*np.cos(q4 + th1) + l5*np.sin(q2)*np.sin(q3)*np.sin(q5)*np.sin(q6 - th2) - l5*np.sin(q2)*np.sin(q4)*np.cos(q3)*np.cos(q6 - th2) + l5*np.sin(q2)*np.sin(q6 - th2)*np.cos(q3)*np.cos(q4)*np.cos(q5) - l5*np.sin(q4)*np.sin(q6 - th2)*np.cos(q2)*np.cos(q5) - l5*np.cos(q2)*np.cos(q4)*np.cos(q6 - th2))*np.sin(q1)
        J_23 = -l3*np.sin(q1)*np.sin(q3)*np.cos(q2) + l3*np.cos(q1)*np.cos(q3) - l4*np.sin(q1)*np.sin(q3)*np.sin(q4 + th1)*np.cos(q2) + l4*np.sin(q4 + th1)*np.cos(q1)*np.cos(q3) - l5*np.sin(q1)*np.sin(q3)*np.sin(q4)*np.cos(q2)*np.cos(q6 - th2) + l5*np.sin(q1)*np.sin(q3)*np.sin(q6 - th2)*np.cos(q2)*np.cos(q4)*np.cos(q5) - l5*np.sin(q1)*np.sin(q5)*np.sin(q6 - th2)*np.cos(q2)*np.cos(q3) - l5*np.sin(q3)*np.sin(q5)*np.sin(q6 - th2)*np.cos(q1) + l5*np.sin(q4)*np.cos(q1)*np.cos(q3)*np.cos(q6 - th2) - l5*np.sin(q6 - th2)*np.cos(q1)*np.cos(q3)*np.cos(q4)*np.cos(q5)
        J_24 = l4*np.sin(q1)*np.sin(q2)*np.sin(q4 + th1) + l4*np.sin(q1)*np.cos(q2)*np.cos(q3)*np.cos(q4 + th1) + l4*np.sin(q3)*np.cos(q1)*np.cos(q4 + th1) + l5*np.sin(q1)*np.sin(q2)*np.sin(q4)*np.cos(q6 - th2) - l5*np.sin(q1)*np.sin(q2)*np.sin(q6 - th2)*np.cos(q4)*np.cos(q5) + l5*np.sin(q1)*np.sin(q4)*np.sin(q6 - th2)*np.cos(q2)*np.cos(q3)*np.cos(q5) + l5*np.sin(q1)*np.cos(q2)*np.cos(q3)*np.cos(q4)*np.cos(q6 - th2) + l5*np.sin(q3)*np.sin(q4)*np.sin(q6 - th2)*np.cos(q1)*np.cos(q5) + l5*np.sin(q3)*np.cos(q1)*np.cos(q4)*np.cos(q6 - th2)
        J_25 = l5*(((np.sin(q1)*np.cos(q2)*np.cos(q3) + np.sin(q3)*np.cos(q1))*np.cos(q4) + np.sin(q1)*np.sin(q2)*np.sin(q4))*np.sin(q5) - (np.sin(q1)*np.sin(q3)*np.cos(q2) - np.cos(q1)*np.cos(q3))*np.cos(q5))*np.sin(q6 - th2)
        J_26 = l5*(-np.sin(q1)*np.sin(q2)*np.sin(q4)*np.cos(q5)*np.cos(q6 - th2) + np.sin(q1)*np.sin(q2)*np.sin(q6 - th2)*np.cos(q4) - np.sin(q1)*np.sin(q3)*np.sin(q5)*np.cos(q2)*np.cos(q6 - th2) - np.sin(q1)*np.sin(q4)*np.sin(q6 - th2)*np.cos(q2)*np.cos(q3) - np.sin(q1)*np.cos(q2)*np.cos(q3)*np.cos(q4)*np.cos(q5)*np.cos(q6 - th2) - np.sin(q3)*np.sin(q4)*np.sin(q6 - th2)*np.cos(q1) - np.sin(q3)*np.cos(q1)*np.cos(q4)*np.cos(q5)*np.cos(q6 - th2) + np.sin(q5)*np.cos(q1)*np.cos(q3)*np.cos(q6 - th2))
        J_27 = 0

        J_31 = 0
        J_32 = -l2*np.sin(q2) - l3*np.cos(q2)*np.cos(q3) + l4*np.sin(q2)*np.cos(q4 + th1) - l4*np.sin(q4 + th1)*np.cos(q2)*np.cos(q3) + l5*np.sin(q2)*np.sin(q4)*np.sin(q6 - th2)*np.cos(q5) + l5*np.sin(q2)*np.cos(q4)*np.cos(q6 - th2) + l5*np.sin(q3)*np.sin(q5)*np.sin(q6 - th2)*np.cos(q2) - l5*np.sin(q4)*np.cos(q2)*np.cos(q3)*np.cos(q6 - th2) + l5*np.sin(q6 - th2)*np.cos(q2)*np.cos(q3)*np.cos(q4)*np.cos(q5)
        J_33 = (l3*np.sin(q3) + l4*np.sin(q3)*np.sin(q4 + th1) + l5*np.sin(q3)*np.sin(q4)*np.cos(q6 - th2) - l5*np.sin(q3)*np.sin(q6 - th2)*np.cos(q4)*np.cos(q5) + l5*np.sin(q5)*np.sin(q6 - th2)*np.cos(q3))*np.sin(q2)
        J_34 = -l4*np.sin(q2)*np.cos(q3)*np.cos(q4 + th1) + l4*np.sin(q4 + th1)*np.cos(q2) - l5*np.sin(q2)*np.sin(q4)*np.sin(q6 - th2)*np.cos(q3)*np.cos(q5) - l5*np.sin(q2)*np.cos(q3)*np.cos(q4)*np.cos(q6 - th2) + l5*np.sin(q4)*np.cos(q2)*np.cos(q6 - th2) - l5*np.sin(q6 - th2)*np.cos(q2)*np.cos(q4)*np.cos(q5)
        J_35 = -l5*((np.sin(q2)*np.cos(q3)*np.cos(q4) - np.sin(q4)*np.cos(q2))*np.sin(q5) - np.sin(q2)*np.sin(q3)*np.cos(q5))*np.sin(q6 - th2)
        J_36 = l5*(np.sin(q2)*np.sin(q3)*np.sin(q5)*np.cos(q6 - th2) + np.sin(q2)*np.sin(q4)*np.sin(q6 - th2)*np.cos(q3) + np.sin(q2)*np.cos(q3)*np.cos(q4)*np.cos(q5)*np.cos(q6 - th2) - np.sin(q4)*np.cos(q2)*np.cos(q5)*np.cos(q6 - th2) + np.sin(q6 - th2)*np.cos(q2)*np.cos(q4))
        J_37 = 0

        J = np.matrix([ [ J_11 , J_12 , J_13 , J_14 , J_15 , J_16 , J_17 ],\
                        [ J_21 , J_22 , J_23 , J_24 , J_25 , J_26 , J_27 ],\
                        [ J_31 , J_32 , J_33 , J_34 , J_35 , J_36 , J_37 ]])
        return J

    def tf_A01(self, r_joints_array):

        q1 = r_joints_array[0]
        l1 = self.l1

        tf = np.matrix([[np.cos(q1) , -np.sin(q1) , 0 ,  0],\
                        [np.sin(q1) ,  np.cos(q1) , 0 ,  0],\
                        [0 ,                    0 , 1 , l1],\
                        [0 ,                    0 , 0 ,  1]])
        return tf

    def tf_A02(self, r_joints_array):
        
        q2 = r_joints_array[1]

        tf_A12 = np.matrix([[ np.cos(q2) , -np.sin(q2) , 0 , 0],\
                            [          0 ,           0 , 1 , 0],\
                            [-np.sin(q2) , -np.cos(q2) , 0 , 0],\
                            [          0 ,           0 , 0 , 1]])
        tf = np.dot( self.tf_A01(r_joints_array), tf_A12 )
        return tf

    def tf_A03(self, r_joints_array):

        q3 = r_joints_array[2]
        l2 = self.l2

        tf_A23 = np.matrix([[np.cos(q3) , -np.sin(q3) ,  0 ,   0],\
                            [         0 ,           0 , -1 , -l2],\
                            [np.sin(q3) ,  np.cos(q3) ,  0 ,   0],\
                            [         0 ,           0 ,  0 ,   1]])
        tf = np.dot( self.tf_A02(r_joints_array), tf_A23 )
        return tf

    def tf_A04(self, r_joints_array):

        q4 = r_joints_array[3]
        l3 = self.l3

        tf_A34 = np.matrix([[np.cos(q4) , -np.sin(q4) ,  0 , l3],\
                            [         0 ,           0 , -1 ,  0],\
                            [np.sin(q4) ,  np.cos(q4) ,  0 ,  0],\
                            [         0 ,           0 ,  0 ,  1]])
        tf = np.dot( self.tf_A03(r_joints_array), tf_A34 )
        return tf

    def tf_A05(self, r_joints_array):

        q5 = r_joints_array[4]
        th1 = self.theta1
        l4 = self.l4

        tf_A45 = np.matrix([[np.cos(q5) ,  -np.sin(q5) ,  0 ,  l4*np.sin(th1)],\
                            [0          ,            0 , -1 , -l4*np.cos(th1)],\
                            [np.sin(q5) ,   np.cos(q5) ,  0 ,               0],\
                            [0          ,            0 ,  0 ,               1]])
        tf = np.dot( self.tf_A04(r_joints_array), tf_A45 )
        return tf

    def tf_A06(self, r_joints_array):

        q6 = r_joints_array[5]
        th2 = self.theta2

        tf_A56 = np.matrix([[np.cos(q6) , -np.sin(q6) ,  0 , 0],\
                            [         0 ,           0 , -1 , 0],\
                            [np.sin(q6) ,  np.cos(q6) ,  0 , 0],\
                            [         0 ,           0 ,  0 , 1]])
        tf = np.dot( self.tf_A05(r_joints_array), tf_A56 )
        return tf

    def tf_A07(self, r_joints_array):

        q7 = r_joints_array[6]
        th2 = self.theta2
        l5 = self.l5

        tf_A67 = np.matrix([[ np.cos(q7) , -np.sin(q7) , 0 , l5*np.sin(th2)],\
                            [          0 ,           0 , 1 , l5*np.cos(th2)],\
                            [-np.sin(q7) , -np.cos(q7) , 1 ,              0],\
                            [          0 ,           0 , 0 ,              1]])
        tf = np.dot( self.tf_A06(r_joints_array), tf_A67 )
        return tf