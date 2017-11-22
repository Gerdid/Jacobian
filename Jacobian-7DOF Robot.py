# -*- coding: utf-8 -*-
"""
Created on Fri Nov  3 12:46:51 2017

@author: gerha
"""
import sympy as sym                     #
import numpy as np                      #
import math as math                     #Libraries import
from mpl_toolkits.mplot3d import Axes3D #
import matplotlib.pyplot as plt         #

#---Constants definition---#
pi=math.pi
#dof=7;      #Definition of DOF of robot
q1,q2,q3,q4,q5,q6,q7=sym.symbols('q1,q2,q3,q4,q5,q6,q7')    #joint variable as symbolic
d3=3        #
d5=5        #Distance between joints (depending on desing)
d7=7        #
minAngle=-90    #Minimum reacheale angle by the joint
maxAngle=91     #Maximum reachable angle by the joint
stepSize=10      #Degrees between each joint movement


#Robot dictionary
#key:number of joint
#[theta,d,alpha,a]
robot=\
{1:[q1,0,-pi/2,0],\
 2:[q2,0,pi/2,0],\
 3:[q3,d3,-pi/2,0],\
 4:[q4,0,pi/2,0],\
 5:[q5,d5,-pi/2,0],\
 6:[q6,0,pi/2,0],\
 7:[q7,d7,0,0]}

#Angular velocity (rad/s)
q=np.array([[3],[2],[9],[8],[5],[4],[7]])


def substitute(q1,q2,q3,q4,q5,q6,q7,dof):
    T=[0,0,0,0,0,0,0]
    q=[math.radians(q1),math.radians(q2),math.radians(q3),math.radians(q4),math.radians(q5),math.radians(q6),math.radians(q7)];     #degrees to radian
    #Evaluation for each joint according to the transformation formula
    for i in range(0,dof):
        currentJoint=robot.get(i+1)
        currentJoint[0]=q[i]
        mat=np.array([[np.cos(q[i]),-np.sin(q[i])*np.cos(currentJoint[2]),np.sin(q[i])*np.sin(currentJoint[2]),currentJoint[3]*np.cos(q[i])],\
                            [np.sin(q[i]),np.cos(q[i])*np.cos(currentJoint[2]),-np.cos(q[i])*np.sin(currentJoint[2]),currentJoint[3]*np.sin(q[i])],\
                            [0,np.sin(currentJoint[2]),np.cos(currentJoint[2]),currentJoint[1]],\
                            [0,0,0,1]])
        T[i]=mat
    return T;

def matrixMultiplication(T,dof):
    for i in range(0,dof-1):
        if i==0:
            matMult=np.dot(T[i],T[i+1])
            temp=matMult
        else:
           matMult=np.dot(temp,T[i+1])
           temp=matMult
    return temp;


if __name__ == '__main__':
    T=substitute(70,20,-30,60,-15,75,10,7);
    
    T0_7=matrixMultiplication(T,7)
    A0_1=T[0]
    A0_2=matrixMultiplication(T,2)  #0A1*1A2
    A0_3=matrixMultiplication(T,3)  #0A1*1A2*2A3
    A0_4=matrixMultiplication(T,4)  #0A1*1A2*2A3*3A4
    A0_5=matrixMultiplication(T,5)  #0A1*1A2*2A3*3A4*4A5
    A0_6=matrixMultiplication(T,6)  #0A1*1A2*2A3*3A4*4A5*5A6
    A0_7=T0_7                       #0A1*1A2*2A3*3A4*4A5*5A6*6A7

    z0_0=[0],[0],[1]
    z0_1=A0_1[0:3,2] 
    z0_2=A0_2[0:3,2]
    z0_3=A0_3[0:3,2]   
    z0_4=A0_4[0:3,2] 
    z0_5=A0_5[0:3,2]   
    z0_6=A0_6[0:3,2]
    z0_7=A0_7[0:3,2]
    
    p0_7=A0_7[0:3,3:4]-z0_0
    p1_7=A0_7[0:3,3:4]-A0_1[0:3,3:4]  
    p2_7=A0_7[0:3,3:4]-A0_2[0:3,3:4]
    p3_7=A0_7[0:3,3:4]-A0_3[0:3,3:4]   
    p4_7=A0_7[0:3,3:4]-A0_4[0:3,3:4]   
    p5_7=A0_7[0:3,3:4]-A0_5[0:3,3:4]   
    p6_7=A0_7[0:3,3:4]-A0_6[0:3,3:4]
       
    J1=np.append(np.cross(z0_0,p0_7,axis=0),z0_0)
    J2=np.append(np.cross(z0_1,p1_7,axis=0),z0_1)
    J3=np.append(np.cross(z0_2,p2_7,axis=0),z0_2)
    J4=np.append(np.cross(z0_3,p3_7,axis=0),z0_3)
    J5=np.append(np.cross(z0_4,p4_7,axis=0),z0_4)
    J6=np.append(np.cross(z0_5,p5_7,axis=0),z0_5)
    J7=np.append(np.cross(z0_6,p6_7,axis=0),z0_6)
    
    J=J1,J2,J3,J4,J5,J6,J7
    J=np.transpose(J)
    
    
    print("---Spatial Velocity---")
    print(np.dot(J,q))

    
    