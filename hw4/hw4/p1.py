#!/usr/bin/env python
import rospy
import math
import time
from random import uniform
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import *
from foundations_hw4.srv import *
    
def callback(data):
    getP = rospy.ServiceProxy('/vrep/youbot/position_sensor', GetPosition)
    rospy.wait_for_service('/vrep/youbot/position_sensor')
    estimates = []
    confidence = []
    
    I = np.array([[1.0,0.0],[0.0,1.0]])
    A = np.array([[1.0,0.0],[0.0,1.0]])
    R = np.array([[uniform(-0.01,0.01),uniform(-0.01,0.01)],[uniform(-0.01,0.01),uniform(-0.01,0.01)]])
    #R = np.array([[0.0,0.0],[0.0,0.0]])
    C = np.array([[1.0,0.0],[0.0,1.0]])
    for i in range(0,5):
        block = getP(i+1)
        umiu = Umiu[i]
        usigma = Usigma[i]
        Loc = block.loc
        Cov = block.cov
        Q = np.array([[Cov[0],Cov[1]],[Cov[2],Cov[3]]])
        #rospy.loginfo(Q)
        z = np.array([[Loc.x],[Loc.y]])
        #rospy.loginfo(z)
        ut = np.dot(A,umiu)
        sigma = np.dot(np.dot(A,usigma),A.T) + R
        K = np.dot(np.dot(sigma,C.T),np.linalg.inv(np.dot(np.dot(C,sigma),C.T) + Q))
        miu = ut + np.dot(K,z-np.dot(C,ut))
        sig = np.dot((I - np.dot(K,C)),sigma)
        Umiu[i] = miu
        Usigma[i] = sig
        point1 = Point(miu[0][0],miu[1][0],0)
        point2 = Point(sig[0][0],sig[1][1],0)
        #rospy.loginfo(point2)
        estimates.append(point1)
        confidence.append(point2)
    #rospy.loginfo(estimates)
    #rospy.loginfo(confidence)
    #rospy.loginfo(Umiu[0])
    return KalmanUpdateResponse(estimates, confidence)
    

def p1_server():
    rospy.init_node('p1_server')
    global Umiu
    global Usigma
    miux = uniform(-0.25,0.25)
    miuy = uniform(-0.25,0.25)
    cxx = 0.3
    cxy = cyx = 0.5
    cyy = 0.4
    Umiu = np.array([[[miux],[miuy]],[[miux],[miuy]],[[miux],[miuy]],[[miux],[miuy]],[[miux],[miuy]]])
    Usigma = np.array([[[cxx,cxy],[cyx,cyy]],[[cxx,cxy],[cyx,cyy]],[[cxx,cxy],[cyx,cyy]],[[cxx,cxy],[cyx,cyy]],[[cxx,cxy],[cyx,cyy]]])
    s=rospy.Service('/p1', KalmanUpdate, callback)
    
    rospy.spin()
    
if __name__ == '__main__':
    p1_server()