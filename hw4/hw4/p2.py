#!/usr/bin/env python
import rospy
import math
import time
import numpy as np
from std_msgs.msg import *
from foundations_hw4.srv import *
    
def callback(data):
    getBomb = rospy.ServiceProxy('/vrep/youbot/bomb_sensor', BombTest)
    rospy.wait_for_service('/vrep/youbot/bomb_sensor')
    distribution = []
    
    Prior = 0.2

    for i in range(0,5):
        guess = getBomb(i+1)
        is_bomb = guess.is_bomb
        dist = guess.dist
        P1 = (4.0 - math.tanh(3.0 * (dist -  1.5)) - math.tanh(3.0)) / 4.0
        P2 = (math.tanh(3.0 * (dist - 1.5)) + math.tanh(3.0)) / 4.0
        #rospy.loginfo(is_bomb)
        if is_bomb == True:
            bel = P1 * Prior / (P1 * Prior + P2 * (1.0 - Prior))
            #rospy.loginfo(1)
        else:
            bel = (1.0 - P1) * Prior / ((1.0 - P1) * Prior + (1.0 - P2) * (1.0 - Prior))
            #rospy.loginfo(2)
        distribution.append(bel)
    sump = 0
    for i in range(0,5):
        sump = sump + distribution[i]
    for i in range(0,5):
        distribution[i] = distribution[i] / sump
    
    return FindBombResponse(distribution)
    

def p2_server():
    rospy.init_node('p2_server')

    s=rospy.Service('/p2', FindBomb, callback)
    
    rospy.spin()
    
if __name__ == '__main__':
    p2_server()