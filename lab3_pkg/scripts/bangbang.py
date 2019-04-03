#!/usr/bin/env python
"""
Starter code for EE106B Turtlebot Lab
Author: Valmik Prabhu, Chris Correa
"""
import rospy
from lab3_pkg.msg import BicycleCommandMsg, BicycleStateMsg
import numpy as np
import tf2_ros
import tf

THRESHOLD_X = 0.05
THRESHOLD_Y = 0.05
THRESHOLD_THETA = 0.1
THRESHOLD_PHI = 0.1

MAX_MAG = 1.5

class BangBang(object):
    """docstring for BangBang"""
    def __init__(self, Kp_x, Kp_y, Kp_theta, Kp_phi):
        self.pub = rospy.Publisher('/bicycle/cmd_vel', BicycleCommandMsg, queue_size=10)
        self.sub = rospy.Subscriber('/bicycle/state', BicycleStateMsg, self.stateListener)
        self.x = None
        self.y = None
        self.theta = None
        self.phi = None
        self.Kp_x = Kp_x
        self.Kp_y = Kp_y
        self.Kp_theta = Kp_theta
        self.Kp_phi = Kp_phi
        self.rate = rospy.Rate(10)

    def run(self):
        for i in range(1):
            if rospy.is_shutdown():
                self.cmd(0,0)
                break
            self.rate.sleep()
            self.turn(1, 1)

    def strafe(self, mag, d):
        self.turn(mag, -d)
        self.rate.sleep()
        self.cmd(2*mag, 0)
        self.rate.sleep()
        self.rate.sleep()
        self.turn(mag, d)
        self.rate.sleep()
        self.cmd(-2*mag, 0)
        self.rate.sleep()
        self.rate.sleep()
        self.cmd(0, 0)
        
    def turn(self, mag, d):
        self.cmd(mag, -d*mag)

        self.rate.sleep()
        self.cmd(-mag, -d*mag)
        self.rate.sleep()
        self.cmd(-mag, d*mag)
        self.rate.sleep()
        self.cmd(mag, d*mag)
        self.rate.sleep()
        self.cmd(0, 0)

    def cmd(self, u1, u2):
        self.pub.publish(BicycleCommandMsg(u1, u2))
        #print('u1 = {}, u2 = {}'.format(u1, u2))

    def stateListener(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.phi = msg.phi
        #print('state: {} {} {} {}'.format(self.x, self.y, self.theta, self.phi))

    def controller(self, target_x, target_y, target_theta, target_phi):
        # Wait until a valid state exists for all state variables
        while self.x == None or self.y == None or self.theta == None or self.phi == None:
            self.rate.sleep()

        # initially send zero command
        self.cmd(0, 0)

        # Calculate angle for inital rotation step
        trajectory_theta = np.arctan2(target_y - self.y, target_x - self.x)
        print('trajectory_theta: {}'.format(trajectory_theta))
        error_theta = trajectory_theta - self.theta

        # Execute initial rotation step
        while np.abs(error_theta) > THRESHOLD_THETA:

            error_theta = trajectory_theta - self.theta
            turn_mag = np.abs(self.Kp_theta * error_theta)
            turn_mag = min(turn_mag, MAX_MAG)
            turn_d = np.sign(error_theta)
            print('error: {}'.format(error_theta))
            print('turn_mag: {}'.format(turn_mag))
            print('turn_d: {}'.format(turn_d))
            if turn_d == 0:
                turn_d = 1

            self.turn(turn_mag, turn_d)
            self.rate.sleep()
        self.cmd(0, 0)

        # Execute rotation step

        # Adjust phi


if __name__ == '__main__':
    rospy.init_node('bangbang', anonymous=False)
    
    b = BangBang(1, 1, 10, 1)

    try:
        b.controller(5, 5, 0, 0)
    except KeyboardInterrupt:
        pass
    #b.run()




