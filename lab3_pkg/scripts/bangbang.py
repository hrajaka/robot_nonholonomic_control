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
import sys
import matplotlib.pyplot as plt

THRESHOLD_DIST = 0.05
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

    def controller(self, target_x, target_y, target_theta, target_phi, plot=True):
        if plot:
            real_x = []
            real_y = []
            real_theta = []
            real_phi = []

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

            if plot:
                real_x.append(self.x)
                real_y.append(self.y)
                real_theta.append(self.theta)
                real_phi.append(self.phi)

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


        # Execute translation step
        target_dist = np.sqrt((target_x-self.x)**2 + (target_y-self.y)**2)
        start_x = self.x
        start_y = self.y
        error_in_dist = target_dist

        while error_in_dist > THRESHOLD_DIST:

            if plot:
                real_x.append(self.x)
                real_y.append(self.y)
                real_theta.append(self.theta)
                real_phi.append(self.phi)

            dist_traveled = np.sqrt((self.x-start_x)**2 + (self.y-start_y)**2)
            error_in_dist = target_dist - dist_traveled
            trans_mag = self.Kp_x * error_in_dist
            if trans_mag > MAX_MAG:
                trans_mag = MAX_MAG
            elif trans_mag < -MAX_MAG:
                trans_mag = -MAX_MAG

            print('error: {}'.format(error_in_dist))
            print('trans_mag: {}'.format(trans_mag))

            self.cmd(trans_mag, 0)
            self.rate.sleep()
        self.cmd(0, 0)


        # Execute rotation step
        error_theta = target_theta - self.theta

        # Execute initial rotation step
        while np.abs(error_theta) > THRESHOLD_THETA:

            if plot:
                real_x.append(self.x)
                real_y.append(self.y)
                real_theta.append(self.theta)
                real_phi.append(self.phi)

            error_theta = target_theta - self.theta
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

        # Adjust phi
        error_phi = target_phi - self.phi

        # Execute initial rotation step
        while np.abs(error_phi) > THRESHOLD_PHI:

            if plot:
                real_x.append(self.x)
                real_y.append(self.y)
                real_theta.append(self.theta)
                real_phi.append(self.phi)

            error_phi = target_phi - self.phi
            turn_mag = np.abs(self.Kp_theta * error_theta)
            turn_mag = min(turn_mag, MAX_MAG)
            turn_d = np.sign(error_phi)
            print('error: {}'.format(error_phi))
            print('turn_mag: {}'.format(turn_mag))
            print('turn_d: {}'.format(turn_d))
            if turn_d == 0:
                turn_d = 1

            self.turn(turn_mag, turn_d)
            self.rate.sleep()
        self.cmd(0, 0)

        if plot:
            plt.figure()

            plt.subplot(223)
            plt.xlabel('timestep')
            plt.ylabel('x')
            plt.grid(True)
            plt.axvline(color='k')
            plt.axhline(color='k')
            plt.plot(range(len(real_x)), real_x)

            plt.subplot(221)
            plt.xlabel('timestep')
            plt.ylabel('x')
            plt.grid(True)
            plt.axvline(color='k')
            plt.axhline(color='k')
            plt.plot(range(len(real_y)), real_y)

            plt.subplot(222)
            plt.xlabel('timestep')
            plt.ylabel('theta')
            plt.grid(True)
            plt.axvline(color='k')
            plt.axhline(color='k')
            plt.plot(range(len(real_theta)), real_theta)

            plt.subplot(224)
            plt.xlabel('timestep')
            plt.ylabel('phi')
            plt.grid(True)
            plt.axvline(color='k')
            plt.axhline(color='k')
            plt.plot(range(len(real_phi)), real_phi)

            plt.tight_layout()
            plt.show()

if __name__ == '__main__':
    rospy.init_node('bangbang', anonymous=False)
    
    b = BangBang(10, 1, 10, 1)
        

    try:
        if len(sys.argv) == 5:
            b.controller(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
        else:
            # b.controller(5, 5, 0, 0)
            print('GIVE ME SOME ARGUMENTS')
    except KeyboardInterrupt:
        pass



    #b.run()




