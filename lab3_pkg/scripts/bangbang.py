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
from std_srvs.srv import Empty as EmptySrv

THRESHOLD_DIST = 0.001
THRESHOLD_THETA = 0.09
THRESHOLD_PHI = 0.05

MAX_MAG = 0.2#1.5

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

        self.u1s = []
        self.u2s = []
        self.time_u = []

    def sleep_num(self, num=1):
        for i in range(num):
            self.rate.sleep()

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
        self.turn(mag, d)
        self.rate.sleep()
        self.cmd(-2*mag, 0)
        self.rate.sleep()
        self.cmd(0, 0)

    def adjust_phi(self, mag, d):
        self.cmd(0, d*mag)
        self.rate.sleep()
        
    def turn(self, mag, d):
        sleep_len = 10
        self.cmd(mag, -d*mag)
        self.sleep_num(sleep_len)
        self.cmd(0, 0)
        self.sleep_num(1)
        self.cmd(-mag, -d*mag)
        self.sleep_num(sleep_len)
        self.cmd(-mag, d*mag)
        self.sleep_num(sleep_len)
        self.cmd(0, 0)
        self.sleep_num(1)
        self.cmd(mag, d*mag)
        self.sleep_num(sleep_len)
        self.cmd(0, 0)
        self.sleep_num(1)

    def cmd(self, u1, u2):
        self.pub.publish(BicycleCommandMsg(u1, u2))
        #print('u1 = {}, u2 = {}'.format(u1, u2))
        self.u1s.append(u1)
        self.u2s.append(u2)
        self.time_u.append(rospy.get_time() - self.time_initial)

    def stateListener(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta
        self.phi = msg.phi
        #print('state: {} {} {} {}'.format(self.x, self.y, self.theta, self.phi))

    def controller(self, target_x, target_y, target_theta, target_phi, plot=True):
        if plot:
            self.time_initial = rospy.get_time()
            time = []
            real_x = []
            real_y = []
            real_theta = []
            real_phi = []

        # Wait until a valid state exists for all state variables
        while self.x == None or self.y == None or self.theta == None or self.phi == None:
            self.rate.sleep()

        print(target_x, target_y, target_theta, target_phi)

        # initially send zero command
        self.cmd(0, 0)

        # Calculate angle for inital rotation step
        if np.abs(target_y - self.y) > 0.001 or np.abs(target_x - self.x) > 0.001:
            trajectory_theta = np.arctan2(target_y - self.y, target_x - self.x)
        else:
            trajectory_theta = self.theta
        print('trajectory_theta: {}'.format(trajectory_theta))
        error_theta = trajectory_theta - self.theta

        # Execute initial rotation step
        print('Executing initial rotation step')
        while np.abs(error_theta) > THRESHOLD_THETA:

            if plot:
                time.append(rospy.get_time() - self.time_initial)
                real_x.append(self.x)
                real_y.append(self.y)
                real_theta.append(self.theta)
                real_phi.append(self.phi)

            error_theta = trajectory_theta - self.theta
            turn_mag = np.abs(self.Kp_theta * error_theta)
            turn_mag = min(turn_mag, MAX_MAG)
            turn_d = np.sign(error_theta)*2
            # print('error: {}'.format(error_theta))
            # print('turn_mag: {}'.format(turn_mag))
            # print('turn_d: {}'.format(turn_d))
            if turn_d == 0:
                turn_d = 1

            self.turn(turn_mag, turn_d)
            self.rate.sleep()
        self.cmd(0, 0)

        print('adjusting phi')
        # set phi to 0
        trajectory_phi = 0
        error_phi = trajectory_phi - self.phi


        while np.abs(error_phi) > THRESHOLD_PHI:

            if plot:
                time.append(rospy.get_time() - self.time_initial)
                real_x.append(self.x)
                real_y.append(self.y)
                real_theta.append(self.theta)
                real_phi.append(self.phi)

            error_phi = trajectory_phi - self.phi
            phi_mag = np.abs(self.Kp_phi * error_phi)
            phi_mag = min(phi_mag, MAX_MAG)
            phi_d = np.sign(error_phi)
            if phi_d == 0:
                phi_d = 1
            self.adjust_phi(phi_mag, phi_d)
            self.rate.sleep()

        # Execute translation step
        print('Executing translation step')
        target_dist = np.sqrt((target_x-self.x)**2 + (target_y-self.y)**2)
        start_x = self.x
        start_y = self.y
        error_in_dist = target_dist

        while error_in_dist > THRESHOLD_DIST:

            if plot:
                time.append(rospy.get_time() - self.time_initial)
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

            #print('error: {}'.format(error_in_dist))
            #print('trans_mag: {}'.format(trans_mag))

            self.cmd(trans_mag, 0)
            self.rate.sleep()
        self.cmd(0, 0)


        # Execute rotation step
        print('Executing rotation step')
        error_theta = target_theta - self.theta

        while np.abs(error_theta) > THRESHOLD_THETA:

            if plot:
                time.append(rospy.get_time() - self.time_initial)
                real_x.append(self.x)
                real_y.append(self.y)
                real_theta.append(self.theta)
                real_phi.append(self.phi)

            error_theta = target_theta - self.theta
            print('Rotation step:')
            print('  target = {}rad'.format(target_theta))
            print('  current = {}rad'.format(self.theta))
            print('  error = {}rad'.format(error_theta))
            turn_mag = np.abs(self.Kp_theta * error_theta)
            turn_mag = min(turn_mag, MAX_MAG)
            turn_d = np.sign(error_theta)*2
            # print('error: {}'.format(error_theta))
            # print('turn_mag: {}'.format(turn_mag))
            # print('turn_d: {}'.format(turn_d))
            if turn_d == 0:
                turn_d = 1

            self.turn(turn_mag, turn_d)
            self.rate.sleep()
        self.cmd(0, 0)

        trajectory_phi = 0
        error_phi = trajectory_phi - self.phi
        while np.abs(error_phi) > THRESHOLD_PHI:

            if plot:
                time.append(rospy.get_time() - self.time_initial)
                real_x.append(self.x)
                real_y.append(self.y)
                real_theta.append(self.theta)
                real_phi.append(self.phi)

            error_phi = trajectory_phi - self.phi
            phi_mag = np.abs(self.Kp_phi * error_phi)
            phi_mag = min(phi_mag, MAX_MAG)
            phi_d = np.sign(error_phi)
            if phi_d == 0:
                phi_d = 1
            self.adjust_phi(phi_mag, phi_d)
            self.rate.sleep()
        

        if plot:
            plt.figure()

            plt.subplot(321)
            plt.xlabel('timestep')
            plt.ylabel('x')
            plt.grid(True)
            plt.axvline(color='k')
            plt.axhline(color='k')
            plt.axhline(target_x, linestyle='--', color='r')
            plt.plot(time, real_x, marker='.')

            plt.subplot(322)
            plt.xlabel('timestep')
            plt.ylabel('y')
            plt.grid(True)
            plt.axvline(color='k')
            plt.axhline(color='k')
            plt.axhline(target_y, linestyle='--', color='r')
            plt.plot(time, real_y, marker='.')

            plt.subplot(323)
            plt.xlabel('timestep')
            plt.ylabel('theta')
            plt.grid(True)
            plt.axvline(color='k')
            plt.axhline(color='k')
            plt.plot(time, real_theta, marker='.')

            plt.subplot(324)
            plt.xlabel('timestep')
            plt.ylabel('phi')
            plt.grid(True)
            plt.axvline(color='k')
            plt.axhline(color='k')
            plt.axhline(target_phi, linestyle='--', color='r')
            plt.plot(time, real_phi, marker='.')

            plt.subplot(325)
            plt.xlabel('timestep')
            plt.ylabel('u1')
            plt.grid(True)
            plt.axvline(color='k')
            plt.axhline(color='k')
            plt.plot(self.time_u, self.u1s, marker='.')

            plt.subplot(326)
            plt.xlabel('timestep')
            plt.ylabel('u2')
            plt.grid(True)
            plt.axvline(color='k')
            plt.axhline(color='k')
            plt.plot(self.time_u, self.u2s, marker='.')

            plt.tight_layout()
            plt.show()

if __name__ == '__main__':

    print 'Waiting for converter/reset service ...',
    rospy.wait_for_service('/converter/reset')
    print 'found!'
    reset = rospy.ServiceProxy('/converter/reset', EmptySrv)
    reset()

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




