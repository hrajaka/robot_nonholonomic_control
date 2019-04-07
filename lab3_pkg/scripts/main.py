#!/usr/bin/env python
"""
Starter code for EE106B Turtlebot Lab
Author: Valmik Prabhu, Chris Correa
"""
import numpy as np
import sys
import argparse

import tf2_ros
import tf
from std_srvs.srv import Empty as EmptySrv
import rospy
from lab3_pkg.msg import BicycleCommandMsg, BicycleStateMsg

from lab3.planners import SinusoidPlanner
import matplotlib.pyplot as plt

class Exectutor(object):
    def __init__(self):
        """
        Executes a plan made by the planner
        """
        self.pub = rospy.Publisher('/bicycle/cmd_vel', BicycleCommandMsg, queue_size=10)
        self.sub = rospy.Subscriber('/bicycle/state', BicycleStateMsg, self.subscribe )
        self.rate = rospy.Rate(100)
        self.state = BicycleStateMsg()
        rospy.on_shutdown(self.shutdown)

    def execute(self, plan):
        """
        Executes a plan made by the planner

        Parameters
        ----------
        plan : :obj:`list` of (time, BicycleCommandMsg, BicycleStateMsg)
        """
        if len(plan) == 0:
            return

        for (t, cmd, state) in plan:
            self.cmd(cmd)
            self.rate.sleep()
            if rospy.is_shutdown():
                break
        self.cmd(BicycleCommandMsg())

    def cmd(self, msg):
        """
        Sends a command to the turtlebot / turtlesim

        Parameters
        ----------
        msg : :obj:`BicycleCommandMsg`
        """
        self.pub.publish(msg)

    def subscribe(self, msg):
        """
        callback fn for state listener.  Don't call me...
        
        Parameters
        ----------
        msg : :obj:`BicycleStateMsg`
        """
        self.state = msg

    def shutdown(self):
        rospy.loginfo("Shutting Down")
        self.cmd(BicycleCommandMsg())

def parse_args():
    """
    Pretty self explanatory tbh
    """
    parser = argparse.ArgumentParser()
    parser.add_argument('-x', type=float, default=0.0, help='Desired position in x')
    parser.add_argument('-y', type=float, default=0.0, help='Desired position in y')
    parser.add_argument('-theta', type=float, default=0.0, help='Desired turtlebot angle')
    parser.add_argument('-phi', type=float, default=0.0, help='Desired angle of the (imaginary) steering wheel')
    return parser.parse_args()

if __name__ == '__main__':
    rospy.init_node('sinusoid', anonymous=False)
    args = parse_args()

    # reset turtlesim state
    print 'Waiting for converter/reset service ...',
    rospy.wait_for_service('/converter/reset')
    print 'found!'
    reset = rospy.ServiceProxy('/converter/reset', EmptySrv)
    reset()
    
    ex = Exectutor()

    print "Initial State"
    print ex.state

    p = SinusoidPlanner(0.3, 0.3, 2, 3)
    goalState = BicycleStateMsg(args.x, args.y, args.theta, args.phi)
    delta_t = 10
    plan = p.plan_to_pose(ex.state, goalState, 0.01, delta_t)
    
    plan_x = []
    plan_y = []
    plan_theta = []
    plan_phi = []
    plan_u1 = []
    plan_u2 = []
    for p_i in plan:
        plan_x.append(p_i[2].x)
        plan_y.append(p_i[2].y)
        plan_theta.append(p_i[2].theta)
        plan_phi.append(p_i[2].phi)
        plan_u1.append(p_i[1].linear_velocity)
        plan_u2.append(p_i[1].steering_rate)

    plt.figure()

    plt.subplot(321)
    plt.xlabel('t')
    plt.ylabel('u1')
    plt.grid(True)
    plt.axvline(color='k')
    plt.axhline(color='k')
    plt.plot(range(len(plan_u1)), plan_u1, color='g')
    plt.axhline(-2, linestyle='--', color='b')
    plt.axhline(2, linestyle='--', color='b')

    plt.subplot(322)
    plt.xlabel('t')
    plt.ylabel('u2')
    plt.grid(True)
    plt.axvline(color='k')
    plt.axhline(color='k')
    plt.plot(range(len(plan_u2)), plan_u2, color='g')
    plt.axhline(-3, linestyle='--', color='b')
    plt.axhline(3, linestyle='--', color='b')

    plt.subplot(323)
    plt.xlabel('t')
    plt.ylabel('x')
    plt.grid(True)
    plt.axvline(color='k')
    plt.axhline(color='k')
    plt.plot(range(len(plan_x)), plan_x, color='r')
    plt.plot(range(len(plan_x)), [goalState.x]*len(plan_x), linestyle='--', color='r')

    plt.subplot(324)
    plt.xlabel('t')
    plt.ylabel('y')
    plt.grid(True)
    plt.axvline(color='k')
    plt.axhline(color='k')
    plt.plot(range(len(plan_y)), plan_y, color='r')
    plt.plot(range(len(plan_y)), [goalState.y]*len(plan_y), linestyle='--', color='r')

    plt.subplot(325)
    plt.xlabel('t')
    plt.ylabel('theta')
    plt.grid(True)
    plt.axvline(color='k')
    plt.axhline(color='k')
    plt.plot(range(len(plan_theta)), plan_theta, color='r')
    plt.plot(range(len(plan_theta)), [goalState.theta]*len(plan_theta), linestyle='--', color='r')

    plt.subplot(326)
    plt.xlabel('t')
    plt.ylabel('phi')
    plt.grid(True)
    plt.axvline(color='k')
    plt.axhline(color='k')
    plt.plot(range(len(plan_phi)), plan_phi, color='r')
    plt.plot(range(len(plan_phi)), [goalState.phi]*len(plan_phi), linestyle='--', color='r')
    plt.axhline(-0.3, linestyle='--', color='b')
    plt.axhline(0.3, linestyle='--', color='b')
    
    plt.tight_layout()
    plt.show()

    print "Predicted Initial State"
    print plan[0][2]
    print "Predicted Final State"
    print plan[-1][2]

    ex.execute(plan)
    print "Final State"
    print ex.state


