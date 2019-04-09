#!/usr/bin/env python
"""
Starter code for EE106B Turtlebot Lab
Author: Valmik Prabhu, Chris Correa
"""
import numpy as np
import sys
import argparse
from scipy.integrate import quad
from scipy.linalg import expm
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
        self.start_time = None
        self.times = []
        self.plan_states = []
        self.actual_states = []


    def execute(self, plan):
        """
        Executes a plan made by the planner

        Parameters
        ----------
        plan : :obj:`list` of (time, BicycleCommandMsg, BicycleStateMsg)
        """
        print('Exectutor starting to exectute plan')

        if len(plan) == 0:
            return

        self.start_time = rospy.get_time()
        for (t, cmd, state) in plan:
            self.times.append(rospy.get_time() - self.start_time)
            self.actual_states.append([self.state.x,
                                       self.state.y,
                                       self.state.theta,
                                       self.state.phi])
            self.plan_states.append([state.x,
                                     state.y,
                                     state.theta,
                                     state.phi])
            self.cmd(cmd)
            self.rate.sleep()
            if rospy.is_shutdown():
                break
        self.cmd(BicycleCommandMsg())
        
        print('Exectutor done exectuting')
        self.plot()

    def plot(self):
        times_arr = np.array(self.times)
        actual_states_arr = np.array(self.actual_states)
        plan_states_arr = np.array(self.plan_states)

        

        plt.figure()

        plt.subplot(221)
        plt.xlabel('t (seconds)')
        plt.ylabel('x (m)')
        plt.grid(True)
        plt.axvline(color='k')
        plt.axhline(color='k')
        plt.plot(times_arr, plan_states_arr[:,0], color='g',
            linestyle='--', label='plan state')
        plt.plot(times_arr, actual_states_arr[:,0], color='r',
            label='actual state')
        plt.legend()
        
        plt.subplot(222)
        plt.xlabel('t (seconds)')
        plt.ylabel('y (m)')
        plt.grid(True)
        plt.axvline(color='k')
        plt.axhline(color='k')
        plt.plot(times_arr, plan_states_arr[:,1], color='g',
            linestyle='--', label='plan state')
        plt.plot(times_arr, actual_states_arr[:,1], color='r',
            label='actual state')
        plt.legend()

        plt.subplot(223)
        plt.xlabel('t (seconds)')
        plt.ylabel('theta (rad)')
        plt.grid(True)
        plt.axvline(color='k')
        plt.axhline(color='k')
        plt.plot(times_arr, plan_states_arr[:,2], color='g',
            linestyle='--', label='plan state')
        plt.plot(times_arr, actual_states_arr[:,2], color='r',
            label='actual state')
        plt.legend()

        plt.subplot(224)
        plt.xlabel('t (seconds)')
        plt.ylabel('phi (rad)')
        plt.grid(True)
        plt.axvline(color='k')
        plt.axhline(color='k')
        plt.plot(times_arr, plan_states_arr[:,3], color='g',
            linestyle='--', label='plan state')
        plt.plot(times_arr, actual_states_arr[:,3], color='r',
            label='actual state')
        plt.legend()

        plt.tight_layout()
        plt.show()


    def execute_closed_loop(self, plan, l, dt=0.01):
        """
        Executes a plan made by the planner with the closed loop controller

        Parameters
        ----------
        plan : :obj:`list` of (time, BicycleCommandMsg, BicycleStateMsg)
        """
        def compute_A(t_index):
            t = plan[t_index][0]
            u1 = plan[t_index][1].linear_velocity
            u2 = plan[t_index][1].steering_rate
            theta = plan[t_index][2].theta
            phi = plan[t_index][2].phi

            A00 = 0
            A01 = 0
            A02 = np.sin(theta) * u1
            A03 = 0

            A10 = 0
            A11 = 0
            A12 = -np.cos(theta) * u1
            A13 = 0

            A20 = 0
            A21 = 0
            A22 = 0
            A23 = (1/l) * (1/np.cos(phi))**2 * u1

            A30 = 0
            A31 = 0
            A32 = 0
            A33 = 0


            A = np.array([[A00, A01, A02, A03], 
                          [A10, A11, A12, A13], 
                          [A20, A21, A22, A23], 
                          [A30, A31, A32, A33], 
                         ])
            return A

        def compute_B(t_index):
            t = plan[t_index][0]
            u1 = plan[t_index][1].linear_velocity
            u2 = plan[t_index][1].steering_rate
            theta = plan[t_index][2].theta
            phi = plan[t_index][2].phi

            B00 = np.cos(theta)
            B01 = 0
         
            B10 = np.sin(theta)
            B11 = 0

            B20 = (1/l) * np.tan(theta)
            B21 = 0

            B30 = 0
            B31 = 1

            B = np.array([[B00, B01], 
                          [B10, B11], 
                          [B20, B21], 
                          [B30, B31], 
                         ])
            return B


        def integrand_00(tau, t, t_index):
            exp_A_tau = expm(-A*tau)
            exp_A_tauT = np.transpose(exp_A_tau)
            return np.exp(6*0.1*(tau-t)) * np.dot( np.dot(exp_A_tau, B), np.dot( BT, exp_A_tauT))[0,0] 

        def integrand_01(tau, t, t_index):
            exp_A_tau = expm(-A*tau)
            exp_A_tauT = np.transpose(exp_A_tau)
            return np.exp(6*0.1*(tau-t)) * np.dot( np.dot(exp_A_tau, B), np.dot( BT, exp_A_tauT))[0,1] 

        def integrand_02(tau, t, t_index):
            exp_A_tau = expm(-A*tau)
            exp_A_tauT = np.transpose(exp_A_tau)
            return np.exp(6*0.1*(tau-t)) * np.dot( np.dot(exp_A_tau, B), np.dot( BT, exp_A_tauT))[0,2] 

        def integrand_03(tau, t, t_index):
            exp_A_tau = expm(-A*tau)
            exp_A_tauT = np.transpose(exp_A_tau)
            return np.exp(6*0.1*(tau-t)) * np.dot( np.dot(exp_A_tau, B), np.dot( BT, exp_A_tauT))[0,3] 


        def integrand_10(tau, t, t_index):
            exp_A_tau = expm(-A*tau)
            exp_A_tauT = np.transpose(exp_A_tau)
            return np.exp(6*0.1*(tau-t)) * np.dot( np.dot(exp_A_tau, B), np.dot( BT, exp_A_tauT))[1,0] 

        def integrand_11(tau, t, t_index):
            exp_A_tau = expm(-A*tau)
            exp_A_tauT = np.transpose(exp_A_tau)
            return np.exp(6*0.1*(tau-t)) * np.dot( np.dot(exp_A_tau, B), np.dot( BT, exp_A_tauT))[1,1] 

        def integrand_12(tau, t, t_index):
            exp_A_tau = expm(-A*tau)
            exp_A_tauT = np.transpose(exp_A_tau)
            return np.exp(6*0.1*(tau-t)) * np.dot( np.dot(exp_A_tau, B), np.dot( BT, exp_A_tauT))[1,2] 

        def integrand_13(tau, t, t_index):
            exp_A_tau = expm(-A*tau)
            exp_A_tauT = np.transpose(exp_A_tau)
            return np.exp(6*0.1*(tau-t)) * np.dot( np.dot(exp_A_tau, B), np.dot( BT, exp_A_tauT))[1,3] 


        def integrand_20(tau, t, t_index):
            exp_A_tau = expm(-A*tau)
            exp_A_tauT = np.transpose(exp_A_tau)
            return np.exp(6*0.1*(tau-t)) * np.dot( np.dot(exp_A_tau, B), np.dot(BT, exp_A_tauT))[2,0] 

        def integrand_21(tau, t, t_index):
            exp_A_tau = expm(-A*tau)
            exp_A_tauT = np.transpose(exp_A_tau)
            return np.exp(6*0.1*(tau-t)) * np.dot( np.dot(exp_A_tau, B), np.dot(BT, exp_A_tauT))[2,1] 

        def integrand_22(tau, t, t_index):
            exp_A_tau = expm(-A*tau)
            exp_A_tauT = np.transpose(exp_A_tau)
            return np.exp(6*0.1*(tau-t)) * np.dot( np.dot(exp_A_tau, B), np.dot(BT, exp_A_tauT))[2,2] 

        def integrand_23(tau, t, t_index):
            exp_A_tau = expm(-A*tau)
            exp_A_tauT = np.transpose(exp_A_tau)
            return np.exp(6*0.1*(tau-t)) * np.dot( np.dot(exp_A_tau, B), np.dot(BT, exp_A_tauT))[2,3] 


        def integrand_30(tau, t, t_index):
            exp_A_tau = expm(-A*tau)
            exp_A_tauT = np.transpose(exp_A_tau)
            return np.exp(6*0.1*(tau-t)) * np.dot( np.dot(exp_A_tau, B), np.dot(BT, exp_A_tauT))[3,0] 

        def integrand_31(tau, t, t_index):
            exp_A_tau = expm(-A*tau)
            exp_A_tauT = np.transpose(exp_A_tau)
            return np.exp(6*0.1*(tau-t)) * np.dot( np.dot(exp_A_tau, B), np.dot(BT, exp_A_tauT))[3,1] 

        def integrand_32(tau, t, t_index):
            exp_A_tau = expm(-A*tau)
            exp_A_tauT = np.transpose(exp_A_tau)
            return np.exp(6*0.1*(tau-t)) * np.dot( np.dot(exp_A_tau, B), np.dot(BT, exp_A_tauT))[3,2] 

        def integrand_33(tau, t, t_index):
            exp_A_tau = expm(-A*tau)
            exp_A_tauT = np.transpose(exp_A_tau)
            return np.exp(6*0.1*(tau-t)) * np.dot( np.dot(exp_A_tau, B), np.dot(BT, exp_A_tauT))[3,3] 


        print("COMPUTING THE INTERESTING MATRICES")

        Pcs = []
        Bs = []
        nb_steps_dt = 5

        for t_index in range(50):#range(np.asarray(plan).shape[0]-1):

            A = compute_A(t_index)
            B = compute_B(t_index)
            AT = np.transpose(A)
            BT = np.transpose(B)
           

            t = plan[t_index][0]
            t_plus_dt = plan[t_index+1][0]
            Hc = np.zeros((4, 4))

            Hc[0,0] = quad(integrand_00, t, t_plus_dt, args=(t, t_index))[0]
            Hc[0,1] = quad(integrand_01, t, t_plus_dt, args=(t, t_index))[0]
            Hc[0,2] = quad(integrand_02, t, t_plus_dt, args=(t, t_index))[0]
            Hc[0,3] = quad(integrand_03, t, t_plus_dt, args=(t, t_index))[0]

            Hc[1,0] = quad(integrand_10, t, t_plus_dt, args=(t, t_index))[0]
            Hc[1,1] = quad(integrand_11, t, t_plus_dt, args=(t, t_index))[0]
            Hc[1,2] = quad(integrand_12, t, t_plus_dt, args=(t, t_index))[0]
            Hc[1,3] = quad(integrand_13, t, t_plus_dt, args=(t, t_index))[0]

            Hc[2,0] = quad(integrand_20, t, t_plus_dt, args=(t, t_index))[0]
            Hc[2,1] = quad(integrand_21, t, t_plus_dt, args=(t, t_index))[0]
            Hc[2,2] = quad(integrand_22, t, t_plus_dt, args=(t, t_index))[0]
            Hc[2,3] = quad(integrand_23, t, t_plus_dt, args=(t, t_index))[0]

            Hc[3,0] = quad(integrand_30, t, t_plus_dt, args=(t, t_index))[0]
            Hc[3,1] = quad(integrand_31, t, t_plus_dt, args=(t, t_index))[0]
            Hc[3,2] = quad(integrand_32, t, t_plus_dt, args=(t, t_index))[0]
            Hc[3,3] = quad(integrand_33, t, t_plus_dt, args=(t, t_index))[0]


            # t = plan[t_index][0]
            # A_t = compute_A(t_index)
            # B_t = compute_B(t_index)
            # A_t_T = np.transpose(A_t)
            # B_t_T = np.transpose(B_t)

            # t_dt_ = plan[t_index + nb_steps_dt][0]
            # A_t_dt_ = compute_A(t_index + nb_steps_dt)
            # B_t_dt_ = compute_B(t_index + nb_steps_dt)
            # A_t_dt__T = np.transpose(A_t_dt)
            # B_t_dt__T = np.transpose(B_t_dt)

            # left_value = np.dot(np.dot(np.expm(-A_t*t), B_t), np.dot(B_t_T, np.transpose(np.expm(-A_t*t))))
            # right_value = np.exp(6*0.1*t_dt-t)



            Pc = np.linalg.inv(Hc)
            Pcs.append(Pc)

            B = compute_B(t_index)

            Bs.append(B)

        print('Exectutor starting to exectute plan')

        print(len(Pcs))
        print(len(Bs))

        GAMMA = 0.1

        if len(plan) == 0:
            return

        self.start_time = rospy.get_time()
        for i, (t, cmd, state) in enumerate(plan):
            self.times.append(rospy.get_time() - self.start_time)
            self.actual_states.append([self.state.x,
                                       self.state.y,
                                       self.state.theta,
                                       self.state.phi])
            self.plan_states.append([state.x,
                                     state.y,
                                     state.theta,
                                     state.phi])

            curr_state = np.array([self.state.x, self.state.y, self.state.theta, self.state.phi]).reshape((4,1))
            plan_state = np.array([state.x, state.y, state.theta, state.phi]).reshape((4,1))
            
            feedback_term = GAMMA * np.dot(np.dot(np.transpose(Bs[i]), Pcs[i]), curr_state - plan_state)

            u1_feedback = cmd.linear_velocity - feedback_term[0]
            u2_feedback = cmd.steering_rate - feedback_term[1]

            self.cmd(BicycleCommandMsg(u1_feedback, u2_feedback))
            self.rate.sleep()
            if rospy.is_shutdown():
                break
        self.cmd(BicycleCommandMsg())
        
        print('Exectutor done exectuting')
        self.plot()
 


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
    
    l = 0.3 
    p = SinusoidPlanner(l, 0.3, 2, 3)
    goalState = BicycleStateMsg(args.x, args.y, args.theta, args.phi)
    delta_t = 10
    '''
    plan = p.plan_to_pose(ex.state, goalState, 0.01, delta_t)
    '''
    full_turn_angle = np.pi/4

    plan = []

    if args.theta > full_turn_angle:
        num_full_turns = int(args.theta / full_turn_angle)
        remainder_turn_angle = args.theta % full_turn_angle

        for turn_nbr in range(num_full_turns):
            temp_goalState = BicycleStateMsg(args.x, args.y, (turn_nbr+1)*full_turn_angle, args.phi)
            temp_plan = p.plan_to_pose(ex.state, temp_goalState, 0.01, delta_t)
            plan.extend(temp_plan)

    temp_plan = p.plan_to_pose(ex.state, goalState, 0.01, delta_t)
    plan.extend(temp_plan)

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


    ex.execute_closed_loop(plan, l)
    # ex.execute(plan)
    
    print "Final State"
    print ex.state


