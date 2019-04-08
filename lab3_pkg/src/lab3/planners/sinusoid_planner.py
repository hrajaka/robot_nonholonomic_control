#!/usr/bin/env python
"""
Starter code for EE106B Turtlebot Lab
Author: Valmik Prabhu, Chris Correa
"""
import numpy as np
from scipy.integrate import quad
import sys
from copy import copy
from copy import deepcopy
import matplotlib.pyplot as plt

import rospy
from lab3_pkg.msg import BicycleCommandMsg, BicycleStateMsg
import tf2_ros
import tf

class SinusoidPlanner():
    def __init__(self, l, max_phi, max_u1, max_u2):
        """
        Turtlebot planner that uses sequential sinusoids to steer to a goal pose

        Parameters
        ----------
        l : float
            length of car
        """
        self.l = l
        self.max_phi = max_phi
        self.max_u1 = max_u1
        self.max_u2 = max_u2

    def plan_to_pose_old(self, start_state, goal_state, dt = 0.01, delta_t=4):
        """
        Plans to a specific pose in (x,y,theta,phi) coordinates.  You
        may or may not have to convert the state to a v state with state2v()
        This is a very optional function.  You may want to plan each component separately
        so that you can reset phi in case there's drift in phi

        Parameters
        ----------
        start_state: :obj:`BicycleStateMsg`
        goal_state: :obj:`BicycleStateMsg`
        dt : float
            how many seconds between trajectory timesteps
        delta_t : float
            how many seconds each trajectory segment should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """

        # This bit hasn't been exhaustively tested, so you might hit a singularity anyways
        max_abs_angle = max(abs(goal_state.theta), abs(start_state.theta))
        min_abs_angle = min(abs(goal_state.theta), abs(start_state.theta))
        
        #if (max_abs_angle > np.pi/2) and (min_abs_angle < np.pi/2):
        #    raise ValueError("You'll cause a singularity here. You should add something to this function to fix it")
        if abs(start_state.phi) > self.max_phi or abs(goal_state.phi) > self.max_phi:
            raise ValueError("Either your start state or goal state exceeds steering angle bounds")

        # We can only change phi up to some threshold
        self.phi_dist = min(
            abs(goal_state.phi - self.max_phi),
            abs(goal_state.phi + self.max_phi)
        )

        # INITIALIZE PATH
        path = []

        # MOVE X
        x_path =        self.steer_x(
                            start_state,
                            goal_state,
                            0,
                            dt,
                            delta_t
                        )
        path.extend(x_path)

        # RESET PHI
        phi_path =      self.steer_phi(
                            path[-1][2],
                            goal_state,
                            path[-1][0] + dt,
                            dt,
                            delta_t
                        )
        path.extend(phi_path)

        # # MOVE ALPHA (theta)
        # start_state_v = self.state2v(path[-1][2])
        # goal_state_v = self.state2v(goal_state)
        # err_alpha = goal_state_v[2] - start_state_v[2]
        # while abs(err_alpha) > 0.02:

        #     start_state_v = self.state2v(path[-1][2])
        #     err_alpha = goal_state_v[2] - start_state_v[2]
        #     step_goal_state_v = start_state_v
        #     step_goal_state_v[2] += 0.01 * np.sign(err_alpha)

        #     print('steer alpha: error = {}'.format(err_alpha))

        #     alpha_path =    self.steer_alpha(
        #                         path[-1][2],
        #                         step_goal_state_v,
        #                         path[-1][0] + dt,
        #                         dt,
        #                         0.3#delta_t
        #                     )
        #     path.extend(alpha_path)
        #     # RESET PHI
        #     phi_path =      self.steer_phi(
        #                         path[-1][2],
        #                         goal_state,
        #                         path[-1][0] + dt,
        #                         dt,
        #                         0.5
        #                     )
        #     path.extend(phi_path)


        # MOVE ALPHA

        start_state = path[-1][2]
        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        #err_alpha = goal_state_v[2] - start_state_v[2]
        err_theta = goal_state.theta - start_state.theta
        print('goal state before:')
        print(goal_state)
        full_turn_angle = np.pi/4
        num_full_turns = int(err_theta / full_turn_angle)
        remainder_turn_angle = err_theta % full_turn_angle

        print('err_theta = {}'.format(err_theta))
        print('full_turn_angle = {}'.format(full_turn_angle))
        print('num_full_turns = {}'.format(num_full_turns))
        print('remainder_turn_angle = {}'.format(remainder_turn_angle))
        alpha_path =    self.steer_alpha(
                            path[-1][2],
                            goal_state,
                            path[-1][0] + dt,
                            dt,
                            delta_t
                        )
        #path.extend(alpha_path)
        for turn_number in range(4): # not sure it makes sense except if we want to go to pi
            path.extend(alpha_path)

        '''
        # Make full turns
        for turn_number in range(num_full_turns):
            #print('hello', start_state_v, goal_state_v, err_alpha)
            goal_state_turn = deepcopy(goal_state)
            goal_state_turn.theta = start_state.theta + (turn_number + 1) * full_turn_angle
            print('current goal state:')
            print(goal_state_turn)
            alpha_path =        self.steer_alpha(
                                path[-1][2],
                                goal_state_turn,
                                path[-1][0] + dt,
                                dt,
                                delta_t
                            )
            path.extend(alpha_path)
        # Make remainder turn
        alpha_path =    self.steer_alpha(
                                path[-1][2],
                                goal_state,
                                path[-1][0] + dt,
                                dt,
                                delta_t
                        )
        path.extend(alpha_path)
        '''

        print('goal state after:')
        print(goal_state)
        # MOVE Y
        y_path =        self.steer_y(
                            path[-1][2],
                            goal_state,
                            path[-1][0] + dt,
                            dt,
                            delta_t
                        )
        path.extend(y_path)

        '''
        path = []
        for p in [x_path, phi_path, alpha_path, y_path]:
            path.extend(p)
        '''
        return path

    def plan_to_pose(self, start_state, goal_state, dt = 0.01, delta_t=4):
        """
        Plans to a specific pose in (x,y,theta,phi) coordinates.  You
        may or may not have to convert the state to a v state with state2v()
        This is a very optional function.  You may want to plan each component separately
        so that you can reset phi in case there's drift in phi

        Parameters
        ----------
        start_state: :obj:`BicycleStateMsg`
        goal_state: :obj:`BicycleStateMsg`
        dt : float
            how many seconds between trajectory timesteps
        delta_t : float
            how many seconds each trajectory segment should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """

        # This bit hasn't been exhaustively tested, so you might hit a singularity anyways
        max_abs_angle = max(abs(goal_state.theta), abs(start_state.theta))
        min_abs_angle = min(abs(goal_state.theta), abs(start_state.theta))
        
        #if (max_abs_angle > np.pi/2) and (min_abs_angle < np.pi/2):
        #    raise ValueError("You'll cause a singularity here. You should add something to this function to fix it")
        if abs(start_state.phi) > self.max_phi or abs(goal_state.phi) > self.max_phi:
            raise ValueError("Either your start state or goal state exceeds steering angle bounds")

        # We can only change phi up to some threshold
        self.phi_dist = min(
            abs(goal_state.phi - self.max_phi),
            abs(goal_state.phi + self.max_phi)
        )

        # INITIALIZE PATH
        path = []

        x_path =        self.steer_x(
                            start_state,
                            goal_state,
                            0,
                            dt,
                            delta_t
                        )
        path.extend(x_path)

        phi_path =      self.steer_phi(
                            path[-1][2],
                            goal_state,
                            path[-1][0] + dt,
                            dt,
                            delta_t
                        )
        path.extend(phi_path)

        alpha_path =      self.steer_alpha(
                            path[-1][2],
                            goal_state,
                            path[-1][0] + dt,
                            dt,
                            delta_t
                        )
        path.extend(alpha_path)

        y_path =        self.steer_y(
                            path[-1][2],
                            goal_state,
                            path[-1][0] + dt,
                            dt,
                            delta_t
                        )
        path.extend(y_path)

        return path

    def steer_x(self, start_state, goal_state, t0 = 0, dt = 0.01, delta_t = 2):
        """
        Create a trajectory to move the turtlebot in the x direction

        Parameters
        ----------
        start_state : :obj:`BicycleStateMsg`
            current state of the turtlebot
        start_state : :obj:`BicycleStateMsg`
            desired state of the turtlebot
        t0 : float
            what timestep this trajectory starts at
        dt : float
            how many seconds between each trajectory point
        delta_t : float
            how many seconds the trajectory should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """

        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        delta_x = goal_state_v[0] - start_state_v[0]

        v1 = delta_x/delta_t
        v2 = 0

        path, t = [], t0
        while t < t0 + delta_t:
            path.append([t, v1, v2])
            t = t + dt
        return self.v_path_to_u_path(path, start_state, dt)

    def steer_phi(self, start_state, goal_state, t0 = 0, dt = 0.01, delta_t = 2):
        """
        Create a trajectory to move the turtlebot in the phi direction

        Parameters
        ----------
        start_state : :obj:`BicycleStateMsg`
            current state of the turtlebot
        goal_state : :obj:`BicycleStateMsg`
            desired state of the turtlebot
        t0 : float
            what timestep this trajectory starts at
        dt : float
            how many seconds between each trajectory point
        delta_t : float
            how many seconds the trajectory should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """

        # ************* IMPLEMENT THIS
        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        delta_phi = goal_state_v[1] - start_state_v[1]

        v1 = 0
        v2 = delta_phi/delta_t

        path, t = [], t0
        while t < t0 + delta_t:
            path.append([t, v1, v2])
            t = t + dt
        return self.v_path_to_u_path(path, start_state, dt)

    def steer_alpha(self, start_state, goal_state, t0 = 0, dt = 0.01, delta_t = 2):
        """
        Create a trajectory to move the turtlebot in the alpha direction.
        Remember dot{alpha} = f(phi(t))*u_1(t) = f(frac{a_2}{omega}*sin(omega*t))*a_1*sin(omega*t)
        also, f(phi) = frac{1}{l}tan(phi)
        See the doc for more math details

        Parameters
        ----------
        start_state : :obj:`BicycleStateMsg`
            current state of the turtlebot
        goal_state : :obj:`BicycleStateMsg`
            desired state of the turtlebot
        t0 : float
            what timestep this trajectory starts at
        dt : float
            how many seconds between each trajectory point
        delta_t : float
            how many seconds the trajectory should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """

        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        delta_alpha = goal_state_v[2] - start_state_v[2]
        print('delta_alpha = {}'.format(delta_alpha))

        omega = 2*np.pi / delta_t

        a2 = min(1, self.phi_dist*omega)
        f = lambda phi: (1/self.l)*np.tan(phi) # This is from the car model
        phi_fn = lambda t: (a2/omega)*np.sin(omega*t) + start_state_v[1]
        integrand = lambda t: f(phi_fn(t))*np.sin(omega*t) # The integrand to find beta
        beta1 = (omega/np.pi) * quad(integrand, 0, delta_t)[0]

        a1 = (delta_alpha*omega)/(np.pi*beta1)

              
        v1 = lambda t: a1*np.sin(omega*(t))
        v2 = lambda t: a2*np.cos(omega*(t))

        path, t = [], t0
        while t < t0 + delta_t:
            path.append([t, v1(t-t0), v2(t-t0)])
            t = t + dt

        '''
        print('------------------')
        print('steer_alpha:')
        print('initial alpha: {}'.format(start_state_v[2]))
        print('goal alpha: {}'.format(goal_state_v[2]))
        print('calculated coefficients:')
        print('  a1 = {}'.format(a1))
        print('  a2 = {}'.format(a2))
        print('  beta1 = {}'.format(beta1))
        print('------------------')
        '''
        return self.v_path_to_u_path(path, start_state, dt)


    def steer_y(self, start_state, goal_state, t0 = 0, dt = 0.01, delta_t = 2):
        """
        Create a trajectory to move the turtlebot in the y direction.
        Remember, dot{y} = g(alpha(t))*v1 = frac{alpha(t)}{sqrt{1-alpha(t)^2}}*a_1*sin(omega*t)
        See the doc for more math details

        Parameters
        ----------
        start_state : :obj:`BicycleStateMsg`
            current state of the turtlebot
        goal_state : :obj:`BicycleStateMsg`
            desired state of the turtlebot
        t0 : float
            what timestep this trajectory starts at
        dt : float
            how many seconds between each trajectory point
        delta_t : float
            how many seconds the trajectory should run for

        Returns
        -------
        :obj:`list` of (float, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """

        def compute_beta1(a1, a2, start_state_v, omega, delta_t, delta_y):
            """
            computes the difference between a1 and delta_y omega / pi beta1
            """
            f = lambda phi: (1/self.l)*np.tan(phi) # This is from the car model

            '''
            test_phi = np.linspace(-a2/(2*omega), a2/(2*omega), 10)
            test_f = f(test_phi)
            print('Testing all possible f(phi)')
            plt.figure()
            plt.axvline(color='k')
            plt.axhline(color='k')
            plt.grid(True)
            plt.plot(test_phi, test_f)
            plt.show()
            '''


            # f2 = lambda tau: f(a2/(2*omega)*np.sin(2*omega*tau)) * a1*np.sin(omega*tau)
            #print('start_state_v[1] = {}'.format(start_state_v[1]))
            f2 = lambda tau: f(a2/(2*omega)*np.sin(2*omega*tau) + start_state_v[1]) * a1*np.sin(omega*tau) 
            '''
            test_tau = np.linspace(-2.1*a1, 2.1*a1, 10)
            test_tau = f2(test_phi)
            print('Testing all possible f2(tau)')
            plt.figure()
            plt.title('f2')
            plt.axvline(color='k')
            plt.axhline(color='k')
            plt.grid(True)
            plt.plot(test_phi, test_f)
            plt.show()
            '''

            g = lambda alpha: alpha/(np.sqrt(1-alpha**2)) # This is from the car model
            # g = lambda alpha: (alpha+start_state_v[2])/(np.sqrt(1-(alpha+start_state_v[2])**2)) # This is from the car model

            integrand = lambda t: g(quad(f2, 0, t)[0] + start_state_v[2]) * a1*np.sin(omega*t)

            ###
            # plt.figure()
            # ts = np.linspace(0, delta_t, 30)
            # for t in ts:
            #     the_quad = quad(f2, 0, t)[0] 
            #     print(the_quad)
            #     plt.plot(t, the_quad)
            # plt.show()
            ###
            #the_quad = quad(f2, 0, delta_t)[0] 
            #print(the_quad)
            ###

            beta1 = (omega/np.pi) * quad(integrand, 0, delta_t)[0]

            return beta1  

        # def compute_final_y(a1, a2, beta1, start_state_v, omega):
        #     print('computing final_y based on initial_y: {}'.format(start_state_v[3]))
        #     return start_state_v[3] + np.pi * a1 * beta1 / omega

        def compute_final_y(a1, a2, start_state_v, omega):
            f = lambda phi: (1/self.l)*np.tan(phi) # This is from the car model

            f2 = lambda tau: f(a2/(2*omega)*np.sin(2*omega*tau) + start_state_v[1]) * a1*np.sin(omega*tau) 
           
            g = lambda alpha: alpha/(np.sqrt(1-alpha**2)) # This is from the car model

            integrand = lambda t: g(quad(f2, 0, t)[0] + start_state_v[2]) * a1*np.sin(omega*t)

            y_final = quad(integrand, 0, delta_t)[0] + start_state_v[3]

            return y_final

        start_state_v = self.state2v(start_state)
        goal_state_v = self.state2v(goal_state)
        print('************************************')
        print('STEER Y ROUTINE')
        print('start state')
        print(start_state_v)
        print('goal state')
        print(goal_state_v)
        print('delta_t')
        print(delta_t)
        delta_y = goal_state_v[3] - start_state_v[3]
        print('delta_y')
        print(delta_y)
        omega = 2*np.pi / delta_t

        if False: #delta_y < 0.1:
            print("I have thresholded")
            a1 = 0
            a2 = 0
        else:
            a2 = min(0.01, self.phi_dist*omega)
            #a2 = 0.1
            # now we do the binary search
            a1_inf = 0
            a1_sup = 2
            # plot the value of final_y against a1 to see if increasing of decreasing
            a1s = np.linspace(a1_inf, a1_sup, 50)
            final_ys = []
            for a1 in a1s:
                #beta1 = compute_beta1(a1, a2, start_state_v, omega, delta_t, delta_y)
                final_ys.append(compute_final_y(a1, a2, start_state_v, omega))
                #print(final_ys[-1])
                #final_ys.append( start_state_v[3] + (np.pi*a1*compute_beta1(a1, a2, start_state_v, omega, delta_t, delta_y))/omega)

            plt.figure()
            plt.title('Binary Search Results')
            plt.xlabel('a1')
            plt.ylabel('y final')
            plt.grid(True)
            plt.axvline(color='k')
            plt.axhline(color='k')
            plt.plot(a1s, final_ys, color='b')
            plt.plot(a1s, goal_state_v[3]*np.ones(a1s.size), linestyle='--', color='r')

            

            for k in range(15):
                a1_middle = float((a1_inf + a1_sup)) / 2
                #print('a1_middle = {}'.format(a1_middle))
                # beta1 = compute_beta1(a1_middle, a2, start_state_v, omega, delta_t, delta_y)
                
                # final_y = start_state_v[3] + (np.pi*a1_middle*beta1)/omega
                final_y = compute_final_y(a1_middle, a2, start_state_v, omega)

                #print('final_y = {}'.format(final_y))
                plt.scatter(a1_middle, final_y, marker='o', color='r')
                if goal_state_v[3] - final_y > 0: 
                    a1_inf = a1_middle
                else:
                    a1_sup = a1_middle
            a1 = a1_middle
            plt.scatter(a1_middle, final_y, marker='o', color='g')
            plt.show()
            print('\n\n')

            print('steer_y:')
            print('initial y: {}'.format(start_state_v[3]))
            print('goal Y: {}'.format(goal_state_v[3]))
            print('calculated coefficients:')
            print('  a1 = {}'.format(a1))
            print('  a2 = {}'.format(a2))
            # print('  beta1 = {}'.format(beta1))

        # at this point we have determined the optimal value of a1

        v1 = lambda t: a1*np.sin(omega*(t))
        v2 = lambda t: a2*np.cos(2*omega*(t))

        path, t = [], t0
        while t < t0 + delta_t:
            path.append([t, v1(t-t0), v2(t-t0)])
            t = t + dt

        return self.v_path_to_u_path(path, start_state, dt)

    def state2v(self, state):
        """
        Takes a state in (x,y,theta,phi) coordinates and returns a state of (x,phi,alpha,y)

        Parameters
        ----------
        state : :obj:`BicycleStateMsg`
            some state

        Returns
        -------
        4x1 :obj:`numpy.ndarray`
            x, phi, alpha, y
        """
        return np.array([state.x, state.phi, np.sin(state.theta), state.y])

    def v_path_to_u_path(self, path, start_state, dt):
        """
        convert a trajectory in v commands to u commands

        Parameters
        ----------
        path : :obj:`list` of (float, float, float)
            list of (time, v1, v2) commands
        start_state : :obj:`BicycleStateMsg`
            starting state of this trajectory
        dt : float
            how many seconds between timesteps in the trajectory

        Returns
        -------
        :obj:`list` of (time, BicycleCommandMsg, BicycleStateMsg)
            This is a list of timesteps, the command to be sent at that time, and the predicted state at that time
        """
        def v2cmd(v1, v2, state):
            u1 = v1/np.cos(state.theta)
            u2 = v2
            return BicycleCommandMsg(u1, u2)

        curr_state = copy(start_state)
        for i, (t, v1, v2) in enumerate(path):
            cmd_u = v2cmd(v1, v2, curr_state)
            path[i] = [t, cmd_u, curr_state]

            curr_state = BicycleStateMsg(
                curr_state.x     + np.cos(curr_state.theta)               * cmd_u.linear_velocity*dt,
                curr_state.y     + np.sin(curr_state.theta)               * cmd_u.linear_velocity*dt,
                curr_state.theta + np.tan(curr_state.phi) / float(self.l) * cmd_u.linear_velocity*dt,
                curr_state.phi   + cmd_u.steering_rate*dt
            )

        return path
