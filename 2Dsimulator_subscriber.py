"""
Inverse kinematics of a two-joint arm
Left-click the plot to set the goal position of the end effector

Author: Daniel Ingram (daniel-s-ingram)
        Atsushi Sakai (@Atsushi_twi)

Ref: P. I. Corke, "Robotics, Vision & Control", Springer 2017, ISBN 978-3-319-54413-7 p102
- [Robotics, Vision and Control  SpringerLink](https://link.springer.com/book/10.1007/978-3-642-20144-8)

"""

import tkinter
import matplotlib
import matplotlib.pyplot as plt
matplotlib.use('GTK3Agg')
import numpy as np
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
import time
import roslib
import sys



# Simulation parameters
Kp = 15
dt = 0.01

# Link lengths
l1 = l2 = 1

# Set initial goal position to the initial end-effector position
x = 2
y = 0

show_animation = True

# if show_animation:
#     plt.ion()


def callback(data) :
    """
    Computes the inverse kinematics for a planar 2DOF arm
    """

    generated_path = Path()
    generated_path = data
    # print(trajectory)
    GOAL_TH=0.0
    theta1=0.0
    theta2=0.0

    for i in range (0,len(generated_path.poses)):
        x = generated_path.poses[i].pose.position.x
        y = generated_path.poses[i].pose.position.y
        
        m = 0

        if show_animation:
            plt.ion()



        while m<5:
            try:
                theta2_goal = np.arccos(
                    (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2))
                theta1_goal = np.math.atan2(y, x) - np.math.atan2(l2 *
                                                                  np.sin(theta2_goal), (l1 + l2 * np.cos(theta2_goal)))

                if theta1_goal < 0:
                    theta2_goal = -theta2_goal
                    theta1_goal = np.math.atan2(
                        y, x) - np.math.atan2(l2 * np.sin(theta2_goal), (l1 + l2 * np.cos(theta2_goal)))

                theta1 = theta1 + Kp * ang_diff(theta1_goal, theta1) * dt
                theta2 = theta2 + Kp * ang_diff(theta2_goal, theta2) * dt

                m = m + 1
            except ValueError as e:
                print("Unreachable goal")

            wrist = plot_arm(theta1, theta2, x, y)

            # check goal
            d2goal = np.hypot(wrist[0] - x, wrist[1] - y)


def plot_arm(theta1, theta2, x, y):  # pragma: no cover
    shoulder = np.array([0, 0])
    elbow = shoulder + np.array([l1 * np.cos(theta1), l1 * np.sin(theta1)])
    wrist = elbow + \
        np.array([l2 * np.cos(theta1 + theta2), l2 * np.sin(theta1 + theta2)])

    if show_animation:
        plt.cla()

        plt.plot([shoulder[0], elbow[0]], [shoulder[1], elbow[1]], 'k-')
        plt.plot([elbow[0], wrist[0]], [elbow[1], wrist[1]], 'k-')

        plt.plot(shoulder[0], shoulder[1], 'ro')
        plt.plot(elbow[0], elbow[1], 'ro')
        plt.plot(wrist[0], wrist[1], 'ro')

        plt.plot([wrist[0], x], [wrist[1], y], 'g--')
        plt.plot(x, y, 'g*')

        plt.xlim(-2, 2)
        plt.ylim(-2, 2)

        plt.show()
        plt.pause(dt)

    return wrist


def ang_diff(theta1, theta2):
    # Returns the difference between two angles in the range -pi to +pi
    return (theta1 - theta2 + np.pi) % (2 * np.pi) - np.pi



def animation():
    from random import random
    global x, y
    for i in range(5):
        x = 2.0 * random() - 1.0
        y = 2.0 * random() - 1.0


def main():  # pragma: no cover


    # for stopping simulation with the esc key.
    fig.canvas.mpl_connect('key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

    rospy.Subscriber("generate_motion_service_node/cartesian_path",  Path, callback)


if __name__ == "__main__":

    rospy.init_node("get_trajectory", anonymous=True)
    
    fig = plt.figure()
    main()
    rospy.spin()