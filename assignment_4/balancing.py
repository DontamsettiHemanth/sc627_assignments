#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np

ANG_MAX = math.pi/18
VEL_MAX = 0.15
rospy.init_node('assign4', anonymous=True)


def velocity_convert(theta, vel_x, vel_y):
    '''
    Robot pose (x, y, theta)  Note - theta in (0, 2pi)
    Velocity vector (vel_x, vel_y)
    '''

    gain_ang = 1  # modify if necessary

    ang = math.atan2(vel_y, vel_x)
    if ang < 0:
        ang += 2 * math.pi

    ang_err = min(max(ang - theta, -ANG_MAX), ANG_MAX)

    v_lin = min(max(math.cos(ang_err) * math.sqrt(vel_x**2 + vel_y**2), -VEL_MAX), VEL_MAX)
    v_ang = gain_ang * ang_err
    return v_lin, v_ang


class Decentralised_bot:
    def __init__(self):
        self.t = 0
        self.num = 0  # concensus number
        self.tolerance = 0.01
        self.lbot = None
        self.rbot = None
        self.bot = None
        self.bot_sub = rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.lbot_sub = rospy.Subscriber('/left_odom', Odometry, self.callback_left_odom)
        self.rbot_sub = rospy.Subscriber('/right_odom', Odometry, self.callback_right_odom)

        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def callback_odom(self, data):
        '''
        Get robot data
        '''
        theta = 2*np.arctan2(data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        x = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        v = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y])
        self.bot = {'x': x, 'th': theta, 'v': v, 'w': data.twist.twist.angular.z}
        self.t = data.header.stamp.secs + data.header.stamp.nsecs*1e-9

    def callback_left_odom(self, data):
        '''
        Get left robot data
        '''
        theta = 2*np.arctan2(data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        x = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        self.lbot = {'x': x, 'th': theta}

    def callback_right_odom(self, data):
        '''
        Get right robot data
        '''
        theta = 2*np.arctan2(data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        x = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        self.rbot = {'x': x, 'th': theta}

    def velocity_convert(self, vel):
        '''
        Velocity vector vel = (vel_x, vel_y)
        '''
        theta = self.bot['th']
        vel_x, vel_y = vel

        gain_ang = 1  # modify if necessary

        ang = np.arctan2(vel_y, vel_x)

        ang_err = min(max(ang - theta, -ANG_MAX), ANG_MAX)

        v_lin = min(max(math.cos(ang_err) * math.sqrt(vel_x**2 + vel_y**2), -VEL_MAX), VEL_MAX)
        v_ang = gain_ang * ang_err
        return v_lin, v_ang

    def consensus(self):
        '''
        Move to midpoint of l and r bots
        TODO Take care of collision
        '''
        if (self.lbot is None) or (self.rbot is None) or (self.bot is None):
            rospy.loginfo("self.bot/rbot/lbot is None")
            vel_msg = Twist()
            bot_i.pub_vel.publish(vel_msg)
            return

        mid = 0.5*(self.lbot['x'] + self.rbot['x'])
        PO = mid - self.bot['x']
        dist = np.sqrt(sum(PO*PO))
        if dist < self.tolerance:
            self.num += 1
            # publishing zero velocities
            vel_msg = Twist()
            self.pub_vel.publish(vel_msg)
            return
        self.num = 0
        v_lin, v_ang = self.velocity_convert(PO)
        # publishing the velocities
        vel_msg = Twist()
        vel_msg.linear.x = v_lin
        vel_msg.angular.z = v_ang
        self.pub_vel.publish(vel_msg)


r = rospy.Rate(30)
bot_i = Decentralised_bot()
path = []
i = 0
while i < 1e5:
    i += 1

vel_msg = Twist()
bot_i.pub_vel.publish(vel_msg)
iter = 0
while (~rospy.is_shutdown()) and (bot_i.num < 10):
    if bot_i.bot is not None:
        bot_i.consensus()
        # store robot path with time stamps (data available in odom topic)
        path.append([bot_i.t, bot_i.bot['x'][0], bot_i.bot['x'][1]])

        iter += 1
        if iter % 1000 == 0:
            rospy.loginfo("Completed %r iterations", iter)
    r.sleep()

if (~rospy.is_shutdown()):
    rospy.loginfo("Success : Reached the goal")
else:
    rospy.logwarn("Failure : Stopped by interruption")

with open("/root/catkin_ws/src/sc627_assignments/assignment_4/output.txt", "a") as file:
    for p in path:
        file.write("\n")
        file.write(str(p)[1:-1])
        pass
    file.write("\n")
exit(1)
