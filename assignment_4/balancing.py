#!/usr/bin/env python
# TODO : use velocity of lbot, rbot to check  concensus
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np

ANG_MAX = math.pi/18
VEL_MAX = 0.15
rospy.init_node('assign4', anonymous=True)


def vc(theta, vel_x, vel_y):
    gain_ang = 1  # modify if necessary

    ang = np.arctan2(vel_y, vel_x)
    ang_err = ang - theta
    if ang_err < -np.pi:
        ang_err += 2*np.pi
    elif ang_err > np.pi:
        ang_err -= 2*np.pi
    if abs(ang_err) < np.pi/2:
        ang_err2 = min(max(ang_err, -ANG_MAX), ANG_MAX)
        v_lin = min(max(math.cos(ang_err2) * math.sqrt(vel_x**2 + vel_y**2), -VEL_MAX), VEL_MAX)
        v_ang = gain_ang * ang_err2
        return v_lin, v_ang
    else:
        if ang_err < 0:
            ang_err2 = min(max(ang_err+np.pi, -ANG_MAX), ANG_MAX)
            v_lin = min(max(-math.cos(ang_err2) * math.sqrt(vel_x**2 + vel_y**2), -VEL_MAX), VEL_MAX)
            v_ang = gain_ang * ang_err2
            return v_lin, v_ang
        elif ang_err > 0:
            ang_err2 = min(max(ang_err-np.pi, -ANG_MAX), ANG_MAX)
            v_lin = min(max(-math.cos(ang_err2) * math.sqrt(vel_x**2 + vel_y**2), -VEL_MAX), VEL_MAX)
            v_ang = gain_ang * ang_err2
            return v_lin, v_ang


class Decentralised_bot:
    def __init__(self):
        now = rospy.Time.now()
        self.StartTime = now.secs
        self.PresTime = now.secs
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
        v = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y])
        self.lbot = {'x': x, 'th': theta, 'v': v, 'w': data.twist.twist.angular.z}

    def callback_right_odom(self, data):
        '''
        Get right robot data
        '''
        theta = 2*np.arctan2(data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        x = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        v = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y])
        self.rbot = {'x': x, 'th': theta, 'v': v, 'w': data.twist.twist.angular.z}

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
        velL = np.sqrt(sum(self.lbot['v']*self.lbot['v'])+(self.lbot['w']*self.lbot['w']))
        velR = np.sqrt(sum(self.rbot['v']*self.rbot['v'])+(self.rbot['w']*self.rbot['w']))
        if (dist < 0.1*self.tolerance) and (velL < 0.1*self.tolerance) and (velR < 0.1*self.tolerance):
            if self.t > 5:
                self.num += 1
            # publishing zero velocities
            vel_msg = Twist()
            self.pub_vel.publish(vel_msg)
            return
        self.num = 0
        v_lin, v_ang = vc(self.bot['th'], PO[0], PO[1])
        # v_lin, v_ang = vc(self.bot['th'], PO[0], 0)  # To restrict to x-axis
        # publishing the velocities
        vel_msg = Twist()
        vel_msg.linear.x = v_lin
        vel_msg.angular.z = v_ang
        self.pub_vel.publish(vel_msg)


r = rospy.Rate(30)
bot_i = Decentralised_bot()
path = []
# i = 0
# while i < 1e5:
#     i += 1

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
file_name = rospy.get_name()
file_name = file_name[7:]

with open("/root/catkin_ws/src/sc627_assignments/assignment_4/output{}.txt".format(file_name), "w") as file:
    for p in path:
        file.write("\n")
        file.write(str(p)[1:-1])
        pass
    file.write("\n")
