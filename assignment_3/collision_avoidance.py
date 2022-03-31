#!/usr/bin/env python

import rospy
from sc627_helper.msg import ObsData
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np

ANG_MAX = math.pi/18
VEL_MAX = 0.15

rospy.init_node('assign3', anonymous=True)

class velocity_obstacles:

    def __init__(self):
        self.t = 0
        self.goal = np.array([5, 0])
        self.obs_xv = None
        self.bot_xv = None
        self.obs_sub = rospy.Subscriber('/obs_data', ObsData, self.callback_obs)  # topic name fixed
        self.bot_sub = rospy.Subscriber('/bot_1/odom', Odometry, self.callback_odom)  # topic name fixed

        self.pub_vel = rospy.Publisher('/bot_1/cmd_vel', Twist, queue_size = 10)
        rospy.sleep(.3)

    def velocity_convert(self, x, y, theta, vel_x, vel_y):
        '''
        Robot pose (x, y, theta)  Note - theta in (0, 2pi)
        Velocity vector (vel_x, vel_y)
        '''

        gain_ang = 1  # modify if necessary

        ang = math.atan2(vel_y, vel_x)
        if ang < 0:
            ang += 2 * math.pi

        ang_err = min(max(ang - theta, -ANG_MAX), ANG_MAX)

        v_lin =  min(max(math.cos(ang_err) * math.sqrt(vel_x ** 2 + vel_y ** 2), -VEL_MAX), VEL_MAX)
        v_ang = gain_ang * ang_err
        return v_lin, v_ang

    def callback_obs(self, data):
        '''
        Get obstacle data in the (pos_x, pos_y, vel_x, vel_y) for each obstacle
            obsData => Obs[] obstacles;
            obs =>  string obs
                    float64 pose_x
                    float64 pose_y
                    float64 vel_x
                    float64 vel_y
        '''
        obs = []
        for i in data.obstacles:
            x = np.array([i.pos_x, i.pos_y])
            v = np.array([i.vel_x, i.vel_y])
            obs.append({x: x, v: v})
        self.obs_xv = obs  # If all obstacles data are sent
        pass

    def callback_odom(self, data):
        '''
        Get robot data
        '''
        theta = 2*np.atan2(data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        x = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        v = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y]) 
        self.bot_xv = {x: x, th: theta, v: v, w: data.twist.twist.angular.z}
        self.t = data.header.stamp.secs + data.header.stamp.nsecs*1e-9
        pass

    def max_v(self, v1, v2):
        if v2[0] > v1[0]:
            return v2
        elif v1[0] > v2[0]:
            return v1
        else:
            PG = self.bot_xv.x - self.goal
            th_g = np.arctan2(PG[1], PG[0])
            if v1[1]-th_g <= v2[1]-th_g:
                return v1
            else:
                return v2


R = 0.15
r = rospy.Rate(30)
v_search = [i*0.01 for i in range(int(100*VEL_MAX), 0, -1)]
th_search = [- ANG_MAX + i*ANG_MAX/10 for i in range(-10, 11)]
VO = velocity_obstacles()
path = []
iter = 0
print "VO done"
d = rospy.Duration(0,10)
#   rospy.sleep(d)
print "sleep done"
while sum((VO.bot_xv.x - VO.goal)*(VO.bot_xv.x - VO.goal)) > 0.05:  # replace with destination reached?
    # calculate collision cone below
    once_free = False  # completed one search of all angles without collision
    free_point = np.array([0, 0])
    iter += 1
    if iter % 100 == 0:
        rospy.loginfo("Completed %r iterations", iter)
    rospy.loginfo("Completed %r iterations", iter)
    for v in v_search:
        for th in th_search:
            free_obs = True  # velocity vector avoids all obstacles?
            for obs in VO.obs_xv:
                # MV strategy
                bot_w = th - VO.bot_xv.th
                bot_v = np.array([v*np.cos(th), v*np.sin(th)])
                v_rel = bot_v - obs.v
                PO = VO.bot_xv.x - obs.x
                th_cri = np.arcsin(R/np.sqrt(sum(PO*PO)))
                alpha = np.arccos(sum(v_rel*PO)/np.sqrt(sum(v_rel*v_rel)*sum(PO*PO)))
                if alpha < th_cri:
                    # misses this obstacle
                    free_obs = False
                    break
            if free_obs:
                once_free = True
                free_point = VO.max_v(free_point, np.array([v, th]))
            pass
        if once_free:
            break

    v_lin, v_ang = VO.velocity_convert(VO.bot_xv.x[0], VO.bot_xv.x[1], VO.bot_xv.th, free_point[0]*np.cos(free_point[1]), free_point[0]*np.sin(free_point[1]))
    # publish the velocities below
    vel_msg = Twist()
    vel_msg.linear.x = v_lin
    vel_msg.angular.z = v_ang
    VO.pub_vel.publish(vel_msg)

    # storing robot path with time stamps (data available in odom topic)
    path.append([VO.t, VO.bot_xv.x])
    r.sleep()

vel_msg = Twist()
VO.pub_vel.publish(vel_msg)
rospy.loginfo("Success : Reached the goal")
exit(1)
