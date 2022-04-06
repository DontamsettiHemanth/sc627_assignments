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


def angle_loop(th):
    while th < -np.pi:
        th += 2*np.pi
    while th > np.pi:
        th -= 2*np.pi
    return th


class velocity_obstacles:

    def __init__(self):
        self.t = 0
        self.goal = np.array([5, 0])
        self.obs_xv = None
        self.bot_xv = None
        self.obs_sub = rospy.Subscriber('/obs_data', ObsData, self.callback_obs)
        self.bot_sub = rospy.Subscriber('/bot_1/odom', Odometry, self.callback_odom)

        self.pub_vel = rospy.Publisher('/bot_1/cmd_vel', Twist, queue_size=10)

    def velocity_convert2(self, x, y, theta, vel_x, vel_y):
        '''
        Robot pose (x, y, theta)  Note - theta in (0, 2pi)
        Velocity vector (vel_x, vel_y)
        '''

        # if theta < 0:
        #     theta += 2 * math.pi

        gain_ang = 1  # modify if necessary

        ang = np.arctan2(vel_y, vel_x)
        # if ang < 0:
        #     ang += 2 * math.pi

        ang_err = min(max(ang - theta, -ANG_MAX), ANG_MAX)

        v_lin = min(max(math.cos(ang_err) * math.sqrt(vel_x ** 2 + vel_y ** 2), -VEL_MAX), VEL_MAX)
        v_ang = gain_ang * ang_err
        return v_lin, v_ang

    def velocity_convert(self, v, th_rel):
        '''
        Velocity vector (||vel||, (th_new - th_old) )
        '''

        gain_ang = 1  # modify if necessary

        ang_err = min(max(th_rel, -ANG_MAX), ANG_MAX)
        v_lin = min(max(math.cos(ang_err)*v, -VEL_MAX), VEL_MAX)
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
            x = np.array([i.pose_x, i.pose_y])
            v = np.array([i.vel_x, i.vel_y])
            obs.append({'x': x, 'v': v})
        self.obs_xv = obs  # If all obstacles data are sent
        pass

    def callback_odom(self, data):
        '''
        Get robot data
        '''
        theta = 2*np.arctan2(data.pose.pose.orientation.z, data.pose.pose.orientation.w)
        x = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
        v = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y])
        self.bot_xv = {'x': x, 'th': theta, 'v': v, 'w': data.twist.twist.angular.z}
        self.t = data.header.stamp.secs + data.header.stamp.nsecs*1e-9
        pass

    def max_v(self, v1, v2):
        PG = -self.bot_xv['x'] + self.goal
        th_g = np.arctan2(PG[1], PG[0])
        alpha = 45*np.pi/180
        th_1 = angle_loop(v1[1] + self.bot_xv['th'] - th_g)
        th_2 = angle_loop(v2[1] + self.bot_xv['th'] - th_g)
        if (-alpha <= th_1) and (th_1 <= alpha):
            if (-alpha <= th_2) and (th_2 <= alpha):

                if v2[0] >= v1[0]+0.01:
                    return v2
                elif v1[0] >= v2[0]+0.01:
                    return v1
                else:
                    if abs(th_1) <= abs(th_2):
                        return v1
                    else:
                        return v2
            else:
                return v1
        elif (-alpha <= th_2) and (th_2 <= alpha):
            return v2
        else:
            print "-"
            return np.array([0, 0])


R = 2*0.15
r = rospy.Rate(30)
v_search = [i*0.01 for i in range(int(100*VEL_MAX), 0, -1)]
v_search.append(0)
th_search = [i*ANG_MAX/10 for i in range(-10, 11)]
VO = velocity_obstacles()
path = []
while (VO.obs_xv is None) or (VO.bot_xv is None):
    pass
vel_msg = Twist()
VO.pub_vel.publish(vel_msg)

iter = 0
Goal_rel = VO.goal - VO.bot_xv['x']
while (~rospy.is_shutdown()) or (sum(Goal_rel*Goal_rel) > 0.1**2):
    once_free = False  # true when a v and th are found without collision with all obs
    free_point = np.array([0, 0])
    # rospy.loginfo("Completed %r iterations", iter)
    for v in v_search:
        for th in th_search:
            free_obs = True  # velocity vector avoids all obstacles?
            bot_th = th + VO.bot_xv['th']
            bot_v = np.array([v*np.cos(bot_th), v*np.sin(bot_th)])
            for obs in VO.obs_xv:
                # collision Checking
                v_rel = bot_v - obs['v']
                PO = -VO.bot_xv['x'] + obs['x']
                th_cri = np.arcsin(R/np.sqrt(sum(PO*PO)))
                alpha = np.arccos(sum(v_rel*PO)/np.sqrt(sum(v_rel*v_rel)*sum(PO*PO)))

                if alpha < th_cri:
                    # possibility to collide this obstacle
                    t_d = 15
                    # check if |v|^2>|PO|^2/t_d^2 for collision
                    if sum(v_rel*v_rel)*(t_d**2) >= np.sum(PO*PO):
                        free_obs = False
                        break
            if free_obs:
                free_point = VO.max_v(free_point, np.array([v, th]))
                if sum(free_point == np.array([0, 0])) != 2:
                    once_free = True
                    if iter % 100 == 0:
                        print free_point
            pass
        if once_free:
            break

    # v_lin, v_ang = VO.velocity_convert(VO.bot_xv['x'][0], VO.bot_xv['x'][1], VO.bot_xv['th'], free_point[0]*np.cos(free_point[1]), free_point[0]*np.sin(free_point[1]))
    v_lin, v_ang = VO.velocity_convert(free_point[0], free_point[1])
    # publish the velocities below
    vel_msg = Twist()
    vel_msg.linear.x = v_lin
    vel_msg.angular.z = v_ang
    VO.pub_vel.publish(vel_msg)

    # storing robot path with time stamps (data available in odom topic)
    path.append([VO.t, VO.bot_xv['x'][0], VO.bot_xv['x'][1]])
    iter += 1
    if iter % 100 == 0:
        rospy.loginfo("Completed %r iterations", iter)
    r.sleep()

vel_msg = Twist()
VO.pub_vel.publish(vel_msg)
rospy.loginfo("Success : Reached the goal")
path.append([VO.t, VO.bot_xv['x'][0], VO.bot_xv['x'][1]])

with open("/root/catkin_ws/src/sc627_assignments/assignment_3/output.txt", "w") as file:
    file.write(str(path[0])[1:-1])
    for p in path[1:]:
        file.write("\n")
        file.write(str(p)[1:-1])
        pass

exit(1)
