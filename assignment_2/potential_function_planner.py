#!/usr/bin/env python

from helper import*
import numpy as np
from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib

# reading input file
inf = "/root/catkin_ws/src/sc627_assignments/assignment_2/input.txt"
with open(inf, 'r') as f:
    change = float
    start = [change(i) for i in f.readline().split(",")]
    start = point(start[0], start[1])
    goal = [change(i) for i in f.readline().split(",")]
    goal = point(goal[0], goal[1])
    stepsize = change(f.readline())
    n = f.readline()
    lines = f.readlines()
    i = 0
    P = []
    obstacles = []  # list of polygons
    while i < len(lines):
        if lines[i] == n:
            obstacles.append(P)
            P = []
        else:
            x, y = [change(xypoint) for xypoint in lines[i].split(",")]
            temp = point(x, y)
            P.append(temp)

        i += 1
    obstacles.append(P)
    tolerance = stepsize*0.5


rospy.init_node('potential_field_mover', anonymous = True)

# Initialize client
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()
# setting result as initial location
result = MoveXYResult()
result.pose_final.x = start.x
result.pose_final.y = start.y
result.pose_final.theta = 0  # in radians (0 to 2pi)


def pub_goal(next, dir, f, path):

    wp = MoveXYGoal()
    wp.pose_dest.x = next.x
    wp.pose_dest.y = next.y
    temp = np.arctan2(dir.y, dir.x)
    if temp < 0:
        temp += 2*np.pi
    wp.pose_dest.theta = temp  # theta is the orientation of robot in radians (0 to 2pi)

    # send waypoint to turtlebot3 via move_xy server
    client.send_goal(wp)
    timeout = 15.0
    done = client.wait_for_result(rospy.Duration(secs=timeout))
    if not done:
        rospy.loginfo("Couldn't complete goal %r! Sending a Goal along the dir vector %r", str(next), str(dir))
        dir = dir.multi(1/dir.norm())
        wp.pose_dest.x -= dir.x*stepsize*0.5
        wp.pose_dest.y -= dir.y*stepsize*0.5
        client.send_goal(wp)  # change angle also if didn't work
        done = client.wait_for_result(rospy.Duration(secs=timeout))
        if not done:
            rospy.logerr("Couldn't go to point %r in %r seconds", str(next), timeout)
            gen_Output(f, path)
            exit(1)
    # getting updated robot location
    result = client.get_result()
    return point(result.pose_final.x, result.pose_final.y), result.pose_final.theta


def GoalDist(pos, goal=goal):
    return (goal - pos).norm()


def F_attract(q, goal=goal):
    d = GoalDist(q)
    zeta = 0.8
    if d <= 2:  # d*_goal = 2
        return (goal - q).multi(zeta)
    else:
        return (goal - q).multi(2*zeta/d)


def F_repulse(q, obstacles=obstacles):
    F_r = point(0,0)
    for i in obstacles:
        d, p_near = computeDistance_P_PointToPolygon(i, q)
        if (0 < d) and (d <= 2):  # Q*_i = 2
            coeff = 0.8*(1.0/d - 1.0/2)/(d**3)  # eta = 0.8
            F_r = F_r + (q - p_near).multi(coeff)
        elif d == 0:  # inside obstacle
            # Not to be executed/reach here 
            rospy.logerr("F_repulse: inside of a virtual obstacle!")
            exit(1)
        # else ==> far from obstacle
    return F_r


def potential_mover(start=start, goal=goal, obstacles=obstacles, stepsize=stepsize):
    current_pos = start
    path = [start]
    f = '/root/catkin_ws/src/sc627_assignments/assignment_2/output.txt'
    max_steps = 200
    j = 1
    while (GoalDist(current_pos) > 2*stepsize) and (j < max_steps):
        dir = F_attract(current_pos) + F_repulse(current_pos)
        next_pos = dir.multi(stepsize/dir.norm()) + current_pos
        current_pos, _ = pub_goal(next_pos, dir, f, path)
        path.append(current_pos)
        j += 1
        if j%100 ==0:
            rospy.loginfo("Took %r steps from start. Not THE END.",j)
    if j < max_steps:
        rospy.loginfo("Almost reached the goal")
        current_pos, _ = pub_goal(goal, (goal - current_pos), f, path)
        path.append(current_pos)
        current_pos, _ = pub_goal(goal, point(0, 0), f, path)
        path.append(current_pos)
        gen_Output(f, path)
        return 'Success', path
    else:
        gen_Output(f, path)
        return "Failure : took more than %r steps".format(max_steps), path


start_time = rospy.get_time()
Final_Status, _ = potential_mover()
end_time = rospy.get_time()
rospy.loginfo("%r, Took %r mins %r secs", Final_Status , (end_time - start_time)//60, (end_time - start_time)%60)
