#!/usr/bin/env python

from helper import*
import numpy as np
from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib

# reading input file
inf = "/root/catkin_ws/src/sc627_assignments/src/assignment_1/input_format.txt"
# in_file = "./input_format.txt"
with open(inf, 'r') as f:
    change = float
    start = [change(i) for i in f.readline().split(", ")]
    start = point(start[0], start[1])
    goal = [change(i) for i in f.readline().split(", ")]
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
            x, y = [change(xypoint) for xypoint in lines[i].split(", ")]
            temp = point(x, y)
            P.append(temp)

        i += 1
    obstacles.append(P)
    tolerance = stepsize*0.5


rospy.init_node('bug_1', anonymous = True)

# Initialize client
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()
# setting result as initial location
result = MoveXYResult()
result.pose_final.x = start.x
result.pose_final.y = start.y
result.pose_final.theta = 0  # in radians (0 to 2pi)


def pub_goal(next, dir):  # replace true with termination condition

    # determine waypoint based on your algo
    # this is a dummy waypoint (replace the part below)
    wp = MoveXYGoal()
    wp.pose_dest.x = next.x
    wp.pose_dest.y = next.y
    wp.pose_dest.theta = np.atan2(dir.y,dir.x) + np.pi  # theta is the orientation of robot in radians (0 to 2pi)

    # send waypoint to turtlebot3 via move_xy server
    client.send_goal(wp)

    client.wait_for_result()

    # getting updated robot location
    result = client.get_result()

    # write to output file (replacing the part below)
    return point(result.pose_final.x, result.pose_final.y), result.pose_final.theta


def GoalDist(pos, goal=goal):
    return (goal - pos).norm()


def TowardsGoal(pos, goal=goal):
    dir = goal - pos
    return dir.multi(stepsize/dir.norm()) + pos


def min_obs_Clearance(pos, obstacles=obstacles):
    mindist = computeDistancePointToPolygon(obstacles[0], pos)
    minp = obstacles[0]
    hit = False
    for P in obstacles[1:]:

        # compute distance from candidate-current-pos to each obstacle
        dist = computeDistancePointToPolygon(P, pos)
        if dist < mindist:
            mindist = dist
            minp = P
    if mindist < tolerance:
        hit = True
    return hit, mindist, minp


def bug1(start=start, goal=goal, obstacles=obstacles, stepsize=stepsize):
    current_pos = start
    path = [start]
    f = '/root/catkin_ws/src/sc627_assignments/src/assignment_1/output_1.txt'
    # f = "./output_1.txt"
    while GoalDist(current_pos) > stepsize:
        next_pos = TowardsGoal(current_pos)
        if len(obstacles) > 0:
            hit, mindist, minp = min_obs_Clearance(next_pos)
            if hit:
                if mindist > 0:
                    dir = (next_pos - current_pos).multi(0.5)
                    next_pos = current_pos + dir
                    current_pos,_ = pub_goal(next_pos, dir)
                    path.append(current_pos)
                rospy.loginfo("Hit: ", hit, mindist, minp[0])
                # circumcircle the obstacle, storing min dist to the goal
                obs_s = current_pos
                min_goal_pos = obs_s
                # first get away from obstacle starting point
                dir, outside = computeTangentVectorToPolygon(minp, current_pos, tolerance)
                next_pos = dir.multi(stepsize) + current_pos
                # Didn't Check collision while circumventing obstacle
                current_pos,_ = pub_goal(next_pos, dir)
                path.append(current_pos)
                if GoalDist(min_goal_pos) > GoalDist(current_pos):
                    min_goal_pos = current_pos
                # circumvent till obstacle is within tolerance
                j = 0
                while (current_pos - obs_s).norm() > tolerance:
                    j += 1
                    if j % 5000 == 0:
                        gen_Output(f, path)
                        return 'circumventing Failure', path
                    dir, outside = computeTangentVectorToPolygon(minp, current_pos,tolerance)
                    if not outside:
                        rospy.loginfo("Inside virtual obstacle: ", j)
                    next_pos = dir.multi(stepsize) + current_pos
                    # Didn't Check collision while circumventing obstacle
                    current_pos,_ = pub_goal(next_pos, dir)
                    path.append(current_pos)
                    if GoalDist(min_goal_pos) > GoalDist(current_pos):
                        min_goal_pos = current_pos
                # circumvent the obstacle untill minDist point is found
                j = 0
                while (current_pos - min_goal_pos).norm() > tolerance:
                    j += 1
                    if j % 5000 == 0:
                        gen_Output(f, path)
                        return 'Circumventing towards min_pos failure', path
                    dir, outside = computeTangentVectorToPolygon(minp, current_pos,tolerance)
                    if not outside:
                        rospy.loginfo("Inside virtual obstacle: ", j)
                    next_pos = dir.multi(stepsize) + current_pos
                    # Didn't Check collision while circumventing obstacle
                    current_pos,_ = pub_goal(next_pos, dir)
                    path.append(current_pos)
                next_pos = TowardsGoal(current_pos)
                hit, _, _ = min_obs_Clearance(next_pos)
                if hit:
                    gen_Output(f, path)
                    return "Failure : No path exits apparantly", path

            else:
                current_pos,_ = pub_goal(next_pos, next_pos - current_pos)
                path.append(current_pos)

        else:
            current_pos,_ = pub_goal(next_pos, next_pos - current_pos)
            path.append(current_pos)
    current_pos,_ = pub_goal(goal, point(0, 0))  # Last step to goal apparently
    path.append(current_pos)
    gen_Output(f, path)
    return "Success", path


Final_Status, _ = bug1()
rospy.loginfo(Final_Status)
