#!/usr/bin/env python

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
from helper import*
import numpy as np


#reading input file
with open("/root/catkin_ws/src/sc627_assignments/src/assignment_1/input_format.txt", 'r') as f:
    change = float
    start = [change(i) for i in f.readline().split(", ")]
    goal = [change(i) for i in f.readline().split(", ")]
    stepsize = change(f.readline())
    n = f.readline()
    lines = f.readlines()
    i = 0
    P = []
    obstacles = [] #list of polygons
    while i < len(lines):
        if i == n:
            obstacles.append(P)
            P = []
        else:
            x, y = [change(xypoint) for xypoint in lines[i].split(", ")]
            temp = point(x, y)
            P.append(temp)

        i += 1
    obstacles.append(P)
    tolerance = stepsize*0.5


rospy.init_node('bugbase', anonymous = True)

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


def bugbase(start=start, goal=goal, obstacles=obstacles, stepsize=stepsize):
    current_position = start
    path = [start]
    dir = goal - current_position
    f = '/root/catkin_ws/src/sc627_assignments/src/assignment_1/output_base.txt'
    while point.dist(current_position, goal) > stepsize:
        candidate_current_position = dir.multi(stepsize/dir.norm()) + current_position
        dists = []
        for P in obstacles:  # compute distance from candidate-current-position to each obstacle
            dists.append(computeDistancePointToPolygon(P, candidate_current_position))
        if min(dists) < tolerance :
            gen_Output(f, path)
            return "Failure: There is an obstacle lying between the start and goal", path
        current_position,_ = pub_goal(candidate_current_position, dir)
        #current_position = candidate_current_position
        path.append(current_position)
    current_position,_ = pub_goal(goal, point(0, 0))  # Last step to goal apparently
    path.append(current_position)
    gen_Output(f, path)
    return "Success!!!", path

Final_Status, _ = bugbase()
rospy.loginfo(Final_Status)