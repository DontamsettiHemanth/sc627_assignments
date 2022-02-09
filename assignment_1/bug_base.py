#!/usr/bin/env python

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
from helper import*
import numpy as np


#reading input file
with open("/root/catkin_ws/src/sc627_assignments/assignment_1/input.txt", 'r') as f:
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


rospy.init_node('bugbase', anonymous = True)

# Initialize client
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()
# setting result as initial location
result = MoveXYResult()
result.pose_final.x = start.x
result.pose_final.y = start.y
result.pose_final.theta = 0  # in radians (0 to 2pi)

def pub_goal(next, dir):

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
        wp.pose_dest.x -= dir.x*tolerance*0.5
        wp.pose_dest.y -= dir.y*tolerance*0.5
        client.send_goal(wp)  # change angle also if didn't work
        if not (client.wait_for_result(rospy.Duration(secs=timeout))):
            rospy.logwarn("Couldn't go to %r in %r secs", str(next), timeout)
    # getting updated robot location
    result = client.get_result()
    return point(result.pose_final.x, result.pose_final.y), result.pose_final.theta


def bugbase(start=start, goal=goal, obstacles=obstacles, stepsize=stepsize):
    current_position = start
    path = [start]
    dir = goal - current_position
    f = '/root/catkin_ws/src/sc627_assignments/assignment_1/output_base.txt'
    while point.dist(current_position, goal) > stepsize:
        candidate_current_position = dir.multi(stepsize/dir.norm()) + current_position
        if len(obstacles) > 0:
            dists = []
            for P in obstacles:  # compute distance from candidate-current-position to each obstacle
                dists.append(computeDistancePointToPolygon(P, candidate_current_position))
            if min(dists) < tolerance:
                gen_Output(f, path)
                return "Failure: There is an obstacle lying between the start and goal", path
        current_position, _ = pub_goal(candidate_current_position, dir)
        path.append(current_position)
    current_position, _ = pub_goal(goal, point(0, 0))  # Last step to goal apparently
    path.append(current_position)
    gen_Output(f, path)
    return "Success!!!", path

start_time = rospy.get_time()
Final_Status,  _ = bugbase()
end_time = rospy.get_time()
rospy.loginfo("%r, Took %r mins %r secs", Final_Status , (end_time - start_time)//60, (end_time - start_time)%60)
