#!/usr/bin/env python

from helper import*
"""
from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib

rospy.init_node('bug1', anonymous=True)

# Initialize client
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()
"""
# reading input file
in_file = "/root/catkin_ws/src/sc627_assignments/src/assignment_1/input_format.txt"
# in_file = "./input_format.txt"
with open(in_file, 'r') as f:
    change = float
    start = [change(i) for i in f.readline().split(", ")]
    print start
    goal = [change(i) for i in f.readline().split(", ")]
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
            x, y = [change(point) for point in lines[i].split(", ")]
            print point(x, y)
            P.append(point(x, y))
            
        i += 1
    obstacles.append(P)

    def GoalDist(pos, goal=goal):
        return (goal - pos).norm()

    def TowardsGoal(pos, goal=goal):
        dir = goal - pos
        return dir.multi(stepsize/dir.norm()) + pos

    def min_obs_Clearance(pos, obstacles=obstacles):
        mindist = computeDistancePointToPolygon(obstacles[0], pos)
        minp = obstacles[0]
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
    tolerance = stepsize/2
    current_pos = start
    path = [start]
    f = '/root/catkin_ws/src/sc627_assignments/src/assignment_1/output_1.txt'
    #f = "./output_1.txt"
    while GoalDist(current_pos) > stepsize:
        next_pos = TowardsGoal(current_pos)
        if len(obstacles) > 0:
            hit, mindist, minp = min_obs_Clearance(next_pos)
            if hit:
                # hit an obstacle: circumcircle the obstacle with storing min dist to the goal while doing so
                obs_s = current_pos
                min_goal_pos = obs_s
                # first get away from obstacle starting point
                dir = computeTangentVectorToPolygon(minp,current_pos)
                current_pos = dir.multi(stepsize/dir.norm()) + current_pos
                # Didn't Check collision while circumventing obstacle
                path.append(current_pos)
                if GoalDist(obs_s) > GoalDist(current_pos):
                    min_goal_pos = current_pos
                # circumvent till obstacle is within tolerance
                while (current_pos - obs_s).norm() > tolerance:
                    dir = computeTangentVectorToPolygon(minp,current_pos)
                    current_pos = dir.multi(stepsize/dir.norm()) + current_pos
                    # Didn't Check collision while circumventing obstacle
                    path.append(current_pos)
                    if GoalDist(obs_s) > GoalDist(current_pos):
                        min_goal_pos = current_pos  
                # circumvent the obstacle untill minDist point is found
                while (current_pos - min_goal_pos).norm() > tolerance:
                    dir = computeTangentVectorToPolygon(minp,current_pos)
                    current_pos = dir.multi(stepsize/dir.norm()) + current_pos
                    # Didn't Check collision while circumventing obstacle
                    path.append(current_pos)
                next_pos = TowardsGoal(current_pos)
                hit, _, _ = min_obs_Clearance(next_pos)
                if hit:
                    gen_Output(f, path)
                    return "Failure :", path

            else:
                current_pos = next_pos
                path.append(current_pos)

        else:
            current_pos = next_pos
            path.append(current_pos)
    path.append(goal)
    gen_Output(f,path)
    return "Success",path

a, b = bug1()
print a
"""
#setting result as initial location
result = MoveXYResult()
result.pose_final.x = 0
result.pose_final.y = 0
result.pose_final.theta = 0 #in radians (0 to 2pi)

while True: #replace true with termination condition

    #determine waypoint based on your algo
    #this is a dummy waypoint (replace the part below)
    wp = MoveXYGoal()
    wp.pose_dest.x = 1 + result.pose_final.x
    wp.pose_dest.y = 1
    wp.pose_dest.theta = 3.14/4 #theta is the orientation of robot in radians (0 to 2pi)

    #send waypoint to turtlebot3 via move_xy server
    client.send_goal(wp)

    client.wait_for_result()

    #getting updated robot location
    result = client.get_result()

    #write to output file (replacing the part below)
    #print result.pose_final.x, result.pose_final.y, result.pose_final.theta
"""
