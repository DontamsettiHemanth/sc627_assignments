#!/usr/bin/env python

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
from helper import*

rospy.init_node('test', anonymous= True)

#Initialize client
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()

#reading input file
with open("/root/catkin_ws/src/sc627_assignments/src/assignment_1/input_format.txt",'r') as f:
    change =float
    start = [change(i) for i in f.readline().split(", ")]
    goal = [change(i) for i in f.readline().split(", ")]
    stepsize = change(f.readline())
    n=f.readline()
    lines =f.readlines()
    i=0
    P=[]
    obstacles = [] #list of polygons
    while i< len(lines):
        if i == n:
            obstacles.append(P)
            P = []
        x,y = [change(i) for i in f.readline().split(", ")]
        P.append(point(x,y))
def gen_Output(f,path): #check the correctness
    with open(f,'w') as file:
        file.write(str(path[0]))
        for p in path[1:]:
            file.write("\n")
            file.write(str(p))
            pass
    

        

def bugbase(start,goal,obstacles,stepsize):
    tolerance = stepsize/2
    current_position = start
    path=[start]
    dir = goal - current_position
    f = '~/catkin_ws/src/sc627_assignments/src/assignment_1/output_base.txt'
    while point.dist(current_position, goal) > stepsize :
        candidate_current_position= dir.multi(stepsize/dir.norm()) + current_position
        
        dists=[]
        for P in obstacles: #compute distance from candidate-current-position to each obstacle
            dists.append(computeDistancePointToPolygon(P,candidate_current_position))
        if min(dists) < tolerance :
            gen_Output(f,path)
            return "Failure: There is an obstacle lying between the start and goal", path

        current_position = candidate_current_position
        path.append(current_position)
    path.append(goal)
    gen_Output(f,path)
    return "Success",path

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
    wp.pose_dest.y = 0
    wp.pose_dest.theta = 0 #theta is the orientation of robot in radians (0 to 2pi)

    #send waypoint to turtlebot3 via move_xy server
    client.send_goal(wp)

    client.wait_for_result()

    #getting updated robot location
    result = client.get_result()

    #write to output file (replacing the part below)
    print(result.pose_final.x, result.pose_final.y, result.pose_final.theta)