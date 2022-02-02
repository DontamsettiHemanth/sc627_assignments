
from helper import*
path = [point(0,0),point(1,1),point(2,2),point(3,3)]
with open("/root/catkin_ws/src/sc627_assignments/src/assignment_1/output_base_format.txt",'w') as file:
    file.write(str(path[0]))
    for p in path[1:]:
        file.write("\n")
        file.write(str(p))
        pass