#!/usr/bin/env python

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
from helper import *
#import other helper files if any


rospy.init_node('test', anonymous= True)

#Initialize client
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()

#read input file
inputs = open("input.txt", "r")
inputs = inputs.read().split("\n")
start = list(map(float, inputs[0].split(",")))
goal = list(map(float, inputs[1].split(",")))
step_size = float(inputs[2])
obstacles = []
for i in inputs[3:]:
    if i == "":
        obstacles.append([])
    else:
        obstacles[-1].append(list(map(float, i.split(","))))

#setting result as initial location
result = MoveXYResult()
result.pose_final.x = start[0]
result.pose_final.y = start[1]
result.pose_final.theta = 0  # in radians (0 to 2pi)
current_pose = np.array([result.pose_final.x, result.pose_final.y])
path = [current_pose.copy()]
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