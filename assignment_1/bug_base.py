#!/usr/bin/env python

from turtle import distance
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
print(start, goal, step_size, obstacles)
#setting result as initial location
result = MoveXYResult()
result.pose_final.x = start[0]
result.pose_final.y = start[1]
result.pose_final.theta = 0  # in radians (0 to 2pi)
current_pose = np.array([result.pose_final.x, result.pose_final.y])
path = [current_pose.copy()]

while distance(goal, [result.pose_final.x, result.pose_final.y]) > step_size: #replace true with termination condition

    #determine waypoint based on your algo
    #this is a dummy waypoint (replace the part below)
    dist_closest_polygon = min(computeDistancePointToPolygon(obstacles[0], [result.pose_final.x, result.pose_final.y]), computeDistancePointToPolygon(obstacles[1], [result.pose_final.x, result.pose_final.y]))
    if computeDistancePointToPolygon(obstacles[0], [result.pose_final.x, result.pose_final.y]) < computeDistancePointToPolygon(obstacles[1], [result.pose_final.x, result.pose_final.y]):
        closest_polygon = obstacles[0]
    else:
        closest_polygon = obstacles[1]
    if dist_closest_polygon < step_size:
        print("Failure : There's an obstacle between current position and goal")
    wp = MoveXYGoal()
    wp.pose_dest.x = result.pose_final.x + np.array(computeTangentVectorToPolygon(closest_polygon, current_pose))[0]*step_size
    wp.pose_dest.y = result.pose_final.y + np.array(computeTangentVectorToPolygon(closest_polygon, current_pose))[1]*step_size
    wp.pose_dest.theta = math.atan2(np.array(computeTangentVectorToPolygon(closest_polygon, current_pose))[1], np.array(computeTangentVectorToPolygon(closest_polygon, current_pose))[0]) #theta is the orientation of robot in radians (0 to 2pi)

    #send waypoint to turtlebot3 via move_xy server
    client.send_goal(wp)

    client.wait_for_result()

    #getting updated robot location
    result = client.get_result()
    current_pose = np.array([result.pose_final.x, result.pose_final.y])
    path.append(current_pose)

    #write to output file (replacing the part below)
    print(result.pose_final.x, result.pose_final.y, result.pose_final.theta)