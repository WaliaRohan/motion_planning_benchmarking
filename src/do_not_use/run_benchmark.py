from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped 
import rclpy

import sys
import os
import time

import pickle
import csv

def main():
    '''Get argument values, convert, call quad.'''
    
    if len(sys.argv) < 1:
        raise Exception("Incorrect usage. Input format: run_benchmark.py exact_path_to_save_location prefix init_x init_y goal_x goal_y")
    save_location = os.path.join(sys.argv[1], sys.argv[2])
    init_x = float(sys.argv[3])
    init_y = float(sys.argv[4])
    goal_x = float(sys.argv[5])
    goal_y = float(sys.argv[6])

    rclpy.init()
    nav = BasicNavigator()

    # ...

    init_pose = PoseStamped()
    init_pose.header.frame_id = 'map'
    # init_pose.header.stamp = nav.get_clock().now().to_msg()
    init_pose.pose.position.x = init_x
    init_pose.pose.position.y = init_y
    init_pose.pose.position.z = 0.00
    init_pose.pose.orientation.x = 0.0
    init_pose.pose.orientation.y = 0.0
    init_pose.pose.orientation.z = 0.0
    init_pose.pose.orientation.w = 1.0

    nav.setInitialPose(init_pose)
    nav.waitUntilNav2Active() # if autostarted, else use lifecycleStartup()

    # ...

    # Bottom: (-2, -0.5, 0.01)
    # Bottom right: (-1.39, -1.68, 0.1)
    # Top: (1.82, -0.58, 0.01)

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    # goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    goal_pose.pose.position.z = 0.00
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    start_time = time.time()
    path = nav.getPath(init_pose, goal_pose)
    end_time = time.time()
    compute_time = end_time - start_time
    
    print("Elapsed time: ", compute_time) 

    fields = ['Time', 'x', 'y', 'z', 'q.x', 'q.y', 'q.z', 'q.w']

    rows = len(path.poses)+2
    cols = len(fields)

    path_arr = [[0]*cols]*rows

    # Add start pose to path_arr
    path_arr[0] = [(init_pose.header.stamp.sec) +
                      (init_pose.header.stamp.nanosec)/1000,
        init_pose.pose.position.x,
        init_pose.pose.position.y,
        init_pose.pose.position.z,
        init_pose.pose.orientation.x,
        init_pose.pose.orientation.y,
        init_pose.pose.orientation.z,
        init_pose.pose.orientation.w]

    # Add goal pose to path_arr
    path_arr[len(path.poses)+1] = [(goal_pose.header.stamp.sec) +
                                    (goal_pose.header.stamp.nanosec)/1000,
        goal_pose.pose.position.x,
        goal_pose.pose.position.y,
        goal_pose.pose.position.z,
        goal_pose.pose.orientation.x,
        goal_pose.pose.orientation.y,
        goal_pose.pose.orientation.z,
        goal_pose.pose.orientation.w]

    for i in range(len(path.poses)):
        pose = path.poses[i]
        header_time = (pose.header.stamp.sec) + (pose.header.stamp.nanosec)/1000
        position = pose.pose.position
        orientation = pose.pose.orientation
        path_arr[i+1] = [header_time,
        position.x,
        position.y,
        position.z,
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w]

    # Save data in this run

    #save_location = '/home/speedracer1702/Projects/ros2_ws/src/nav2_benchmark/data/test_run/data.csv'

    os.makedirs(save_location)

    pickle_path_file = os.path.join(save_location, sys.argv[2] + "_raw_path.bin")
    pickle_time_file = os.path.join(save_location, sys.argv[2] + "_raw_time.bin")

    # Save raw data
    with open(pickle_path_file, 'wb') as f:
        pickle.dump(path, f) 

    with open(pickle_time_file, 'wb') as f:
        pickle.dump(compute_time, f)
    
    csv_file = os.path.join(save_location, sys.argv[2] + "_path_data.csv")

    # Save csv file
    with open(csv_file, 'w') as csvfile: 
        # creating a csv writer object 
        csvwriter = csv.writer(csvfile) 
            
        # writing the fields 
        csvwriter.writerow(fields) 
            
        # writing the data rows 
        csvwriter.writerows(path_arr)

if __name__ == "__main__":
    main()