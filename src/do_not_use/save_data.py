import pickle # https://java2blog.com/write-variable-to-file-python/#Using_the_pickledump_function
import csv
import sys
import os

'''Read and analyze path data stored as pickle dump files'''

def main():
    '''Get argument values, convert, call quad.'''
    
    if len(sys.argv) < 1:
        raise Exception("No save location provided. Input format: save_data.py exact_path_to_save_location prefix")
    save_location = os.path.join(sys.argv[1], sys.argv[2])

    ### Recover data from experiment - not required for actual experiment
    path_file = '/home/speedracer1702/Projects/ros2_ws/src/nav2_benchmark/data/test_run/data.txt'
    start_fin_file = '/home/speedracer1702/Projects/ros2_ws/src/nav2_benchmark/data/test_run/start_fin.txt'

    with open(path_file, 'rb') as f:
        path = pickle.load(f) 

    with open(start_fin_file, 'rb') as f:
        start_fin = pickle.load(f)

    # dummy compute time
    compute_time = 1000000


    fields = ['Time', 'x', 'y', 'z', 'q.x', 'q.y', 'q.z', 'q.w']

    rows = len(path.poses)+2
    cols = len(fields)

    path_arr = [[0]*cols]*rows

    # Add start pose to path_arr
    start_pose = start_fin.poses[0]
    path_arr[0] = [(start_pose.header.stamp.sec) +
                      (start_pose.header.stamp.nanosec)/1000,
        start_pose.pose.position.x,
        start_pose.pose.position.y,
        start_pose.pose.position.z,
        start_pose.pose.orientation.x,
        start_pose.pose.orientation.y,
        start_pose.pose.orientation.z,
        start_pose.pose.orientation.w]

    # Add goal pose to path_arr
    goal_pose = start_fin.poses[1]
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
        time = (pose.header.stamp.sec) + (pose.header.stamp.nanosec)/1000
        position = pose.pose.position
        orientation = pose.pose.orientation
        path_arr[i+1] = [time,
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