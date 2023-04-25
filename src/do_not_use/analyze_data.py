import pickle
import math

'''Read and analyze path data stored as pickle dump files'''

### Recover data from experiment
path_file = '/home/speedracer1702/Projects/ros2_ws/src/nav2_benchmark/data/test_run/data.txt'
start_fin_file = '/home/speedracer1702/Projects/ros2_ws/src/nav2_benchmark/data/test_run/start_fin.txt'

# https://java2blog.com/write-variable-to-file-python/#Using_the_pickledump_function
with open(path_file, 'rb') as f:
    path = pickle.load(f) 

with open(start_fin_file, 'rb') as f:
    start_fin = pickle.load(f)

### Save this data in csv with given filename

path_length = 0

for i in range(len(path.poses) - 1):
    current_pose = path.poses[i]
    next_pose = path.poses[i+1]
    
    p = [current_pose.pose.position.x, current_pose.pose.position.y]
    q = [next_pose.pose.position.x, next_pose.pose.position.y]

    path_length = path_length + math.dist(p, q)

print(path_length)
