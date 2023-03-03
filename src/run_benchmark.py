from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped 
import rclpy

def main():
    rclpy.init()
    nav = BasicNavigator()

    # ...


    init_pose = PoseStamped()
    init_pose.header.frame_id = 'map'
    init_pose.header.stamp = nav.get_clock().now().to_msg()
    init_pose.pose.position.x = -1.310
    init_pose.pose.position.y = -0.610
    init_pose.pose.position.z = 0.024
    init_pose.pose.orientation.x = 0.0
    init_pose.pose.orientation.y = 0.0
    init_pose.pose.orientation.z = 0.0
    init_pose.pose.orientation.w = 1.0

    nav.setInitialPose(init_pose)
    nav.waitUntilNav2Active() # if autostarted, else use lifecycleStartup()

    # ...

    # Bottom right: (-1.39, -1.68, 0.1)
    # Top: (1.82, -0.58, 0.01)

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = 2.10
    goal_pose.pose.position.y =  1.07
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    path = nav.getPath(init_pose, goal_pose)
    smoothed_path = nav.smoothPath(path)

    # ...

    # nav.goToPose(goal_pose)
    # while not nav.isTaskComplete():
    #   feedback = nav.getFeedback()
    #   if feedback.navigation_duration > 600:
    #     nav.cancelTask()

    # # ...

    # result = nav.getResult()
    # if result == TaskResult.SUCCEEDED:
    #     print('Goal succeeded!')
    # elif result == TaskResult.CANCELED:
    #     print('Goal was canceled!')
    # elif result == TaskResult.FAILED:
    #     print('Goal failed!')
