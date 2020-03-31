#!/usr/bin/python2

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
import numpy as np
from std_srvs.srv import Empty
import math
import pickle 

right = 0
right_front = 0
front = 0
left = 0
behind = 0

pub = ""

def scan_callback(msg):
    global right, right_front, front, left, behind
    right = (msg.ranges[0] + msg.ranges[315] + msg.ranges[45]) / 3
    right_front = (msg.ranges[45] + msg.ranges[0]) / 2
    front = (msg.ranges[90] + msg.ranges[45] + msg.ranges[135]) / 3
    left = (msg.ranges[180] + msg.ranges[135] + msg.ranges[225]) / 3
    behind = msg.ranges[270]


def close_far_l(val):
    if val <= 0.5: return 0 # close
    else: return 1 # far

def close_far_rf(val):
    if val <= 1.2: return 0 # close
    else: return 1 # far
    
def close_far_ext(val):
    # return 0 for too close, 1 for med, 2 for far, 3 for too far
    if val <= 0.5: return 0 # too close
    elif val <= 0.6: return 1 # close
    elif val <= 0.8: return 2 # medium
    elif val <= 1.2: return 3 # far
    return 4                  # too far 

def orientation():
    nearest = min(front, left, right, behind)
    if nearest == front: return 0     # approaching
    if nearest == behind: return 1    # moving away
    if nearest == right: return 2     # parallel
    return 3

def get_discrete_state():
    state = rospy.wait_for_message('/scan', LaserScan)
    scan_callback(state)

    toReturn = []
    toReturn.append(close_far_ext(right))
    toReturn.append(close_far_rf(right_front))
    toReturn.append(close_far_ext(front)) 
    toReturn.append(close_far_l(left))
    toReturn.append(orientation())
    print(toReturn)
    return toReturn

def step(action):
    print(action)
    # declare a publisher to move the robot
    new_pose = Pose2D()
    if action == 0: 
        new_pose.x = 0.3
    elif action == 1: # left
        new_pose.theta = math.pi / 4
    else: # right
        new_pose.theta = - math.pi / 4
    pub.publish(new_pose)

def get_max_action(q_table, state):
    max_rew = 0
    max_action = 0

    rew_0 = q_table[state[0]][state[1]][state[2]][state[3]][state[4]][0]
    rew_1 = q_table[state[0]][state[1]][state[2]][state[3]][state[4]][1]
    rew_2 = q_table[state[0]][state[1]][state[2]][state[3]][state[4]][2]

    max_rew = max(rew_0, rew_1, rew_2)
    if (rew_0 == rew_1 and rew_1 == rew_2): max_action = np.random.randint(0,3)
    if max_rew == rew_0: max_action = 0
    elif max_rew == rew_1: max_action = 1
    else: max_action = 2

    return (max_rew, max_action)

def main():
    
    try:
        # initialize publisher and subscribers
        global pub 
        pub = rospy.Publisher('/triton_lidar/vel_cmd', Pose2D, queue_size=10)
        rospy.init_node('triton_wall_follow', anonymous=True)

        q_file = open('q_table_pickle', 'rb')
        q_table = pickle.load(q_file)
        print(q_file)
        
        while not rospy.is_shutdown():  
       
            current_state = get_discrete_state()
            action = get_max_action(q_table, current_state)
            step(action)

        
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated" )
   
if __name__ == '__main__':
    main()
