#!/usr/bin/python2

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
import numpy as np
from std_srvs.srv import Empty
import math
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

right = 0
right_front = 0
front = 0
left = 0
behind = 0

pub = ""

def scan_callback(msg):
    global right, right_front, front, left, behind
    print("calculating right")
    right = (msg.ranges[0] + msg.ranges[315] + msg.ranges[45]) / 3
    print("calculating right front")
    right_front = (msg.ranges[45] + msg.ranges[0]) / 2
    print("calculating front")
    front = (msg.ranges[90] + msg.ranges[45] + msg.ranges[135]) / 3
    print("calculating left")
    left = (msg.ranges[180] + msg.ranges[135] + msg.ranges[225]) / 3
    print("calculating behind")
    behind = msg.ranges[270]
 

def close_far_l(val):
    to_return = 0
    if val <= 0.5: to_return = 0 # close
    else: to_return = 1 # far
    return to_return

def close_far_rf(val):
    to_return = 0
    if val <= 1.2: to_return = 0 # close
    else: to_return = 1 # far
    return to_return
    
def close_far_ext(val):
    to_return = 0
    # return 0 for too close, 1 for med, 2 for far, 3 for too far
    if val <= 0.5: to_return = 0 # too close
    elif val <= 0.6: to_return = 1 # close
    elif val <= 0.8: to_return = 2 # medium
    elif val <= 1.2: to_return = 3 # far
    else: to_return = 4            # too far
    return to_return 

def orientation():
    to_return = 0
    nearest = min(front, left, right, behind)
    if nearest == front: to_return = 0     # approaching
    if nearest == behind: to_return = 1    # moving away
    if nearest == right: to_return = 2     # parallel
    else: to_return = 3
    return to_return

def get_discrete_state():
    
    
    try: 
        print("waiting for message")
        state = rospy.wait_for_message('/scan', LaserScan, timeout=1000)
        print("call callback function")
        scan_callback(state)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")

    toReturn = []
    print("appending right")
    toReturn.append(close_far_ext(right))
    print("appending right-front")
    toReturn.append(close_far_rf(right_front))
    print("appending front")
    toReturn.append(close_far_ext(front)) 
    print("appending left")
    toReturn.append(close_far_l(left))
    print("appending orientation")
    toReturn.append(orientation())
    
    return toReturn

def move(action):
    
    # declare a publisher to move the robot
    new_pose = Pose2D()
    if action == 0: 
        new_pose.x = 0.3
    elif action == 1: # left
        new_pose.theta = math.pi / 4
    else: # right
        new_pose.theta = - math.pi / 4
    pub.publish(new_pose)

def calculate_reward(state):
    reward = 0
    if state[0] == 0 or state[0] == 4: reward = -1
    elif state[2] == 0: reward = -1
    elif state[3] == 0: reward = -1

    return reward 

def get_max_action(q_table, state):
    max_rew = 0
    max_action = 0

    rew_0 = q_table[state[0]][state[1]][state[2]][state[3]][state[4]][0]
    rew_1 = q_table[state[0]][state[1]][state[2]][state[3]][state[4]][1]
    rew_2 = q_table[state[0]][state[1]][state[2]][state[3]][state[4]][2]

    max_rew = max(rew_0, rew_1, rew_2)
    if (rew_0 == rew_1 and rew_1 == rew_2): max_action = np.random.randint(0,3)
    elif max_rew == rew_0: max_action = 0
    elif max_rew == rew_1: max_action = 1
    else: max_action = 2

    return (max_rew, max_action)

def get_location():

    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        model = GetModelStateRequest()
        model.model_name = 'triton_lidar'
        result = get_model_srv(model)
        return result.pose.position

    except rospy.ROSInterruptException:
        rospy.loginfo("could not get model state") 


def check_equal(location):
    ERROR = 0.00001
    equal = False 

    if abs(location[0].x - location[1].x) < ERROR:
        if abs(location[1].x - location[2].x) < ERROR:
            if abs(location[0].y - location[1].y) < ERROR:
                if abs(location[1].y - location[2].y) < ERROR:
                    equal = True

    return equal  

def reset_simulation():
    rospy.wait_for_service('/gazebo/reset_simulation')
    try: 
        reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        reset_sim()
    except rospy.ROSInterruptException:
        rospy.loginfo("could not reset simulation")

def save_table(table):
    file = open('q_table_pickle', 'ab')
    pickle.dump(table, file) 
    file.close()
    print("saving q_table as a pickle")


def main():

    # variables for updating q_table 
    LEARNING_RATE = 0.2
    DISCOUNT = 0.8

    # variables for while loop termination
    EPISODES = 150 
    STEPS = 10000
    step = 0
    done = False
    last_3_loc = []
    counter = 0

    # variables for learning decay
    epsilon = 0
    epsilon_0 = 0.9
    DECAY = 0.985
    
    try:
        # initialize publisher and subscribers
        global pub 
        pub = rospy.Publisher('/triton_lidar/vel_cmd', Pose2D, queue_size=10)
        rospy.init_node('triton_train', anonymous=True)

        q_table = np.zeros([5,2,5,2,4,3])
        
        for episode in range(EPISODES):
            # reset simulation
            reset_simulation()
            step = 0 
            done = False

            current_state = get_discrete_state()

            # update epsilon
            epsilon = epsilon_0 * (DECAY ** episode)

            while done == False:

                counter +=1

                # choose an action
                action = 0 
                if np.random.random() > epsilon:
                    action = get_max_action(q_table, current_state)[1]
                    print("action chosen with get max")
                else:
                    action = np.random.randint(0, 3)
                    print("action chosen randomly")

                if action > 3: 
                    print("ERROR with rand number")
                    done = True
                # take action and observe new state
                move(action)
                print("getting new state")
                
                new_state = get_discrete_state()

                # calculate reward 
                print("calculating reward")
                reward = calculate_reward(new_state)

                # update q_table
                print("calculating max_future_q")
                max_future_q = get_max_action(q_table, new_state)[0]
                print("getting current_q")
                current_q = q_table[current_state[0]][current_state[1]][current_state[2]][current_state[3]][current_state[4]][action]
                print("calculating new_q")
                new_q = current_q + LEARNING_RATE * (reward + DISCOUNT * max_future_q - current_q)
                print("updating q_table")
                q_table[current_state[0]][current_state[1]][current_state[2]][current_state[3]][current_state[4]][action] = new_q 
                print("updating current_state")
                current_state = new_state

                # print statements to track progress
                print("Episode: ", episode)
                print("Step: ", step)
                print("Action: ", action)
                print("State: ", current_state)
                print("Count: ", counter)
                print("Location: ", get_location())

                # check termination
                if len(last_3_loc) == 3:
                    last_3_loc.pop(0)
                    last_3_loc.append(get_location())
                    done = check_equal(last_3_loc)
                else:
                    last_3_loc.append(get_location()) 
                step = step + 1
                if step == STEPS: done = True
 

        save_table(q_table)
        
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated")
   
if __name__ == '__main__':
    main()
