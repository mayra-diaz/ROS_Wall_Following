#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from nav_msgs.msg import Path
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

import json
import random as rd

dir = '../Stingray-Simulation/catkin_ws/src/stingray_sim/src/'
qtable_dir = dir+'qtable48-4.json'
#dir = 'src/stingray_sim/src/'
starting_positions = [[0,0], [-3,-3], [2,-2], [1.3,-0.6], [3.4,1.4], [-0.03,-3], [-3.2,2.8], [-3.5, 0.9]]
get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
model = GetModelStateRequest()
model.model_name='triton_lidar'


def change_position(position):
    pose_pub=rospy.Publisher ('/gazebo/set_model_state', ModelState,queue_size=1)
    model_state = ModelState()
    model_state.model_name = 'triton_lidar'
    model_state.pose.position.x = position[0]
    model_state.pose.position.y = position[1]
    model_state.pose.position.z = 0.0
    model_state.pose.orientation.x = 0
    model_state.pose.orientation.y = 0
    model_state.pose.orientation.z = 0
    model_state.pose.orientation.w = 0
    pose_pub.publish(model_state)

def on_ground():
    result = get_model_srv(model)
    return abs(result.pose.position.z)<0.2

def write_json():
    global qtable
    global e
    json_object = json.dumps(qtable, indent=4)
    with open(qtable_dir, "w") as outfile:
        outfile.write(json_object)
    with open(dir+'e.txt', "w") as outfile:
        outfile.write(str(e))

reset_world = rospy.ServiceProxy("/gazebo/reset_world", Empty)

qtable = {}
with open(qtable_dir, 'r') as openfile:
    qtable = json.load(openfile)

# right, left, forward
#actions = [[0.1, -0.2], [0.1,0.2], [0.25,0]]

# right, left, forward, backward
actions = [[0.1, -0.2], [0.1,0.2], [0.25,0], [-0.25,0]]

state ='0000'
prev_state = state
same_position = 0
prev_action = 0
e = 0.9
dec = 0.985
lrate = 0.3
dfactor = 0.7
reward_value = 1
penalty_value = -0.8
training = True

laser_data = None
poses = None

def set_data(data):
    global laser_data
    laser_data =  data.ranges

def set_pose(data):
    global poses
    poses = data.poses

def check_position():
    global poses
    cx = abs(poses[-1].pose.position.x-poses[-2].pose.position.x)
    cy = abs(poses[-1].pose.position.y-poses[-2].pose.position.y)
    return (cx<0.001 and cy<0.001)

def set_state():
    global state
    global prev_state
    global same_position
    global laser_data
    prev_state = state
    rdg =  [-25,36]
    crdg = [45,66]
    cdg =  [75, 106]
    ldg =  [130, 210]
    avgr =  sum(laser_data[rdg[0]:len(laser_data)])+sum(laser_data[0:crdg[1]]) / (rdg[1]-rdg[0])
    avgrc = sum(laser_data[crdg[0]:crdg[1]]) / (crdg[1]-crdg[0])
    avgc =  sum(laser_data[cdg[0]:cdg[1]])   / (cdg[1]-cdg[0])
    avgl =  sum(laser_data[ldg[0]:ldg[1]])   / (ldg[1]-ldg[0])
    r =  0 if avgr<=0.2  else 1 if avgr<0.6 else 2 if avgr<1.4 else 3
    rc = 0 if avgrc<=0.2 else 1
    c =  0 if avgc<=0.2 else 1 if avgc<1.2 else 2
    l =  0 if avgl<=0.2  else 1
    state = str(l)+str(c)+str(rc)+str(r)
    same_position = 3 if (sum([state[i]=='0' for i in range(4)])>1) else same_position+1 if check_position() else 0


def get_action():
    global e
    global state
    mx = max(qtable[state])
    gp = rd.randrange(1000)/1000
    if training and (e>gp):
        return rd.randrange(len(actions))
    return qtable[state].index(mx)


def update_qtable():
    global state
    global prev_state
    global prev_action
    r = penalty_value if (sum([state[i]=='0' for i in range(4)])) else reward_value if (state[3]=='2') else 0
    old_value = qtable[prev_state][prev_action]
    qtable[prev_state][prev_action] = old_value + lrate*(r + dfactor*max(qtable[state]) - old_value)


def move(steps):
    pub = rospy.Publisher('/triton_lidar/vel_cmd', Twist, queue_size=10)
    rate = rospy.Rate(2)
    vel = Twist()
    rospy.sleep(2)
    global training
    global state
    global prev_action
    global poses
    while not rospy.is_shutdown() and steps!=0 and same_position<3 and on_ground():
        set_state()
        if training:
            update_qtable()
        rospy.loginfo(state)
        ac = get_action()
        prev_action = ac
        vel.linear.x =  0
        vel.linear.y =  float(actions[ac][0])
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = float(actions[ac][1])
        pub.publish(vel)
        steps -=1
        rate.sleep()
    print(state,same_position)
    print(poses[-1])


def robot_movement(episodes):
    global e
    global dec
    global same_position
    global starting_positions
    for i in range(episodes):
        print(i)
        change_position(starting_positions[i%len(starting_positions)])
        e = e*dec
        print(e)
        same_position = 0
        move(300)
        print('moved\n')
        write_json()


if __name__ == '__main__':
    try:
        rospy.init_node('robot_movement', anonymous=True)
        print("Init")
        laser_sub = rospy.Subscriber("/scan", LaserScan, set_data)
        path_sub = rospy.Subscriber('/triton/path', Path, set_pose)
        training = False
        while laser_data is None:
            rospy.sleep(1)
        change_position(starting_positions[-1])
        robot_movement(501)
    except rospy.ROSInterruptException:
        print("Error")