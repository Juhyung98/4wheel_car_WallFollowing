#!/usr/bin/python

import rospy
import math
import time
from std_msgs.msg import Int32MultiArray
from rostopic import ROSTopicHz

xycar_sub = Int32MultiArray()
xycar_sub.data = [0,0,0,0,0]
hz_list = [0]
last = 0
hz = 0.0
rate = 0.0
accident = False

def callback(msg):
    global last, hz_list, hz, rate
    last = rospy.Time.from_sec(time.time())
    hz_list.insert(0,last.to_sec())
    xycar_sub.data[0] = msg.data[0]
    xycar_sub.data[1] = msg.data[1]
    xycar_sub.data[2] = msg.data[2]
    xycar_sub.data[3] = msg.data[6]
    xycar_sub.data[4] = msg.data[7]
    print(msg.data)
    # if angle not last_angle:
    hz = hz_list[0]-hz_list[1]
    rate = 1/hz
    print(rate)
    hz_list.pop()

def collision():
    global accident
    for i in xycar_sub.data:
        if i < 20 and 0 < i:
            accident = True
    return accident


# rth = ROSTopicHz.get_hz(topic=/ultrasonic)
rospy.init_node('guide')
motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
ultra_sub = rospy.Subscriber('ultrasonic', Int32MultiArray, callback)
xycar_msg = Int32MultiArray()

if rate == 0:
    rate = 123
r = rospy.Rate(rate)

while not rospy.is_shutdown():
    angle_last = 0
    velocity = 100
    if xycar_sub.data[3]+ xycar_sub.data[2] > xycar_sub.data[4] + xycar_sub.data[0]:
        angle_cur = 20 - angle_last
        turn = 'RIGHT'
        #angle_last+=1
    elif xycar_sub.data[3]+ xycar_sub.data[2] < xycar_sub.data[4] + xycar_sub.data[0]:
        angle_cur = -20 + angle_last
        turn = 'LEFT'
        #angle_last-=1
    else:
        angle_cur = 0
    if collision() == True:
        temp = min(xycar_sub.data)
        idx = xycar_sub.data.index(temp)
    while float(temp)*11/10 > xycar_sub.data[idx]:
        velocity = -100
    if turn == 'RIGHT':
        angle_cur -=5
    else:
        angle_cur +=5
    xycar_msg.data = [angle_cur, velocity]
    motor_pub.publish(xycar_msg)
    accident = False

    # angle_cur = angle_last
    xycar_msg.data = [angle_cur, velocity]
    motor_pub.publish(xycar_msg)
    if not(angle_last == angle_cur):
        r.sleep()
        
