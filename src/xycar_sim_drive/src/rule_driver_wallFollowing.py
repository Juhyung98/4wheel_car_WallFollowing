#!/usr/bin/python

import rospy
import math
from std_msgs.msg import Int32MultiArray


xycar_sub = Int32MultiArray()
xycar_sub.data = [0,0,0,0,0]
accident = False

def callback(msg):
    xycar_sub.data[0] = msg.data[0]
    xycar_sub.data[1] = msg.data[1]
    xycar_sub.data[2] = msg.data[2]
    xycar_sub.data[3] = msg.data[6]
    xycar_sub.data[4] = msg.data[7]

    print(msg.data)
  

def collision():
    global accident
    for i in xycar_sub.data:
        if i < 48 + 2 and 0 < i:    ## radius of boundary circle = 20*sqrt(13) ## 2 = additional space
            accident = True
    return accident        

def getTargetSensor(NEAR_WALL, right_sensor, left_sensor) :

    if NEAR_WALL == 'LEFT_WALL':
        targetSensor = left_sensor
    elif NEAR_WALL == 'RIGHT_WALL':
        targetSensor = right_sensor
    else:
        targetSensor = left_sensor

    return targetSensor

def wallFollowing(targetSensor, val):
    global angle_cur

    if targetSensor > 150:
        angle_cur -= val
        
    elif targetSensor < 150:
        angle_cur += val
       
    else:
        angle_cur = 0
       

def turnDirection(vector):
    global angle_cur, value

    value = abs(vector) * 9 / 200

    if vector > 0:
        #val = vector * 9 / 200
        angle_cur = value
        turn = 'RIGHT'

    elif vector < 0:
        #val = ((-vector) * 9 / 200)
        angle_cur = value
        turn = 'LEFT'

    else:
        turn = 'STRAIGHT'
        angle_cur = 0    
    
    return turn

# rth = ROSTopicHz.get_hz(topic=/ultrasonic) 
rospy.init_node('guide')
motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
ultra_sub = rospy.Subscriber('ultrasonic', Int32MultiArray, callback)
xycar_msg = Int32MultiArray()


while not rospy.is_shutdown():
 
    val = 1.0
    value = 1.0
    angle_cur = 0
    velocity = 100  

    right_sensor = xycar_sub.data[3] + xycar_sub.data[2]
    left_sensor = xycar_sub.data[4] + xycar_sub.data[0]

    NEAR_WALL = 'TWO_WALLS'

    if left_sensor == 0 or right_sensor == 0:
        left_sensor = 1
        right_sensor = 1
   
    vector  = float(right_sensor) - float(left_sensor)
    val = vector * 18 / 200

    if right_sensor > left_sensor:
        NEAR_WALL = 'LEFT_WALL'
    elif right_sensor < left_sensor:
        NEAR_WALL = 'RIGHT_WALL'
    else:
        NEAR_WALL = 'TWO_WALLS'

    targetSensor = getTargetSensor(NEAR_WALL, right_sensor, left_sensor)
    wallFollowing(targetSensor, val)

    if right_sensor > 500 or left_sensor > 500:
        turn = turnDirection(vector)    


    if collision() == True:
        temp = min(xycar_sub.data)
        idx = xycar_sub.data.index(temp)
        while float(temp)+0.32 > xycar_sub.data[idx]: # 1 rate would be best!
            velocity = -100
            if turn == 'RIGHT':
                angle_cur -= value 
            elif turn == 'LEFT':
                angle_cur -= value
            else:
                turn = 'STRAIGHT'
                angle_cur = 0   
            # angle_cur = 0 
            xycar_msg.data = [angle_cur, velocity]
            motor_pub.publish(xycar_msg)  
        accident = False    

    xycar_msg.data = [angle_cur, velocity]
    motor_pub.publish(xycar_msg)
