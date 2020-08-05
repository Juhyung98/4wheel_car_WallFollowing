#!/usr/bin/python

import rospy
import math
import time
from std_msgs.msg import Int32MultiArray


xycar_sub = Int32MultiArray()
xycar_sub.data = [0,0,0,0,0]
timeList = [0]
errorList = [0]
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

       
        if i < 48 + 2 and 0 < i: 
            accident = True

    return accident        


def get_errorControl(vector):
    
    global errorList
    global error
    
    desired_vector = 0
    # errorSum = 0
    
    error = vector - desired_vector

    if error > 300:
        error -= error * 2 / 3
    print("error")
    print(error)

    errorList.insert(0, error)
    errorPrev = errorList[1]

    errorControl = error - errorPrev
    
    # if errorControl is not 0:
    #     errorControl = 0.0

    # print("errorControl")
    # print(errorControl)
    # if errorControl > 100:
    #     errorControl = 100

    errorList.pop()
    
    return errorControl


def get_dt():
    global timeList

    error_Time = rospy.Time.from_sec(time.time())
    timeList.insert(0, error_Time.to_sec())
    errorPrev_Time = timeList[1]
    dt = error_Time.to_sec() - errorPrev_Time
    timeList.pop()

    return dt



def calculate_PID(error, errorControl, dt):

    kp = 0.87                  # float(17) / 200
    #ki = float(11) / 200
    kd = float(17) / 200

    integral_output = 0

    proportional_output = kp * error 
    #integral_output = integral_output + ki * error * dt
    derivative_output = kd * (errorControl / dt)

    presaturated_output = proportional_output + derivative_output 
    # presaturated_output = proportional_output + derivative_output + integral_output

    # print("presaturated_output")
    # print(presaturated_output)

    return presaturated_output


# rth = ROSTopicHz.get_hz(topic=/ultrasonic) 
rospy.init_node('guide')
motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
ultra_sub = rospy.Subscriber('ultrasonic', Int32MultiArray, callback)
xycar_msg = Int32MultiArray()


while not rospy.is_shutdown():

    val = 1.0
    angle_cur = 0
    velocity = 100  
    error = 0.0

    right_sensor = xycar_sub.data[3] + xycar_sub.data[2]
    left_sensor = xycar_sub.data[4] + xycar_sub.data[0]

    if left_sensor == 0 or right_sensor == 0:
        left_sensor = 1
        right_sensor = 1
   
    vector  = float(right_sensor) - float(left_sensor)


    errorControl = get_errorControl(vector)
    dt = get_dt()
    presaturated_output = calculate_PID(error, errorControl, dt)


    if vector > 0:
        angle_cur = presaturated_output
        turn = 'RIGHT'

    elif vector < 0:
        angle_cur = presaturated_output
        turn = 'LEFT'

    else:
        turn = 'STRAIGHT'
        angle_cur = 0    


    if collision() == True:
        temp = min(xycar_sub.data)
        idx = xycar_sub.data.index(temp)

        while float(temp)+0.32 > xycar_sub.data[idx]: # 1 rate would be best!
            velocity = -100
            if turn == 'RIGHT':
                angle_cur = -presaturated_output 
            elif turn == 'LEFT':
                angle_cur = -presaturated_output
            else:
                turn = 'STRAIGHT'
                angle_cur = 0  
            # print("collision presaturated_output")
            # print(presaturated_output)  
            xycar_msg.data = [angle_cur, velocity]
            motor_pub.publish(xycar_msg)  
        accident = False    
        # print("collision angle_cur")
        # print(angle_cur)
    # print("angle_cur")
    # print(angle_cur)
    xycar_msg.data = [angle_cur, velocity]
    motor_pub.publish(xycar_msg)