#!/usr/bin/python

import rospy
import math
import time
from std_msgs.msg import Int32MultiArray


def callback(msg):

    ###                                      ###
    ### As we don't use index [3], [4], [5]  ###
    ### Make new data array                  ###
    ### msg.data[3], msg.data[4], msg.data[5] is always 0 in this situation
    ###                                      ###

    xycar_sub.data[0] = msg.data[0]
    xycar_sub.data[1] = msg.data[1]
    xycar_sub.data[2] = msg.data[2]
    xycar_sub.data[3] = msg.data[6]
    xycar_sub.data[4] = msg.data[7]

    print(msg.data)
  


###                                                        ### 
### Get valueForCentering to make angle                    ###
###                                                        ### 
def get_valueForCentering():
    global valueForCentering

    right_sensor = xycar_sub.data[3] + xycar_sub.data[2]
    left_sensor = xycar_sub.data[4] + xycar_sub.data[0]

    # if left_sensor == 0 or right_sensor == 0:
    #     left_sensor = 1
    #     right_sensor = 1
   
    valueForCentering  = float(right_sensor) - float(left_sensor)

    return valueForCentering


###                                                        ### 
### Get 'error' and 'errorControl' value for PID control   ###
###                                                        ### 

def get_errorControl(vector):
    global error, errorList          ## record error
    
    desired_vector = 0
    
    error = vector - desired_vector

    if error > 300:                 ## compressed error (D control)
        error -= error * 0.7
    # print("error")
    # print(error)

    errorList.insert(0, error)
    errorPrev = errorList[1]

    errorControl = error - errorPrev

    errorList.pop()                     ## errorList[1] is always errorPrev, errorList[[0] is always error(current error)
    
    return errorControl


# def get_dt():
#     global timeList     ## record time

#     error_Time = rospy.Time.from_sec(time.time())
#     timeList.insert(0, error_Time.to_sec())
#     errorPrev_Time = timeList[1]
#     dt = error_Time.to_sec() - errorPrev_Time
#     timeList.pop()                      ## timeList[1] is always errorPrev_Time, errorList[[0] is always error_Time(current error time)

#     return dt




###                                     ###
### Following codes are for PID control ###
###                                     ###

def calculate_PID(error, errorControl): #, dt):
    global constant_PresaturatedToAnagle

    kp = 5.0                  
    kd = 1.0        #float(17) / 200
    constant_PresaturatedToAnagle =  float(1) / 28
    
    proportional_output = kp * error                        ## P control
    derivative_output = kd * (errorControl) # / dt)         ## D control
    presaturated_output = proportional_output + derivative_output       ## PD control


    ### If you want to add I control, add following codes ###

    ## ki = float(11) / 200
    ## integral_output = 0
    ## integral_output = integral_output + ki * error * dt
    ## presaturated_output = proportional_output + derivative_output + integral_output

    print("presaturated_output")
    print(presaturated_output)

    return presaturated_output





###                                     ### 
### Chcek the possibility of accident   ###
###                                     ### 

def check_collision():
    global accidentPossibility

    for i in xycar_sub.data:
        if i < 48 + 2 and 0 < i:            ## radius of car's boundary circle is 20*sqrt(13) (=about 48)
                                            ## 2 = additional space
            accidentPossibility = True

    return accidentPossibility        




###                                       ###
### Avoiding obstacle (escape collision)  ###
###                                       ###

def escape_collision():
    global turn, presaturated_output, accidentPossibility

    while float(temp)+0.32 > xycar_sub.data[idx]:          # 0.32 is the max distance per sec when topic hz is 132hz
            velocity = -100
            if turn == 'RIGHT':
                angle_cur = -presaturated_output #* 0.9
            elif turn == 'LEFT':
                angle_cur = +presaturated_output #* 0.9
            else:
                turn = 'STRAIGHT'
                angle_cur = 0  
            # print("collision presaturated_output")
            # print(presaturated_output)  
            xycar_msg.data = [angle_cur, velocity]
            motor_pub.publish(xycar_msg)  
    
    accidentPossibility = False 

# rth = ROSTopicHz.get_hz(topic=/ultrasonic)



xycar_sub = Int32MultiArray()
xycar_sub.data = [0,0,0,0,0]
timeList = [0]
errorList = [0]
accidentPossibility = False
angle_cur = 0
velocity = 100      ## you can change velocity  
error = 0.0

rospy.init_node('guide')
motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
ultra_sub = rospy.Subscriber('ultrasonic', Int32MultiArray, callback)
xycar_msg = Int32MultiArray()


while not rospy.is_shutdown():

    valueForCentering = get_valueForCentering()
    errorControl = get_errorControl(valueForCentering)
    #dt = get_dt()
    presaturated_output = abs(calculate_PID(error, errorControl)) #, dt))

    print("presaturated_output")
    print(presaturated_output)

    if valueForCentering > 0:
        turn = 'RIGHT'
        angle_cur = presaturated_output * constant_PresaturatedToAnagle
        

    elif valueForCentering < 0:
        turn = 'LEFT'
        angle_cur = -presaturated_output * constant_PresaturatedToAnagle
        

    else:
        turn = 'STRAIGHT'
        angle_cur = 0    


    if check_collision() == True:
        temp = min(xycar_sub.data)
        idx = xycar_sub.data.index(temp)

        escape_collision()  

        # print("collision angle_cur")
        # print(angle_cur)
        
    print("angle_cur")
    print(angle_cur)

    xycar_msg.data = [angle_cur, velocity]
    motor_pub.publish(xycar_msg)