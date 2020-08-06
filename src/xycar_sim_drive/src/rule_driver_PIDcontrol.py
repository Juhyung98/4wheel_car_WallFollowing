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

        ## As we don't use index [3], [4], [5]
    ## Make new data array

    xycar_sub.data[0] = msg.data[0]
    xycar_sub.data[1] = msg.data[1]
    xycar_sub.data[2] = msg.data[2]
    xycar_sub.data[3] = msg.data[6]
    xycar_sub.data[4] = msg.data[7]

    print(msg.data)
  

def check_collision():
    global accidentPossibility

    for i in xycar_sub.data:
        if i < 48 + 2 and 0 < i:         ## radius of car's boundary circle is 20*sqrt(13) (=about 48)
                                            ## 2 = additional space
            accidentPossibility = True

    return accidentPossibility        


def get_errorControl(vector):
    global errorList          ## record error
    global error
    
    desired_vector = 0
    
    error = vector - desired_vector

    if error > 300:
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



def calculate_PID(error, errorControl, dt):

    kp = 5.0                
    #ki = float(11) / 200
    kd = 1.0 #float(17) / 200

    constant_PresaturatedToAnagle =  float(1) / 28
    #integral_output = 0
    proportional_output = kp * error 
    #integral_output = integral_output + ki * error * dt
    derivative_output = kd * (errorControl) # / dt)

    presaturated_output = proportional_output + derivative_output 
    # presaturated_output = proportional_output + derivative_output + integral_output

    print("presaturated_output")
    print(presaturated_output)

    return presaturated_output



def escape_collision():
    global turn

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
rospy.init_node('guide')
motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
ultra_sub = rospy.Subscriber('ultrasonic', Int32MultiArray, callback)
xycar_msg = Int32MultiArray()


while not rospy.is_shutdown():

    angle_cur = 0
    velocity = 100      ## you can change velocity  
    error = 0.0

    right_sensor = xycar_sub.data[3] + xycar_sub.data[2]
    left_sensor = xycar_sub.data[4] + xycar_sub.data[0]

    if left_sensor == 0 or right_sensor == 0:
        left_sensor = 1
        right_sensor = 1
   
    vector  = float(right_sensor) - float(left_sensor)


    errorControl = get_errorControl(vector)
    dt = get_dt()
    presaturated_output = abs(calculate_PID(error, errorControl, dt))
    print("presaturated_output")
    print(presaturated_output)

    if vector > 0:
        turn = 'RIGHT'
        angle_cur = presaturated_output * constant_PresaturatedToAnagle
        

    elif vector < 0:
        turn = 'LEFT'
        angle_cur = -presaturated_output * constant_PresaturatedToAnagle
        

    else:
        turn = 'STRAIGHT'
        angle_cur = 0    


    if check_collision() == True:
        temp = min(xycar_sub.data)
        idx = xycar_sub.data.index(temp)

        # while float(temp)+0.32 > xycar_sub.data[idx]:          # 0.32 is the max distance per sec when topic hz is 132hz
        #     velocity = -100
        #     if turn == 'RIGHT':
        #         angle_cur = -presaturated_output #* 0.9
        #     elif turn == 'LEFT':
        #         angle_cur = +presaturated_output #* 0.9
        #     else:
        #         turn = 'STRAIGHT'
        #         angle_cur = 0  
        #     # print("collision presaturated_output")
        #     # print(presaturated_output)  
        #     xycar_msg.data = [angle_cur, velocity]
        #     motor_pub.publish(xycar_msg)  
        # accident = False
          
        escape_collision()  

        # print("collision angle_cur")
        # print(angle_cur)
        
    print("angle_cur")
    print(angle_cur)
    xycar_msg.data = [angle_cur, velocity]
    motor_pub.publish(xycar_msg)