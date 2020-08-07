#!/usr/bin/python

import rospy
import math
from std_msgs.msg import Int32MultiArray


def callback(msg):

    ### As we don't use index [3], [4], [5]                                   
    ### Make new data array                                                   
    ### msg.data[3], msg.data[4], msg.data[5] is always 0 in this situation   
  
    xycar_sensor.data[0] = msg.data[0]
    xycar_sensor.data[1] = msg.data[1]
    xycar_sensor.data[2] = msg.data[2]
    xycar_sensor.data[3] = msg.data[6]
    xycar_sensor.data[4] = msg.data[7]

    print(msg.data)
  



###                                          ### 
### Get valueForCentering to make angle      ###
### We want to make 'valueForCentering == 0' ### 
### Do PID control using valueForCentering   ###   
###                                          ###         

def get_valueForCentering():
    global valueForCentering

    right_sensor = xycar_sensor.data[3] + xycar_sensor.data[2]
    left_sensor = xycar_sensor.data[4] + xycar_sensor.data[0]
   
    valueForCentering  = float(right_sensor) - float(left_sensor)

    return valueForCentering




###                                                        ### 
### Get 'error' and 'errorControl' value for PID control   ###
###                                                        ### 

def get_errorControl(valueForCentering):
    global error, errorList          ## record error
    
    desired_valueForCentering = 0
    
    error = valueForCentering - desired_valueForCentering

    if error > 500:                 ## compressed error (help D control)
        error = error * 0.5

    errorList.insert(0, error)
    errorPrev = errorList[1]

    errorControl = error - errorPrev

    errorList.pop()                 ## errorList[1] is always errorPrev, errorList[[0] is always error(current error)
    
    return errorControl




###                                                                                  ###
### Following codes are for PID control                                              ###
###                                                                                  ###
### If you want to add 'I' control, add following codes in function                  ###
### If you don't add, it's PD control                                                ###
###                                                                                  ###
### ki = float(11) / 200    ## golden ratio ki for I control                         ###
### integral_output = 0                                                              ###
### integral_output = integral_output + ki * error                                   ###
### presaturated_output = proportional_output + derivative_output + integral_output  ###
###                                                                                  ###

def calculate_PD(error, errorControl): 
    global constant_PresaturatedToAngle

    kp = 10.0                                               ## golden ratio kp for P control
    kd = 5.0                                                ## golden ratio kd for D control
    constant_PresaturatedToAngle =  float(1) / 46           ## constant_PresaturatedToAngle size is related with kp
    
    proportional_output = kp * error                        ## P control
    derivative_output = kd * (errorControl)                 ## D control
    presaturated_output = proportional_output + derivative_output       ## PD control


    
    return presaturated_output




###                                     ### 
### Chcek the possibility of accident   ###
###                                     ### 

def check_collision():
    global accidentPossibility, collisionRange

    carWidth = 48
    carLength = 136
    carDiagonal = math.sqrt(carWidth * carWidth + carLength * carLength)    ## radius of car's boundary circle is about 20*sqrt(13) in this situation
    rollAheadDistance = (carDiagonal - carWidth) / 2                           ## rollAheadDistance is about 48 in this situation
    additionalSpace = 2

    collisionRange = rollAheadDistance + additionalSpace         
                                    
    for sensorLength in xycar_sensor.data:
        if sensorLength < collisionRange and 0 < sensorLength:           
            accidentPossibility = True

    return accidentPossibility        




###                                       ###
### Avoiding obstacle (escape collision)  ###
###                                       ###

def escape_collision():
    global turn, presaturated_output, accidentPossibility, collisionRange, constant_PresaturatedToAngle

    accidentPossibility_sensor = min(xycar_sensor.data)
    accidentPossibility_sensor_idx = xycar_sensor.data.index(accidentPossibility_sensor)

    while collisionRange > xycar_sensor.data[accidentPossibility_sensor_idx]:         
            velocity = -25
            if turn == 'RIGHT':
                angle = -presaturated_output * constant_PresaturatedToAngle
            elif turn == 'LEFT':
                angle = +presaturated_output * constant_PresaturatedToAngle
            else:
                turn = 'STRAIGHT'
                angle = 0  

            xycar_msg.data = [angle, velocity]
            motor_pub.publish(xycar_msg)  

    accidentPossibility = False 


##
## initialize
##
xycar_sensor = Int32MultiArray()
xycar_sensor.data = [0,0,0,0,0]
errorList = [0]
accidentPossibility = False
angle = 0
velocity = 25      ## you can change velocity  
error = 0.0

rospy.init_node('guide')
motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)
ultra_sub = rospy.Subscriber('ultrasonic', Int32MultiArray, callback)
xycar_msg = Int32MultiArray()


while not rospy.is_shutdown():

    valueForCentering = get_valueForCentering()
    errorControl = get_errorControl(valueForCentering)
    presaturated_output = abs(calculate_PD(error, errorControl))

    if valueForCentering > 0:
        turn = 'RIGHT'
        angle = presaturated_output * constant_PresaturatedToAngle

    elif valueForCentering < 0:
        turn = 'LEFT'
        angle = -presaturated_output * constant_PresaturatedToAngle

    else:
        turn = 'STRAIGHT'
        angle = 0    


    if check_collision() == True:
        escape_collision()  

    xycar_msg.data = [angle, velocity]
    motor_pub.publish(xycar_msg)