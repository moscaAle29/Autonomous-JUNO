#!/usr/bin/env python3
import rospy
from rospy.numpy_msg import numpy_msg

from sensor_msgs.msg import NavSatFix
import numpy as np

from std_msgs.msg import Float64
from std_msgs.msg import Float32
from shared_objects.Motor import Stepper

import math

#from time import sleep
motor=Stepper("/dev/ttyACM0")
SIZE=10
TRANSMISSION_RATIO= 9
MAX_ANGLE=45
step_n=0
#values= np.zeros((SIZE,), dtype=Float64)
mean=0
index=0
md=0
    
# values[index]=data.data
#         index+=1
#         if index >=SIZE:
#             index=0
#         mean=np.mean(values)
#         step=int(mean/step_angle)*256

def callback_stepangle(data):
    global motor
    global step_n
    global mean
    
    #step_n +=1
    step_angle=1.8 #angle 
    #if(step_n > 2):
        
        #step_n=0
    angle=data.data
        #mean= mean/2
        #step=int(mean/step_angle*256)
        #mean=data.data
    print("angle",angle)
        
        # Move motor with saturation
    if angle > MAX_ANGLE:
        step=int((MAX_ANGLE/step_angle)*256)
        print(step)
            #print(step)
        motor.move_stepper(-int(step*TRANSMISSION_RATIO))
    elif angle < -MAX_ANGLE:
        step=int(-(MAX_ANGLE/step_angle)*256)
        print(step)
        #print(step)
        motor.move_stepper(-int(step*TRANSMISSION_RATIO))
    else:
        step=int((angle/step_angle)*256)
        print(step)
        motor.move_stepper(-int(step*TRANSMISSION_RATIO))
            #print(step)
        
    #else:
        #values[index]=data.data
        #index+=1
        #mean+=data.data


def callback_brake(data):
    global brake
    #brake.move_stepper()
    brake.brake()
    pass
        
    
def main_loop():
            
    rospy.Subscriber("KalmanAngle", Float64, callback_stepangle)
    #rospy.Subscriber("brake_command", Float64, callback_brake) 
    rospy.spin()     
      
if __name__ == '__main__':
    rospy.init_node('steering_brake_node', anonymous=True)

    #----------create Motor component -----------
    #motor=Stepper("/dev/ttyACM0",0,12000,500000,-50000)
    #brake=Stepper("/dev/ttyACM1")

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try: 
            main_loop()  
        except rospy.ROSException: 
            rate.sleep()
            continue