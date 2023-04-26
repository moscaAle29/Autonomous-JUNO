#!/usr/bin/env python3
import rospy
from rospy.numpy_msg import numpy_msg

from sensor_msgs.msg import NavSatFix
import numpy as np

from std_msgs.msg import Float64
from std_msgs.msg import Float32

import math

from time import sleep
motor=None
step_n=0
mean=0
module=rospy.get_param("/core_module_active") # gps, distance module 1 if active, 0 otherwise
alive=rospy.get_param("/active_node")["core"]
pub_dir_gps_comp=None
md=0


def callback_direction(data): 
    
    global steering
    heading=data.data
    global md
    global pub_dir_gps_comp
    md1=float(md)
    steering=steering_angle(md1,heading)
    pub_dir_gps_comp.publish(Float64(steering)) #this message is sent to kalman filter
    #print(f"steering angle is {steering}") 
    
    
    


def callback_stepangle(data):
    global motor
    global step_n
    global mean
    
    step_n +=1
    step_angle=1.8 #angle 
    if(step_n > 2):
        step_n=0
        angle=data.data
        mean+=data.data
        mean= mean/3
        step=int(mean/step_angle)*256
        mean=data.data
        #print("motor step",step,"angle",mean,"data",data.data)
        
        # Move motor with saturation
        if mean > 90:
            step=int(90/step_angle)*256
            motor.move_stepper(int(step*3))
        elif mean < -90:
            step=int(-90/step_angle)*256
            motor.move_stepper(int(step*3))
        else:
            motor.move_stepper(int(step*3))
            pass
        
    else:
        mean+=data.data


def callback_brake(data):
    global brake
    #brake.move_stepper()
    brake.brake()
    pass
    
def callback_steer(steer):
    global steering
    steering=steer
        
    
def main():
    #rate = rospy.Rate()
#----------create gps component -------------
    
    if(module["GPS"]):
        global pub_dir_gps_comp
        #gps=GPS_USB("/dev/ttyACM0",9600)
        #gps.read_data()
        #pub_gps=rospy.Publisher('gps_coordinate',NavSatFix)
        #pub_speed_gps=rospy.Publisher('gps_speed',Float64)
        #pub_dir_gps=rospy.Publisher('gps_direction',Float64)
        #pub_dir_gps_comp=rospy.Publisher('gps_direction_comp',Float64)#computed gpx direction
        target=0 
        #list_point=gps_parser('/home/jose/catkin_ws/src/juno_ros_scripts/src/Core/track.gpx')
        #list_point=gps_parser('/home/jose/catkin_ws/src/juno_ros/juno_ros_scripts/src/Core/out_gps_nogPARKING_MOD2.gpx')
        rospy.Subscriber("heading",Float32,callback_direction)
        rospy.Subscriber("core/Steering",Float32,callback_steer)
        rospy.Subscriber("KalmanAngle", Float64, callback_stepangle)
        #motor_direction=0
    
            
#----------create Motor component -----------
    
    global motor, brake
    from Motor import Stepper
    #motor=Stepper("/dev/ttyACM0",0,12000,500000,-50000)
    motor=Stepper("/dev/ttyACM1")
    brake=Stepper("/dev/ttyACM2")
    rospy.Subscriber("KalmanAngle", Float64, callback_stepangle)
    rospy.Subscriber("brake_command", Float64, callback_brake)

#----------create DISTANCE component ------------- 
    if(module["distance"]): #to create the object
         pub_distance=rospy.Publisher('sensor_distance',Float64)

#---------Loop Starts Here ------------------------         
    rospy.init_node('initial_core', anonymous=True)
    #while not rospy.is_shutdown():
        #rospy.Subscriber("KalmanAngle", Float64, callback_stepangle)
        #rospy.spin()
#----------GPS component-----------         
        # if(module["GPS"]): #if gps module is active
        #         global md
                 
        #         gps.read_data() #read gpx data
        #         msg=gps.return_data()
        #         publish_m=NavSatFix()
        #         publish_m.latitude=msg["RMC"]["latitude"][0]
        #         if(msg["RMC"]["latitude"][1]=="S"):
        #             publish_m.latitude*=-1
        #         publish_m.longitude= msg["RMC"]["longitude"][0]
        #         if(msg["RMC"]["longitude"][1]=="W"):
        #             #publish_m.latitude*=-1
        #             pass
        #         publish_m.altitude=msg["RMC"]["altitude"]
        #         pub_gps.publish(publish_m)
        #         pub_speed_gps.publish(Float64(msg["VTG"]["speed_km"]))
        #         #print(publish_m.latitude)
        #         #print(float(publish_m.longitude))
        #         #pub_dir_gps.publish(Float64(msg["RMC"]["direction"]))
        #         current_dist,motor_direction,passed=gps_direction([float(publish_m.latitude),float(publish_m.longitude)],list_point[target])
        #         #subscribe to arduino heading value 
        #         md=motor_direction
                
               
        #         #print('Current Latitude {}; Current Longitude: {}'.format(float(publish_m.latitude), float(publish_m.longitude)))
        #         #print('Target Latitude {}; Target Longitude: {}'.format(list_point[target][0], list_point[target][1]))
        #         #print('Distance {}; Checkpoint: {}'.format(current_dist, target))
        #         if(passed):
        #                 target+=1
        #                 if(target==len(list_point)):
        #                     target-=1
    #rospy.sleep(0.5)
    #rate.sleep()
                
if __name__ == '__main__': 
 try: 
    #if(alive):
        main()  
 except rospy.ROSInterruptException: 
    pass
