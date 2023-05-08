import numpy as np
import pandas as pd
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import Twist
import math
import tf2_ros
import rospy
from std_msgs.msg import Float64
    
def locate_origin(data, transform):
   """
   -- Locate Origin --
   
   Locate cell index nearest to car position, by doing location/resolution:
   [m]/[m/cell]
   """
   
   x_idx = round((data.info.origin.position.x + transform.transform.translation.x) / data.info.resolution) # check the 180 deg rotation
   y_idx = round((data.info.origin.position.y + transform.transform.translation.y) / data.info.resolution)


   return int(x_idx), int(y_idx)    
    
def get_direction (start_x, start_y, target_x, target_y, heading_angle):
   start_x = abs(start_x)
   start_y = abs(start_y)
   target_x = abs(target_x)
   target_y = abs(target_y)
   
   d_x=target_x-start_x # + because they're negative
   d_y=target_y-start_y
   print(f'x distance: {d_x}')
   print(f'y distance: {d_y}')
   
   direction_angle = math.degrees(math.atan2(d_y,abs(d_x)))  #angle in degrees
   steer_angle = math.degrees(heading_angle)-direction_angle

   if steer_angle < -180:
      steer_angle = 360+(steer_angle) 
      #print(steer_angle)
   elif steer_angle > 180:
      steer_angle = steer_angle-360


   print(f'heading angle: {math.degrees(heading_angle)}')
   print(f'direction angle:{direction_angle}')
   print(f'steer angle: {steer_angle}')

   return steer_angle
 
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.degrees(math.atan2(t0, t1))
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.degrees(math.asin(t2))
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.degrees(math.atan2(t3, t4))
     
        return roll_x, pitch_y, yaw_z

def callback_position(msg):
   global pos_x, pos_y, heading
   # locate car in map
   try:
      transform = tfBuffer.lookup_transform('map', 'camera_link', rospy.Time()) # will return the transform of camera link wirt map

   except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      steer_pub.publish(0)
      return
   
   x=transform.transforms.rotation.x
   y=transform.transforms.rotation.y
   z=transform.transforms.rotation.z
   w=transform.transforms.rotation.w
   _,_,heading=euler_from_quaternion(x,y,z,w)
   
   pos_x, pos_y = locate_origin(msg, transform)

def callback_path(path):
   objective_x=path[0].poses.pose.position.x/0.05
   objective_y=path[0].poses.pose.position.y/0.05
   
   steering_angle=get_direction(pos_x,pos_y,objective_x,objective_y,heading)
   msg=Float64()
   msg.data=steering_angle
   steer_pub.publish(msg)
   pass

def callback_cmd(cmd):
   steering_angle=cmd.angular.z
   #steering_angle=steering_angle*math.pi/180
   steer_pub.publish(steering_angle)

def main_loop():
    # main loop
    #rospy.Subscriber('/planner/move_base/TebLocalPlannerROS/local_plan', Path, callback_path)
    #rospy.Subscriber('/rtabmap/proj_map', OccupancyGrid, callback_position)
    rospy.Subscriber('/planner/cmd_vel', Twist, callback_cmd)
    rospy.spin()
 
if __name__ == '__main__':
    # place one-time defs here
    pos_x=0
    pos_y=0
    heading=0
    rospy.init_node('obstacle_avoidance', anonymous=True)
    steer_pub = rospy.Publisher('KalmanAngle', Float64, queue_size=1)
    rate = rospy.Rate(10.0)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    #from this point on leave as is 
    while not rospy.is_shutdown():
        try:
            main_loop()
        except rospy.ROSException :
            rate.sleep()
            continue

