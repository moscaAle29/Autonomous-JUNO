import numpy as np
import pandas as pd
from nav_msgs.msg import OccupancyGrid
import math
import tf2_ros
import rospy
from std_msgs.msg import Float64

if __name__=="main":
    rospy.init_node('obstacle_avoidance',anonymous=True)
    steer_pub = rospy.Publisher('KalmanAngle', Float64, queue_size=1)
    rate = rospy.Rate(10.0)
    # tf listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    
def callback_occupancy(grid):
    print(grid)
    
def main_loop():
    # main loop
    rospy.Subscriber('/rtabmap/grid_map', OccupancyGrid, callback_occupancy)
    rospy.spin()
    
    while not rospy.is_shutdown():
        try:

            main_loop()
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
