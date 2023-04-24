import rospy
# import also messages types, for example 
from std_msgs.msg import Float64

def callback_func(value):
    #define the actions to perform on received data, in this case value
    # NB you can still define and use functions
    pass

def main_loop():
    # main loop, we define just the subscribers (non blocking functions) 
    rospy.Subscriber('<topic_to_subscribe_to>', Float64, callback_func)
    rospy.spin()

if __name__ == '__main__':
    # place one-time defs here
    rospy.init_node('<node_name>', anonymous=True)
    steer_pub = rospy.Publisher('<topic_to_subscribe_to>', Float64, queue_size=1)
    rate = rospy.Rate(10.0)
    #from this point on leave as is 
    while not rospy.is_shutdown():
        try:
            main_loop()
        except rospy.ROSException :
            rate.sleep()
            continue