import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import torch
import utils_model
import utils_path
import math

bridge = CvBridge()
model = utils_model.initialize_model()
lookahead_distance = 10.0  # Lookahead distance of 10 meters
wheelbase = 1.6  # Wheelbase of the vehicle

def image_callback(data):
    global steer_pub
    # convert ROS Image message to OpenCV image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    input_tensor, shapes = utils_model.preprocessing_image(cv_image)
    with torch.no_grad():
        _, _, _, _, seg = model(input_tensor)
    mask = utils_model.preprocessing_mask(seg, shapes)
    line_edges = utils_path.processing_mask(mask)
    lateral_distance, _ = utils_path.computing_lateral_distance(line_edges)

    # Calculate the steering angle using Pure Pursuit algorithm
    ld_squared = lookahead_distance**2
    alpha = math.atan2(lateral_distance, ld_squared)
    steering_angle = math.atan(2 * wheelbase * math.sin(alpha) / lookahead_distance)

    # Publish the steering angle
    steer_msg = Float64()
    steer_msg.data = steering_angle
    steer_pub.publish(steer_msg)

def main_loop():
    # main loop, we define just the subscribers (non-blocking functions)
    image_sub = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('image_subscriber', anonymous=True)
    steer_pub = rospy.Publisher('<topic_to_publish_to>', Float64, queue_size=1)
    rate = rospy.Rate(10.0)
    # from this point on, leave as is
    while not rospy.is_shutdown():
        try:
            main_loop()
        except rospy.ROSException:
            rate.sleep()
            continue
