
import sys
import os

sys.path.append(os.path.abspath(os.path.join('.')))
sys.path.append(os.path.dirname(__file__))

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import torch
import math

import cv2
import numpy as np
import torch
import os
from torchvision import transforms
from HybridNets.backbone import HybridNetsBackbone
from HybridNets.utils.utils import letterbox, Params
import matplotlib.pyplot as plt

use_cuda = torch.cuda.is_available()
#-----------------change on each computer-------------------------------------------------------------------------------------
params = Params(f'/home/alemomo/Scrivania/H2politO/catkin_ws/src/juno_aug/src/HybridNets/projects/bdd100k.yml')
#-----------------------------------------------------------------------------------------------------------------------------
color_list_seg = {}
steer_msg = Float64()

for seg_class in params.seg_list:
    color_list_seg[seg_class] = list(np.random.choice(range(256), size=3))

import cv2
import matplotlib.pyplot as plt
import numpy as np
import pickle

SIMULATION = True
LANE_METERS = 6                 # da cambiare
Y_TEN_METERS = 555
LANE_PIXELS = None
LATERAL_DISTANCE = 0
scale_factor = None

def load_camera_calib(sim=False):
    if not sim:
        # for the D435i camera
        mtx = [[616.397, 0, 329.849],
                [0, 616.239, 230.878],
                [0, 0, 1]]
        dist = [-0.166739, 0.015824, -0.000173, -0.001353, 0.050206]
    else:
        # for the simulation
        mtx = [[1395.35, 0, 640],
                [0, 1395.35, 360],
                [0, 0, 1]]
        dist = [0, 0, 0, 0, 0]
    return np.array(mtx), np.array(dist)

def undistort(img, mtx, dist):
    '''
    Undistorts an image
    :param img (ndarray): Image, represented an a numpy array
    :param mtx: Camera calibration matrix
    :param dist: Distortion coeff's
    :return : Undistorted image
    '''
    
    undistort = cv2.undistort(img, mtx, dist, None, mtx)
    return undistort

def warp_image(img, warp_shape, src, dst):
    '''
    Performs perspective transformation (PT)
    :param img (ndarray): Image
    :param warp_shape: Shape of the warped image
    :param src (ndarray): Source points
    :param dst (ndarray): Destination points
    :return : Tuple (Transformed image, PT matrix, PT inverse matrix)
    '''
    M = cv2.getPerspectiveTransform(src, dst)
    invM = cv2.getPerspectiveTransform(dst, src)
    
    warped = cv2.warpPerspective(img, M, warp_shape, flags=cv2.INTER_CUBIC)
    return warped, M, invM

def eye_bird_view(img, mtx, dist, d=350):
    ysize = img.shape[0]
    xsize = img.shape[1]
    
    undist = undistort(img, mtx, dist)

    # src = np.float32([                # ROI Carla_Parth
    #     (371,300),                    # in alto a dx
    #     (271,300),                    # in alto a sx
    #     (71,450),                    # in basso a sx
    #     (571,450)                    # in basso a dx
    # ])

    # src = np.float32([                  # Juno_denerg_resize
    #     (486, 284),
    #     (410, 284),
    #     (35, 480),
    #     (861, 480)
    # ])

    # src = np.float32([               
    #     (1044, 252),
    #     (881, 252),
    #     (75, 665),
    #     (1845, 665)
    # ])


    src = np.float32([                  # Juno_denerg original size (1280, 720)
        (694.0, 350.0),
        (586.0, 350.0),
        (50.0, 675.0),
        (1230.0, 675.0)
    ])

    # src = np.float32([                   # (1280, 720)
    #     (694.0, 240.0),
    #     (586.0, 240.0),
    #     (50.0, 475.0),
    #     (1230.0, 475.0)
    # ])

    # src = np.float32([                   # Simulation (1280, 720)
    #     (694.0, 490.0),
    #     (586.0, 490.0),
    #     (50.0, 675.0),
    #     (1230.0, 675.0)
    # ])

    # src = np.float32([                   # Simulation 2 (1280, 720)
    #     (694.0, 368.0),
    #     (586.0, 368.0),
    #     (50.0, 675.0),
    #     (1230.0, 675.0)
    # ])

    # src = np.float32([
    #     (347.0, 127.0),
    #     (293.0, 127.0),
    #     (25.0, 253.0),
    #     (615.0, 253.0)
    # ])

    dst = np.float32([
        (xsize - d, 0),
        (d, 0),
        (d, ysize),
        (xsize - d, ysize)
    ])

    warped, M, invM = warp_image(undist, (xsize, ysize), src, dst)
    return warped

def processing_mask(mask, show=False):
    mtx, dist = load_camera_calib(sim=SIMULATION)
    warped = eye_bird_view(mask, mtx, dist, d=490)

    # _, mask_th = cv2.threshold(warped, 1, 255, cv2.THRESH_BINARY)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (16, 16))
    res_morph = cv2.morphologyEx(warped, cv2.MORPH_CLOSE, kernel)
    line_edges = cv2.Canny(res_morph, 100, 100)
    if show:
        plt.imshow(line_edges)
        plt.show()
    return line_edges

def computing_lateral_distance(line_edges, y=Y_TEN_METERS, show=False):
    global LANE_PIXELS
    global LATERAL_DISTANCE
    global scale_factor
    white_pixels = np.nonzero(line_edges[y, :])[0]
    if len(white_pixels) == 0:
        return LATERAL_DISTANCE
    if len(white_pixels) == 1:
        if white_pixels[0] > line_edges.shape[1]//2:
        #     x_coords_points = 0, white_pixels[0]
        # else:
        #     x_coords_points = white_pixels[0], line_edges.shape[1]
        # if white_pixels[0] > line_edges.shape[1]//2:
            x_coords_points = white_pixels[0]-LANE_PIXELS, white_pixels[0]
        else:
            x_coords_points = white_pixels[0], white_pixels[0]+LANE_PIXELS
    elif len(white_pixels) > 2:
        max_diff = float('-inf')
        max_diff_indices = None

        for i in range(len(white_pixels) - 1):
            diff = abs(white_pixels[i] - white_pixels[i+1])
            if diff > max_diff:
                max_diff = diff
                max_diff_indices = (i, i+1)
        x_coords_points = white_pixels[max_diff_indices[0]], white_pixels[max_diff_indices[1]]
    else:
        x_coords_points = white_pixels[0], white_pixels[1]
    posm = y, (x_coords_points[1]+x_coords_points[0])//2

    if show:
        cv2.circle(line_edges, posm[::-1], 2, (255, 255, 255), 1)
        plt.imshow(line_edges)
        plt.show()

    middle_image = line_edges.shape[1]//2
    lateral_distance = posm[1] - middle_image
    if not LANE_PIXELS:
        LANE_PIXELS = x_coords_points[1] - x_coords_points[0]   
        scale_factor = LANE_METERS / LANE_PIXELS               
    
    later_distance_meters = lateral_distance * scale_factor
    LATERAL_DISTANCE = later_distance_meters
    return later_distance_meters

def preprocessing_image(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    resized_shape = params.model['image_size']

    normalize = transforms.Normalize(
        mean=params.mean, std=params.std
    )
    transform = transforms.Compose([
        transforms.ToTensor(),
        normalize,
    ])

    if isinstance(resized_shape, list):
        resized_shape = max(resized_shape)
    shapes = []

    h0, w0 = image.shape[:2]  
    r = resized_shape / max(h0, w0) 
    input_img = cv2.resize(image, (int(w0 * r), int(h0 * r)), interpolation=cv2.INTER_AREA)
    h, w = input_img.shape[:2]

    (input_img, _), ratio, pad = letterbox((input_img, None), resized_shape, auto=True,
                                            scaleup=False)

    if use_cuda:
        input_tensor = transform(input_img).unsqueeze(0).cuda()
    else:
        input_tensor = transform(input_img).unsqueeze(0).cpu()   
    shapes.append(((h0, w0), ((h / h0, w / w0), pad)))  
    return input_tensor, shapes

def initialize_model():
    MULTICLASS_MODE: str = "multiclass"
    print(os.system("pwd"))

    anchors_ratios = params.anchors_ratios
    anchors_scales = params.anchors_scales
    obj_list = params.obj_list
    seg_list = params.seg_list

    #-----------------change on each computer-------------------------------------------------------------------------------------
    weights_path = '/home/alemomo/Scrivania/H2politO/catkin_ws/src/juno_aug/src/HybridNets/weights/hybridnets.pth'
    #-----------------------------------------------------------------------------------------------------------------------------
    state_dict = torch.load(weights_path, map_location='cuda' if use_cuda else 'cpu')

    weight_last_layer_seg = state_dict['segmentation_head.0.weight']

    seg_mode = MULTICLASS_MODE

    model = HybridNetsBackbone(compound_coef=3, num_classes=len(obj_list), ratios=eval(anchors_ratios),
                            scales=eval(anchors_scales), seg_classes=len(seg_list), seg_mode=seg_mode)           # lasciare None sulla backbone è ok

    model.load_state_dict(state_dict)
    model.requires_grad_(False)
    model.eval()

    if use_cuda:
        model = model.cuda()
    else:
        model = model.cpu()
    return model

def preprocessing_mask(seg, shapes, show=False):
    _, seg_mask = torch.max(seg, 1)
    seg_mask_ = seg_mask[0].squeeze().cpu().numpy()
    pad_h = int(shapes[0][1][1][1])
    pad_w = int(shapes[0][1][1][0])
    seg_mask_ = seg_mask_[pad_h:seg_mask_.shape[0]-pad_h, pad_w:seg_mask_.shape[1]-pad_w]
    seg_mask_ = cv2.resize(seg_mask_, dsize=shapes[0][0][::-1], interpolation=cv2.INTER_NEAREST)
    color_seg = np.zeros((seg_mask_.shape[0], seg_mask_.shape[1], 3), dtype=np.uint8)
    for index, seg_class in enumerate(params.seg_list):
        color_seg[seg_mask_ == index+1] = color_list_seg[seg_class]
    color_seg = color_seg[..., ::-1]  

    color_mask = np.mean(color_seg, 2)
    _, color_mask_bn = cv2.threshold(color_mask,0,255, cv2.THRESH_BINARY)
    if show:
        plt.imshow(color_mask_bn)
        plt.show()
    return color_mask_bn.astype('uint8')

def image_callback(data):
    global steer_pub
    # convert ROS Image message to OpenCV image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    print(cv2.imwrite('/home/alemomo/image.png',cv_image))
    input_tensor, shapes = preprocessing_image(cv_image)
    with torch.no_grad():
        _, _, _, _, seg = model(input_tensor)
    mask = preprocessing_mask(seg, shapes, show=False)
    print(cv2.imwrite('/home/alemomo/mask.png',mask))
    line_edges = processing_mask(mask, show=False)
    
    #print(cv2.imwrite('/home/alemomo/line_edges.png',line_edges))
    lateral_distance = computing_lateral_distance(line_edges, show=False)

    # Calculate the steering angle using Pure Pursuit algorithm
    distance_to_waypoint = longitudinal_distance**2 + lateral_distance**2
    rad_steering_angle = math.atan2(2 * wheelbase * lateral_distance, distance_to_waypoint )  


    degree_steering_angle = math.degrees(rad_steering_angle)
    print(f"--------------------------\n{lateral_distance=}\n{degree_steering_angle=}")
    # Publish the steering angle
    steer_msg.data = degree_steering_angle
    # steer_msg.data = 0
    steer_pub.publish(steer_msg)

def main_loop():
    # main loop, we define just the subscribers (non-blocking functions)
    image_sub = rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('path_planning', anonymous=True)
    bridge = CvBridge()
    model = initialize_model()
    longitudinal_distance = 10.0  # Lookahead distance of 10 meters
    wheelbase = 1.6  # Wheelbase of the vehicle
    steer_pub = rospy.Publisher('KalmanAngle', Float64, queue_size=1)
    rate = rospy.Rate(10.0)
    # from this point on, leave as is
    while not rospy.is_shutdown():
        try:
            main_loop()
        except rospy.ROSException:
            rate.sleep()
            continue
