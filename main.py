import os
import sys

sys.path.append(os.path.abspath(os.path.join('.')))
sys.path.append(os.path.dirname(__file__))

import torch
import utils_model
import utils_path
import cv2
import math

model = utils_model.initialize_model()
lookahead_distance = 10.0  # Lookahead distance of 10 meters
wheelbase = 1.6  # Wheelbase of the vehicle

if __name__ == '__main__':
    img = cv2.imread("/home/alemomo/Scrivania/H2politO/catkin_ws/src/juno_aug/src/worst_case.jpeg", cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
    #img = cv2.resize(img, (1280, 720), fx=0 ,fy=0) 
    input_tensor, shapes = utils_model.preprocessing_image(img)
    with torch.no_grad():
        _, _, _, _, seg = model(input_tensor)
    mask = utils_model.preprocessing_mask(seg, shapes, show=True)
    line_edges = utils_path.processing_mask(mask, show=True)
    lateral_distance = utils_path.computing_lateral_distance(line_edges, show=True)
    print(lateral_distance)
    ld_squared = lookahead_distance**2
    alpha = math.atan2(lateral_distance, ld_squared)
    rad_steering_angle = math.atan(2 * wheelbase * math.sin(alpha) / lookahead_distance)    
    degree_steering_angle = rad_steering_angle * 180 / math.pi
    print(degree_steering_angle)