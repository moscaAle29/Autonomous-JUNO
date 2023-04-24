import cv2
import matplotlib.pyplot as plt
import numpy as np
import pickle
import os


def load_camera_calib():
    if os.path.exists('D435i.p'):
        with open('D435i.p', mode='rb') as f:
            data = pickle.load(f)
            mtx, dist = data['mtx'], data['dist']
    else:
        mtx = [[616.397, 0, 329.849],
                [0, 616.239, 230.878],
                [0, 0, 1]]
        dist = [-0.166739, 0.015824, -0.000173, -0.001353, 0.050206]
    return mtx, dist

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

    # src = np.float32([               
    #     (487, 160),
    #     (411, 160),
    #     (35, 356),
    #     (861, 356)
    # ])

    # src = np.float32([                # ROI Carla_Parth
    #     (371,300),                    # in alto a dx
    #     (271,300),                    # in alto a sx
    #     (71,450),                    # in basso a sx
    #     (571,450)                    # in basso a dx
    # ])

    # src = np.float32([               
    #     (487, 120),
    #     (411, 120),
    #     (35, 316),
    #     (861, 316)
    # ])

    # src = np.float32([               
    #     (1044, 252),
    #     (881, 252),qqq
    #     (75, 665),
    #     (1845, 665)
    # ])


    # src = np.float32([                  # Juno_denerg_resize
    #     (486, 284),
    #     (410, 284),
    #     (35, 480),
    #     (861, 480)
    # ])

    # src = np.float32([                  # Juno_denerg original size (1280, 720)
    #     (694.0, 399.0),
    #     (586.0, 399.0),
    #     (50.0, 675.0),
    #     (1230.0, 675.0)
    # ])

    src = np.float32([                   # (1280, 720)
        (694.0, 240.0),
        (586.0, 240.0),
        (50.0, 475.0),
        (1230.0, 475.0)
    ])

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
    mtx, dist = load_camera_calib()
    warped = eye_bird_view(mask, mtx, dist, d=475)

    # _, mask_th = cv2.threshold(warped, 1, 255, cv2.THRESH_BINARY)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    res_morph = cv2.morphologyEx(warped, cv2.MORPH_CLOSE, kernel)
    line_edges = cv2.Canny(res_morph, 100, 200)
    if show:
        plt.imshow(line_edges)
        plt.show()
    return line_edges

def computing_lateral_distance(line_edges, y=470, show=False):
    white_pixels = np.nonzero(line_edges[y, :])[0]
    if len(white_pixels) > 2:
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
    points_car = np.nonzero(line_edges[line_edges.shape[0]-30, :])[0]          # da cambiare
    x_coords_points_car = points_car[0], points_car[-1]
    posm_car = line_edges.shape[0]-30, (x_coords_points_car[1]+x_coords_points_car[0])//2

    if show:
        cv2.circle(line_edges, posm[::-1] , 2, (255, 255, 255), 1)
        cv2.circle(line_edges, posm_car[::-1] , 2, (255, 255, 0), 1)
        plt.imshow(line_edges)
        plt.show()

    middle_image = line_edges.shape[1]//2
    lateral_distance = posm[1] - middle_image
    car_rel_position = posm_car[1] - middle_image

    lane_meters = 8                         # da cambiare
    scale_factor = lane_meters / (x_coords_points[1] - x_coords_points[0])
    later_distance_meters = lateral_distance * scale_factor
    return later_distance_meters, car_rel_position

    # scala_di_conversione = larghezza_strda_in_metri / larghezza_strada_in_pixel(bird eye view)
    # lateral_distance_in_metri = lateral_distance_in_pixel * scala_di_conversione  