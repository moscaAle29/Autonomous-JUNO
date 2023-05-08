import cv2
import matplotlib.pyplot as plt
import numpy as np
import pickle

SIMULATION = False
LANE_METERS = 14                 # da cambiare
Y_TEN_METERS = 470
LANE_PIXELS = None

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

    src = np.float32([               
        (1044, 252),
        (881, 252),
        (75, 665),
        (1845, 665)
    ])


    # src = np.float32([                  # Juno_denerg original size (1280, 720)
    #     (694.0, 399.0),
    #     (586.0, 399.0),
    #     (50.0, 675.0),
    #     (1230.0, 675.0)
    # ])

    # src = np.float32([                   # (1280, 720)
    #     (694.0, 240.0),
    #     (586.0, 240.0),
    #     (50.0, 475.0),
    #     (1230.0, 475.0)
    # ])

    src = np.float32([                   # Simulation (1280, 720)
        (694.0, 490.0),
        (586.0, 490.0),
        (50.0, 675.0),
        (1230.0, 675.0)
    ])

    src = np.float32([                   # Simulation 2 (1280, 720)
        (694.0, 368.0),
        (586.0, 368.0),
        (50.0, 675.0),
        (1230.0, 675.0)
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
    mtx, dist = load_camera_calib(sim=SIMULATION)
    warped = eye_bird_view(mask, mtx, dist, d=490)

    # _, mask_th = cv2.threshold(warped, 1, 255, cv2.THRESH_BINARY)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    res_morph = cv2.morphologyEx(warped, cv2.MORPH_CLOSE, kernel)
    line_edges = cv2.Canny(res_morph, 100, 100)
    if show:
        plt.imshow(line_edges)
        plt.show()
    return line_edges

def computing_lateral_distance(line_edges, y=Y_TEN_METERS, show=False):
    global LANE_PIXELS
    white_pixels = np.nonzero(line_edges[y, :])[0]
    if len(white_pixels) == 1:
        if white_pixels[0] > line_edges.shape[1]//2:
            x_coords_points = 0, white_pixels[0]
        else:
            x_coords_points = white_pixels[0], line_edges.shape[1]
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
        cv2.circle(line_edges, posm[::-1] , 2, (255, 255, 255), 1)
        plt.imshow(line_edges)
        plt.show()

    middle_image = line_edges.shape[1]//2
    lateral_distance = posm[1] - middle_image
    if not LANE_PIXELS:
        LANE_PIXELS = x_coords_points[1] - x_coords_points[0]       
        print(LANE_PIXELS)            
    scale_factor = LANE_METERS / LANE_PIXELS
    later_distance_meters = lateral_distance * scale_factor
    return later_distance_meters

    # scala_di_conversione = larghezza_strda_in_metri / larghezza_strada_in_pixel(bird eye view)
    # lateral_distance_in_metri = lateral_distance_in_pixel * scala_di_conversione  