#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from utils.camera_info import camera2lidar, lidar2camera
from utils.local_planner import get_cost_map

theta_y = 10.0*np.pi/180.

pitch_rotationMat = np.array([
    [np.cos(theta_y),  0., np.sin(theta_y)],
    [       0.,        1.,         0.     ],
    [-np.sin(theta_y), 0., np.cos(theta_y)],
])

pix_width = 0.05
map_x_min = 0.0
map_x_max = 10.0
map_y_min = -10.0
map_y_max = 10.0
lim_z = -1.5

width = int((map_x_max-map_x_min)/pix_width)
height = int((map_y_max-map_y_min)/pix_width)
u_bias = int(np.abs(map_y_max)/(map_y_max-map_y_min)*height)
v_bias = int(np.abs(map_x_max)/(map_x_max-map_x_min)*width)


def project(x, y):
    u = -x/pix_width + u_bias
    v = -y/pix_width + v_bias
    result = np.array([u, v])
    mask = np.where((result[0] < width) & (result[1] < height))
    result = result[:, mask[0]]
    return result.astype(np.int16)


def perspective_mapping():
    point = read_pcd("D:\documents\SRTP\\test\inputipm\\4.pcd")
    point2 = point[0]
    h = np.size(point2,0)
    l = np.size(point2,1)
    point_cloud = np.zeros((l,h))
    for i in range(h):
        for j in range(l):
            point_cloud[j][i] = point2[i][j]

    point_cloud = np.dot(pitch_rotationMat, point_cloud)
    img = lidar2camera(point_cloud)
    cv2.imwrite("D:\documents\SRTP\\test\\answeripm\ipmans1.jpg", img)

if __name__ == '__main__':
    input_img = cv2.imread("C:\Users\13910\Desktop\pm1.png")
   #perspective_mapping()