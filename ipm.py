#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import os
from tqdm import tqdm

fx = 711.642238
fy = 711.302135
s = 0.0
x0 = 644.942373
y0 = 336.030580

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

cameraMat = np.array([
        [fx,  s, x0],
        [0., fy, y0],
        [0., 0., 1.]
])

theta_y = 10.0*np.pi/180.

pitch_rotationMat = np.array([
    [np.cos(theta_y),  0., np.sin(theta_y)],
    [       0.,        1.,         0.     ],
    [-np.sin(theta_y), 0., np.cos(theta_y)],
])

rotationMat = np.array([
    [0., -1., 0.],
    [0.,  0., -1.],
    [1.,  0., 0.],
])
translationMat = np.array([0.0, 0.0, 0.0])

theta_x = np.arctan2(rotationMat[2][1], rotationMat[2][2])

rotationMat = np.dot(rotationMat, np.linalg.inv(pitch_rotationMat))


def camera2lidar(image_uv):
    rotation = np.linalg.inv(np.dot(cameraMat, rotationMat))
    translation = np.dot(cameraMat, translationMat)
    translation = np.dot(rotation, translation)
    R = rotation
    T = translation
    roadheight = -1.3

    u = image_uv[0]
    v = image_uv[1]

    zi = (T[2]+roadheight)/(R[2][0]*u + R[2][1]*v + R[2][2])
    xl = (R[0][0]*u + R[0][1]*v + R[0][2])*zi - T[0]
    yl = (R[1][0]*u + R[1][1]*v + R[1][2])*zi - T[1]
    trans_pc = np.array([
            xl,
            yl,
            [roadheight]*image_uv.shape[1]
            ])
    return trans_pc


def project(x, y):
    u = -x/pix_width + u_bias
    v = -y/pix_width + v_bias
    result = np.array([u, v])
    mask = np.where((result[0] < width) & (result[1] < height))
    result = result[:, mask[0]]
    return result.astype(np.int16)


def inverse_perspective_mapping(data):
    data = cv2.resize(data, (1280, 720), interpolation=cv2.INTER_AREA)
    res = np.where(data > 100)
    data = np.stack([res[1], res[0]])
    trans_pc = camera2lidar(data)

    img = np.zeros((width, height, 1), np.uint8)
    img.fill(0)
    res = np.where(
        (trans_pc[0] > map_x_min) & (trans_pc[0] < map_x_max) & (trans_pc[1] > map_y_min) & (trans_pc[1] < map_y_max))
    trans_pc = trans_pc[:, res[0]]
    u, v = project(trans_pc[0], trans_pc[1])
    img[u, v] = 255
    kernel = np.ones((5, 5), np.uint8)
    img = cv2.dilate(img, kernel, iterations=1)
    img = cv2.erode(img, kernel, iterations=1)
    return img


def mkdir(path):
    os.makedirs(path, exist_ok=True)


if __name__ == '__main__':
    save_path = "C:/Users/13910/Desktop/Yuquan_Position_Acquirer-master/"
    files = os.listdir(save_path + "pm")

    mkdir(save_path + "ipm")
    for f in tqdm(files):
        input_img = cv2.imread(save_path + "pm/" + f)
        img = inverse_perspective_mapping(input_img)
        cv2.imwrite(save_path + 'ipm/' + f[:-4] + '.png', img)