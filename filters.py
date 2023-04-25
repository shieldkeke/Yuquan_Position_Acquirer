# -*- coding: utf-8 -*-
import cv2
import numpy as np


class KalmanFilter:
    '''4-dim state 2-dim output kf X, Y, Vx, Vy'''
    def __init__(self, init_state=[0,0,0,0]) -> None:
        self.kf = cv2.KalmanFilter(4, 2)
        # set measurement matrix and transition matrix
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        # set init state
        # self.kf.statePost = np.array(init_state)
        self.kf.statePost = np.array([[np.float32(init_state[0])], [np.float32(init_state[1])], [np.float32(init_state[2])], [np.float32(init_state[3])]], np.float32)
        self.kf.processNoiseCov = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32) * 0.03
        self.kf.measurementNoiseCov = np.array([[1, 0], [0, 1]], np.float32) *1 # 0.00003

    def update(self, coordX, coordY):
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        updated = self.kf.correct(measured)
        predicted = self.kf.predict()
        x_pre, y_pre = predicted[0][0], predicted[1][0]
        x, y = updated[0][0], updated[1][0]
        return x, y

class KalmanFilter_W:
    '''4-dim state 2-dim output kf yaw w'''
    def __init__(self, init_state=[0, 0]) -> None:
        self.kf = cv2.KalmanFilter(2, 1)
        # set measurement matrix and transition matrix
        self.kf.measurementMatrix = np.array([[1, 0]], np.float32)
        self.kf.transitionMatrix = np.array([[1, 1], [0, 1]], np.float32)
        # set init state
        # self.kf.statePost = np.array(init_state)
        self.kf.statePost = np.array([[np.float32(init_state[0])], [np.float32(init_state[1])]], np.float32)
        self.kf.processNoiseCov = np.array([[1, 0], [0, 1]], np.float32) * 0.03
        self.kf.measurementNoiseCov = np.array([[1]], np.float32) * 1 # 0.00003

    def update(self, yaw):
        measured = np.array([[np.float32(yaw)]])
        updated = self.kf.correct(measured)
        predicted = self.kf.predict()
        yaw_pre = predicted[0][0]
        yaw = updated[0][0]
        return yaw, yaw_pre
    
    
if __name__ == "__main__":
    kf = KalmanFilter([2,8,0,0])
    print(kf.update(2,8))
    print(kf.update(2,8))

    print(kf.update(2,8))
