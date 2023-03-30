# -*- coding: utf-8 -*-
import cv2
import numpy as np


class KalmanFilter:
    '''4-dim state 2-dim output kf'''
    def __init__(self, init_state=[0,0,0,0]) -> None:
        self.kf = cv2.KalmanFilter(4, 2)
        self.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.kf.statePost =  np.array(init_state, np.float32)

    def update(self, coordX, coordY):
        measured = np.array([[np.float32(coordX)], [np.float32(coordY)]])
        updated = self.kf.correct(measured)
        predicted = self.kf.predict()
        x_pre, y_pre = predicted[0], predicted[1]
        x, y = updated[0], updated[1]
        return x, y
    
    
if __name__ == "__main__":
    kf = KalmanFilter([2,8,0,0])
    print(kf.update(2,8))
    print(kf.update(2,8))

    print(kf.update(2,8))
