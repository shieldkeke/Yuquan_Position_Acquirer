import numpy as np
import cv2

width = 1280
height = 720
fx = 711.642238
fy = 711.302135
s = 0.0
x0 = 644.942373
y0 = 336.030580

vehicle_width = 2.0
half_vehicle_width = vehicle_width / 2

cameraMat = np.array([
        [fx,  s, x0],
        [0., fy, y0],
        [0., 0., 1.]
])
rotationMat = np.array([
    [0.,     0.,   1.],
    [-1.,    0.,   0.],
    [0.,    -1.,   0.],
])

theta_y = -10.0*np.pi/180.
roll_rotationMat = np.array([
    [1.,              0.,               0.],
    [0., np.cos(theta_y), -np.sin(theta_y)],
    [0., np.sin(theta_y),  np.cos(theta_y)],
])
rotationMat = np.dot(rotationMat, roll_rotationMat)
rotationMat = np.linalg.inv(rotationMat)
# print(f"rotation: {rotationMat}")
translationMat = np.array([0., 1.3, 0.])


def pi2pi(theta, theta0=0.0):
	while(theta > np.pi + theta0):
		theta = theta - 2.0 * np.pi
	while(theta < -np.pi + theta0):
		theta = theta + 2.0 * np.pi
	return theta


class Pose(object):
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw


# get the pm img
def car2camera(point, rotationMat=rotationMat, translationMat=translationMat):
    point_matrix = np.array([point.x, point.y, 0.]).T
    trans_pc = np.dot(rotationMat, point_matrix) + translationMat.T
    image_uv = (trans_pc[0] * fx / trans_pc[2] + x0, trans_pc[1] * fy / trans_pc[2] + y0 )
    point = (int(image_uv[0]), int(image_uv[1]))
    return point


# change the coordinate
def world2car(pos, future):
    t_matrix = np.array([[np.cos(pos.yaw), -np.sin(pos.yaw), pos.x],
                         [np.sin(pos.yaw),  np.cos(pos.yaw), pos.y],
                         [              0,                0,     1]])
    t_matrix = np.linalg.inv(t_matrix)
    new_traj = []

    for point in future:
        point_matrix = np.array([point.x, point.y, 1.])
        point_matrix = np.dot(t_matrix, point_matrix)

        new_pose = Pose(point_matrix[0], point_matrix[1], point.yaw-pos.yaw)
        new_traj.append(new_pose)

    return new_traj


# get the traj whose length is ??m
def cut_traj(start_index, traj, length=20):
    dis = 0.
    new_traj = [traj[start_index]]
    for i in range(start_index, len(traj)-1):
        dis = dis + np.sqrt((traj[i].x - traj[i+1].x)**2 + (traj[i].y - traj[i+1].y)**2)
        if dis > length:
            return new_traj
        new_traj.append(traj[i+1])
    return new_traj


def data_augmentation(traj):
    result_list = []
    for i in range(len(traj) - 1):
        p1 = traj[i]
        p2 = traj[i+1]
        if i / len(traj) < 0.4:
            min_dist = 0.008
        elif i / len(traj) < 0.6:
            min_dist = 0.016
        else:
            min_dist = 0.08
        _result_list = getLinearPose(p1, p2, min_dist)
        result_list.extend(_result_list)

    return result_list


def getLinearPose(pose1, pose2, min_dist):
    x1, x2 = pose1.x, pose2.x
    y1, y2 = pose1.y, pose2.y
    yaw1, yaw2 = pose1.yaw, pose2.yaw
    dis = np.sqrt((pose1.x - pose2.x) ** 2 + (pose1.y - pose2.y) ** 2)
    total = int(dis / min_dist)
    tt = np.arange(total) / total

    x, y = tt * x2 + (1 - tt) * x1, tt * y2 + (1 - tt) * y1
    yaw = pi2pi(yaw2 - yaw1) * tt + yaw1

    new_pose_list = []
    for i in range(total):
        new_pose = Pose(x[i], y[i], yaw[i])
        new_pose_list.append(new_pose)

    return new_pose_list


def open_pos(path):
    pos_file = open(path, "r")
    traj = []
    t = []
    while True:
        line = pos_file.readline()
        if not line:
            break
        line = line.split()
        pose = Pose(eval(line[1]), eval(line[2]), eval(line[3]))
        traj.append(pose)
        t.append(line[0])
    return t, traj


def limit(num, num_min, num_max):
    if num_min > num_max:
        num_min, num_max = num_max, num_min
    if num < num_min:
        return num_min
    elif num > num_max:
        return num_max
    else:
        return num


def drawLineInImage(car_pos, img):
    car_vec = np.array([car_pos.x, car_pos.y])
    theta = car_pos.yaw + np.pi/2
    start_vec = np.array([np.cos(theta)*half_vehicle_width, np.sin(theta)*half_vehicle_width]) + car_vec
    theta = car_pos.yaw - np.pi/2
    end_vec = np.array([np.cos(theta) * half_vehicle_width, np.sin(theta) * half_vehicle_width]) + car_vec
    start = Pose(start_vec[0], start_vec[1], 0)
    end = Pose(end_vec[0], end_vec[1], 0)
    start = car2camera(start)
    end = car2camera(end)

    # if start[0]>width or start[0]<0 or start[1]>height or start[1]<0 or end[0]>width or end[0]<0 or end[1]>height or end[1]<0:
    #     return
    if (start[0]>width and end[0]>width) or (start[1]>height and end[1]>height) or (start[0]<0 and end[0]<0) or (start[1]<0 and end[1]<0):
        return
    else:
        start = (limit(start[0], 0, width), limit(start[1], 0, height))
        end = (limit(end[0], 0, width), limit(end[1], 0, height))

    cv2.line(img, start, end, (255, 255, 255), thickness=2)


def getPM(traj):
    img = np.zeros((height, width, 3), np.uint8)
    for point in traj:
        drawLineInImage(point, img)

    kernel = np.ones((6, 6), np.uint8)
    img = cv2.dilate(img, kernel, iterations=1)
    img = cv2.erode(img, kernel, iterations=1)
    img = cv2.resize(img, (width // 2, height // 2), interpolation=cv2.INTER_CUBIC)
    img = cv2.resize(img, (width, height), interpolation=cv2.INTER_CUBIC)
    return img


if __name__ == '__main__':
    file_path = "pos.txt"
    t, traj = open_pos(file_path)

    for i in range(len(t)):
        path = cut_traj(i, traj)
        path = world2car(path[0], path[1:])

        path = data_augmentation(path)
        img = getPM(path)

        cv2.imwrite('pm\\'+ t[i] + '.png', img)
