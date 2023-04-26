import numpy as np
import math
from sensor_msgs.msg import PointCloud2, Imu
from gps_common.msg import GPSFix
import rospy
import os

### real world GPS coordinate corresponding to the pixel in "map.png"

left_up_gps = {"x": 120.11807369666667, "y": 30.263599648333336}
right_down_gps = {"x": 120.11863341150001, "y": 30.261879024000002}
left_up_pic = {"x": 425, "y": 566}
right_down_pic = {"x": 455, "y": 678}

left_down_gps = {"x": 120.11749962616665, "y": 30.262226684333335}
right_up_gps = {"x": 120.11923214966667, "y": 30.2632707255}
left_down_pic = {"x": 387, "y": 652}
right_up_pic = {"x": 489, "y": 590}


class GPSItem:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.longtitude = x
        self.latitude = y
        
    def data(self):
        return self.x, self.y
    
    def dis(self, other):
        x1, y1 = self.gps2xy_ellipse()
        x2, y2 = other.gps2xy_ellipse()
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)
    
    def gps2xy_ellipse(self): #ellipsoid

        ref_lon = 120.11 * math.pi / 180
        ref_lat = 30.26 * math.pi / 180
        lon = self.x * math.pi / 180
        lat = self.y * math.pi / 180
        
        MACRO_AXIS = 6378137
        MINOR_AXIS = 6356752
        a = MACRO_AXIS ** 2
        b = MINOR_AXIS ** 2
        c = math.tan(ref_lat) ** 2
        d = (1/math.tan(ref_lat)) ** 2
        x = a / math.sqrt(a + b*c)
        y = b / math.sqrt(b + a*d)
        c = math.tan(lat) ** 2
        d = (1/math.tan(lat)) ** 2
        m = a / math.sqrt(a + b*c)
        n = b / math.sqrt(b + a*d)

        y_c = math.sqrt((x - m)**2 + (y - n)**2)

        c = math.tan(ref_lat) ** 2
        x_c = a/math.sqrt(a + b*c) * (lon - ref_lon)

        return x_c, y_c

class location_ekf():
    def __init__(self):
        """ merge gps and imu data """

        # State Equation Update Rule
        # x + v/ψ˙(−sin(ψ) + sin(dtψ˙+ψ))
        # y + v/ψ˙(cos(ψ) − cos(dtψ˙+ψ))
        # dtψ˙+ ψ
        # dta + v

        self.X = []
        self.P = np.eye(4) * 0.1 # state covariance
        self.M = np.array([[0.005,0],[0,0.01]]) # motion noise w, a
        # self.Q = np.eye(3) * 0.05 # measurement noise
        self.Q = np.eye(2) * 0.05 # measurement noise
        self.init_flag = False

    def init(self, x, y, t, v):
        self.X = np.array([x, y, t, v])
        self.init_flag = True

    def predict(self, u, dt):

        if not self.init_flag:
            return
        
        u = np.array(u)
        x, y, t, v = self.X
        w, a = u
        
        # state prediction
        if w == 0:
            self.X = np.array([x + v * math.cos(t) * dt,
                               y + v * math.sin(t) * dt,
                               t,
                               v + a*dt])
        else:
            self.X = np.array([x + v/w * (math.sin(t + w*dt) - math.sin(t)),
                               y + v/w * (-math.cos(t + w*dt) + math.cos(t)),
                               t + w*dt,
                               v + a*dt])

        # jacobi matrix of state , F = dF/dx
        if w == 0:
            G = np.array([[1, 0, -v * math.sin(t) * dt, math.cos(t) * dt],
                           [0, 1, v * math.cos(t) * dt,  math.sin(t) * dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
        else:
            G = np.array([[1, 0, -v/w * math.cos(t) + v/w * math.cos(t + w*dt), - math.sin(t)/w + math.sin(t + w*dt)/w],
                           [0, 1, -v/w * math.sin(t) + v/w * math.sin(t + w*dt), math.cos(t)/w - math.cos(t + w*dt)/w],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])    

        # jacobi matrix of motion noise , V = dF/du    
        if w == 0:
            V = np.array([[-0.5*dt**2*math.sin(t), 0],
                          [ 0.5*dt**2*math.cos(t), 0],                      
                          [dt, 0],
                          [0, dt]])
        else:
            V = np.array([[v/w**2 * (math.sin(t) - math.sin(t + w*dt)) + v/w * math.cos(t + w*dt) * dt, 0],
                          [-v/w**2 * (math.cos(t) - math.cos(t + w*dt)) + v/w * math.sin(t + w*dt) * dt, 0],                      
                          [dt, 0],
                          [0, dt]])
        
        R = V @ self.M @ V.T
        
        # state convariance prediction
        self.P = G @ self.P @ G.T + R

    def update(self, z):
        # measurement update
        z = np.array(z)
        x, y, t, v = self.X
        x_, y_ = z
        H = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0]])
        # x_, y_ ,t_, = z
        # H = np.array([[1, 0, 0, 0],
        #               [0, 1, 0, 0],
        #               [0, 0, 1, 0]])
        K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + self.Q)
        self.X = self.X + K @ (z - H @ self.X)
        self.P = (np.eye(4) - K @ H) @ self.P

    def get_state(self):
        return self.X

def get_time(data):
    t = float(data.header.stamp.secs) + float(data.header.stamp.nsecs)/1000000000
    return t

def gps_callback(data):
    global pos
    gps = GPSItem(data.longitude, data.latitude)
    x, y = gps.gps2xy_ellipse() # world coordinate
    dx = x - pos[0]
    dy = y - pos[1]
    v = math.sqrt(dx**2 + dy**2)
    gps_yaw = math.atan2(dy, dx)

    if not ekf.init_flag and pos != [0,0,0] and v > 0.05:
        ekf.init(x, y, gps_yaw, v)
    
    if ekf.init_flag:
        # ekf.update([x, y, gps_yaw])
        ekf.update([x, y])
        pos = ekf.get_state()[:3]
    else:
        pos = [x, y, gps_yaw]

    t = get_time(data)
    gps_file.write(f"{t:7f} {x} {y} {gps_yaw} {v}\n")
    ekf_file.write(f"{t:7f} {pos[0]} {pos[1]} {pos[2]}\n")

def imu_callback(data):
    global last_t
    t = get_time(data)
    if last_t == 0:
        last_t = t
        return
    dt = t - last_t
    last_t = t
    ekf.predict([data.angular_velocity.z, data.linear_acceleration.x], dt)
    imu_file.write(f"{t:7f} {data.angular_velocity.z} {data.linear_acceleration.x}\n")

def mkdir(path):
    os.makedirs(save_path+path, exist_ok=True)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("jzhw/gps/fix", GPSFix, gps_callback)
    rospy.Subscriber("os_cloud_node/imu",Imu, imu_callback)
    print("successfully initialized")
    rospy.spin()

def open_file():
    gps_file = open(gps_path, "w")
    imu_file = open(imu_path, "w")
    ekf_file = open(ekf_path, "w")
    return gps_file, imu_file, ekf_file

def close_file():
    gps_file.close()
    imu_file.close()
    ekf_file.close()

if __name__ == "__main__":

    ONLINE = True
    save_path = "../data5/"
    mkdir("state")
    imu_path = save_path + "state/imu.txt"
    gps_path = save_path + "state/gps.txt"
    ekf_path = save_path + "state/ekf.txt"
    gps_file, imu_file, ekf_file = open_file()
    ekf = location_ekf()

    global pos, last_t
    last_t = 0
    pos = [0,0,0]

    listener()
    close_file()

