
import rospy
from gps_common.msg import GPSFix
from sensor_msgs.msg import Image as Img
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge

# import open3d as o3d
from PIL import Image 
import matplotlib.pyplot as plt
import numpy as np
import math
import cv2


bridge = CvBridge() 
img = Image.open('map.png')
fig, ax = plt.subplots()
line, = ax.plot([], [])  # empty line
point, = ax.plot([], [], 'bo')

xs = []
ys = []

left_up_gps = {"x": 120.11807369666667, "y": 30.263599648333336}
right_down_gps = {"x": 120.11863341150001, "y": 30.261879024000002}
left_up_pic = {"x": 425, "y": 566}
right_down_pic = {"x": 455, "y": 678}

# 120.11923214966667 30.2632707255 489 590 right up
# 120.11749962616665 30.262226684333335 387 652 left down
# 120.11863341150001 30.261879024000002 455 678 right down
# 120.11807369666667 30.263599648333336 425 566 left up

class GPSItem:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.longtitude = x
        self.latitude = y
        
    def data(self):
        return self.x, self.y
    
    def gps2xy(self): #sphere
        
        # x:north y:east
        
        CONSTANTS_RADIUS_OF_EARTH = 6371000

        ref_lon = left_up_gps["x"]
        ref_lat = left_up_gps["y"]
        
        lon = self.x
        lat = self.y

        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        ref_lat_rad = math.radians(ref_lat)
        ref_lon_rad = math.radians(ref_lon)

        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)
        ref_sin_lat = math.sin(ref_lat_rad)
        ref_cos_lat = math.cos(ref_lat_rad)

        cos_d_lon = math.cos(lon_rad - ref_lon_rad)

        arg = np.clip(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon, -1.0, 1.0)
        c = math.acos(arg)

        k = 1.0
        if abs(c) > 0:
            k = (c / math.sin(c))

        x = float(k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH)
        y = float(k * cos_lat * math.sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH)

        return y,x 
    
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

class RealDataSaver:

    def __init__(self) -> None:
        self.global_img = []
        self.global_gps = []
        self.global_time = []
        self.time = 0
        self.gps_right_down = GPSItem(right_down_gps["x"], right_down_gps["y"])
        self.gps_left_up = GPSItem(left_up_gps["x"], left_up_gps["y"])


        self.listener()
        plt.imshow(img, animated= True)
        plt.savefig("gps.png")
        plt.show()
        

    def save_data(self):
        pass
    
    def draw_nav(self, gps): #draw global nav img

        # x_gps, y_gps = gps.data()
        # left_up_x, left_up_y = self.gps_left_up.data()
        # right_down_x, right_down_y = self.gps_right_down.data()

        x_gps, y_gps = gps.gps2xy_ellipse()
        left_up_x, left_up_y = self.gps_left_up.gps2xy_ellipse()
        right_down_x, right_down_y = self.gps_right_down.gps2xy_ellipse()

        center_x = int((x_gps - left_up_x) / (right_down_x - left_up_x) * (right_down_pic["x"] - left_up_pic["x"]) + left_up_pic["x"])
        center_y = int((y_gps - left_up_y) / (right_down_y - left_up_y) * (right_down_pic["y"] - left_up_pic["y"]) + left_up_pic["y"])
        xs.append(center_x)
        ys.append(center_y)
        line.set_data(xs, ys)
        line.figure.canvas.draw()

    def get_time(self, data):
        self.global_time = data.header.stamp.secs

    def gps_callback(self, data):
        self.draw_nav(GPSItem(data.longitude, data.latitude))
        self.global_gps = [data.longitude, data.latitude]
        self.get_time(data)
        self.save_data()
    
    def lidar_callback(self, data):
        pc = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
        self.global_scan = np.array(list(pc))
        # np.save("1.npy", self.global_scan)
        # o3d.io.write_point_cloud("1.pcd",pc2.read_points(data))

    def img_callback(self, data):
        self.global_img = bridge.imgmsg_to_cv2(data, "bgr8")
        # cv2.imwrite("1.jpg",self.global_img)

    def listener(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("jzhw/gps/fix", GPSFix, self.gps_callback)
        rospy.Subscriber("camera/color/image_raw", Img, self.img_callback)
        rospy.Subscriber("os_cloud_node/points",PointCloud2, self.lidar_callback)
        rospy.spin()

if __name__ == '__main__':
    RealDataSaver()