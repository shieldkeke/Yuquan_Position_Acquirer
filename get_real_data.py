
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
    
    def gps2xy(self):
    
        latitude = self.latitude * math.pi/180
        longtitude = self.longtitude *math.pi/180

        #the radius of the equator
        radius = 6378137
        #distance of the two poles
        distance = 6356752.3142

        #reference
        base = 30.26 * math.pi/180

        radius_square = pow(radius,2)
        distance_square = pow(distance,2)

        e = math.sqrt(1 - distance_square/radius_square)
        e2 = math.sqrt(radius_square/distance_square - 1)

        cosb0 = math.cos(base)
        N = (radius_square / distance) / math.sqrt( 1+ pow(e2,2)*pow(cosb0,2))
        K = N*cosb0

        sinb = math.sin(latitude)
        tanv = math.tan(math.pi/4 + latitude/2)
        E2 = pow((1 - e*sinb) / (1+ e* sinb),e/2)
        xx = tanv * E2

        xc = K * math.log(xx)
        yc = K * longtitude
        return xc,yc

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

        x_gps, y_gps = gps.gps2xy()
        left_up_x, left_up_y = self.gps_left_up.gps2xy()
        right_down_x, right_down_y = self.gps_right_down.gps2xy()

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