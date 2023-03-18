
import rospy
from gps_common.msg import GPSFix
from sensor_msgs.msg import Image as Img
from sensor_msgs.msg import PointCloud2
from scout_msgs.msg import ScoutStatus
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
import os

# import open3d as o3d
from PIL import Image 
import matplotlib.pyplot as plt
import numpy as np
import math
import cv2

### if we draw global nav to vertify our gps module and data 
# img = Image.open('map.png')
fig, ax = plt.subplots()
line, = ax.plot([], [])  # empty line
point, = ax.plot([], [], 'bo')

xs = []
ys = []

### real world GPS coordinate corresponding to the pixel in "map.png"

left_up_gps = {"x": 120.11807369666667, "y": 30.263599648333336}
right_down_gps = {"x": 120.11863341150001, "y": 30.261879024000002}
left_up_pic = {"x": 425, "y": 566}
right_down_pic = {"x": 455, "y": 678}

left_down_gps = {"x": 120.11749962616665, "y": 30.262226684333335}
right_up_gps = {"x": 120.11923214966667, "y": 30.2632707255}
left_down_pic = {"x": 387, "y": 652}
right_up_pic = {"x": 489, "y": 590}

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
    
    def dis(self, other):
        x1, y1 = self.gps2xy_ellipse()
        x2, y2 = other.gps2xy_ellipse()
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)
    
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

    def __init__(self, save_path="/home/chenyeke/srtp/data/") -> None:
        self.global_img = []
        self.global_nav = []
        self.global_time = []
        self.global_scan = []
        self.global_pos = [0, 0, 0] # x, y, yaw
        self.global_vel = [0, 0, 0] # x, y, angle_vel
        self.global_acc = [0, 0, 0] #x, y
        self.time = 0
        self.gps_right_down = GPSItem(right_down_gps["x"], right_down_gps["y"])
        self.gps_left_up = GPSItem(left_up_gps["x"], left_up_gps["y"])
        self.save_path = save_path
        self.bridge = CvBridge() 
        self.mkdir('')
        self.mkdir('img/')
        self.mkdir('pcd/')
        self.mkdir('nav/')
        self.mkdir('state/')
        self.pos_file = open(save_path+'state/pos.txt', 'w+')
        self.vel_file = open(save_path+'state/vel.txt', 'w+')
        self.acc_file = open(save_path+'state/acc.txt', 'w+')
        self.angular_vel_file = open(save_path+'state/angular_vel.txt', 'w+')

        self.listener()
        # plt.imshow(img, animated= True)
        # plt.savefig("gps.png")
        # plt.show()
    
    def __del__(self):
        self.pos_file.close()
        self.vel_file.close()
        self.acc_file.close()
        self.angular_vel_file.close()
        cv2.destroyAllWindows()

    def mkdir(self, path):
        os.makedirs(self.save_path+path, exist_ok=True)

    def save_data(self):
        index = str(self.global_time)
        cv2.imwrite(self.save_path+'img/'+index+'.png', self.global_img)
        cv2.imwrite(self.save_path+'nav/'+index+'.png', self.global_nav)
        np.save(self.save_path+'pcd/'+str(self.global_time)+'.npy', self.global_scan)
        self.pos_file.write(index+'\t'+
                   str(self.global_pos[0])+'\t'+ # x
                   str(self.global_pos[1])+'\t'+ # y
                   str(self.global_pos[2])+'\t'+'\n') #yaw
        self.vel_file.write(index+'\t'+
                    str(self.global_vel[0])+'\t'+
                    str(self.global_vel[1])+'\t'+'\n')
        self.acc_file.write(index+'\t'+
                    str(self.global_acc[0])+'\t'+
                    str(self.global_acc[1])+'\t'+'\n')
        self.angular_vel_file.write(index+'\t'+
                    str(self.global_vel[2])+'\t'+'\n')
        
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
   
    def get_nav(self, gps):
        img = Image.open('navigation.png')
        map = Image.open('map.png')

        x_gps, y_gps = gps.gps2xy_ellipse()
        left_up_x, left_up_y = self.gps_left_up.gps2xy_ellipse()
        right_down_x, right_down_y = self.gps_right_down.gps2xy_ellipse()
        center_x = int((x_gps - left_up_x) / (right_down_x - left_up_x) * (right_down_pic["x"] - left_up_pic["x"]) + left_up_pic["x"])
        center_y = int((y_gps - left_up_y) / (right_down_y - left_up_y) * (right_down_pic["y"] - left_up_pic["y"]) + left_up_pic["y"])
        
        center_x = center_x * img.width / map.width
        center_y = center_y * img.height/ map.height
        img = img.rotate(angle=self.global_pos[2], center=(center_x, center_y))
        
        cut_len = 20 # we cut +-20meters
        cut_range =  cut_len * (math.sqrt((right_down_pic["x"] - left_up_pic["x"])**2 + (right_down_pic["y"] - left_up_pic["y"])**2)/(self.gps_right_down.dis(self.gps_left_up))) #20 meters * (pixels per meter in "map.png")
        pic_cut_range = int(cut_range*img.height/map.height)

        #we ensure the height of the picture equals to 20 meters ()i n the real world
        #but we arbitrary scaled the width (use coefficient k)of the picture for rationality of nav pic
        k = 0.8
        y_offset = 40

        img = img.crop(box=(center_x-pic_cut_range*k, center_y-pic_cut_range-y_offset, center_x+pic_cut_range*k, center_y-y_offset))
        img = img.resize((300, 240), Image.ANTIALIAS)
        self.global_nav = img

    def get_time(self, data):
        self.global_time = float(data.header.stamp.secs) + float(data.header.stamp.nsecs)/1000000000

    def gps_callback(self, data):
        gps = GPSItem(data.longitude, data.latitude)
        # self.draw_nav(gps)
        global_x, global_y = gps.gps2xy_ellipse() # world coordinate
        dx = global_x - self.global_pos[0]
        dy = global_y - self.global_pos[1]
        global_yaw = math.atan2(dy, dx)
        self.global_pos = [global_x, global_y, global_yaw]
        self.get_time(data)
        self.get_nav(gps)
        self.save_data()
    
    def lidar_callback(self, data):
        pc = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
        self.global_scan = np.array(list(pc))

    def status_callback(self, data):
        # because our car is Four-wheel differentia
        vel = data.linear_velocity
        self.global_acc[0] = vel * math.cos(self.global_pos[2]) - self.global_vel[0]
        self.global_acc[1] = vel * math.sin(self.global_pos[2]) - self.global_vel[1]
        self.global_vel[0] = vel * math.cos(self.global_pos[2]) # yaw
        self.global_vel[1] = vel * math.sin(self.global_pos[2])
        self.global_vel[2] = data.angular_velocity
        
    def img_callback(self, data):
        self.global_img = self.bridge.imgmsg_to_cv2(data, "bgr8")

    def listener(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("jzhw/gps/fix", GPSFix, self.gps_callback)
        rospy.Subscriber("camera/color/image_raw", Img, self.img_callback)
        rospy.Subscriber("os_cloud_node/points",PointCloud2, self.lidar_callback)
        rospy.Subscriber("scout_status",ScoutStatus, self.lidar_callback)
        rospy.spin()

if __name__ == '__main__':

    # test for GPS module

    # p1 = GPSItem(left_up_gps["x"], left_up_gps["y"])
    # p2 = GPSItem(right_down_gps["x"], right_down_gps["y"])
    # p3 = GPSItem(left_down_gps["x"], left_down_gps["y"])
    # p4 = GPSItem(right_up_gps["x"], right_up_gps["y"])
    # print(p1.dis(p3) + p3.dis(p2) + p2.dis(p4) + p4.dis(p1))

    RealDataSaver()