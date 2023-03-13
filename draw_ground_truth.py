
import rospy
from gps_common.msg import GPSFix
from PIL import Image
import matplotlib.pyplot as plt
import threading

img = Image.open('map.png')
fig, ax = plt.subplots()
line, = ax.plot([], [])  # empty line
point, = ax.plot([], [], 'bo')

xs = []
ys = []

# left_up_gps = {"x": 120.129917, "y": 30.272621}
# right_down_gps = {"x": 120.133689, "y": 30.266071}
# left_up_pic = {"x": 457, "y": 248}
# right_down_pic = {"x": 667, "y": 669}

left_up_gps = {"x": 120.11807369666667, "y": 30.263599648333336}
right_down_gps = {"x": 120.11863341150001, "y": 30.261879024000002}
left_up_pic = {"x": 425, "y": 566}
right_down_pic = {"x": 455, "y": 678}

# 120.11923214966667 30.2632707255 489 590 right up
# 120.11749962616665 30.262226684333335 387 652 left down
# 120.11863341150001 30.261879024000002 455 678 right down
# 120.11807369666667 30.263599648333336 425 566 left up
class GPS_item:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def draw(gps):
    center_x = int((gps.x-left_up_gps["x"])/(right_down_gps["x"]-left_up_gps["x"]) * (right_down_pic["x"]-left_up_pic["x"]) + left_up_pic["x"])
    center_y = int((gps.y - left_up_gps["y"]) / (right_down_gps["y"] - left_up_gps["y"]) * (right_down_pic["y"] - left_up_pic["y"]) + left_up_pic["y"])
    # print(center_x, center_y)
    xs.append(center_x)
    ys.append(center_y)
    line.set_data(xs, ys)
    line.figure.canvas.draw()

def callback(data):
    # rospy.loginfo(rospy.get_caller_id()+"I heard %s",data)
    # print(data.longitude, data.latitude)
    draw(GPS_item(data.longitude, data.latitude))

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("jzhw/gps/fix", GPSFix, callback)
    rospy.spin()

if __name__ == '__main__':
    ros_thread = threading.Thread(target=listener())
    ros_thread.start()
    plt.imshow(img, animated= True)
    plt.show()