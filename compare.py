from PIL import Image

Image.MAX_IMAGE_PIXELS = None
import matplotlib.pyplot as plt
import math
from tqdm import tqdm

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

    def gps2xy_ellipse(self):  # ellipsoid

        ref_lon = 120.11 * math.pi / 180
        ref_lat = 30.26 * math.pi / 180
        lon = self.x * math.pi / 180
        lat = self.y * math.pi / 180

        MACRO_AXIS = 6378137
        MINOR_AXIS = 6356752
        a = MACRO_AXIS ** 2
        b = MINOR_AXIS ** 2
        c = math.tan(ref_lat) ** 2
        d = (1 / math.tan(ref_lat)) ** 2
        x = a / math.sqrt(a + b * c)
        y = b / math.sqrt(b + a * d)
        c = math.tan(lat) ** 2
        d = (1 / math.tan(lat)) ** 2
        m = a / math.sqrt(a + b * c)
        n = b / math.sqrt(b + a * d)

        y_c = math.sqrt((x - m) ** 2 + (y - n) ** 2)

        c = math.tan(ref_lat) ** 2
        x_c = a / math.sqrt(a + b * c) * (lon - ref_lon)

        return x_c, y_c

class EKFItem:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class RealDataDrawer:
    def __init__(self, type="gps") -> None:
        self.gps_right_down = GPSItem(right_down_gps["x"], right_down_gps["y"])
        self.gps_left_up = GPSItem(left_up_gps["x"], left_up_gps["y"])

        self.type = type
        self.traj = []
        self.xs = []
        self.ys = []

    def open_file(self, path):
        pos_file = open(path, "r")
        while True:
            line = pos_file.readline()
            if not line:
                break
            line = line.split()
            if self.type == "gps":
                t = GPSItem(eval(line[1]), eval(line[2]))
            elif self.type == "ekf":
                t = EKFItem(eval(line[1]), eval(line[2]))
            self.traj.append(t)

    def draw_nav(self):  # draw global nav img
        print(self.type + " is drawing")
        for point in tqdm(self.traj):
            if self.type == "gps":
                x_gps, y_gps = point.gps2xy_ellipse()
            elif self.type == "ekf":
                x_gps, y_gps = point.x, point.y
            left_up_x, left_up_y = self.gps_left_up.gps2xy_ellipse()
            right_down_x, right_down_y = self.gps_right_down.gps2xy_ellipse()

            center_x = (x_gps - left_up_x) / (right_down_x - left_up_x) * (right_down_pic["x"] - left_up_pic["x"]) + left_up_pic["x"]
            center_y = (y_gps - left_up_y) / (right_down_y - left_up_y) * (right_down_pic["y"] - left_up_pic["y"]) + left_up_pic["y"]
            self.xs.append(center_x)
            self.ys.append(center_y)
        print("point load complete")


if __name__ == '__main__':
    # test for GPS module
    path = "data/"
    img = Image.open(path + 'map.png')
    gps_drawer = RealDataDrawer(type="gps")
    ekf_drawer = RealDataDrawer(type="ekf")

    gps_drawer.open_file(path + "gps.txt")
    ekf_drawer.open_file(path + "ekf.txt")

    gps_drawer.draw_nav()
    ekf_drawer.draw_nav()

    new_fig, new_ax = plt.subplots(figsize=(img.width / 100, img.height / 100))  # make it proportional divided by 100 because Large numbers are not valid
    # new_ax.set(xlim=[0, img.width], ylim=[img.height, 0])
    # plt.imshow(img)
    plt.axis('off')
    new_ax.plot([x for x in gps_drawer.xs], [y for y in gps_drawer.ys], color='red', linewidth=1, label="gps data")
    new_ax.plot([x for x in ekf_drawer.xs], [y for y in ekf_drawer.ys], color='blue', linewidth=1, label="ekf data")
    plt.legend()
    plt.show()
