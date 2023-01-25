from PIL import Image
from math import *

left_up_gps = {"x": 120.129917, "y": 30.272621}
right_down_gps = {"x": 120.133689, "y": 30.266071}
left_up_pic = {"x": 457, "y": 248}
right_down_pic = {"x": 667, "y": 669}

'''单位换算部分'''
real_meters = 20
earth_R = 6378137
theta = acos(cos(left_up_gps["y"]*pi/180)*cos(right_down_gps["y"]*pi/180)*cos(left_up_gps["x"]*pi/180-right_down_gps["x"]*pi/180)+sin(left_up_gps["y"]*pi/180)*sin(right_down_gps["y"]*pi/180))
# print(earth_R*theta)
rdis = ((left_up_pic["x"]-right_down_pic["x"])**2+(left_up_pic["y"]-right_down_pic["y"])**2)**0.5/(earth_R*theta)
cut_range = int(rdis*real_meters)
# print(cut_range)
y_offset = 26


def cut_navigation(gps, yaw):
    img = Image.open('navigation.png')
    center_x = int((gps.x-left_up_gps["x"])/(right_down_gps["x"]-left_up_gps["x"]) * (right_down_pic["x"]-left_up_pic["x"]) + left_up_pic["x"])
    center_y = int((gps.y - left_up_gps["y"]) / (right_down_gps["y"] - left_up_gps["y"]) * (right_down_pic["y"] - left_up_pic["y"]) + left_up_pic["y"])
    center_x = center_x * img.width / 829
    center_y = center_y * img.height /755
    img = img.rotate(angle=yaw, center=(center_x, center_y))
    pic_cut_range = int(cut_range*img.height/755)
    print(pic_cut_range)
    img = img.crop(box=(center_x-pic_cut_range*0.625, center_y-pic_cut_range-y_offset, center_x+pic_cut_range*0.625, center_y-y_offset))
    img = img.resize((300, 240), Image.ANTIALIAS)
    return img


class GPS_item:
    def __init__(self, x, y):
        self.x = x
        self.y = y


if __name__ == "__main__":
    the_gps = GPS_item(120.13068884676356, 30.275299720398763)
    img = cut_navigation(the_gps, 105)
    img.save('after_cut.png')