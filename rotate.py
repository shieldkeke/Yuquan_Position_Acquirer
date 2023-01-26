from PIL import Image

left_up_gps = {"x": 120.129917, "y": 30.272621}
right_down_gps = {"x": 120.133689, "y": 30.266071}
left_up_pic = {"x": 457, "y": 248}
right_down_pic = {"x": 667, "y": 669}

cut_range = 10


def cut_navigation(gps, yaw):
    img = Image.open('navigation.png')
    img = img.resize((829, 755))
    print(img.width, img.height)
    center_x = int((gps.x-left_up_gps["x"])/(right_down_gps["x"]-left_up_gps["x"]) * (right_down_pic["x"]-left_up_pic["x"]) + left_up_pic["x"])
    center_y = int((gps.y - left_up_gps["y"]) / (right_down_gps["y"] - left_up_gps["y"]) * (right_down_pic["y"] - left_up_pic["y"]) + left_up_pic["y"])
    print(center_x, center_y)
    img = img.rotate(angle=yaw, center=(center_x, center_y))
    img.save("after_rotate.png")
    img = img.crop(box=(center_x-cut_range, center_y-cut_range, center_x+cut_range, center_y+cut_range))
    return img


class GPS_item:
    def __init__(self, x, y):
        self.x = x
        self.y = y


if __name__ == "__main__":
    the_gps = GPS_item(120.13076417401567, 30.275332343777958)
    img = cut_navigation(the_gps, 90)
    img.save('after_cut.png')