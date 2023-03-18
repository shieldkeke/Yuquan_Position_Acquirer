
import matplotlib.pyplot as plt
from matplotlib.patches import Arc
from PIL import Image

class LineDrawer:
    def __init__(self, ax) -> None:
        line, = ax.plot([], [])  # empty line
        point, = ax.plot([], [], 'bo')

        self.line = line
        self.point = point
        self.xs = list(line.get_xdata())
        self.ys = list(line.get_ydata())
        
        self.Display_Points = []
        self.GPS_Points = []
        self.SHOW_LINE = True
        self.READ_FROM_LAST = False

        self.left_up_gps = {"x": 120.11807369666667, "y": 30.263599648333336}
        self.right_down_gps = {"x": 120.11863341150001, "y": 30.261879024000002}
        self.left_up_pic = {"x": 425, "y": 566}
        self.right_down_pic = {"x": 455, "y": 678}

        self.left_down_gps = {"x": 120.11749962616665, "y": 30.262226684333335}
        self.right_up_gps = {"x": 120.11923214966667, "y": 30.2632707255}
        self.left_down_pic = {"x": 387, "y": 652}
        self.right_up_pic = {"x": 489, "y": 590}

        self.callback = line.figure.canvas.mpl_connect('button_press_event', self)


    def save_data(self, gps_point, dis_point):
        self.GPS_Points.append(gps_point)
        self.xs.append(dis_point[0])
        self.ys.append(dis_point[1])
        self.Display_Points.append(dis_point)
        if self.SHOW_LINE:
            self.line.set_data(self.xs, self.ys)
            self.line.figure.canvas.draw()
    
    def undo(self):
        if len(self.Display_Points)>0:
            self.Display_Points.pop()
            self.GPS_Points.pop()
            self.xs.pop()
            self.ys.pop()
        if self.SHOW_LINE:
            self.line.set_data(self.xs, self.ys)
            self.line.figure.canvas.draw()

    def write_to_file(self):
        file=open('gps_data.csv',mode='w')
        for point in self.GPS_Points:
            file.write(f'{point[0]} {point[1]}\n')
        file.close()
        
        # file=open('pic_data.csv',mode='w')
        # for point in self.Display_Points:
        #     file.write(f'{point[0]} {point[1]}\n')
        # file.close()

    def draw_navigation(self):
        plt.clf()
        new_fig, new_ax = plt.subplots()
        map = Image.open('map.png')
        k = 1 # for more thin lines(seem to be useless)
        new_ax.set(xlim=[0, map.width * k], ylim=[map.height * k, 0]) 
        plt.axis('off')

        new_ax.plot([x*k for x in self.xs], [y*k for y in self.ys], color='red', linewidth=1)
        plt.savefig('navigation.png', bbox_inches='tight', pad_inches=0, dpi=4000)# for the high quality of picture

    def __call__(self, event):
        # undo last point if [middle button]
        if event.button == 2:
            self.undo()
            return

        # if [left button] or [right button] from image coordinate to GPS coordinate
        x = (event.xdata - self.left_up_pic["x"]) / (self.right_down_pic["x"] - self.left_up_pic["x"]) * (self.right_down_gps["x"] - self.left_up_gps["x"]) + self.left_up_gps["x"]
        y = (event.ydata - self.left_up_pic["y"]) / (self.right_down_pic["y"] - self.left_up_pic["y"]) * (self.right_down_gps["y"] - self.left_up_gps["y"]) + self.left_up_gps["y"]    
        print("my position: " ,event.button, event.xdata, event.ydata) 
        print("gps: ", x, y)

        
        self.point.set_data(event.xdata, event.ydata) # show point
        self.line.figure.canvas.draw()
        
        # if [right button], save and draw it
        if event.button == 3:
            self.save_data((x, y),(event.xdata, event.ydata))

#def cut_navigation(gps, yaw_angle):


if __name__ == "__main__":
    img = Image.open('map.png')
    fig, ax = plt.subplots()
    l = LineDrawer(ax)
    plt.imshow(img, animated= True)

    try:
        plt.show()
        l.write_to_file()
    except:
        l.write_to_file()

    l.draw_navigation()
    
    #457,248 -> 120.129917,30.272621
    #667,669 -> 120.133689,30.266071

    #http://api.map.baidu.com/lbsapi/getpoint/index.html