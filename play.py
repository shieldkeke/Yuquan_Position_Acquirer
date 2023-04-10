import cv2
import os
# play the pics in the folder with x fps
def play(folder, x = 24, save = False, save_name = 'output'):
    files = os.listdir(folder)
    files.sort()
    for file in files:
        img = cv2.imread(folder + '/' + file)
        cv2.imshow('img', img)
        cv2.waitKey(1000//x)
    if save:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(save_name + '.mp4', fourcc, 20.0, (1280, 720))
        for file in files:
            img = cv2.imread(folder + '/' + file)
            out.write(img)
        out.release()
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    play('/home/chenyeke/srtp/data/pm', 24, True, 'before_filt')
    play('/home/chenyeke/srtp/data/pm_filt', 24, True, 'after_filt')