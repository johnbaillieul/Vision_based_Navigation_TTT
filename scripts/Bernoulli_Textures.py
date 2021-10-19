# Import all libraries we will use
import random
import numpy as np
import cv2


def create_image(p):
    # let's create a heigth x width matrix with all pixels in black color
    heigth = 1080
    width = 1920
    diameter = 50
    x_correction = int(0.7 * diameter / 2)
    y_correction = int(0.7 * diameter / 2)

    img = np.ones((heigth, width, 3), np.uint8)*255

    hcount = int(diameter/2)

    while hcount < (heigth-3):
        wcount = int(diameter/2)
        while wcount < (width-3):
            if random.uniform(0, 1) >= (1-p):
                shape = random.uniform(0, 3)
                if shape < 1.0:
                    cv2.circle(img, (wcount, hcount), int(diameter/2), [0, 0, 0], -1)
                elif shape < 2.0:
                    cv2.rectangle(img, (wcount - x_correction, hcount - y_correction), (wcount + x_correction, hcount +
                                                                                        y_correction), [0, 0, 0], -1)
                else:
                    pt1 = (wcount, hcount-y_correction)
                    pt2 = (wcount-x_correction, hcount+y_correction)
                    pt3 = (wcount+x_correction, hcount+y_correction)
                    triangle_cnt = np.array([pt1, pt2, pt3])
                    cv2.drawContours(img, [triangle_cnt], 0, (0, 0, 0), -1)
            # img[hcount, wcount] = [255, 255, 255]
            wcount += diameter
        hcount += diameter

    p = int(p * 100)
    # save our image as a "jpg" image
    cv2.imwrite("bernoulli" + str(p) + "M" + ".png", img)


if __name__ == '__main__':
    create_image(0.08)
