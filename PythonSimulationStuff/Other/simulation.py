import cv2
import numpy as np
from math import pi, cos, sin, exp


leftMouseIsHeld = False
WIDTH = 600
HEIGHT = 500
CENTER = (int(HEIGHT/2), int(WIDTH/2))
angle1 = 0
angle2 = 0
lengthOfLine = 70
mask = None


def KChanged(K=0):
    #print("K", K)
    #K = cv2.getTrackbarPos("K Selector", "Figure")
    pass


def reCalculateArm():
    global mask

    mask = np.zeros((WIDTH, HEIGHT, 3), dtype=np.uint8)
    line1EOL = getEndofLine(CENTER, angle1)
    mask = cv2.line(mask, CENTER, line1EOL, color=(255, 0, 0), thickness=3)
    mask = cv2.line(mask, line1EOL, getEndofLine(line1EOL, angle2), color=(0, 255, 0), thickness=3)

    cv2.imshow("Figure", mask)


def angle1Changed(newAngle1=0):
    global angle1
    angle1 = newAngle1
    reCalculateArm()
    

def angle2Changed(newAngle2=0):
    global angle2
    angle2 = newAngle2
    reCalculateArm()


def toRad(degrees):
    return degrees * pi / 180


def sine(degrees):
    return sin(toRad(degrees))


def cosine(degrees):
    return cos(toRad(degrees))


def bar_movement(event, x, y, flags, param):
    global leftMouseIsHeld

    if event==cv2.EVENT_LBUTTONDOWN:
        leftMouseIsHeld = True
        print("Mouse down at", x, y)
    elif event==cv2.EVENT_LBUTTONUP:
        leftMouseIsHeld = False
        print("Moving up at", x, y)
    elif event==cv2.EVENT_MOUSEMOVE:
        if leftMouseIsHeld:
            print("Moving to", x, y)


def getEndofLine(start_of_line, angle):
    xEOL = -sine(angle)*lengthOfLine + start_of_line[1]
    yEOL = cosine(angle)*lengthOfLine + start_of_line[0]

    return ( int(yEOL) , int(xEOL) )

def main():
    global mask

    cv2.namedWindow("Figure")
    cv2.createTrackbar("K Selector", "Figure", 50, 100, KChanged)
    cv2.createTrackbar("Angle 1 Selector", "Figure", angle1, 360, angle1Changed)
    cv2.createTrackbar("Angle 2 Selector", "Figure", angle2, 360, angle2Changed)
    #cv2.setTrackbarPos("K Selector", "Figure", 50)

    reCalculateArm()

    cv2.setMouseCallback("Figure", bar_movement)

    #while True:

    cv2.waitKey(0)

if __name__ == '__main__':  
    main()