import cv2
import numpy as np
from math import pi, cos, sin
import keyboard
from time import time
from values import *
import pickle


# load saved data if exist
loaded = False
try:
    with open('values.pickle', 'rb') as f:
        imp = pickle.load(f)
        loaded = True
        print("Loaded Data", imp)
except FileNotFoundError as e:
    pass


# save to pickle
if not loaded:
    with open('values.pickle', 'wb') as f:
        pickle.dump(imp, f)
        print("First time launch", imp)
        
    
    # Human-Robot Interfacing software prototype
    #   using impedance Control & Trajectory Optimization techniques

    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # references
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    #   - https://drive.google.com/drive/folders/1OpBOQRvFL2EBWvRAA2XpjNUty3MaOqUA?usp=sharing

    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # Variables Shared
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%
mask = None
Fh = np.empty((0, 2), int) # force applied to robot

def initXandXd():
    x = np.empty((0, 2), int) # actual position
    xd = np.empty((0, 2), int) # desired position
    return x, xd

def resetXAndXd():
    x, xd = initXandXd()

    x = np.append(x, np.array([[int(WIDTH/2), int(HEIGHT/2)]]), axis=0) # actual position
    xd = np.append(xd, np.array([[int(WIDTH/2 + WIDTH/4), int(HEIGHT/2 + HEIGHT/4)]]), axis=0) # desired position
    return x, xd

x, xd = initXandXd()
x, xd = resetXAndXd()

time_app_started_at = 0
def millis():
    return int(time()*1000) - time_app_started_at
time_app_started_at = millis() # so i can make millis(), count ever since the app started, maybe it'll be good for a line by line execution scheme?


def ImpedenceControl():
    global Fh

    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # Shared Variables
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    i = len(xd) - 1 # number of sampling times
    e = xd - x # array of errors between all positions of x and xd

    # for my code the error is considered as the affecting force, just bcz it's the simplest way to go about it
    #Fh = e.copy() # TODO: DONT FORGET TO GET RID OF THIS za3ma simulating gravity
    # let's say Fh is gravity
    Fh = np.append(Fh, np.array([[ 1000*9.8*(WIDTH-x[i][0]), 1000*9.8*(HEIGHT-x[i][1]) ]]), axis=0)

    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # Trajactory Adaptation
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # Operation - Trajactory Adaptation
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Epsilon1 = imp["K1"] * e*e # performance evaluation index

    # Index function J
    J = 0
    for j in range(0, i):
        J = J + Epsilon1[j] * np.exp( - imp["Mu1"] * (i - j) * imp["dT"] )

    Alpha1 = 1 - np.exp( -imp["v1"] * J ) # variable to be designed according to a specific system to adjust the user's interaction profile
    Beta1 = imp["M1"] * Alpha1 # an open parameter to regulate the interaction force

    # final relation between x, xd and Fh
    xd_trajectory_adaptation = x + Beta1 * Fh # oh we experienced disturbing force Fh, let's see where we should move next to reduce Fh, we find xd


    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # Adaptive impedance Control
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # we must differentiate X axis and Y axis separately from each other then put them back together
    A1 = np.diff([ A1[0] for A1 in e ])
    A2 = np.diff([ A2[1] for A2 in e ])
    e_derivative = np.empty((0, 2), int)
    e_derivative = np.insert(e_derivative, 0, e[0], axis=0)
    for i in range(0, len(A1)):
        e_derivative = np.insert(e_derivative, 0, np.array([A1[i], A2[i]]), axis=0)
    # because the python differentiation doesn't insert the first element that start from zero we add it

    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    # Operation - Adaptive impedance Control
    #%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Epsilon2 = imp["K2"] * e*e # performance evaluation index

    # Index function J2
    J2 = 0
    for j in range(0, i):
        J2 = J2 + Epsilon2[j] * np.exp( - imp["Mu2"] * (i - j) * imp["dT"] )

    Alpha2 = np.exp( -imp["v2"] * J2 ) # variable to be designed according to a specific system to adjust the user's interaction profile
    
    Beta2 = imp["M2"] * Alpha2 # paramter to regulate the stiffness of the impedence controller
    deltaK = Beta2 * abs(Fh)
    Kc = imp["Kc0"] + deltaK # variable stiffness
    Fim = Kc * e + imp["Kd"] * e_derivative # it's u, input of the impedance control

    # final relation between x, xd and Fh
    xd_impedence_control = x + Beta2 * Fh # oh we experienced disturbing force Fh, let's see where we should move next to reduce Fh, we find xd


    return xd_trajectory_adaptation, xd_impedence_control


def getLast(x):
    return tuple(x[len(x)-1])


def K2Changed(K2=0):
    global imp
    imp["K2"] = turnFromPercent(K2, K2Full, K2Max)

def M2Changed(M2=0):
    global imp
    imp["M2"] = turnFromPercent(M2, M2Full, M2Max)

def Kc0Changed(Kc0=0):
    global imp
    imp["Kc0"] = turnFromPercent(Kc0, Kc0Full, Kc0Max)

def KdChanged(Kd=0):
    global imp
    imp["Kd"] = turnFromPercent(Kd, KdFull, KdMax)

def v2Changed(v2=0):
    global imp
    imp["v2"] = turnFromPercent(v2, v2Full, v2Max)

def dTChanged(dT=0):
    global imp
    imp["dT"] = turnFromPercent(dT, dTFull, dTMax)

def Mu2Changed(Mu2=0):
    global imp
    imp["Mu2"] = turnFromPercent(Mu2, Mu2Full, Mu2Max)


def turnToPercent(value, full, max):
    return int( value/full * max )

def turnFromPercent(percentage, full, max):
    return percentage*full/max


def main():
    global mask, x, xd, Fh

    print("Press esc to exit")
    print("Going to save values every", generationTime/1000, "seconds")

    cv2.namedWindow("Figure")
    cv2.createTrackbar("v2", "Figure", turnToPercent(imp["v2"], v2Full, v2Max), v2Max, v2Changed)
    cv2.createTrackbar("dT", "Figure", turnToPercent(imp["dT"], dTFull, dTMax), dTMax, dTChanged)
    cv2.createTrackbar("K2", "Figure", turnToPercent(imp["K2"], K2Full, K2Max), K2Max, K2Changed)
    cv2.createTrackbar("M2", "Figure", turnToPercent(imp["M2"], M2Full, M2Max), M2Max, M2Changed)
    cv2.createTrackbar("Mu2", "Figure", turnToPercent(imp["Mu2"], Mu2Full, Mu2Max), Mu2Max, Mu2Changed)
    cv2.createTrackbar("Kc0", "Figure", turnToPercent(imp["Kc0"], Kc0Full, Kc0Max), Kc0Max, Kc0Changed)
    cv2.createTrackbar("Kd", "Figure", turnToPercent(imp["Kd"], KdFull, KdMax), KdMax, KdChanged)

    previous_sample = 0
    first_launch = True
    current_generation_start_time = millis()
    current_sample = millis()
    update_available = False

    while True:

        # impedence control stuff
        if not first_launch:
            current_sample = millis()
            if current_sample - previous_sample >= imp["dT"]:
                update_available = True
                
                xd_trajectory_adaptation, xd_impedence_control = ImpedenceControl()

                # TODO: you're only using xd_impedence_control in the line before   
                x = np.append(x, np.array([[ int(xd_impedence_control[0][0]), int(xd_impedence_control[0][1]) ]]), axis=0) # just keep all coordinates as int
                xd = np.append(xd, np.array([xd[len(xd)-1]]) , axis=0) # i guess we keep desiring the same spot until we reach it?

                previous_sample = current_sample

        if first_launch or update_available:
            print("latest x", x[len(x)-1])
            update_available = False
            # draw Ui stuff
            mask = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
            mask = cv2.circle(mask, getLast(xd), 10, (0, 255, 0), 2)
            mask = cv2.circle(mask, getLast(x), 5, (255, 255, 255), 2)
            cv2.imshow("Figure", mask)

        first_launch = False

        # if generation time ended restart, just so u can be adjusting model values live and see them act on the item throughout the whole action from start to finish
        if current_sample - current_generation_start_time > generationTime:
            x, xd = resetXAndXd()
            Fh, _ = initXandXd()
            first_launch = True
            current_generation_start_time = current_sample

            with open('values.pickle', 'wb') as f:
                pickle.dump(imp, f)
                print("Saving", imp)


        cv2.waitKey(1)
        if keyboard.is_pressed('esc') == True:
            break


if __name__ == '__main__':  
    main()