# file:///C:/Users/Kream/Desktop/SmartImpedanceControl/ResearchPapers/3.pdf
import cv2
import numpy as np
from math import sqrt, floor, ceil
import keyboard
from multiprocessing import Process, Pipe




# physics info
L1 = 70
L2 = 70

# window size 
width = 400
height = 400

center = (int(width/2), int(height/2))
theta1 = np.pi*1/4
theta2 = np.pi*1/4
motor1_angle = int(theta1*180/np.pi)
motor2_angle = int(theta2*180/np.pi)

# so motor angles are set automatically on launch since it'll be different than motor1_angle
current_motor1_angle = motor1_angle
current_motor2_angle = motor2_angle




sensor1_value, sensor2_value, sensor3_value, sensor4_value = (0, 0, 0, 0)
distance_applied_by_sensor1, distance_applied_by_sensor2, distance_applied_by_sensor3, distance_applied_by_sensor4 = (0, 0, 0, 0)

s1_angle = np.pi/2 #sensor1_angle
s2_angle = 0
s3_angle = np.pi
s4_angle = -np.pi/2
    
dist = 10 # steps_following_force

steps_following_force = 0.1 # factor, potentially where force 0-1024 range might go into after cleaning it up
distance_from_desired = 0 # this must be set on a cooldown

def getSensorInfluence(actual, theta1, theta2):
    
    sensor1 = (int(dist * np.sin(theta1+theta2+s1_angle)), int(dist * np.cos(theta1+theta2+s1_angle)) )
    sensor2 = (int(dist * np.sin(theta1+theta2+s2_angle)), int(dist * np.cos(theta1+theta2+s2_angle)) )
    sensor3 = (int(dist * np.sin(theta1+theta2+s3_angle)), int(dist * np.cos(theta1+theta2+s3_angle)) )
    sensor4 = (int(dist * np.sin(theta1+theta2+s4_angle)), int(dist * np.cos(theta1+theta2+s4_angle)) )
    
    return sensor1, sensor2, sensor3, sensor4

def getSensorPositions(actual, sensor1, sensor2, sensor3, sensor4):
    
    sensor1_pos = (int(actual[0] + sensor1[0]), int(actual[1] + sensor1[1]) )
    sensor2_pos = (int(actual[0] + sensor2[0]), int(actual[1] + sensor2[1]) )
    sensor3_pos = (int(actual[0] + sensor3[0]), int(actual[1] + sensor3[1]) )
    sensor4_pos = (int(actual[0] + sensor4[0]), int(actual[1] + sensor4[1]) )
    
    return sensor1_pos, sensor2_pos, sensor3_pos, sensor4_pos




# inverse kin
# https://github.com/AymenHakim99/Forward-and-Inverse-Kinematics-for-2-DOF-Robotic-arm?fbclid=IwAR3Mu8nFWDik95ROO-O-ViZtPJ8EQK5ItO9Y9rz37tiFY2LDuernw1n67jM

import sympy as smp

# maybe we choose the signs base on which is closest? but also the one within possible range idk

x, y, a1, a2 = smp.symbols('x, y, a1, a2')

th2 = smp.acos( (x**2 + y**2 - a1**2 - a2**2) / (2*a1*a2) )
theta2_f = smp.lambdify( (x, y, a1, a2) , th2)


costh2, sinth2 = smp.symbols('costh2, sinth2')

# +0.00001 is to avoid division by zero
th1 = smp.atan(y/(x+0.00001)) - smp.atan(a2*sinth2/(a1+a2*costh2))
theta1_f = smp.lambdify( (x, y, a1, a2, costh2, sinth2) , th1)





def inverseKin(px, py):
    
    # we gotta offset desired from the center
    px -= center[0]
    py -= center[1]
    
    # restricting arm to the maximum reach possible
    desired_circle = sqrt(px**2 + py**2)
    possible_circle = L1+L2 # maximum radius
    ratio = desired_circle / possible_circle
    
    if ratio > 1:
        if px < 0:
            px = ceil(px/ratio) # because floor for negative numbers is the opposite lmao
        else:
            px = floor(px/ratio)
            
        if py < 0:
            py = ceil(py/ratio) # because floor for negative numbers is the opposite lmao
        else:
            py = floor(py/ratio)
        
        desired_circle = sqrt(px**2 + py**2)
        possible_circle = sqrt((L1+L2)**2 + 0**2)
        ratio = desired_circle / possible_circle
        
    theta2 = theta2_f(py, px, L1, L2)
    theta1 = theta1_f(py, px, L1, L2, np.cos(theta2), np.sin(theta2))
    
    if py < 0:
        theta1 -= np.pi
    
    ogtheta1 = theta1
    ogtheta2 = theta2
        
    # trying to flip the approach to desired
    desired_point_tawila = sqrt(px**2+py**2)
    desired_point_angle = np.arccos(py/(desired_point_tawila+0.00001)) # +0.00001 to avoid division by zero
    if px < 0:
        desired_point_angle = - desired_point_angle
        
    theta1 += 2*(abs(theta1-desired_point_angle))
    theta2 = -theta2
    
    return theta1, theta2, ogtheta1, ogtheta2 # this function provides both approaches to the desired point so u can choose based on convenience

def forwardKin(theta1, theta2):
    x = L2*np.sin(theta1 + theta2) + L1*np.sin(theta1)
    y = L2*np.cos(theta1 + theta2) + L1*np.cos(theta1)
    return list((int(x) + center[0], int(y) + center[1]))

def forwardElbowKin(theta1):
    x = L1*np.sin(theta1)
    y = L1*np.cos(theta1)
    return list((int(x) + center[0], int(y) + center[1]))

def calculateEverything():
    global sensor1_pos, sensor2_pos, sensor3_pos, sensor4_pos
    global distance_applied_by_sensor1, distance_applied_by_sensor2, distance_applied_by_sensor3, distance_applied_by_sensor4
    global sensor1, sensor2, sensor3, sensor4
    global actual, theta1, theta2, end_effector, elbow, ogend_effector, ogelbow, motor1_angle, motor2_angle
    
    theta1, theta2, ogtheta1, ogtheta2 = inverseKin(actual[0], actual[1])  # this function provides both approaches to the desired point so u can choose based on convenience
    # calculate sensor locations for drawing on the gui
    sensor1, sensor2, sensor3, sensor4 = getSensorInfluence(desired, theta1, theta2)
        
    # now let's influence actual based on force
    if sensor1_value != 0: # DEBUGGING ONLY MAKE MOVING BASED ON APPLIED FORCE        print("
        distance_applied_by_sensor1 += steps_following_force
        actual[0] -= steps_following_force * sensor1[0]
        actual[1] -= steps_following_force * sensor1[1]
    else:
        # if we still got offset let's remove it
        if distance_applied_by_sensor1 > 0:
            distance_applied_by_sensor1 -= steps_following_force
            actual[0] += steps_following_force * sensor1[0]
            actual[1] += steps_following_force * sensor1[1]

    if sensor2_value != 0: # DEBUGGING ONLY MAKE MOVING BASED ON APPLIED FORCE
        distance_applied_by_sensor2 += steps_following_force
        actual[0] -= steps_following_force * sensor2[0]
        actual[1] -= steps_following_force * sensor2[1]
    else:
        # if we still got offset let's remove it
        if distance_applied_by_sensor2 > 0:
            distance_applied_by_sensor2 -= steps_following_force
            actual[0] += steps_following_force * sensor2[0]
            actual[1] += steps_following_force * sensor2[1]
    if sensor3_value != 0: # DEBUGGING ONLY MAKE MOVING BASED ON APPLIED FORCE
        
        distance_applied_by_sensor3 += steps_following_force
        actual[0] -= steps_following_force * sensor3[0]
        actual[1] -= steps_following_force * sensor3[1]
    else:
        # if we still got offset let's remove it
        if distance_applied_by_sensor3 > 0:
            distance_applied_by_sensor3 -= steps_following_force
            actual[0] += steps_following_force * sensor3[0]
            actual[1] += steps_following_force * sensor3[1]
    if sensor4_value != 0: # DEBUGGING ONLY MAKE MOVING BASED ON APPLIED FORCE
        distance_applied_by_sensor4 += steps_following_force
        actual[0] -= steps_following_force * sensor4[0]
        actual[1] -= steps_following_force * sensor4[1]
    else:
        # if we still got offset let's remove it
        if distance_applied_by_sensor4 > 0:
            distance_applied_by_sensor4 -= steps_following_force
            actual[0] += steps_following_force * sensor4[0]
            actual[1] += steps_following_force * sensor4[1]
            
    # scotch sadly
    # if all sensors don't have any force on them then make the head go back to its desired 
    # TODO make it go there slowly
    if sensor1_value == 0 and sensor2_value == 0 and sensor3_value == 0 and sensor4_value == 0 \
        and distance_applied_by_sensor1 <= 0 and distance_applied_by_sensor2 <= 0 and distance_applied_by_sensor3 <= 0 and distance_applied_by_sensor4 <= 0:
        actual = desired.copy()

    # just for drawing
    sensor1_pos, sensor2_pos, sensor3_pos, sensor4_pos = getSensorPositions(actual, sensor1, sensor2, sensor3, sensor4)
    
    # update the thetas after we have influenced the actual end effector position
    theta1, theta2, ogtheta1, ogtheta2 = inverseKin(actual[0], actual[1])  # this function provides both approaches to the desired point so u can choose based on convenience

    # BLUE ONE
    # limitation programming to be able to extend the arm past that 180 since the 2nd link can still turn more 
    theta1LimitReached = False
    if -np.pi < theta1 and theta1 < -np.pi/2:
        theta1 = -np.pi/2
        theta1LimitReached = True
    if theta1 > np.pi/2 or theta1 < -np.pi:
        theta1 = np.pi/2
        theta1LimitReached = True

    elbow = forwardElbowKin(theta1)

    # the case where theta1 reaches its limit
    if theta1LimitReached:
        centered_desired_x = actual[0] - center[0]
        centered_desired_y = actual[1] - center[1]

        centered_elbow_x = elbow[0] - center[0]
        centered_elbow_y = elbow[1] - center[1]

        tawila = sqrt((centered_desired_x-centered_elbow_x)**2 + (centered_desired_y-centered_elbow_y)**2)
        theta2 = np.pi - np.arccos((centered_desired_x-centered_elbow_x) / tawila)
        if actual[1] < center[1] and actual[0] < center[0]:
            theta2 = -theta2
        if actual[1] < center[1] and actual[0] > center[0]:
            theta2 = np.pi - theta2
        if actual[1] > center[1] and actual[0] > center[0]:
            theta2 = -np.pi +  theta2


    deg_theta2 = theta2*180/np.pi
    if deg_theta2 < -155:
        deg_theta2 = -155
    if deg_theta2 > -155+180:
        deg_theta2 = -155+180
    theta2 = deg_theta2*np.pi/180
    deg_theta2 = -(deg_theta2-(-155+180))

    end_effector = forwardKin(theta1, theta2)

    # determine the angles of the motors based on angles of thetas
    deg_theta1 = theta1*180/np.pi
    temp_motor1_angle = int(deg_theta1+90)
    if -180 < temp_motor1_angle and temp_motor1_angle < 0:
        temp_motor1_angle = 0
    elif temp_motor1_angle < -180 or temp_motor1_angle > 180:
        temp_motor1_angle = 180

    motor1_angle, motor2_angle = [temp_motor1_angle, int(deg_theta2)] # i moved it to here just so they're both updated at once



    # GREEN ONE
    # limitation programming to be able to extend the arm past that 180 since the 2nd link can still turn more
    '''
    ogtheta1LimitReached = False
    if -np.pi < ogtheta1 and ogtheta1 < -np.pi/2:
        ogtheta1 = -np.pi/2
        ogtheta1LimitReached = True
    if ogtheta1 > np.pi/2 or ogtheta1 < -np.pi:
        ogtheta1 = np.pi/2
        ogtheta1LimitReached = True

    ogelbow = forwardElbowKin(ogtheta1)

    # the case where theta1 reaches its limit
    if ogtheta1LimitReached:
        centered_desired_x = x - center[0]
        centered_desired_y = y - center[1]

        centered_ogelbow_x = ogelbow[0] - center[0]
        centered_ogelbow_y = ogelbow[1] - center[1]

        tawila = sqrt((centered_desired_x-centered_ogelbow_x)**2 + (centered_desired_y-centered_ogelbow_y)**2)
        ogtheta2 = np.pi - np.arccos((centered_desired_x-centered_ogelbow_x) / tawila)
        if y < center[1] and x < center[0]:
            ogtheta2 = -ogtheta2
        if y < center[1] and x > center[0]:
            ogtheta2 = np.pi - ogtheta2
    # TODO: find ogtheta2 that lets the second link follow the desired point with the first link fixed at its limitation

    ogend_effector = forwardKin(ogtheta1, ogtheta2)
    '''

clicking = False
def desiredIntroduced(event, x, y, flags, param):
    global clicking, desired, actual, theta1, theta2
    if event==cv2.EVENT_LBUTTONUP:
        clicking = False
    elif event==cv2.EVENT_LBUTTONDOWN or (clicking and event==cv2.EVENT_MOUSEMOVE): # or event==cv2.EVENT_MOUSEMOVE
        if event==cv2.EVENT_LBUTTONDOWN:
            clicking = True
        
        # FIXING A SPECIAL CASE
        # if we were in top right then it tries to snap to the top right quadrant
        if actual[0]-center[0]>0 and actual[1]-center[1]>0 and x-center[0]>0 and y-center[1]<0: # to fix the snipping issue on top right
            if theta2 < -2.14:
                return
        
        desired = [x, y]
        actual = [x, y]
        
        
end_effector = forwardKin(theta1, theta2)
elbow = forwardElbowKin(theta1)
ogend_effector = forwardKin(theta1, theta2)
ogelbow = forwardElbowKin(theta1)
desired = end_effector.copy()
actual = end_effector.copy()
sensor1, sensor2, sensor3, sensor4 = getSensorInfluence(actual, theta1, theta2) 
sensor1_pos, sensor2_pos, sensor3_pos, sensor4_pos = getSensorPositions(actual, sensor1, sensor2, sensor3, sensor4)







from serial import Serial
from serial import EIGHTBITS
from serial import PARITY_NONE
from serial import STOPBITS_ONE
from time import time

port = "COM10"
def communicationThread():
    global process, haltingProducer, fromArduinoProducer, locationUpdateConsumer
    
    # init serial
    serial, stop = Setup_serial(port)
    if serial != None:
        print("Serial opened!")
    else:
        print("Could not open serial! returning")
        return
    
    previous_command_time = 0
    # run communication
    while True and not stop:
        # sending location updates to arduino
        if locationUpdateConsumer.poll():
            val = locationUpdateConsumer.recv()
            if val!=None:
                # since main thread sent us a string of these two values for compactability
                current_motor1_angle, current_motor2_angle = val.split(" ")
                current_motor1_angle, current_motor2_angle = (int(current_motor1_angle), int(current_motor2_angle))

                # send it to arduino
                print("Applying angles:", current_motor1_angle, "°", current_motor2_angle, "°")
                send_command(serial, "ANGLES " + str(current_motor1_angle) + " " + str(current_motor2_angle))

        # FUTURE: check for force reports in order for us to decide where to take the robot arm
        bundle = Read(serial)
        message = Clean(bundle)
        if message: # skip what's below this line
            fromArduinoProducer.send(message)

        # maybe not necessary because process
        cv2.waitKey(1)

        # for quitting
        if keyboard.is_pressed('esc'):
            haltingProducer.send(None)
            process.join()
            break
        
def Read(serial):
    try:
        return serial.readline() # read a byte string
    except Exception as e:
        print("Failed to read error:", e)

        
def Clean(b):
    if b!=None:
        #string_n = b.decode('utf-8', errors='replace')  # decode byte string into Unicode
        try:
            return b.decode().rstrip()  # decode byte string into Unicode
        except UnicodeDecodeError:
            return None

def Setup_serial(port):
    while True:
        # for quitting
        if keyboard.is_pressed('esc'):
            return None, False

        # might be unecessary
        cv2.waitKey(1)

        try:
            '''
            ser = Serial(
            port = port,
            baudrate = 9600,
            bytesize = EIGHTBITS,
            parity = PARITY_NONE,
            stopbits = STOPBITS_ONE,
            timeout = 1,
            xonxoff = False,
            rtscts = False,
            dsrdtr = False,
            writeTimeout = 2
            )
            '''
            ser = Serial(port = port, baudrate = 9600, timeout = 0.001)
            if(ser.isOpen() == False):
                ser.open()
            else:
                return ser, False
        except FileNotFoundError as e:
            print("You're either not connected or the port", port, "is wrong")
        except Exception as e:
            print("Error connecting to serial with error e=", e)
        print("Attempting to open serial")
        

def send_command(serial, command):
    serial.write(bytes(command + '\n', 'utf-8'))
    return serial






# allow the user to edit the desired point real time
cv2.destroyAllWindows()

def draw():
    window = np.ones((height, width, 3), dtype=np.uint8) #*255

    window = cv2.line(window, tuple(center), tuple(elbow), (255, 0, 0), 6)
    window = cv2.line(window, tuple(elbow), tuple(end_effector), (255, 0, 0), 6)
    #window = cv2.line(window, tuple(center), tuple(desired), (255, 255, 255), 6)
    
    #window = cv2.line(window, tuple(center), tuple(ogelbow), (0, 255, 0), 6)
    #window = cv2.line(window, tuple(ogelbow), tuple(ogend_effector), (0, 255, 0), 6)
    
    # draw force sensors
    window = cv2.circle(window, sensor1_pos, 2, (0, 255, 0), 2)
    window = cv2.circle(window, sensor2_pos, 2, (226, 253, 48), 2)
    window = cv2.circle(window, sensor3_pos, 2, (48, 82, 253), 2)
    window = cv2.circle(window, sensor4_pos, 2, (253, 48, 239), 2)
    
    window = cv2.circle(window, tuple(desired), 5, (255, 255, 255), 2)   # we have casual coordinates but we want zero to be the middle of screen with a range of -4 to 4 just bcz it works with our other values
    return window


def mainLoop(locationUpdateProducer, haltingConsumer, fromArduinoConsumer):
    global sensor1_value, sensor2_value, sensor3_value, sensor4_value # sensors
    global current_motor1_angle, motor1_angle, current_motor2_angle, motor2_angle # motors
    global previous_report # time
    
    # start main Gui thread
    cv2.namedWindow('output', cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('output', desiredIntroduced)

    while True:

        # check for msgs received from arduino
        if fromArduinoConsumer.poll():
            val = fromArduinoConsumer.recv()
            if val!=None:
                # process msg, either it's a confirmation to a command or applied force reporting
                if val.startswith("Ok, upd"):
                    print(val)
                    
                elif val.startswith("Reporting, forces"):
                    print(val)
                    
                elif val.startswith("OOF"):
                    print(val)
                    
                # process msg, either it's a confirmation to a command or applied force reporting
                elif val.startswith("Ok, started"):
                    print(val)

                else:
                    print("Arduino printed:", val)

        # debugging: applying forces to sensors manually to testonly
        if keyboard.is_pressed('k'):
            sensor1_value = 1024
        else:
            sensor1_value = 0

        if keyboard.is_pressed('o'):
            sensor2_value = 1024
        else:
            sensor2_value = 0

        if keyboard.is_pressed('l'):
            sensor3_value = 1024
        else:
            sensor3_value = 0

        if keyboard.is_pressed('m'):
            sensor4_value = 1024
        else:
            sensor4_value = 0

        calculateEverything()

        # report new positions, current default: 25 per second
        if motor1_angle != current_motor1_angle or motor2_angle != current_motor2_angle:
            
            if time() - previous_report > period_of_reports:
                previous_report = time()
                # save the data
                current_motor1_angle, current_motor2_angle = (motor1_angle, motor2_angle)
                locationUpdateProducer.send(str(current_motor1_angle) + " " + str(current_motor2_angle))


        # display simulation
        window = draw()
        cv2.imshow('output', window)

        # for no lag
        cv2.waitKey(1)

        # halting check
        if haltingConsumer.poll():
            val = haltingConsumer.recv()
            if val==None:
                break


position_reports_per_second = 10
period_of_reports = 1 / position_reports_per_second
previous_report = 0

if __name__ == '__main__':
    (locationUpdateConsumer, locationUpdateProducer) = Pipe(False)
    (fromArduinoConsumer, fromArduinoProducer) = Pipe(False)
    (haltingConsumer, haltingProducer) = Pipe(False)
    process = Process(target=mainLoop, args=(locationUpdateProducer, haltingConsumer, fromArduinoConsumer))
    process.start()

    communicationThread()

    # clean up
    cv2.destroyAllWindows()

