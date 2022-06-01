import numpy as np
from multiprocessing import Process, Pipe
from multiprocessing.sharedctypes import Value, Array
import cv2
import keyboard
import time

# https://docs.python.org/3/library/multiprocessing.html#sharing-state-between-processes

height = 400
width = 400

index = 0

# https://stackoverflow.com/questions/23816222/python-shared-string-memory-for-multiprocessing
def mainLoop(consumer, producer2):
    global index
    # allow the user to edit the desired point real time
    cv2.destroyAllWindows()
    cv2.namedWindow('output', cv2.WINDOW_AUTOSIZE)
    while True:
        window = np.ones((height, width, 3), dtype=np.uint8) *255
        cv2.imshow('output', window)

        if consumer.poll():
            val = consumer.recv()
            if val==None:
                producer2.send("closing")
                break
            producer2.send("looping")
            #print("received in thread", val)

        # for quitting
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

            
        if keyboard.is_pressed('k'):
            index += 1
            print(index)
            

if __name__ == '__main__':
    (consumer,producer) = Pipe(False)
    (consumer2,producer2) = Pipe(False)
    process = Process(target=mainLoop, args=(consumer,producer2))
    process.start()

    # start main Gui thread
    while True:
        # for quitting
        #print(time.time())
        #if keyboard.is_pressed('f'):
        #producer.send(time.time())
        if keyboard.is_pressed('v'):
            producer.send(None)
            process.join()
            if consumer2.poll():
                val = consumer2.recv()
                print(val)
            break

            
        if keyboard.is_pressed('l'):
            index += 1
            print(index)

    # clean up
    cv2.destroyAllWindows()