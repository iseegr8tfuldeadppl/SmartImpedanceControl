import cv2
import numpy as np

cv2.namedWindow('output', cv2.WINDOW_AUTOSIZE)
width = 400
height = 400
window = np.zeros((height, width, 3), dtype=np.uint8)

while True:
    cv2.imshow('output', window)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cv2.destroyAllWindows()