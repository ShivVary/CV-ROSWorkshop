import cv2  
import numpy as np

# create video capture object
capture = cv2.VideoCapture("File_path")  

while True:
    retval, frame = capture.read()  # retval is a boolean for successful read

    # Exit loop once the end of the video is reached
    if not retval:
        break
    
    cv2.imshow("Display name", frame)
    
    # Close if 'd' is pressed
    if cv2.waitKey(17) & 0xFF == ord('d'):  
        break

# Release the video capture object and close all OpenCV windows
capture.release()
cv2.destroyAllWindows()
