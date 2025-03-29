import cv2
import numpy as np
from skimage.measure import label, regionprops

# Specify your image name and/or directory
aImgFileName = "cv_pics/red_9.jpg"
Image = cv2.imread(aImgFileName)

# Convert BGR to HSV
HSVImage = cv2.cvtColor(Image, cv2.COLOR_BGR2HSV)

# Define lower and upper range of HSV values
HSV_LOWER = np.array([170, 150, 70])
HSV_UPPER = np.array([179, 255, 255])

# Mask image with defined HSV values
HsvMask = cv2.inRange(HSVImage, HSV_LOWER, HSV_UPPER)

# Construct kernel for morphological operation
Shape = cv2.MORPH_RECT
KSize = (5, 5)  # 5x5 kernel
kernel = cv2.getStructuringElement(Shape, KSize)

# Perform closing operation
mode = cv2.MORPH_CLOSE
MorphedImage = cv2.morphologyEx(HsvMask, mode, kernel)

# Label connected components
LabelIm = label(MorphedImage)
props = regionprops(LabelIm)
thick = 2

# Loop through all detected regions
for region in props:
    # Get bounding box
    minr, minc, maxr, maxc = region.bbox

    # Get centroid and convert to integers for OpenCV
    centroid = tuple(map(int, region.centroid))

    # Draw bounding box
    cv2.rectangle(Image, (minc, minr), (maxc, maxr), (255, 0, 0), thick)

    # Draw centroid as a black dot
    cv2.circle(Image, centroid, 5, (0, 0, 0), -1)

    # Label object near the centroid
    black = (255, 255, 255)
    text_position = (centroid[0] - 10, centroid[1] - 10)
    cv2.putText(Image, "Object", text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, black , thick)

# Display result
cv2.imshow("Detected Objects", Image)
cv2.waitKey(0)
cv2.destroyAllWindows()
