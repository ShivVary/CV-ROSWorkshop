import tools as tool
import cv2
from CompVisParams import *
import matplotlib.pyplot as plt
from skimage.measure import label, regionprops, regionprops_table

def main(file_name):
    # Instantiate the tools
    filters = tool.CFilters()
    masker = tool.CMaskOp()
    morpher = tool.CMorphTools()
    identifier = tool.CObjectID()

    # Set file path
    file_name = "cv_pics/smarties.png"

    # Get raw image
    raw_image = cv2.imread(file_name)

    # Gray scale
    gray_scaled = filters.GrayScaler(raw_image)

    # Gaussian filter
    noise_filtered = filters.GaussianBlur(gray_scaled)

    # Simple thresholder
    t = masker.SimpleThresholder(noise_filtered,170,255,aType="BINARY_INV")

    # Morph
    m = morpher.MorphImage(t,aMode="CLOSE",aKSize=(8,8),aShape="ELLIPSE")

    # Titles for each image
    image_dict = {
        "Raw Image": raw_image,
        "Thresholded (FILTERED)": t,
        "Morphed + Filtered": m,
    }

    fig, axes = plt.subplots(1, len(image_dict), figsize=(18, 4))

    for ax, (title, img) in zip(axes, image_dict.items()):
        ax.imshow(img, cmap="gray" if len(img.shape) == 2 else None)
        ax.set_title(title)
        ax.axis("off")

    plt.tight_layout()
    plt.show()

if __name__=="__main__":
    file_name = "cv_pics/smarties.png"
    main(file_name)
