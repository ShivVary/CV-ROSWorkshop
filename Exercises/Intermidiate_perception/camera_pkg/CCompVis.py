#!/usr/bin/env python3
"""
Main class that encapsulates all computer vision. Design principle for this modules is as follows:

- All functionalities/ modules of computer vision operations should be encapsulated as TOOLS
    in tools.py then instantiate it as a member of CCompVis class here

- Imagine CCompVis class as an actual Computer built for just computer vision. The tools are its apps and
    self.mImage is the memory -> dont worry to much about this tho but u get the picture :)))

- Use the tools and define class methods as CCompVis as designated tasks. 
- Datas tructures should be clear and succinct and delcared in CompVisParams.py


@author: Thomas, Alvin
"""
import cv2
import os
import numpy as np
from camera_pkg.tools import *
from camera_pkg.CompVisParams import *
import matplotlib.pyplot as plt

class CCompVis:
    '''
    Class encapsulates all functionalities of a computer vision module
    - Read raw image with specified file name
    - Pre processing
    - Masking operations
    - Morphological operations
    - Object identification tool
    
    '''
    def __init__(self):
        self.mImage = None                      # Raw image storage

        # SET OF TOOLS
        self.mPreProcessTool = CPreProcess()    # Pre-processing tool
        self.mMasker = CMaskOp()                # Image masking tool
        self.mMorphTools = CMorphTools()        # Morphing tools 
        self.mIDTool = CObjectID()              # Object identifcation tools 

    def ReadRawImage(self,aImgFileName):
        """
        Read image with specified path. Check if image exist and successfully 
        read

        Parameters
        ----------
        aImgFileName (str) : image path
        
        Return
        ______
        (bool) - True = image successfully read
               - False = image not read 
        """
        # Check if image path is correct
        if os.path.exists(aImgFileName):
            self.mImage = cv2.imread(aImgFileName)
            self.mFile = aImgFileName
            # Check if image is correctly read
            if self.mImage is not None:
                #print(f"{aImgFileName} is successfuly read!")
                return True
            
            else:
                print("Image found but NOT READ")
                return False
            
        else:
            print(f"{aImgFileName} DOES NOT EXIST!")
            return False

    def AllCylinderDetector(self):
        """
        Function for detect all 8 cylinders (Look in CompVisParams.py to find all colours).
        Returns a dataset that contains which colour are found and their bbox and centroids 
        Draws the bbox and centroids on the map 

        COMP VIS PROCEDURES
        Filtering:
            - Sharpening to preseve edges
            - Red blue - only sharpening, 
            - Black whole shebang
            
        Masking:
            - Convert colour spaces and do a pixel matching operation to 
            fully construct cylinders objects
            - Close ,Open, Erode, Dilate bit image
        
        Contours/ Centroids:
            - Find contours, Centroids
            - Centroids = Area, number of pixels, position of centre
            - Get Bounding box coordinates to get max size 
        
        Post Processing:
            - Label colour and bounding box 
        
        NOTE FROM ASSIGNMENT 3: THIs is for all 8 cylinders. Still applicable for Major project
        Note -> HOWEVER, for major project, for each colour, more then 1 bounding boxes will be 
        painted in the camera frame as Major project accounts for multiple objects of the same
        colour 
        """
        # PRE PROCESSING
        # Check if everything is set up
        assert self.mImage is not None, "No image has been provided"

        # # Split the HSV image into channels
        h, s, v = cv2.split(self.mImage)

        # Calculate the mean brightness (value channel)
        mean_brightness = np.mean(v)
        
        # perform pre processing for image
        # each colour may require different pre-processed image 
        DarkenedImg = self.mPreProcessTool.Darken(self.mImage,3.5)
        BrightenedImg = self.mPreProcessTool.Brighten(self.mImage,1.5)
        
        #_____________________
        # MASKNG AND MORPHING
        OPEN = MORPH_OPERATION.OPEN
        RECT = MORPH_SHAPE.RECTANGLE
        
        # For lilac, Marron and Red, there is special treatment 
        Cylinders = CYLINDERS()
        for colour, data in Cylinders.AllColours.items():
            # Perform masking on all colours
            SpecialColours = (data["color"] == "lilac") or (data["color"] == "maroon") or (data["color"] == "red")
            if SpecialColours:
                if mean_brightness > 105:
                    data["M"] = self.mMasker.MaskImage(colour,self.mImage,True)

                else:
                    # For lilac, brigten the images
                    if not (data["color"] == "lilac"):
                        data["M"]= self.mMasker.MaskImage(colour,BrightenedImg)
                    
                    else:
                        data["M"]= self.mMasker.MaskImage(colour,self.mImage)

            elif data["color"] == "black":
                data["M"]= self.mMasker.MaskImage(colour,DarkenedImg)
            
            else:
                data["M"]= self.mMasker.MaskImage(colour,self.mImage)
            
            data["M"] = self.mMorphTools.MorphImage(data["M"],RECT,OPEN)
            
            # Use the masks to find regions that may resemble cylinders
            PotentialRegions = self.mIDTool.GetProperties(data["M"])

            # Identify Coloured objects (Legos and Cylinders)
            if len(PotentialRegions) != 0:
                data["F"] = True
                self.mIDTool.FindColours(PotentialRegions,data)
        
        # DRAW THE IMAGES 
        self.mIDTool.Visualise(Cylinders.AllColours,self.mImage)

        # DATASET 
        return Cylinders.AllColours

    def MaskStacker(self,aCylinders):
        '''
        Display all 8 binary images from masking 8 colours
            First Row = Yellow, Red, Maroon, Lilac
            Second Row = Blue, Green, Magneta, Black

            Also put text indicating their color on top left corner 
        
        !!! Use for debugging masks or tuning HSV color space 
        '''
        # Stack on all masks
        AllColours = list(aCylinders.keys())

        # Define the position (top-left corner)
        position = (10, 30)  # (x, y) coordinates

        # Define font, scale, color, and thickness
        font = cv2.FONT_HERSHEY_SIMPLEX

        # Draw the text on the image
        for color,data in aCylinders.items():
            cv2.putText(data["M"],color.colour_str,position,font,1.0,(255,255,255),1)
        
        firstRow = aCylinders[AllColours[0]]["M"]
        secRow = aCylinders[AllColours[4]]["M"]
        
        for i in range(1,int(len(AllColours)/2)):
            firstRow = np.hstack((firstRow,aCylinders[AllColours[i]]["M"]))
            secRow= np.hstack((secRow,aCylinders[AllColours[i+4]]["M"]))
        
        MaskStack = np.vstack((firstRow,secRow))

        return MaskStack
