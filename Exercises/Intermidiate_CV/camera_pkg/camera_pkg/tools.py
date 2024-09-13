#!/usr/bin/env python3
"""
File Contains  Tools:
    - CPreProcess: Perform equalisation, blurring, sharpening and darken
    - CMaskOp: Perform masking operations
    - CMorphTools: perform morphin operations
    - CObjectID: Find cylinders
    
@author: Thomas, Alvin
"""
import cv2 
import numpy as np
from camera_pkg.CompVisParams import *
from skimage.measure import label, regionprops

class CPreProcess:
    def __init__ (self):
        pass

    def LABnHSVEqualise(self,aImage):
        '''
        Equalising L from LAB then equalise the V from HSV of the prior equalised
        image.
         - L equalising => robust to lightign
         - V equalising + amplifys => bring more "trueness" to the colours
        
        Initial intention is to reduce the reflectiveness caused by lighting on
        the cylinders as well as equalising lighting. However, it worked against
        our favour especially with RED and BLUE. However, it helped with black
        cylinders
        
        Parameters
        ----------
        aImage = a RGB image
        
        Return
        ______
        FinalImage = equalised image
        
        '''
        assert aImage is not None ,"Invalid image input"
        CLAHE = cv2.createCLAHE(clipLimit=1.5)
        
        # LAB EQUALISATION 
        LAB = cv2.cvtColor(aImage, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(LAB)
        
        NewL = CLAHE.apply(l)
        
        # Convert back to RGB to convert to HSV later
        # Merge the CLAHE-enhanced L channel with AB
        NewLAB= cv2.merge((NewL, a, b))
        
        # Convert back to BGR color space
        NewLabImage = cv2.cvtColor(NewLAB, cv2.COLOR_LAB2BGR)
        
        # # HSV EQUALISATION
        HSV = cv2.cvtColor(NewLabImage, cv2.COLOR_BGR2HSV)
        h,s,v = cv2.split(HSV)
        NewS = CLAHE.apply(s)
        
        NewSatAmp = np.clip(NewS * 2, 0, 255)
        
        #Merge the CLAHE-enhanced V channel HS to get final image
        NewHSV = cv2.merge((h,NewS,v))
        
        FinalImage = cv2.cvtColor(NewHSV, cv2.COLOR_HSV2BGR)
        
        return FinalImage
    
    def GaussianBlur(self,aImage, aKernel=(5,5)):
        '''
        Perform gausian blur

        Parameters
        ----------
        aImage : RGB image
        aKernel : n x n kernel size. The default is (5,5).

        Returns
        -------
        Blur : Blur RGB image
        '''
        # Check for corect inputs
        assert aImage is not None ,"Invalid image input"
        assert isinstance(aKernel,tuple),"Invalid kernel input"
        
        Blur = cv2.GaussianBlur(aImage, aKernel, 0)

        return Blur
    
    def Sharpening (self,aImage,aKernel=(5,5)):
        '''
        Perform sharpening

        Parameters
        ----------
        aImage : RGB image
        aKernel : n x n kernel size. The default is (5,5).

        Returns
        -------
        Blur : Blur RGB image
        '''
        assert aImage is not None ,"Invalid image input"
        assert isinstance(aKernel,tuple),"Invalid kernel input"
        
        # Default kernel for sharpening
        kernel = np.ones(aKernel,np.float32)/25
        Sharpened = cv2.filter2D(aImage, -1, kernel)
        
        return Sharpened
    
    def Darken(self,aImage,aScale = 1.5):
        '''
        Lower brightness of functions

        Parameters
        ----------
        aImage : RGB image
        aScale : 1.5
        
        Returns
        -------
        Blur : Blur RGB image
        '''
        assert aImage is not None, " Image is invalid"
        # Convert image to float32
        ImFloat = aImage.astype(np.float32)
        
        # Darken image 
        DarkIm = ImFloat /aScale
        DarkIm  = np.clip(DarkIm , 0, 255)
        
        # Convert back to 8 bit
        DarkIm  = np.uint8(DarkIm )
        
        return DarkIm
    
    def Brighten(self,aImage,aScale = 1.5):
        '''
        Increase brightness of functions

        Parameters
        ----------
        aImage : RGB image
        aScale : 1.5
        
        Returns
        -------
        Blur : Blur RGB image
        '''
        assert aImage is not None, " Image is invalid"
        # Convert image to float32
        ImFloat = aImage.astype(np.float32)
        
        # Darken image 
        BriIm = ImFloat * aScale
        BriIm  = np.clip(BriIm , 0, 255)
        
        # Convert back to 8 bit
        BriIm  = np.uint8(BriIm )
        
        return BriIm
    

class CMaskOp:
    def __init__(self):
        pass

    def MaskImage(self,aColour,aImage,aModProfile = False):
        '''
        Masked image based on HSV color profile given. HSV values can be changed
        in CompVisParam.py file

        Parameters
        ----------
        aColour : Colour profile that contains hsv values for masking
        aImage : RGB image
        aProfile: Default or modified HSV values of colour

        Returns
        -------
        HsvMask : Binary image (0,1)

        '''
        # parse inputs
        assert aImage is not None,"Invalid image"
        #assert aColour is COLOUR_PROFILE, "Invalid colour profile input"
        
        HSVImage = cv2.cvtColor(aImage, cv2.COLOR_BGR2HSV)
        HSVMask = None

        if aModProfile:
            HSVMask = cv2.inRange(HSVImage,aColour.HSV_MODIFIED[0,:],aColour.HSV_MODIFIED[1,:])

        else:
            HSVMask = cv2.inRange(HSVImage,aColour.HSV_DEFAULT[0,:],aColour.HSV_DEFAULT[1,:])

        assert HSVMask is not None, "Masking operation FAILED"
        
        return  HSVMask
        

class CMorphTools:
    def __init__(self):
        pass
    
    def MorphImage(self,aBinImage,aShape,aMode,aKSize = (5,5)):
        '''
        Perform morphological images based on specified 
        
        Parameters
        ----------
        aBinImage : Binary inmage
        aShape :  Kernel Shape, RECTANGLE, CROSS, ELLISPE
        aMode : OPEN,CLOSE,DILATE,ERODE
        aKSize : Kernel size n x n
            DESCRIPTION. The default is (5,5).

        Returns
        -------
        MorphedImage 

        '''
        assert aBinImage is not None ,"Invalid binary image input"
        assert aShape in MORPH_SHAPE.SHAPE_LIST ,"Invalid kernel shape"
        assert aMode in MORPH_OPERATION.MORPH_OPS, "Invalid morph operation key"
        
        # Construct kernel based on shapes and size
        kernel = cv2.getStructuringElement(aShape, aKSize)
        
        # Perform specified operation
        MorphedImage = cv2.morphologyEx(aBinImage, aMode, kernel)
        
        # return image
        return MorphedImage

class CObjectID:
    def __init__(self):
        self.mCTL = np.array([[np.cos(0.208),-np.sin(0.208),0.0026],
                              [np.sin(0.208),np.cos(0.208),0.012322],
                              [0,0,1]])
        
    def GetProperties(self,aBinImage):
        '''
        Get regions with filled area > 2000

        Parameters
        ----------
        aBinImage : Binary image

        Returns
        -------
        FilteredRegions : list of regions 

        '''
        assert aBinImage is not None ,"Invalid binary image input"
        
        LabelIm = label(aBinImage)
        
        regions = regionprops(LabelIm)    
        FilteredRegions = []
        if regions:
            for r in regions:
                if r.filled_area > 500:
                    FilteredRegions.append(r)
        
        return FilteredRegions
        
    def FindColours(self,aRegions,aCylinderData):
        '''
        Find the actual cylinder

        Parameters
        ----------
        aRegions : Regions of cylinder

        Returns
        -------
        Data that contains all identified objects with their bounding boxes and centroids

        '''
        aCylinderData["BB"] = []
        aCylinderData["C"]= []
        # Iterate through all regions
        # Find both legos and cylinders
        for region in aRegions:
            BBoxCoords = region.bbox
            
            x1 = BBoxCoords[1]
            x2 = BBoxCoords[3]
                    
            y1 = BBoxCoords[0]
            y2 = BBoxCoords[2]

            aCylinderData["BB"].append(((x1,x2),(y1,y2)))

            # Use bounding box to calculate centroid coordinates
            # Calculate centroid
            midX = int((x2+x1)/2)
            midY = int((y2+y1)/2)
            
            aCylinderData["C"].append((midX,midY))


    def Visualise(self,aCylindersFound,aImage):
        '''
        Draw bounding box,on frame

        Parameters
        ----------
        aFound: a list of colour found
        aImage : RGB image
    
        '''
        assert aImage is not None, 'Invalid image input'

        # Params for drawing on pictures
        font = cv2.FONT_HERSHEY_SIMPLEX 
        fontScale = 1
        thickness = 2
        
        for key,data in aCylindersFound.items():
            if data["F"]:
                for i in range(len(data["BB"])):
                    bb = data["BB"][i]
                    x1 = bb[0][0]
                    x2 = bb[0][1]

                    y1 = bb[1][0]
                    y2 = bb[1][1]

                    LWRatio = abs(y2 - y1)/abs(x2-x1)

                    centroid = data["C"][i]

                    # Visualise found cylinders on screen
                    if LWRatio > (1.4):
                        if (x2 <= 640) and (y2 <=480):
                            # Draw bounding box
                            cv2.rectangle(aImage, (x1,y1),
                                                (x2,y2), data["BGR"],thickness)
                            
                            # Draw centroid
                            cv2.circle(aImage, centroid  , 5, data["BGR"], -1)
