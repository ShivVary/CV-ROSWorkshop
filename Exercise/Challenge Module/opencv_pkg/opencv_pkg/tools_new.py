#!/usr/bin/env python3
"""
File Contains  Tools:
    - CFilters: Smoothing images, darken and brighten images
    - CMaskOp: Perform masking + Thresholding operations
    - CMorphTools: Perform Morphin operations (Its morphin time)
    - CObjectID: Find objects and visualise

File contains utility functions which select vital tools for you to do Open CV tasks
You can modify these functions to your liking. 
"""
import cv2
import numpy as np
from CompVisParams import *
from skimage.measure import label, regionprops,regionprops_table

class CFilters:
    def __init__ (self):
        '''
        Class contains Image Filtering Methods, all contained in functions to include
        common additional functionality for functional quality assurance.
        '''
        self.conversions = {
            "RGB": cv2.COLOR_BGR2RGB,
            "HSV": cv2.COLOR_BGR2HSV,
            "YUV": cv2.COLOR_BGR2YUV,
            "LAB": cv2.COLOR_BGR2LAB,
            "HLS": cv2.COLOR_BGR2HLS,
            "XYZ": cv2.COLOR_BGR2XYZ
        }

        self.GRAY_CONVERSIONS = {
        "BGR2GRAY": cv2.COLOR_BGR2GRAY,
        "RGB2GRAY": cv2.COLOR_RGB2GRAY,
        "BGRA2GRAY": cv2.COLOR_BGRA2GRAY,
        "RGBA2GRAY": cv2.COLOR_RGBA2GRAY,
        "BayerBG2GRAY": cv2.COLOR_BayerBG2GRAY,
        "BayerGB2GRAY": cv2.COLOR_BayerGB2GRAY,
        "BayerRG2GRAY": cv2.COLOR_BayerRG2GRAY,
        "BayerGR2GRAY": cv2.COLOR_BayerGR2GRAY,
        "YUV2GRAY_420": cv2.COLOR_YUV2GRAY_420,
        "YUV2GRAY_YVYU": cv2.COLOR_YUV2GRAY_YVYU,
        "YUV2GRAY_UYVY": cv2.COLOR_YUV2GRAY_UYVY,
        "YUV2GRAY_YUYV": cv2.COLOR_YUV2GRAY_YUYV,
    }

    def GrayScaler(self,aImage, aSpace="BGR2GRAY")-> np.ndarray:
        '''
        Perform grayscaling images depending on original colour space
        Parameters
        ----------
            aImage : RGB image
            aSpace : String value inicating keys for colour spaces.
                Default is BGR

        Returns
        -------
            grayImage : Grayscaled Image
        '''
        assert aImage is not None ,"Invalid image input"
        assert aSpace in self.GRAY_CONVERSIONS, "Invalid Colour Space input"

        grayImage = cv2.cvtColor(aImage, self.GRAY_CONVERSIONS[aSpace])

        return grayImage

    def Blur(self, aImage: np.ndarray, aKernelSize: tuple = (5, 5)) -> np.ndarray:
        """
        Applies an averaging blur filter (cv2.blur).

        Parameters:
            aImage (np.ndarray): Input image.
            aKernelSize (tuple): Kernel size (width, height) for blurring.

        Returns:
            np.ndarray: Blurred image.
        """
        assert aImage is not None, "Invalid image."
        assert isinstance(aKernelSize, tuple) and len(aKernelSize) == 2, "Kernel size must be a tuple (width, height)."
        return cv2.blur(aImage, aKernelSize)

    def GaussianBlur(self, aImage: np.ndarray, aKernelSize: tuple = (5, 5), aSigmaX: float = 0) -> np.ndarray:
        """
        Applies a Gaussian blur filter.

        Parameters:
            aImage (np.ndarray): Input image.
            aKernelSize (tuple): Kernel size (width, height) for Gaussian blur.
            aSigmaX (float): Standard deviation in X direction.

        Returns:
            np.ndarray: Blurred image.
        """
        assert aImage is not None, "Invalid image."
        assert isinstance(aKernelSize, tuple) and len(aKernelSize) == 2, "Kernel size must be a tuple (width, height)."
        return cv2.GaussianBlur(aImage, aKernelSize, aSigmaX)

    def MedianBlur(self, aImage: np.ndarray, aKernelSize: int = 5) -> np.ndarray:
        """
        Applies a median blur filter.

        Parameters:
            aImage (np.ndarray): Input image.
            aKernelSize (int): Kernel size (must be an odd integer).

        Returns:
            np.ndarray: Blurred image.
        """
        assert aImage is not None, "Invalid image."
        assert isinstance(aKernelSize, int) and aKernelSize % 2 == 1, "Kernel size must be an odd integer."
        return cv2.medianBlur(aImage, aKernelSize)

    def BilateralFilter(self, aImage: np.ndarray, aDiameter: int = 9, aSigmaColor: float = 75, aSigmaSpace: float = 75) -> np.ndarray:
        """
        Applies bilateral filtering, which preserves edges while reducing noise.

        Parameters:
            aImage (np.ndarray): Input image.
            aDiameter (int): Diameter of the pixel neighborhood.
            aSigmaColor (float): Filter sigma in the color space.
            aSigmaSpace (float): Filter sigma in the coordinate space.

        Returns:
            np.ndarray: Blurred image.
        """
        assert aImage is not None, "Invalid image."
        assert isinstance(aDiameter, int) and aDiameter > 0, "Diameter must be a positive integer."
        return cv2.bilateralFilter(aImage, aDiameter, aSigmaColor, aSigmaSpace)
    
    def Sharpening (self,aImage,aKernel=(5,5))-> np.ndarray:
        '''
        Perform sharpening

        Parameters
        ----------
            aImage : RGB image
            aKernel : n x n kernel size. The default is (5,5).

        Returns
        -------
            Sharpened : Sharpened RGB image
        '''
        assert aImage is not None ,"Invalid image input"
        assert isinstance(aKernel,tuple),"Invalid kernel input"
        
        # Default kernel for sharpening
        kernel = np.ones(aKernel,np.float32)/25
        Sharpened = cv2.filter2D(aImage, -1, kernel)
        
        return Sharpened
    
    def Darken(self,aImage,aScale = 1.5)-> np.ndarray:
        '''
        Lower brightness of functions

        Parameters
        ----------
            aImage : Image
            aScale : default 1.5
        
        Returns
        -------
            DarkIm: Darken Image
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
    
    def Brighten(self,aImage,aScale = 1.5)-> np.ndarray:
        '''
        Increase brightness of functions

        Parameters
        ----------
            aImage : Image
            aScale : default is 1.5
        
        Returns
        -------
            BriIm: Brighted Image
        '''
        assert aImage is not None, " Image is invalid"
        # Convert image to float32
        ImFloat = aImage.astype(np.float32)
        
        # Darken image 
        BriIm = ImFloat * aScale
        BriIm  = np.clip(BriIm , 0, 255)
        
        # Convert back to 8 bit
        BriIm  = np.uint8(BriIm)
        
        return BriIm
    
class CMaskOp:
    def __init__(self):
        """
        Class for masking operations on binary images using various color spaces.
        """
        # Class attribute: Mapping of color space names to OpenCV conversion codes
        self.COLOR_CONVERSIONS = {
            "RGB": cv2.COLOR_BGR2RGB,
            "HSV": cv2.COLOR_BGR2HSV,
            "YUV": cv2.COLOR_BGR2YUV,
            "LAB": cv2.COLOR_BGR2LAB,
            "HLS": cv2.COLOR_BGR2HLS,
            "XYZ": cv2.COLOR_BGR2XYZ
        }

        # Simple Thresholding types
        self.THRESHOLDER_TYPE = {
            "BINARY": cv2.THRESH_BINARY,
            "BINARY_INV": cv2.THRESH_BINARY_INV,
            "TRUNC": cv2.THRESH_TRUNC,
            "TOZERO": cv2.THRESH_TOZERO,
            "TOZERO_INV": cv2.THRESH_TOZERO_INV,
        }   

        # Adaptive Thresolding types
        self.ADAPTIVE_THRESH ={
            "MEAN": cv2.ADAPTIVE_THRESH_MEAN_C,
            "GAUSSIAN": cv2.ADAPTIVE_THRESH_GAUSSIAN_C
        }

    def SimpleThresholder(self, aGrayImage, aThreshold, aMax, aType="BINARY")-> np.ndarray:
        '''
        Simple Thresholding operation using grayscale images.

        Parameters:
        -----------
            aGrayImage : np.ndarray
                The input grayscale image.

            aThreshold : int or float
                The threshold value.

            aMax : int or float
                The maximum value to use with thresholding.

            aType : str, optional
                The thresholding type (default is "BINARY").
                Supported types: "BINARY", "BINARY_INV", "TRUNC", "TOZERO", "TOZERO_INV", "OTSU", "TRIANGLE".

        Returns:
        --------
            thresholded_image : np.ndarray
                The result of the thresholding operation.
        '''
        assert isinstance(aMax, int), "Max value must be an integer "
        assert isinstance(aThreshold, int), "Threshold must be an integer "
        assert isinstance(aMax, int), "Max value must be an integer "
        assert 0 <= aThreshold <= 255, f"Threshold value {aThreshold} is out of range [0, 255]."
        assert 0 <= aMax <= 255, f"Max value {aMax} is out of range [0, 255]."
        assert aType in list(self.THRESHOLDER_TYPE.keys()), f"Invalid threshold type"

        # Apply the thresholding operation
        _, tImage= cv2.threshold(aGrayImage, aThreshold, aMax, self.THRESHOLDER_TYPE[aType])

        return tImage

    def AdaptiveThresholder(self, aGrayImage, aMax, aMethod="GAUSSIAN", aType="BINARY", aBlockSize=11, aC=2)-> np.ndarray:
        '''
        Adaptive Thresholding operation for grayscale images.

        Parameters:
        -----------
            aGrayImage : np.ndarray
                The input grayscale image.

            aMax : int or float
                The maximum value to use with thresholding (must be in range [0, 255]).

            aMethod : str, optional
                The adaptive thresholding method (default is "GAUSSIAN").
                Supported methods: "MEAN", "GAUSSIAN".

            aType : str, optional
                The thresholding type (default is "BINARY").
                Supported types: "BINARY", "BINARY_INV".

            aBlockSize : int, optional
                Size of the neighborhood area for local threshold calculation (must be **odd** and ≥3).

            aC : int or float, optional
                A constant subtracted from the calculated threshold (can be positive or negative).

        Returns:
        --------
            thresholded_image : np.ndarray
                The result of the adaptive thresholding operation.
        '''
        assert aGrayImage is not None, "Image is invalid"
        assert len(aGrayImage.shape) == 2 or (len(aGrayImage.shape) == 3 and aGrayImage.shape[2] == 1), "Image is not grayscale."
        assert isinstance(aMax, int), "Max value must be an integer "
        assert 0 <= aMax <= 255, f"Max value {aMax} is out of range [0, 255]."
        assert aMethod in list(self.ADAPTIVE_THRESH.keys()), f"Invalid method"
        assert aType in list(self.THRESHOLDER_TYPE.keys()), f"Invalid threshold type"
        assert isinstance(aBlockSize, int) and aBlockSize % 2 == 1 and aBlockSize >= 3, "Block size must be an odd integer ≥ 3."
        assert isinstance(aC, int), "C must be an integer or float."

        # Select OpenCV adaptive method
        method = self.ADAPTIVE_THRESH[aMethod]

        # Select thresholding type
        thresholdType = self.THRESHOLDER_TYPE[aType]

        # Apply adaptive thresholding
        tImage = cv2.adaptiveThreshold(aGrayImage, aMax, method, thresholdType, aBlockSize, aC)

        return tImage


    def MaskImage(self, aColour, aImage: np.ndarray, aSpace: str = "HSV", aModProfile: bool = False) -> np.ndarray:
        """
        Masks an image based on the given colour space and colour profile.

        Parameters:
        -----------
            aColour : Colour Space numbers for masking operation

            aImage : np.ndarray
                The input image in RGB/BGR format.

            aSpace : str, optional
                The color space to use for masking (default is "HSV").
                Supported values: "RGB", "HSV", "YUV", "LAB", "HLS", "XYZ".

            aModProfile : bool, optional
                Whether to use the modified colour range values (default is False).

        Returns:
        --------
            mask: A binary mask (0 and 255) where the selected color range is white.
        """
        assert aImage is not None, " Image is invalid"
        assert aSpace in list[self.COLOR_CONVERSIONS.keys()], "Invalid Colour Space input"

        # Convert image to the selected color space
        convertedImage = cv2.cvtColor(aImage, self.COLOR_CONVERSIONS[aSpace])

        # Select colour ranges based on prefernces
        colourRange = None
        if hasattr(aColour,"HSV_MODIFIED"):
            colourRange = getattr(aColour, "HSV_MODIFIED") if aModProfile else getattr(aColour, "HSV_DEFAULT")
        
        else:
            colourRange =getattr(aColour, "HSV_DEFAULT")

        # Ensure color range is a NumPy array with shape (2,3)
        if not isinstance(colourRange, np.ndarray) or colourRange.shape != (2, 3):
            raise ValueError(f"{aColour} range should be a NumPy array of shape (2, 3).")

        ####
        try:
            # Create the binary mask
            mask = cv2.inRange(convertedImage, colourRange[0], colourRange[1])
            return mask
        except:
            # Ehh what the fuck
            raise RuntimeError("Masking operation failed.")



class CMorphTools:
    def __init__(self):
        
        pass
    
    def MorphImage(self,aBinImage,aMode,aShape="RECTANGLE",aKSize = (5,5))-> np.ndarray:
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
        assert aShape in list(MORPH_SHAPE.SHAPE_LIST.keys()) ,"Invalid kernel shape"
        assert aMode in list(MORPH_OPERATION.MORPH_OPS.keys()), "Invalid morph operation key"
        
        # Construct kernel based on shapes and size
        shape = MORPH_SHAPE.SHAPE_LIST[aShape]
        mode = MORPH_OPERATION.MORPH_OPS[aMode]
        kernel = cv2.getStructuringElement(shape, aKSize)
        
        # Perform specified operation
        MorphedImage = cv2.morphologyEx(aBinImage, mode , kernel)
        
        # return image
        return MorphedImage

class CObjectID:
    def __init__(self):
        pass
    def FilterRegions(self,aBinImage):
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
                if r.filled_area > 1500:
                    FilteredRegions.append(r)
        
        return FilteredRegions


    def Visualise(self, aRegions, aImage):
        '''
        Draw bounding boxes on an image.

        Parameters
        ----------
            aRegions : list
                List of detected regions (from skimage.measure.regionprops).

            aImage : np.ndarray
                RGB image on which bounding boxes will be drawn.

        Returns
        -------
            image_copy : np.ndarray
                Image with bounding boxes drawn.
        '''
        assert aImage is not None, "Invalid image input"
        assert isinstance(aRegions, list), "Invalid region input"
        
        # Create a copy of the input image to avoid modifying the original
        image_copy = aImage.copy()
        
        # Set thickness for bounding box
        thickness = 2

        for props in aRegions:
            minr, minc, maxr, maxc = props.bbox
            cv2.rectangle(image_copy, (minc, minr), (maxc, maxr), (0, 0, 255), thickness)

        return image_copy  # Return the modified image instead of modifying in-place


            