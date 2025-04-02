#!/usr/bin/env python
"""
File contains PARAMETERS FOR functions in tools.py to utilise
Change if you want
"""
import cv2
import numpy as np
from dataclasses import dataclass,field

@dataclass(frozen=True)
class MORPH_SHAPE:
    RECTANGLE = cv2.MORPH_RECT
    ELLIPSE = cv2.MORPH_ELLIPSE
    CROSS = cv2.MORPH_CROSS

    # Dictionary for easy key-based access
    SHAPE_LIST = {
        "RECTANGLE": RECTANGLE,
        "ELLIPSE": ELLIPSE,
        "CROSS": CROSS,
        }

dataclass(frozen=True)
class MORPH_OPERATION:
    CLOSE = cv2.MORPH_CLOSE
    OPEN = cv2.MORPH_OPEN
    DILATE = cv2.MORPH_DILATE
    ERODE = cv2.MORPH_ERODE
    GRADIENT = cv2.MORPH_GRADIENT
    TOP_HAT = cv2.MORPH_TOPHAT
    BLACK_HAT = cv2.MORPH_BLACKHAT

    # Dictionary to allow key-based access
    MORPH_OPS = {
        "CLOSE": CLOSE,
        "OPEN": OPEN,
        "DILATE": DILATE,
        "ERODE": ERODE,
        "GRADIENT": GRADIENT,
        "TOP_HAT": TOP_HAT,
        "BLACK_HAT": BLACK_HAT,
    }

# All colours and labels
# Base class for colour profiles
@dataclass
class COLOUR_PROFILE:
    '''
    base class for colour profiles - immutable, only stores relavent parameters for CV to 
                                    do its tasks 
    '''
    label: int             = field(default_factory=int)
    colour_str: str        = field(default_factory=str)
    BGR: tuple             = field(default_factory=tuple)
    HSV_DEFAULT: np.array  = field(default_factory=np.array)
    HSV_MODIFIED: np.array = field(default_factory=np.array)


# HSV VALUES FOR SOME COLOURS
@dataclass
class YELLOW(COLOUR_PROFILE):
    label: int = 0
    colour_str: str = "yellow"
    BGR: tuple = (0,255,255)
    HSV_DEFAULT: np.array =  np.array([[18,20,5],[35,255,200]])

@dataclass
class RED(COLOUR_PROFILE):
    label: int = 1
    colour_str: str = "red"
    BGR: tuple = (255,0,0)
    HSV_DEFAULT: np.array =  np.array([[170,150,70],[179,255,255]])
    HSV_MODIFIED: np.array = np.array([[173,110,180],[179,255,255]])

@dataclass
class MAROON(COLOUR_PROFILE):
    label: int = 2
    colour_str: str = "maroon"
    BGR: tuple = (255,150,150)
    HSV_DEFAULT: np.array =  np.array([[168,180,104],[179,255,190]])
    HSV_MODIFIED: np.array = np.array([[164,227,98],[178,243,140]])

@dataclass
class LILAC(COLOUR_PROFILE):
    label: int = 3
    colour_str: str = "lilac"
    BGR: tuple = (255, 255, 255)
    HSV_DEFAULT: np.array =  np.array([[100,0,200],[179,70,255]])
    HSV_MODIFIED: np.array = np.array([[144,0,44],[158,54,94]])

@dataclass
class BLUE(COLOUR_PROFILE):
    label: int = 4
    colour_str: str = "blue"
    BGR: tuple = (255,0,0)
    HSV_DEFAULT: np.array =  np.array([[90,120,30],[120,255,255]])

@dataclass
class GREEN(COLOUR_PROFILE):
    label: int = 5
    colour_str: str = "green"
    BGR: tuple = (0,255,0)
    HSV_DEFAULT: np.array =  np.array([[40,10,20],[75,255,255]])

@dataclass
class MAGENTA(COLOUR_PROFILE):
    label: int = 6
    colour_str: str = "magenta"
    BGR: tuple = (255,0,255)
    HSV_DEFAULT: np.array =  np.array([[142,100,30],[162,255,255]])

@dataclass
class BLACK(COLOUR_PROFILE):
    label: int = 7
    label: int = 7
    colour_str: str = "black"
    BGR: tuple = (0,0,0)
    HSV_DEFAULT: np.array =  np.array([[19,75,17],[117,100,28]])
#------------------------------------------------------------
class CYLINDERS:
    '''
    This class stores relevant colour profiles of all cylinders in A3. 
    U can request specific colours by inputting a list of colour profiles
    Otherwise it returns ALL. 

    For every colour profile, it contains the following data and its key:
    <color> = colour string
    <BGR>   = tuple of Blue Green Red colour values (0-255)
    <BB>    = tuple of tuples ((x1,x2),(y1,y2)) bounding box coordinates
    <C>     = tuple of (x,y) centroid pixels
    <M>     = 640 x 480 array of 1 and 0s, Masked image
    <F>     = boolean True/False whether the object is identified in frame
    <R>     = (x,y) relative measurements from lidar
    '''
    def __init__(self) -> None:
        self.AllColours = {
            YELLOW:None,
            RED:None,
            MAROON:None,
            LILAC:None,
            BLUE:None,
            GREEN:None,
            MAGENTA:None,
            BLACK:None
        }

        for color,_ in self.AllColours.items():
            self.AllColours[color] = {"ID":color.label,"color":color.colour_str,"BGR":color.BGR,"BB":None,"C":None,"M":None,"F":False,"R":None}
        
        
    def GetAllColours(self):
        return self.AllColours

    def GetSpecificColours(self,aColourList):
        '''
        Insert a list of colour profile class to request colours. 
        '''
        if len(aColourList) == 0:
            print("No colour requested, ur getting all colours, bitch-ass")
            self.GetAllColours()

        else:
            NewParams = {}
            for color in aColourList:
                NewParams[color] = {"ID":color.label,"color":color.colour_str,"BGR":color.BGR,"BB":None,"C":None,"M":None,"F":False,"R":None}
        
            return NewParams
    

        

    