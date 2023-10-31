from PyQt5.QtGui import QImage, QPixmap
from PyQt5.uic import loadUi
from PyQt5.QtCore import pyqtSlot, QTimer,QThread,pyqtSignal
from PyQt5.QtWidgets import QDialog
from PyQt5.QtMultimedia import QCameraInfo

import cv2 as cv
import numpy as np
from ipdb import set_trace as pst
import matplotlib.pyplot as plt
from PIL import Image as im

# from pathlib import Path
# import sys

# sys.path.append(Path("../comp-vision").resolve().as_posix())
import vision_func as vf
import os, sys

# if sys.platform.startswith("linux") and ci_and_not_headless:
#     os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH")
# if sys.platform.startswith("linux") and ci_and_not_headless:
#     os.environ.pop("QT_QPA_FONTDIR")

class imagingComponents():
    '''
    Used to display, change and upload images into the GUI
    '''
    _camlist = []
    try:
        _camera =[0,QCameraInfo.defaultCamera().deviceName]
    except:
        _camera = [0,0]
    _image = r"GUI/default.png"
    # _image = "comp_vision/img/100-ohm-res.png"
    def __init__(self):
        imagingComponents.setImg(self,imagingComponents._image)
        imagingComponents.setAvailableCams(self)

    def setAvailableCams(self):
        camlist = []
        for i, c in enumerate(QCameraInfo.availableCameras()):
        
            camlist.append([i, c.deviceName])
        print(camlist)

        imagingComponents._camlist = camlist
    def getAvailableCams(self):
        '''
        Returns available cams
        '''
        return imagingComponents._camlist
    def setCam(self,cam):
        '''
        Sets the webcam being used
        '''
        imagingComponents._camera = cam
        
    def getCam(self):
        '''
        Returns camera being used
        '''
        return imagingComponents._camera
    
    def setImg(self,image):
        '''
        Initialises the image to be used 
        '''
        imagingComponents._image = image

    def getResistanceValue(self):
        '''
        Returns the value of resistor
        '''
        return str(imagingComponents._value)
    def getColourBands(self):
        '''
        Returns an array of colour bands 
        '''
        c = imagingComponents._colours
        # Only gets the 'text' of the array 
        #IF USING GETCONTOURS U NEED BELOW CODE
        ##################################################
        # colournames = [array[2] for array in c]    
        # ############################################   
        return c
    def getTolerance(self):
        '''
        Returns tolerance value of resistor 
        '''
        return str(imagingComponents.tolerance)

    def getImg(self):
        '''
        Returns image to be analysed
        '''
        return imagingComponents._image
    
    def getMask(self):
        '''
        Gets the cropped img of the component 
        ''' 

        mask = imagingComponents._mask
        value = im.fromarray(mask)
        return value
    
    def findColours(self):
        '''
        Using Jared's colour code to find colour bands 
        '''
                # Color and value mappings
    #         colours = [
    #         [(0, 73, 0)      , (179, 255, 18)     ,'BLACK'],
    #         [(0, 139, 0)    , (9, 218, 125)    ,'BROWN'],
    #         [(0, 112, 151)    , (5, 255, 255)     ,'RED'],
    #         [(15, 175, 0)   , (25, 255, 255)    ,'ORANGE'],
    #         [(30, 170, 100) , (40, 250, 255)  ,'YELLOW',],
    #         [(38, 40, 0)    , (90, 225, 255)    ,'GREEN'],
    #         [(68, 61, 0)    , (121, 255, 255)   ,'BLUE'],
    #         [(120, 40, 100) , (140, 250, 220) ,'PURPLE'],
    #         [(0, 0, 50)     , (179, 50, 80)   ,'GREY'],
    #         [(0, 0, 90)     , (179, 15, 250)  ,'WHITE'],
    #         [(18, 118, 52)  , (21, 255, 255)    ,'GOLD'],
    # ]
        colours = [
        [(0, 0, 0)      , (179, 194, 78)     ,'BLACK', (50, 50, 50)],
        [(0, 129, 45)    , (11, 255, 144)    ,'BROWN', (0, 0, 139)],
        [(0, 140, 99)    , (5, 255, 255)     ,'RED', (0, 0, 255)],
        [(12, 175, 0)   , (25, 255, 255)    ,'ORANGE', (0, 165, 255)],
        [(30, 170, 100) , (40, 250, 255)  ,'YELLOW',(0, 255, 255)],
        [(38, 40, 0)    , (90, 225, 255)    ,'GREEN', (0, 255, 0)],
        [(68, 61, 0)    , (121, 255, 255)   ,'BLUE', (255, 0, 0)],
        [(120, 40, 100) , (140, 250, 220) ,'PURPLE', (255, 0, 255)],
        [(0, 0, 50)     , (179, 50, 80)   ,'GREY', (128, 128, 128)],
        [(0, 0, 90)     , (179, 15, 250)  ,'WHITE', (255, 255, 255)],
        [(16, 116, 136)  , (16, 116, 136)    ,'GOLD', (0, 215, 255)],
        ]
        # [(16, 116, 136)  , (17, 201, 198)    ,'GOLD', (0, 215, 255)],
        # tolerance mappings
        tolerances = {10: '5%', 9: '10%', 8: '20%', 1: '1%', 2: '2%'}
        print(imagingComponents._image)
        # imagingComponents._image = imagingComponents._image.replace("GUI\\","")
        print("PRINTT")
        print(imagingComponents._image)
        img = cv.imread(imagingComponents._image)
        outputimg = img
        # Check if the image is in portrait mode
        if img.shape[0] > img.shape[1]:
            # Rotate the image by 90 degrees
            img = cv.rotate(img, cv.ROTATE_90_CLOCKWISE)

        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        rows, cols = img.shape[:2]
        img = hsv[rows//2-15:rows//2+15, 5:cols-5]  # Update to your submat
        img = cv.bilateralFilter(img, 5, 80, 80)  # bilateral filter
        # Calculate total image area
        total_image_area = img.shape[0] * img.shape[1]

        # Mask for blue colour
        blue_lower = np.array([68, 61, 0])
        blue_upper = np.array([121, 255, 255])
        blue_mask = cv.inRange(img, blue_lower, blue_upper)
        blue_area = cv.countNonZero(blue_mask)

        # If blue takes more than 20% of image, ignore blue bands
        ignore_blue = blue_area > 0.2 * total_image_area
        areas = {}
        max_areas = {}
        resistor_values = {}
        threshold_distance = 10  # define a threshold distance for merging close color bands
    # create an empty image with the same shape as the original image
        overlay_img = np.zeros_like(img)

        overlay_img = np.zeros((*img.shape[:2], 3), dtype=np.uint8)

        for i, colour in enumerate(colours):
            if colour[2] == 'BLUE' and ignore_blue:
                continue
            mask = cv.inRange(img, colour[0], colour[1])
            contours, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv.contourArea(contour)
                if area > 20:
                    M = cv.moments(contour)
                    cx = int(M['m10'] / M['m00'])
                    close_key = next((k for k in max_areas if abs(k - cx) <= threshold_distance), None)
                    if close_key is not None and max_areas[close_key] > area:
                        continue
                    max_areas[cx if close_key is None else close_key] = area
                    resistor_values[cx if close_key is None else close_key] = i

            # create a color mask
            color_mask = np.zeros_like(overlay_img)
            color_mask[mask > 0] = colours[i][3]  # set the color where mask is not zero


            # add the color mask to the overlay image
            overlay_img = cv.add(overlay_img, color_mask)

        # Resize and display the overlay image
        overlay_img = cv.resize(overlay_img, (overlay_img.shape[1]*4, overlay_img.shape[0]*4)) # increase size 4 times
        cv.imshow("Overlayed Masks", overlay_img)
        # cv.waitKey(0)

        # sort the resistors by x location
        resistor_values = {k: resistor_values[k] for k in sorted(resistor_values)}
        resistor_colours = [colours[resistor_values[key]][2] for key in sorted(resistor_values)]
        
        print(resistor_values)
        # find the resistor value


        if len(resistor_values) >= 4:
            keys = list(resistor_values.keys())
            tens = resistor_values[keys[0]]
            units = resistor_values[keys[1]]
            multiplier = resistor_values[keys[2]]
            try:
                tolerance = tolerances[resistor_values[keys[3]]]
            except:
                tolerance = 'flip'
            if tens == 10 or tolerance == 'flip':
                print("Image upside down, flipping...")
                resistor_values = dict(reversed(list(resistor_values.items())))
                keys = list(resistor_values.keys())
                print(resistor_values)
                tens = resistor_values[keys[0]]
                units = resistor_values[keys[1]]
                multiplier = resistor_values[keys[2]]
                resistor_colours = [colours[resistor_values[key]][2] for key in resistor_values]
                tolerance = tolerances[resistor_values[keys[3]]]
                
            # print("The resistance is: ", resistor_val, " with tolerance of ", tolerance)
        elif len(resistor_values) == 3:
            image_width = overlay_img.shape[1]/4
            positions = list(resistor_values.keys())
            first_band_position = positions[0]
            third_band_position = image_width - positions[2]
            print(first_band_position)
            print(third_band_position)
            # Get the image width to calculate distances from the edges
            print('cum bum')
            print(image_width) 
            keys = list(resistor_values.keys())

            # Compare distances of the first and third color bands from the image edges
            if first_band_position > third_band_position:
                # Flip the positions list if the first band is further from both edges
                resistor_values = dict(reversed(list(resistor_values.items())))
                keys = list(resistor_values.keys())
                print('passed inside wtf')
            # Assign variables based on the updated positions list
            print(resistor_values)
            tens = resistor_values[keys[0]]
            units = resistor_values[keys[1]]
            multiplier = resistor_values[keys[2]]
            resistor_colours = [colours[resistor_values[key]][2] for key in resistor_values]
            tolerance = tolerances[10]  # Hard code 5 

        else:
            print("Not enough bands detected to calculate the resistor value.")
        
        # print(resistor_colours[2])
        try:
            if resistor_colours[2] == "GOLD":
                multiplier =-1
        except:
            print("dont worry")
        print(tens)
        print(units)
        print(multiplier)
        resistor_val = (tens * 10 + units) * (10 ** multiplier)
        
        

        imagingComponents._value = resistor_val
        imagingComponents._colours = resistor_colours
        imagingComponents.tolerance = tolerance

        print(imagingComponents._value)
        print(imagingComponents._colours)
        print(imagingComponents.tolerance)
        img = cv.cvtColor(outputimg,cv.COLOR_BGR2RGB)
        cropimg = im.fromarray(img)
        return cropimg
    

    def getContours(self):
        '''
        Using vision_func.py, get contours of current image 
        '''
        """Find resistor bands"""
        # scale = 2
        dimensions = (576, 224)
        # need to read image in grey to get cropped mask
        initial = vf.read_img(imagingComponents._image, grey=True)
        if initial.shape[0] > initial.shape[1]:
            initial = cv.rotate(initial, cv.ROTATE_90_CLOCKWISE)
        initial = cv.resize(initial, dimensions, interpolation=cv.INTER_AREA)
        initial = cv.GaussianBlur(initial, (111,111), cv.BORDER_DEFAULT)
        mask = vf.test_masking(initial, opt='rec')
        x,y,w,h = cv.boundingRect(mask)

        bgr = vf.read_img(imagingComponents._image)
        if bgr.shape[0] > bgr.shape[1]:
            bgr = cv.rotate(bgr, cv.ROTATE_90_CLOCKWISE)
        bgr = cv.resize(bgr, dimensions, interpolation=cv.INTER_AREA)
        # bgr = white_balance(img=bgr, w_patch=w_patch)
        bgr = cv.GaussianBlur(bgr, (111,111), cv.BORDER_DEFAULT)
        cv.imshow('blur', bgr)
        img = cv.cvtColor(bgr, cv.COLOR_BGR2HSV)
        crop = img[y:y+h, x:x+w]
        bgr = bgr[y:y+h, x:x+w]
        initial = initial[y:y+h, x:x+w]
            

        def valid_contour(c):
            """Check for correct area and aspect ratio of contour"""
            # min_area = (40*(crop.shape[1]/crop.shape[0]))/(546/11)
            min_area = 70 
            if (cv.contourArea(c) < min_area):
                return False
            else:
                x,y,w,h = cv.boundingRect(c)
                asp_rat = float(w)/h
                # print(asp_rat)
                if (asp_rat > 5.5 or asp_rat < 2.2):
                    return False
            return True
        
        def find_bands():
            
            thresh = cv.adaptiveThreshold(initial, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY, 59, 5)
            thresh = cv.bitwise_not(thresh)

            band_pos = []
            colour_check = vf.colours

            for clr in colour_check:
                mask = cv.inRange(crop, clr[0], clr[1])
                if (clr[2] == 'RED'): # combine 2 red csv ranges
                    # print('inred')
                    redMask2 = cv.inRange(crop, vf.red_top_lower, vf.red_top_upper)
                    mask = cv.bitwise_or(redMask2, mask, mask)

                mask = cv.bitwise_and(mask, thresh, mask=mask)
                # cv.imshow('mask1', mask)
                contours, heirarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)[-2:]
                # cv.imshow('contour', contours)
                # print(contours)
                    # filter contours
                for i in range(len(contours) - 1, -1, -1):
                    if (valid_contour(contours[i])):
                        left = tuple(contours[i][contours[i][:,:,0].argmin()][0])
                        # make array with band position and associated colour
                        band_pos += [left + tuple(clr[2:])]
                        cv.circle(bgr, left, 5, (255,0,255), -1)
                    else:
                        contours = list(contours)
                        contours.pop(i)
                
                vf.con = cv.drawContours(bgr, contours, -1, clr[-1], 3)

            # cv.imshow('conturs', bgr)
            imagingComponents._contours = cv.cvtColor(bgr, cv.COLOR_BGR2RGB)
            return sorted(band_pos, key=lambda tup: tup[0])
        
        def resistance(sorted_bands):
            """return resulting ohms of resistor"""
            strVal = ""
            intVal = 0
            if (len(sorted_bands) > 0):
                if (len(sorted_bands) in [3,4,5]):
                    for band in sorted_bands[:-2]:
                        # values of numbers except multiplier (last band)
                        strVal += str(band[3])
                    intVal = int(strVal)
                    # add number of zeros corresponding to multiplier colour value
                    intVal *= 10**sorted_bands[-2][3]
                if (sorted_bands[-1][2] == 'ORANGE'):
                    tol = '5%'
                else:
                    tol = '10%'

                return intVal, tol
            else:
                return "N/A"
        

        for j in range(len(crop)):
            sorted_bands = find_bands()
            result = resistance(sorted_bands)
        # print(sorted_bands)
        
        imagingComponents._colours = sorted_bands
        imagingComponents._value = result[0]
        imagingComponents.tolerance = result[1]

        # print(result)

    
        value = im.fromarray(vf.con)
        contours = im.fromarray(imagingComponents._contours)
        # cv.imshow('ohhh', imagingComponents._contours)

        return contours

class Camera(QThread):
    image_signal = pyqtSignal(QImage)
    _imaging = imagingComponents
    def __init__(self):
        super().__init__()
        self._running = True
    def run(self):
        currentcam = self._imaging.getCam(self)[0]
        self.cap = cv.VideoCapture(currentcam)
         
        while self._running:
            if currentcam !=self._imaging.getCam(self)[0]:
                currentcam = self._imaging.getCam(self)[0]
                self.cap = cv.VideoCapture(currentcam)
            ret, frame = self.cap.read()
            if ret:
                h, w, ch = frame.shape
                bytes_per_line = ch * w
                qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
                
                # Emit the signal with the QImage
                self.image_signal.emit(qt_image)
    
    def stop(self):
        self._running = False
        # self.cap.release()
