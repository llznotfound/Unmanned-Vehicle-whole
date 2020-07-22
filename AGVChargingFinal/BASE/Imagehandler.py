# -*- coding: utf-8 -*-
"""
Created on  Jun 16 14:28:26 2016

@author: Ankit Singh
"""
from PatternFinding import PatternFinding
from FindingOrientationOfContours import FindingOrientationOfContours
from AffineTransformation import AffineTransformation,PerspectiveTransformation
import numpy as np

import cv2 as cv
import os.path


class Imagehandler(object):
    def __init__(self, img):
        self.Image = img

    def __convertImagetoBlackWhite(self):
        #self.Image = cv.imread(self.ImagePath, cv.IMREAD_COLOR)
        self.imageOriginal = self.Image
        if self.Image is None:
            print ('some problem with the image')
        else:
            print ('Image Loaded')

        # self.Image = cv.cvtColor(self.Image, cv.COLOR_BGR2GRAY)
        hsv = cv.cvtColor(self.Image, cv.COLOR_BGR2HSV)
        # define range of blue color in HSV
        low = np.array([35, 43, 46])
        up = np.array([80, 255, 255])
        # Threshold the HSV image to get only blue colors
        self.Image = cv.inRange(hsv, low, up)

        # self.Image = cv.GaussianBlur(self.Image, (3, 3), 1)
        cv.imshow("beforenormalize", self.Image)
        # cv.normalize(self.Image, self.Image, 255, 0, cv.NORM_MINMAX, cv.CV_8U)
        #self.Image = cv.equalizeHist(self.Image)
        cv.imshow("normalize", self.Image)

        # self.Image = cv.adaptiveThreshold(
        #     self.Image,
        #     255,                    # Value to assign
        #     cv.ADAPTIVE_THRESH_GAUSSIAN_C,# Mean threshold
        #     cv.THRESH_BINARY,
        #     15,                     # Block size of small area
        #     -2,                      # Const to substract
        # )
        # _, self.Image = cv.threshold(self.Image, 170, 255, cv.THRESH_BINARY)

        cv.imshow("binary", self.Image)
        element = cv.getStructuringElement(cv.MORPH_RECT, (9, 9))
        self.Image = cv.morphologyEx(self.Image, cv.MORPH_CLOSE, element)

        cv.imshow("binaryAfter", self.Image)

        return self.Image


    def WritingImage(self, image, path, imageName):
        if image is None:
            print ('Image is not valid.Please select some other image')
        else:
            image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
            print (path + imageName)
            #why would you want to imshow your images. Let them be there in Results folder
            cv.imwrite(path + imageName, image)
            # cv.imshow(imageName, image)
            # cv.waitKey(0)ž
            # cv.destroyAllWindows()

    def GetImageContour(self):
        thresholdImage = self.__convertImagetoBlackWhite()  #B & W with adaptive threshold
        thresholdImage = cv.Canny(thresholdImage, 20, 60) #Edges by canny edge detection
        #cv.imshow("thresholdImage", thresholdImage)
        thresholdImage, contours, hierarchy = cv.findContours(
            thresholdImage, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        self.Contours = contours
        # uncomment this to see the contours on the image
        cv.drawContours(self.imageOriginal, contours, -1, (0,255,0), 1)
        cv.imshow("thresholdImage", self.imageOriginal)
        """
        patternFindingObj=PatternFinding()
        areas= [cv.contourArea(contour) for contour in contours]
        for index in range(len(contours)):
            IsPattern=self.IsPossibleQRContour(index)
            if IsPattern is True:
                x,y,w,h=cv.boundingRect(contours[index])
                cv.rectangle(self.imageOriginal,(x,y),(x+w,y+h),(0,0,255),2)
                cv.imshow("hello",self.imageOriginal)
        maxAreaIndex=np.argmax(areas)
        x,y,w,h=cv.boundingRect(contours[maxAreaIndex])
        cv.rectangle(self.image2,(x,y),(x+w,y+h),(0,255,0),2)
        cv.imshow("hello",self.imageOriginal)
        cv.waitKey(0)
        cv.destroyAllWindows()"""
        contour_group = (thresholdImage, contours, hierarchy)
        return contour_group

    def QRCodeInImage(self):
        patternFindingObj = PatternFinding(self.GetImageContour(), self.imageOriginal)
        patterns = patternFindingObj.FindingQRPatterns(3)
        if patterns == None:
            return None, self.imageOriginal, None
        else:
            if len(patterns) < 3:
                print ('patterns unable to find')
                self.TransformImage = self.imageOriginal
                return None, self.TransformImage
            contourA = patterns[0]
            contourB = patterns[1]
            contourC = patterns[2]
            orientationObj = FindingOrientationOfContours()
            rotationAngle, massQuad , ORIENTATION = orientationObj.FindOrientation(contourA, contourB, contourC)
            tl=massQuad.tl
            tr=massQuad.tr
            bl=massQuad.bl
            # print("massQuad:", massQuad)
            br = np.float32([tr[0] + bl[0] - tl[0], tr[1] + bl[1] - tl[1]])
            affineTransformObj = AffineTransformation(self.imageOriginal, ORIENTATION)
            self.TransformImage = affineTransformObj.Calangle(rotationAngle, massQuad)
            return rotationAngle, self.TransformImage, [tl, tr, bl, br]



if __name__ == '__main__':
    # pass
    hdl=Imagehandler("./Input/qr3.jpg")
    hdl.GetImageContour()
    #cv.imshow("test",hdl.TransformImage)
    cv.waitKey(0)