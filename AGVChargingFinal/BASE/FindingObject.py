"""
Created on  Jul 16 21:05:31 2016

@author: Feiyang
# 这里包含两种模版匹配方案，SIFT和ORB算法，建立在Opencv模版匹配代码基础上。
"""
import cv2
import numpy as np
from helper import utils
from matplotlib import pyplot as plt

class FindingObject(object):
    def __init__(self, templatePictureName):
        self.template = cv2.imread(templatePictureName, 0)
        if self.template is None:
            print('Unable to read the Image. Please provide the image file')
        self.frame = None
        self.matchesMask = None
        sift = cv2.xfeatures2d.SIFT_create()
        self.kp1_sf, self.des1_sf = sift.detectAndCompute(self.template, None)
        orb = cv2.ORB_create()
        self.kp1_ob, self.des1_ob = orb.detectAndCompute(self.template, None)

    def PlotResult(self, kp1, kp2, good):
        plt.ion()  # 开启interactive mode
        draw_params = dict(matchColor=(0, 255, 0),  # draw matches in green color
                           singlePointColor=None,
                           matchesMask=self.matchesMask,  # draw only inliers
                           flags=2)
        plt.imshow(self.frame)
        img3 = cv2.drawMatches(self.template, kp1, self.frame, kp2, good, None, **draw_params)
        plt.imshow(img3, 'gray'), plt.show()
        plt.pause(1)
        plt.close()
    # this is the public model used to find objects once features are extract from pictures
    def FindingObject(self, matches, kp1, kp2, percentage=0.7, plot=False, MIN_MATCH_COUNT=13, expect_size=400):
        # store all the good matches as per Lowe's ratio test.
        good = []
        for m, n in matches:
            if m.distance < percentage * n.distance:
                good.append(m)
                if len(good) > 5 * MIN_MATCH_COUNT:
                    break
        print("matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT))
        if len(good) > MIN_MATCH_COUNT:
            src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
            dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
            M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
            self.matchesMask = mask.ravel().tolist()
            h, w = self.template.shape
            pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
            if M is not None:
                dst = cv2.perspectiveTransform(pts, M)
                # bound RIO in frame
                dst = np.int32(dst)
                x_max, x_min, y_max, y_min = utils.findmaxmin(dst)
                RIOBox = [x_max, x_min, y_max, y_min]
                if (np.int32(RIOBox) < 0).any() or abs(x_max - x_min) < 2 or abs(y_max - y_min < 2):
                    RIOBox = [self.frame.shape[1], 0, self.frame.shape[0], 0]
                    ratio = 1
                else:
                    ratio = expect_size / (x_max - x_min)

            else:
                print("Enough matches found, but no suitable transform")
                RIOBox = [self.frame.shape[1], 0, self.frame.shape[0], 0]
                ratio = 1

            if plot:
                self.PlotResult(kp1, kp2, good)

        else:
            print("Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT))
            return self.frame, [self.frame.shape[1], 0, self.frame.shape[0], 0], 1

        return self.frame, RIOBox, ratio

#This function uses SIFT features
#It will return the cropped image of the template, ROI point coordinates and ratio, otherwise the whole picture will be returned
    def  SIFTFindingObject(self,frame, expect_size = 400, MIN_MATCH_COUNT = 13, plot = False):
        self.frame = frame
        img2 = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        # Initiate SIFT detector
        sift = cv2.xfeatures2d.SIFT_create()
        #sift = cv2.xfeatures2d.SURF_create(float(4000))#uncommented to use surf instead of SIFT

        # find the keypoints and descriptors with SIFT
        kp2, des2 = sift.detectAndCompute(img2, None)
        if kp2 is None or des2 is None:
            RIOBox = [img2.shape[1], 0, img2.shape[0], 0]
            ratio = 1
            print("SIFT with no result!!!!")
            return self.frame, RIOBox, ratio
        #find homograph of template in the frame
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
        search_params = dict(checks=50)
        flann = cv2.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(self.des1_sf, des2, k=2)

        return self.FindingObject(
            matches, self.kp1_sf, kp2, 0.7, plot=plot, MIN_MATCH_COUNT=MIN_MATCH_COUNT, expect_size=expect_size)

#This function uses ORB features
#It will return the cropped image of the template, ROI point coordinates and ratio, otherwise the whole picture will be returned
    def OBGFindingObject(self, frame, expect_size = 400, MIN_MATCH_COUNT = 10, plot = False):
        self.frame = frame
        img2 = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        # Initiate ORB detector
        orb = cv2.ORB_create()
        # 对frame图像检测特征和描述符
        kp2, des2 = orb.detectAndCompute(img2, None)
        if kp2 is None or des2 is None:
            RIOBox = [img2.shape[1], 0, img2.shape[0], 0]
            ratio = 1
            print("OBG with no result!!!!")
            return self.frame, RIOBox, ratio
        bf = cv2.BFMatcher()
        matches = bf.knnMatch(self.des1_ob, des2, k=2)
        return self.FindingObject(
            matches, self.kp1_ob, kp2, 0.7, plot=plot, MIN_MATCH_COUNT=MIN_MATCH_COUNT, expect_size=expect_size)
