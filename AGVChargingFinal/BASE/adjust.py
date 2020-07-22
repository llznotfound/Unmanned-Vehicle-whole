# %load testV5.py

from camera_cal.ipcamCapture import ipcamCapture
import numpy as np
import math
import cv2
import time
from Imagehandler import Imagehandler
from FindingObject import FindingObject
from matplotlib import pyplot as plt
from helper import socket_test as st
# from pyzbar import pyzbar
from helper import utils
from camera_cal import helpers
import sys


def modify(img):
    pts1 = np.float32([[494, 167], [185, 830], [1793, 809], [1439, 173]])
    pts2 = np.float32([[200, 200], [200, 1000], [1200, 1000], [1200, 200]])
    M = cv2.getPerspectiveTransform(pts1, pts2)
    dst = cv2.warpPerspective(img, M, (1400, 1400))
    return dst


def CarPositionDetect(frame):
    obj = Imagehandler(frame)
    try:
        rotationAngle, TransformImage, massQuad = obj.QRCodeInImage()
        # print("Angle is:", rotationAngle)
    except ZeroDivisionError:
        rotationAngle, TransformImage, massQuad = None, frame, None
        print('QR Code not found in image ')
    if TransformImage is None:
        print('Image is not generated')
    cv2.imshow("TakeCar", TransformImage)
    return rotationAngle, massQuad

def ZoomIn(RIOBox, frame):
        #  zoom in RIO
        # print("The bounding box is: ", RIOBox)
        RIO = frame[RIOBox[3]:RIOBox[2], RIOBox[1]:RIOBox[0]]
        if RIO.shape[1] > 2:
            ratio = 400 / RIO.shape[1]
            RIO = cv2.resize(RIO, (400, np.int32(ratio * RIO.shape[0])), interpolation=cv2.INTER_LINEAR)
        else:
            ratio = 1
        return ratio, RIO

def main():
    URL = "rtsp://admin:abcd1234@192.168.8.1:554/Streaming/Channels/101?transportmode=unicast"
    ipcam = ipcamCapture(URL)  # a class specialize in dealing with web camera
    ipcam.start()
    img = ipcam.getframe()
    flag = False
    time.sleep(1)
    #initialize distortion parameters
    mtx, dist = helpers.get_camera_cal()
    nxt_RIOBox = None
    findQR = FindingObject('4.png')  # load template for SIFT
    findQR2 = FindingObject('2.png')
    findQR1 = FindingObject('1.png')
    cnt = 0  # test time for nxtRIO
    ratio = 1
    cnt = 0

    #设定终点位置
    x_max = 422
    y_mid = 856


    x, y = None, None

    basetcp = st.BaseTCP()
    while ipcam.CamStatus():
        if not ipcam.readable:  # check whether the current frame is the latest
            continue
        cnt = cnt+1
        frameOriginal = ipcam.getframe()
        frameOriginal = cv2.undistort(frameOriginal, mtx, dist, None, mtx)
        cv2.namedWindow("txt", 0)
        cv2.resizeWindow("txt", 640, 480)
        cv2.imshow("txt", frameOriginal)
        # c = cv2.waitKey(0)
        frame = modify(frameOriginal)
        display = frame  # 重要：区别display 和 frameOriginal，以防错误修改Original Frame。display用于展示结果，

        # if nxt_RIOBox is not generated before, use OBG to find RIO
        if nxt_RIOBox == None:
            _, nxt_RIOBox, ratio = findQR1.SIFTFindingObject(frame,MIN_MATCH_COUNT=16,plot=False)
            if nxt_RIOBox == [frame.shape[1], 0, frame.shape[0], 0]:
                _, nxt_RIOBox, ratio = findQR2.SIFTFindingObject(frame, MIN_MATCH_COUNT=16, plot=False)
                if nxt_RIOBox == [frame.shape[1], 0, frame.shape[0], 0]:
                    _, nxt_RIOBox, ratio = findQR.SIFTFindingObject(frame, MIN_MATCH_COUNT=16, plot=False)

            print('!!!!reinitilize!!!!!')

        RIOBox = nxt_RIOBox
        ratio, RIO = ZoomIn(RIOBox, frame)          #  zoom in RIO
        rotationAngle, massQuad = CarPositionDetect(RIO)            # search for QRcode in the RIO
        # print("massQuad size:", massQuad)
        # strin = input("wait for input...")
        if RIOBox != None:
            cv2.rectangle(display, (RIOBox[1], RIOBox[3]), (RIOBox[0], RIOBox[2]), (0, 255, 0), 2)

        # bounding the QRcode in the Original frame, and predict the next RIO
        OrignalPt = []
        if massQuad is not None and RIOBox is not None:
            for i in range(4):
                OrignalPt.append(utils.ReturnToOrg([RIOBox[1], RIOBox[3]], massQuad[i], ratio))
        else:
            print("RIOBox not generated")
            cv2.namedWindow("frameOriginal", 0)
            cv2.resizeWindow("frameOriginal", 640, 480)
            cv2.imshow("frameOriginal", frame)
            cnt += 1
            #连续10张图没有在ROI中检测到车辆位置，重新初始化
            if cnt >= 10:
                nxt_RIOBox = None
                cnt = 0
            c = cv2.waitKey(1)
            continue

        Picx = int((OrignalPt[1][0] + OrignalPt[2][0]) / 2)
        Picy = int((OrignalPt[1][1] + OrignalPt[2][1]) / 2)
        Planpt = utils.PicToPlan([Picx, Picy], [frame.shape[1], frame.shape[0]])

        rotationAngle = utils.PicAngleToPlanAngle(rotationAngle)

        RIOBox = utils.findmaxmin(OrignalPt)
        nxt_RIOBox = utils.findNextRIO(RIOBox, [frame.shape[1], frame.shape[0]], n=4)
        # break
        # draw the boundary, angle, and RIO for debug

        if x is not None:
            for jj in range(len(x) - 1):
                pt1 = utils.PlanToPic(utils.Realworld2Camera([x[jj], y[jj]]), [frame.shape[1], frame.shape[0]])
                pt2 = utils.PlanToPic(utils.Realworld2Camera([x[jj+1], y[jj+1]]), [frame.shape[1], frame.shape[0]])
                cv2.line(display, (int(pt1[0]), int(pt1[1])), (int(pt2[0]), int(pt2[1])), (255, 0, 0), 5)


        cv2.circle(display, (Picx, Picy), 5, (100, 100, 100), - 1)
        print("Place: {}, {}".format(Picx, Picy))
        # cv2.circle(display, (Picz[0], Picz[1]), 5, (100, 200, 100), - 1)
        cv2.circle(display, (OrignalPt[0][0], OrignalPt[0][1]), 5, (0, 255, 0), - 1)
        cv2.circle(display, (OrignalPt[1][0], OrignalPt[1][1]), 5, (255, 0, 0), - 1)
        cv2.circle(display, (OrignalPt[2][0], OrignalPt[2][1]), 5, (0, 0, 255), - 1)
        cv2.putText(display,
                    "x is: " + str(Planpt[0]), (30, 70), cv2.FONT_HERSHEY_PLAIN, 1.5,
                    (0, 255, 0), 1, 8, 0)
        cv2.putText(display,
                    "y is: " + str(Planpt[1]), (30, 90), cv2.FONT_HERSHEY_PLAIN, 1.5,
                    (0, 255, 0), 1, 8, 0)
        cv2.putText(display,
                    "angle is: " + str(rotationAngle), (30, 50), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0), 1, 8, 0)
        cv2.rectangle(display, (RIOBox[1], RIOBox[3]), (RIOBox[0], RIOBox[2]), (255, 0, 255), 2)
        cv2.rectangle(display, (nxt_RIOBox[1], nxt_RIOBox[3]), (nxt_RIOBox[0], nxt_RIOBox[2]), (255, 0, 255), 2)
        cv2.namedWindow("frameOriginal", 0)
        cv2.imshow("frameOriginal", display)
        c = cv2.waitKey(1)
        # if cnt == 2:
        #     cnt = 0
        print(Planpt[0] - x_max, Planpt[1]-y_mid, rotationAngle)
        # PicRealWorld = utils.camer2Realworld(Planpt[0], Planpt[1])
        print(Planpt)

        # -------------------细调节部分代码-----------------------
        basetcp.questionMotion()
        if basetcp.CheckState():
            time.sleep(2)
            if 90-5 >= rotationAngle > 0:
                basetcp.send("L2")
            elif Planpt[1] - y_mid > 130:
                basetcp.send("B2")  # 35cm
            elif 180 >= rotationAngle > 96:
                basetcp.send("R2")
            elif Planpt[1] - y_mid > 38:
                basetcp.send("B1")  # 8cm
            elif 88 > rotationAngle > 90 - 5:
                    basetcp.send("L1")
            elif 96 > rotationAngle > 92:
                basetcp.send("R1")
            elif Planpt[0] - x_max>= 60:
                basetcp.send("G2")  # 18cm
            elif Planpt[0] - x_max<= -60:
                basetcp.send("C2")  # 18cm
            # elif Planpt[0]- x_max >= 16:
            #     basetcp.send("G1")  # 4cm
            # elif Planpt[1] - y_mid > 4:
            #     basetcp.send("Bs")  # 8cm
            elif Planpt[1] - y_mid < -7:
                basetcp.send("F")  # 8c

            else:
                print("complete!!!")
                break



        #plan path for car:
        if c & 0xFF == ord('q'):
            ipcam.stop()
            break

if __name__ == '__main__':
    main()
