

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


# p键截图 q键退出
#输入：原始图像
#输出：视角转为地面正上方的图片，大小统一放到（1400， 1400）
#注意pts2中点x方向差距与y方向差距的比例要与实际的比例相同。例如，这里是x方向差距为1200 - 200 = 1000， 
#y方向差距为1000-200 = 800，对应实际场地的长度x：y = 5:4
def modify(img):
    pts1 = np.float32([[494, 167], [185, 830], [1793, 809], [1439, 173]])
    pts2 = np.float32([[200, 200], [200, 1000], [1200, 1000], [1200, 200]])
    M = cv2.getPerspectiveTransform(pts1, pts2)
    dst = cv2.warpPerspective(img, M, (1400, 1400))
    return dst

#输入：ROI
#输出：车辆朝向，坐标
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

#统一将图片放缩到400作为长边长
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
    # 无线摄像头的网络地址，格式：rtsp://用户名:密码@ip地址:端口号/Streaming/Channels/101?transportmode=unicast
    URL = "rtsp://admin:abcd1234@192.168.8.1:554/Streaming/Channels/101?transportmode=unicast"
    # URL = "20200604_20200604172740_20200604173358_172738.mp4"
    #单独设立一个线程读取相机图片，避免时延问题
    ipcam = ipcamCapture(URL)  # a class specialize in dealing with web camera
    ipcam.start()
    img = ipcam.getframe()
    flag = False
    time.sleep(1)
    #initialize distortion parameters
    mtx, dist = helpers.get_camera_cal()
    nxt_RIOBox = None
    #一组车辆背面花纹的图片
    findQR = FindingObject('4.png')  # load template for SIFT
    findQR2 = FindingObject('2.png')
    findQR1 = FindingObject('1.png')
    cnt = 0  # test time for nxtRIO
    ratio = 1
    x, y = None, None
    (FinalX, FinalY, FinalAngle) = (80+23, 80+25, 90)

    basetcp = st.BaseTCP() #初始化socket通信

    while ipcam.CamStatus():
        if not ipcam.readable:  # check whether the current frame is the latest
            continue
        cnt = cnt+1
        #----------------------图片与处理步骤————————————————————————
        frameOriginal = ipcam.getframe() #读取一张图片
        frameOriginal = cv2.undistort(frameOriginal, mtx, dist, None, mtx)#消除桶形畸变
        cv2.namedWindow("txt", 0)
        cv2.resizeWindow("txt", 640, 480)
        cv2.imshow("txt", frameOriginal)
        # c = cv2.waitKey(0)
        frame = modify(frameOriginal)
        display = np.copy(frame)  # 重要：区别display 和 frameOriginal，以防错误修改Original Frame。display用于展示结果，

        #----------------------确定ROI————————————————————————
        # if nxt_RIOBox is not generated before, use sift to find RIO
        if nxt_RIOBox == None:
            _, nxt_RIOBox, ratio = findQR1.SIFTFindingObject(frame,MIN_MATCH_COUNT=16,plot=False)
            if nxt_RIOBox == [frame.shape[1], 0, frame.shape[0], 0]:
                _, nxt_RIOBox, ratio = findQR2.SIFTFindingObject(frame, MIN_MATCH_COUNT=16, plot=False)
                if nxt_RIOBox == [frame.shape[1], 0, frame.shape[0], 0]:
                    _, nxt_RIOBox, ratio = findQR.SIFTFindingObject(frame, MIN_MATCH_COUNT=16, plot=False)

            print('!!!!reinitilize!!!!!')

        RIOBox = nxt_RIOBox
        ratio, RIO = ZoomIn(RIOBox, frame)          #  zoom in RIO

        # ----------------------确定车辆在ROI中的位置————————————————————————
        rotationAngle, massQuad = CarPositionDetect(RIO)            # search for QRcode in the RIO
        if RIOBox != None:
            cv2.rectangle(display, (RIOBox[1], RIOBox[3]), (RIOBox[0], RIOBox[2]), (0, 255, 0), 2)

        # ----------------------确定车辆在图片中的位置，并预测next_ROI————————————————————————
        # bounding the QRcode in the Original frame, and predict the next RIO
        # 将ROI中二维码的位置变换回到原始图片中
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
            if cnt >= 10:
                nxt_RIOBox = None
                cnt = 0
            c = cv2.waitKey(1)
            continue

        Picx = int((OrignalPt[1][0] + OrignalPt[2][0]) / 2)
        Picy = int((OrignalPt[1][1] + OrignalPt[2][1]) / 2)
        #坐标变换，调整坐标y轴方向
        Planpt = utils.PicToPlan([Picx, Picy], [frame.shape[1], frame.shape[0]])
        rotationAngle = utils.PicAngleToPlanAngle(rotationAngle)

        # 预测下一个ROI
        RIOBox = utils.findmaxmin(OrignalPt)
        nxt_RIOBox = utils.findNextRIO(RIOBox, [frame.shape[1], frame.shape[0]], n=4)

        #---draw the boundary, angle, RIO, and planned path for debug----------
        if x is not None:
            for jj in range(len(x) - 1):
                pt1 = utils.PlanToPic(utils.Realworld2Camera([x[jj], y[jj]]), [frame.shape[1], frame.shape[0]])
                pt2 = utils.PlanToPic(utils.Realworld2Camera([x[jj+1], y[jj+1]]), [frame.shape[1], frame.shape[0]])
                cv2.line(display, (int(pt1[0]), int(pt1[1])), (int(pt2[0]), int(pt2[1])), (255, 0, 0), 5)
        cv2.circle(display, (Picx, Picy), 5, (100, 100, 100), - 1)
        print("Place: {}, {}".format(Picx, Picy))
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
        #
        # ----------------------车辆运动部分————————————————————————
        # 将车辆在图像中的坐标转为世界坐标（这时历史遗留问题，Pic2Plan和camer2Realworld可以合并）
        PicRealWorld = utils.camer2Realworld(Planpt[0], Planpt[1])
        print(PicRealWorld)
        basetcp.questionMotion() #查询车辆运动状态
        if basetcp.CheckState(): #如果车辆静止
            m, n, x, y = basetcp.send_xy((PicRealWorld[0], PicRealWorld[1], rotationAngle), (FinalX, FinalY, FinalAngle))
            print(x, y)
            if x is not None:
                for jj in range(len(x) - 1):
                    pt1 = utils.PlanToPic(utils.Realworld2Camera([x[jj], y[jj]]), [frame.shape[1], frame.shape[0]])
                    pt2 = utils.PlanToPic(utils.Realworld2Camera([x[jj + 1], y[jj + 1]]), [frame.shape[1], frame.shape[0]])
                    # print(pt1, pt2)
                    cv2.line(display, (int(pt1[0]), int(pt1[1])), (int(pt2[0]), int(pt2[1])), (233, 233, 100), 5)
                for jj in range(len(m) - 1):
                    pt1 = utils.PlanToPic(utils.Realworld2Camera([m[jj], n[jj]]), [frame.shape[1], frame.shape[0]])
                    pt2 = utils.PlanToPic(utils.Realworld2Camera([m[jj + 1], n[jj + 1]]), [frame.shape[1], frame.shape[0]])
                    cv2.line(display, (int(pt1[0]), int(pt1[1])), (int(pt2[0]), int(pt2[1])), (255, 0, 255), 5)
            cv2.imshow("frameOriginal", display)
            c = cv2.waitKey(1)
            sys.stdout.flush()
            strin = input("wait for input...") #人工审核路径规划结果
            print(strin)
            if (strin == "a"):                  #如果觉得可以，准许车辆运动
                print("go")
                basetcp.GO()
            else: print(strin)

        #plan path for car:
        if c & 0xFF == ord('q'):
            ipcam.stop()
            break

if __name__ == '__main__':
    main()
