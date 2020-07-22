import numpy as np
#所有工具函数和坐标变换均在此文件中

#寻找四个坐标中最大最小的坐标
def findmaxmin(dst):
    dst = np.squeeze(dst)
    dst.reshape(4, 2)
    print(dst.shape)
    x_list = np.array([pt[0] for pt in dst])
    x_max = np.max(x_list)
    x_min = np.min(x_list)
    y_list = np.array([pt[1] for pt in dst])
    y_max = np.max(y_list)
    y_min = np.min(y_list)
    # print("the maximum minimum is: {}, {}, {}, {}".format(x_max, x_min, y_max, y_min))
    return x_max, x_min, y_max, y_min

#预测下一个ROI区域范围
def findNextRIO(RIOBox, sizePic, n = 2):
    nxt_x_min = np.int32(RIOBox[1] - (RIOBox[0] - RIOBox[1]) * (n-1)/2)
    nxt_x_min = nxt_x_min if nxt_x_min > 0 else 0
    nxt_x_max = np.int32(RIOBox[0] + (RIOBox[0] - RIOBox[1]) * (n-1)/2)
    nxt_x_max = nxt_x_max if nxt_x_max < sizePic[0] else sizePic[0]
    nxt_y_min = np.int32(RIOBox[3] - (RIOBox[2] - RIOBox[3]) * (n-1)/2)
    nxt_y_min = nxt_y_min if nxt_y_min > 0 else 0
    nxt_y_max = np.int32(RIOBox[2] + (RIOBox[2] - RIOBox[3]) * (n-1)/2)
    nxt_y_max = nxt_y_max if nxt_y_max < sizePic[1] else sizePic[1]
    return [nxt_x_max, nxt_x_min, nxt_y_max, nxt_y_min]

#将裁减出的ROI坐标变换回原图中
def ReturnToOrg(OriginPoint, pt, ratio):
    return [OriginPoint[0] + np.int32(pt[0]/ratio), OriginPoint[1] + np.int32(pt[1]/ratio)]

#将坐标系从y轴向下转为y轴向上
def PicToPlan(Picpt, PicSize):
    return [Picpt[0], PicSize[1] -Picpt[1]]
 
#将坐标系从y轴向上转会y轴向下   
def PlanToPic(Planpt, PicSize):
    return [Planpt[0], PicSize[1] -Planpt[1]]

#标定需要修改这里
#图像坐标到世界标定过程：记录一组frameOriginal图像上x，y读数。记录一组车辆在真实世界中以某个点为原点的相对坐标位置。用excel拟合对应点的线性关系式
def camer2Realworld(x, y):
    return [(x*0.3175 - 30.333), (y * 0.3292 - 165.29)]
def Realworld2Camera(pt):
    return [int((pt[0]+30.333)/0.3175), int((pt[1]+165.29)/0.3292)]

#二维码角度变化成车辆角度。
def PicAngleToPlanAngle(angle):
    return 360-(angle + 45)%360