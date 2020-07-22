# coding: utf-8
import cv2
import numpy as np
from helper import takePictureCmd as tp, helpers

# tp.getpciture()
img = tp.getpciture()
mtx, dist = helpers.get_camera_cal()
img = cv2.undistort(img, mtx, dist, None, mtx)
# cv2.imshow('img', img)
# cv2.waitKey(0)
print(img.shape)
# img = cv2.resize(img, (1080, 720))
# print(img.shape)

# print img.shape

# rtmp://rtmp01open.ys7.com/openlive/c1614c164a6b4eb8a5978bffd51136fd.hd
# ffmpeg -i rtmp://rtmp01open.ys7.com/openlive/c1614c164a6b4eb8a5978bffd51136fd.hd -f image2 -y picture.jpeg
def modify(img):
    pts1 = np.float32([[656, 23], [293, 716], [1907, 824], [1523, 50]])
    pts2 = np.float32([[0, 0], [0, 1000], [900, 1000], [900, 0]])
    M = cv2.getPerspectiveTransform(pts1, pts2)
    dst = cv2.warpPerspective(img, M, (900, 1000)) # 4.5m， 5.0m
    return dst

def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "%d,%d" % (x, y)
        cv2.circle(img, (x, y), 1, (255, 0, 0), thickness=-1)
        cv2.putText(img, xy, (x, y), cv2.FONT_HERSHEY_PLAIN,
                    1.0, (0, 0, 0), thickness=1)
        print("x:", x)
        print("y:", y)
        cv2.imshow("img", img)


# img = modify(img)
cv2.namedWindow("img", 0)
cv2.resizeWindow("img", 640, 480)
cv2.setMouseCallback("img", on_EVENT_LBUTTONDOWN)
cv2.imshow("img", img)

while (True):
    try:
        cv2.waitKey(100)
    except Exception:
        cv2.destroyAllWindows()
        break

cv2.waitKey(0)
cv2.destroyAllWindows()
