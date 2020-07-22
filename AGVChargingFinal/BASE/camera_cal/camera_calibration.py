# Camera calibration file, include two function.
# main(), do the camera cal and write the result to a pickle file for further use
# test(), check the if the pickle file and the calibration parameters work
import numpy as np
import cv2
import glob
import matplotlib.pyplot as plt
import pickle                   # 用于序列化和反序列化Python对象结构的二进制协议
import os

def calibrate(drawconer=False):
    '''
	read the calibration image and do the camera calibration
	and output the result to a pickle file.
	if drawconer is True, will draw the corner on the chessboard file and save it to another folder.
	'''
    # !!! IMPORTANT, set the nx, ny according the calibration chessboard pictures.
    nx = 8
    ny = 6

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0), ...(6,5,0)
    objp = np.zeros((nx * ny, 3), np.float32)
    objp[:, :2] = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d points in real world space
    imgpoints = []  # 2d pionts in image plane.

    # Make a list of calibration images
    images = glob.glob('my_image/WechatIMG*.jpeg')
    print("Reading the calibration file...")
    # Step through the list and search for chessboard corners
    for idx, fname in enumerate(images):
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        print("Searching corners on ", fname, "...")
        ret, corners = cv2.findChessboardCorners(gray, (nx, ny), None)

        # If found, add object points, image points
        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)

            if drawconer:
                cv2.drawChessboardCorners(img, (nx, ny), corners, ret)
                write_name = 'corners_found' + str(idx) + '.jpeg'
                cv2.imwrite(write_name, img)
                cv2.imshow('img', img)
                cv2.waitKey(500)
    cv2.destroyAllWindows()

    # Get image size
    img_size = (img.shape[1], img.shape[0])

    # Do camera calibration given object points and image points
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None)

    # Save the camera calibration result for later use (we won't worry about rvecs / tvecs)
    print("Saving the parameter to file...>>camera_cal.p")
    dist_pickle = {}
    dist_pickle["mtx"] = mtx
    dist_pickle["dist"] = dist
    pickle_file = open("my_image/camera_cal.p", "wb")
    pickle.dump(dist_pickle, pickle_file)
    pickle_file.close()
    mean_error = 0

    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        mean_error += error
    print( "total error: {}".format(mean_error/len(objpoints)) )

def test():
    """
	read the pickle file on disk and implement undistor on image
	show the oringal/undistort image
	"""
    print("Reading the pickle file...")

    loc = os.path.dirname(os.path.realpath(__file__)) + "/my_image/camera_cal.p"
    pickle_file = open(loc, "rb")
    dist_pickle = pickle.load(pickle_file)
    mtx = dist_pickle["mtx"]
    dist = dist_pickle["dist"]
    pickle_file.close()
    images = glob.glob('my_image/WechatIMG*.jpeg')
    for idx, fname in enumerate(images): 
	    print("Reading the sample image...")
	    # loc = os.path.dirname(os.path.realpath(__file__)) + "/my_image/WechatIMG15.jpeg"
	    img = cv2.imread(fname)
	    img_size = (img.shape[1], img.shape[0])
	    dst = cv2.undistort(img, mtx, dist, None, mtx)

	    # dst = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
	    # Visualize undistortion
	    print("Visulize the result...")
	    # plt.figure()
	    f, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
	    ax1.imshow(img), ax1.set_title('Original Image', fontsize=15)
	    ax2.imshow(dst), ax2.set_title('Undistored Image', fontsize=15)
	    plt.show()


if __name__ == '__main__':
#     calibrate(drawconer=False)  # read the chessboard file and get mtx, dist and write to pickle fiel
  calibrate(drawconer=True)
  # test()	# read the pickle file and undistort an image.
