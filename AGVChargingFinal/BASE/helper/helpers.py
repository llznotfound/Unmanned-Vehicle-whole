import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import pickle
import glob
import os


def get_camera_cal():
    """
	Read the picke file and return the mtx, dist to caller
	"""
    loc = os.path.abspath(os.path.dirname(os.path.dirname(__file__))) + "/camera_cal/camera_cal.p"
    print(loc)
    pickle_file = open(loc, "rb")
    dist_pickle = pickle.load(pickle_file)
    mtx = dist_pickle["mtx"]
    dist = dist_pickle["dist"]
    pickle_file.close()

    return mtx, dist

# if __name__ == 'main':
get_camera_cal()