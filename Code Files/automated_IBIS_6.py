"""
Kay Hutchinson
11/6/2019
Automated surgical endoscope for Raven II
Goal:
Reduce surgeon workload and provide optimum viewpoint.
Summary:
Takes images from the ZED Mini stereoscopic camera, passes them through
a trained MRCNN to identify and localize objects in the left and right
images, sends these pixel coordinates to another function to determine
the location of the object relative to the base of the IBIS camera arm,
determines the centroid of the objects of interest, uses a control
algorithm to direct the IBIS to point the camera at this centroid.

11/20/19
on Server 1: cd to
/media/yq/6d7f1ede-896c-43f1-a2b8-b74fc9a3d86b/home/yq/Desktop/Devon/zed-python-api-legacy
model.py is in /examples/mrcnn/    live_camera.py files are in /examples/
in terminal, run:
python3 examples/automated_IBIS_5.py splash --weights=/media/yq/0d57f6b9-4507-4edb-9800-537ac5258c2b/home/hdd_files/samin/dsn2019/prelimstudy/Suturing/logs/surgery20191111T2135/mask_rcnn_surgery_0020.h5
python3 examples/live_camera.py splash --weights=/media/yq/0d57f6b9-4507-4edb-9800-537ac5258c2b/home/hdd_files/samin/dsn2019/prelimstudy/Suturing/logs/surgery20191111T2135/mask_rcnn_surgery_0020.h5

12/03/19: folder and command
/media/yq/6d7f1ede-896c-43f1-a2b8-b74fc9a3d86b/home/yq/Desktop/Devon/zed-python-api-legacy/examples$
python3 automated_IBIS_5.py splash --weights=/media/yq/0d57f6b9-4507-4edb-9800-537ac5258c2b/home/hdd_files/samin/dsn2019/prelimstudy/Suturing/logs/surgery20191122T2113/mask_rcnn_surgery_0020.h5

12/06/19:
fixed math issues in control code

Mask R-CNN
Train on the surgery robot dataset.

Copyright (c) 2018 Matterport, Inc.
Licensed under the MIT License (see LICENSE for details)
Written by Waleed Abdulla

------------------------------------------------------------

Usage: import the module (see Jupyter notebooks for examples), or run from
       the command line as such:

    #Train a new model starting from pre-trained COCO weights
    python surgery.py train --dataset=/home/.../mask_rcnn/data/surgery/ --weights=coco

    #Train a new model starting from pre-trained ImageNet weights
    python surgery.py train --dataset=/home/.../mask_rcnn/data/surgery/ --weights=imagenet

    # Continue training the last model you trained. This will find
    # the last trained weights in the model directory.
    python surgery.py train --dataset=/home/.../mask_rcnn/data/surgery/ --weights=last

    #Detect and color splash on a image with the last model you trained.
    #This will find the last trained weights in the model directory.
    python surgery.py splash --weights=last --image=/home/...../*.jpg

    #Detect and color splash on a video with a specific pre-trained weights of yours.
    python sugery.py splash --weights=/home/.../logs/mask_rcnn_surgery_0030.h5  --video=/home/simon/Videos/Center.wmv
"""
# Libraries
import serial
import time
from time import sleep
import csv
import re
import math
import io
import cv2
import multiprocessing
from multiprocessing import Process

# For ZED
import pyzed.camera as zcam
import pyzed.types as tp
import pyzed.core as core
import pyzed.defines as sl

camera_settings = sl.PyCAMERA_SETTINGS.PyCAMERA_SETTINGS_BRIGHTNESS
str_camera_settings = "BRIGHTNESS"
step_camera_settings = 1

# MRCNN
import os
import sys
import json
import datetime
import numpy as np
import skimage.draw
import keras.backend as K
from skimage.viewer import ImageViewer
from matplotlib import pyplot as plt
# Root directory of the project
ROOT_DIR = os.path.abspath("../../")

# Import Mask RCNN
sys.path.append(ROOT_DIR)  # To find local version of the library
from mrcnn.config import Config
from mrcnn import model as modellib, utils
from mrcnn import visualize
#from mrcnn.model import MaskRCNN

### for some reason, this works
import tensorflow as tf

config = tf.ConfigProto()
config.gpu_options.allow_growth = True
tf.keras.backend.set_session(tf.Session(config=config))
###

DEFAULT_LOGS_DIR = os.path.join(ROOT_DIR, "logs")

# Definitions for communicating with IBIS Ardunio
# Credits to Erick, Asha, and Ray
port = "/dev/ttyACM0"
baudrate = 9600
ser = serial.Serial(port, baudrate)
x = "a"

# txt file to store data from IBIS
file_name = "serial_data.txt"
file_create = open(file_name, "w+")

# note: updates entire txt and csv file every 0.1 sec
start_time = time.time()

# Constants for object position calculations
# Constants
f = 700     # focal distance (pixels)
S = 63      # eye separation (mm)

mmToInch = 0.0393701    # 1 mm = 0.039701 inches

# MRCNN class id names
class_names = ['Left Grasper', 'Right Grasper', 'Red Block', 'Green Block', 'Background']
############################################################
#  Configurations
############################################################

class SurgeryConfig(Config):
    """Configuration for training on the toy dataset.
    Derives from the base Config class and overrides some values.
    """
    # Give the configuration a recognizable name
    NAME = "surgery"

    # We use a GPU with 12GB memory, which can fit two images.
    # Adjust down if you use a smaller GPU.
    IMAGES_PER_GPU = 4

    # Number of classes (including background)
    NUM_CLASSES = 1 + 4  # Background + objects

    # Number of training steps per epoch
    STEPS_PER_EPOCH = 500

    # Skip detections with < 90% confidence
    DETECTION_MIN_CONFIDENCE = 0.9

# Functions
def detect_and_color_splash(model, image_path=None, video_path=None, out_dir=''):
    assert image_path or video_path
    #print(K.eval(model.optimizer.lr))
    class_names = ['Left Grasper', 'Right Grasper', 'Red Block', 'Green Block', 'Background']

    # Image or video?
    if image_path:
        # Run model detection and generate the color splash effect
        print("Running on {}".format(args.image))
        # Read image
        image = skimage.io.imread(args.image)
        # Detect objects
        r = model.detect([image], verbose=1)[0]
        # Color splash
        # splash = color_splash(image, r['masks'])

        # hb 10/21 - coordinates
        print(r['rois'])

        visualize.display_instances(image, r['rois'], r['masks'], r['class_ids'],
                                    class_names, r['scores'], making_image=True)
        file_name = 'splash.png'
        # Save output
        # file_name = "splash_{:%Y%m%dT%H%M%S}.png".format(datetime.datetime.now())
        # save_file_name = os.path.join(out_dir, file_name)
        # skimage.io.imsave(save_file_name, splash)
    elif video_path:
        import cv2
        # Video capture
        vcapture = cv2.VideoCapture(video_path)
        # width = int(vcapture.get(cv2.CAP_PROP_FRAME_WIDTH))
        # height = int(vcapture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        width = 1600
        height = 1600
        fps = vcapture.get(cv2.CAP_PROP_FPS)
        # Define codec and create video writer
        file_name = "splash_{:%Y%m%dT%H%M%S}.wmv".format(datetime.datetime.now())
        vwriter = cv2.VideoWriter(file_name,
                                  cv2.VideoWriter_fourcc(*'MJPG'),
                                  fps, (width, height))

        count = 0
        success = True
        #For video, we wish classes keep the same mask in frames, generate colors for masks
        colors = visualize.random_colors(len(class_names))
        while success:
            print("frame: ", count)
            # Read next image
            plt.clf()
            plt.close()
            success, image = vcapture.read()
            if success:
                # OpenCV returns images as BGR, convert to RGB
                image = image[..., ::-1]
                # Detect objects
                r = model.detect([image], verbose=0)[0]
                # Color splash
                # splash = color_splash(image, r['masks'])

                splash = visualize.display_instances(image, r['rois'], r['masks'], r['class_ids'],
                                                     class_names, r['scores'], colors=colors, making_video=True)
                # Add image to video writer
                vwriter.write(splash)
                count += 1
        vwriter.release()
    print("Saved to ", file_name)

# Read IBIS and log data
def logIBIS(position):
    # check that data is readable
    initialized = False
    first_chunk_ready = False

    #key = ''
    while True:
    #while key != 113:   # for 'q' key
        #print("read")
        x = ser.readline()
        #print(str(x)[2:-5]+"\n")   # prints directly from Arduino

        # append to file
        file_out = open(file_name, "a")
        # -5 accounts for the \r\n' at the end, 2 accounts for b'
        file_out.write(str(x)[2:-5]+"\n")
        file_out.close()

        # check if file is ready yet
        file_in = open(file_name, "r")
        curr_line = file_in.readline()
        if not (initialized and first_chunk_ready): # initialized:
            while curr_line and curr_line != 'IBIS initialized\n':   # can have speedup by not checking this every time
                curr_line = file_in.readline()
            if curr_line == 'IBIS initialized\n':
                initialized = True
                curr_line = file_in.readline()
                while curr_line and curr_line != '\n':
                    curr_line = file_in.readline()
                if curr_line == '\n':
                    first_chunk_ready = True
                    # if you uncomment this, it will break the while True
                    # break
        file_in.close()

        if initialized and first_chunk_ready:
            txt_to_csv(position)

# txt to csv for logging IBIS data
def txt_to_csv(position):
    file_in = open(file_name,"r")      ################### UPDATE THIS WITH FILE NAME

    #print('readlines:',file_in.readlines())  #check for if it's reading anything
    while file_in.readline() != 'IBIS initialized\n': #waits for IBIS initialized to start
        continue
    file_in.readline()
    #file_in = open(file_name,"r")
    lines = [line for line in file_in.readlines() if line]
    file_in.close()
    #print('lines:',lines)
    chunks = [[] for x in range(len(lines))]
    i = 0

    #print('chunks1',chunks)
    #prev_line = ''
    for line in lines:
        chunks[i].append(line)
        if line == 'kinUpdate\n':# && prev_line == '\n':  #filters chunks based on \nkinUpdate
            i+=1
        #prev_line = line

    #takes into account negatives and numbers with/without decimal points
    #a number like 0009 is still counted as a number
    format = re.compile('[a-z A-Z]*: .*')   #basic format for if it's a var to add to the log header
    format_num = re.compile('^ *-?[0-9]+\.?[0-9]*( [ a-zA-Z]*)?$')   #finds portion of string that matches with number
    format_num_only = re.compile('-?[0-9]+\.?[0-9]*')
    format_num_tuple = re.compile(' *-?[0-9]+\.?[0-9]* *, *-?[0-9]+\.?[0-9]*')  #matches if it's a tuple of 2

    #make headers
    #need to adjust this part to account for the random start place in the txt file
    headers = []
    for val in chunks[0]:       #jk didn't do -> #changed to using chunks[2] since sometimes chunks[1] would be super long and weird
        if format.match(val) is not None:
            headers.append(val.split(":")[0])
    # print(headers)
    # exit()

    cols = [[] for l in range(len(chunks)-1)] #-1 to account for first val in chunks

    for a in range(1,len(chunks)):
        for b in range(len(chunks[a])):
            if (format.match(chunks[a][b]) is not None):# and (chunks[a][b].split(":")[0]==headers[len(cols[a-1])]):
                #second part of if statement checks for in case another value with colon gets added
                raw_chunk = chunks[a][b].split(": ")[1][0:-1]   #thing at the end gets rid of \n
                toAppend = raw_chunk                            #value to be appended
                tuple_check = format_num_tuple.search(raw_chunk)
                num_check = format_num.search(raw_chunk)
                if tuple_check is not None:                  #check if tuple
                    tup_vals = tuple_check.group(0).split(",")  #split tuple into list for now
                    first_val = float(tup_vals[0])
                    second_val = float(tup_vals[1])
                    toAppend = (first_val,second_val)       #update toAppend
                elif num_check is not None:                  #check if number
                    toAppend = float(format_num_only.search(raw_chunk).group(0))   #update toAppend to matched num
                cols[a-1].append(toAppend)

                #print("col: ")
                #print(cols[a-1])
                if len(cols[a-1]) == 6:   # 6 is the current number of columns written to the csv
                    #print(cols[a-1])

                    for idx, n in enumerate(cols[a-1]):
                        position[idx] = n
                        #print(position[idx])
                    # write cols[a-1] to a multiprocessing Array so controlIBIS can access the IBIS position


    cols = [x for x in cols if x] #clear empty lists from cols
    #list(filter([], cols))
    #print(position[:])

    #with open('/home/student/Documents/Research/IBIS/log.csv', 'w') as out_file:
    with open('/media/yq/6d7f1ede-896c-43f1-a2b8-b74fc9a3d86b/home/yq/Desktop/Devon/zed-python-api-legacy/examples/log.csv', 'w') as out_file:
        writer = csv.writer(out_file)
        writer.writerow((h for h in headers))
        writer.writerows((col for col in cols))

# sort lists from MRCNN
def sort_objects(list):
    seed = list[0]
    sorted = [seed]
    list.remove(seed)
    for i in range(len(list)):
        tosort = list[0]
        list.remove(tosort)
        for j in range(len(sorted)):
            if tosort[1][0] < sorted[j][1][0]:
                sorted.insert(j, tosort)
                break
            elif j < len(sorted)-1:
                pass
            else:
                sorted.append(tosort)
    return sorted

# Object positions relative to IBIS, returns list of x, y, z coords
def object_position(pos, Lp, Rp):
    # extract position from pos array
    xCam = pos[2]
    yCam = pos[3]
    tBaseraw = pos[1]     # in radians where forward is 1.57 rad
    xCamVec = pos[4]
    yCamVec = pos[5]

    # Offset for tBase
    tBase = tBaseraw - 1.57

    # extract pixel coords from lists
    Lpx = Lp[0]
    Lpy = Lp[1]
    Rpx = Rp[0]
    Rpy = Rp[1]

    #print(xCam, yCam, tBaseraw, xCamVec, yCamVec, tBase, Lpx, Lpy, Rpx, Rpy)

    # Position of the camera and camera vector wrt IBIS base, accounting for base servo position
    Cam = np.array([xCam*math.cos(tBase), yCam, xCam*math.sin(tBase)])
    CamVec = np.array([xCamVec*math.cos(tBase), yCamVec, xCamVec*math.sin(tBase)])

    # Create perpendicular vectors to the camera vector
    CamVecO = CamVec
    ChO = np.array([-CamVecO.item(2), 0, CamVecO.item(0)])
    CvO = np.cross(ChO, CamVecO)

    # Create unit vectors
    COnorm = CamVecO/np.linalg.norm(CamVecO)
    ChOnorm = ChO/np.linalg.norm(ChO)
    CvOnorm = CvO/np.linalg.norm(CvO)


    # From the stereoimages, calculate the distance to the object and the vertical and horizontal displacements
    # First, the distance to the object
    dX = np.abs(Lpx - Rpx)
    d = (f*S*mmToInch)/dX

    # Second, the horizontal displacement, wrt the right eye, b/c most people are right eye dominant
    # if statement corrects for the sign of the displacement
    if (np.abs(Lpx - 640) > np.abs(Rpx-640)):
        sign = 1
    else:
        sign = -1

    XO = sign*(d*Rpx)/f
    h = 1.24 + XO # horizontal distance

    # Third, the vertical displacement
    Hf = 360                        # pixels from middle of image to top
    Hp = 360 - Rpy                  # pixels from middle of image to point
    hf = d * np.tan(tBase)          # inches from middle of the image to top
    hp = (Hp/Hf)* hf                # inches from middle of image to point

    Pobject = Cam + COnorm*d + ChOnorm*h + CvOnorm*hp

    #print("Pobject")
    #print(Pobject)
    return Pobject

# Project objects onto a plane (u, v) containing the centroid of the objects and
# perpendicular to the camera vector
# send commands to IBIS
def project_to_cplane_and_control(pos, object_centers, centroid):
    # extract position from pos array
    xCam = pos[2]
    yCam = pos[3]
    tBaseraw = pos[1]     # in radians where forward is 1.57 rad
    xCamVec = pos[4]
    yCamVec = pos[5]

    # Offset for tBase
    tBase = tBaseraw - 1.57

    # Position of the camera and camera vector wrt IBIS base, accounting for base servo position
    Cam = np.array([xCam*math.cos(tBase), yCam, xCam*math.sin(tBase)])
    CamVec = np.array([xCamVec*math.cos(tBase), yCamVec, xCamVec*math.sin(tBase)])

    # Create perpendicular vectors to the camera vector
    CamVecO = CamVec
    ChO = np.array([-CamVecO.item(2), 0, CamVecO.item(0)])
    CvO = np.cross(ChO, CamVecO)

    # Create unit vectors
    COnorm = CamVecO/np.linalg.norm(CamVecO)
    ChOnorm = ChO/np.linalg.norm(ChO)
    CvOnorm = CvO/np.linalg.norm(CvO)

    # Matrix of unit vector components
    # ho is ChOnorm, vo is CvOnorm, no is COnorm
    # ([[xho, xvo, xno], [yho, yvo, yno], [zho, zvo, zno]])
    unitvecmatrix = np.array([  [ChOnorm[0], CvOnorm[0], COnorm[0]],
                                [ChOnorm[1], CvOnorm[1], COnorm[1]],
                                [ChOnorm[2], CvOnorm[2], COnorm[2]]])

    # The plane (u, v) containing the centroid and perpendicular to
    # the camera vector (COnorm) is described by:
    # COnorm * (x, y, z)' = COnorm * centroid

    # The location of the camera image center on the (u, v) plane is
    # the projection of the camera vector to the plane
    # Distance of the camera from the (u, v) plane
    dcam = (np.dot(COnorm, centroid))-(np.dot(COnorm, Cam))

    # Project the camera image  center onto the (u, v) plane using
    # P' = P + d*COnorm,  this is the origin of the (u, v) plane
    camprojpoint = Cam + dcam*COnorm

    print(f"camera: {Cam}")
    # print(f"camera focus on plane: {camprojpoint}")

    # For each object in the object_centers list, find the distance to the
    # (u, v) plane and calculate its coordinates on this plane relative
    # to the origin (defined as the projection of the camera vector onto
    # the (u, v) plane)

    # To calculate the average distance between the centroid and object projs
    rtot =  0
    n = len(object_centers)


    for obj in range(len(object_centers)):
        # Get coordinates of object from the list
        objpoint = object_centers[obj][1]

        # The distance, d, of each object to this plane is given by:
        # d = ((COnorm * pobj)-(COnorm*centroid))/||COnorm||
        # magnitdue of a unit normal vector is 1, so ||COnorm|| = 1
        d = ((np.dot(COnorm, centroid))-(np.dot(COnorm, objpoint)))

        # Project the object onto the (u, v) plane using P' = P + d*COnorm
        projpoint = objpoint + d*COnorm

        # Find vectors from camprojpoint to projpoint
        planevec = projpoint - camprojpoint

        # Break down into (u, v) components using lin alg
        uvcoords = np.linalg.solve(unitvecmatrix, planevec)

        # Find distance on (u, v) plane between centroid and
        # projection of object
        # vector from centroid to projection of object
        veccp = centroid - projpoint
        r = np.sqrt(veccp.dot(veccp))

        rtot += r

        # print(f"obj: {objpoint}")
        # print(f"uv coords: {uvcoords}")
        # print("\n")


    # Average radius of circle of objects in plane
    avgr = rtot / n

    # Calculate camera projection to centroid vector in (u, v) plane
    centroiduvvect = centroid - camprojpoint

    # Break down into (u, v) components using lin alg
    centuv = np.linalg.solve(unitvecmatrix, centroiduvvect)

    # Calculate the max size of the visible plane with the centroid
    # (middle to top of rectangle) (tan(60 deg) = 1.732)
    vertvis = 1.732 * dcam

    # Radius of desired view, currently defined at 50%
    rvis = 0.5 * vertvis

    # Threshold
    threshold = 0.1 * vertvis


    # Send commands to IBIS
    # I, O, U, D, L, R in the format b'I'

    # Control I/O for zoom
    if (avgr - rvis) > threshold:
        # X out
        ser.write(b'O')
        print("Command: Out")
    if (rvis - avgr) > threshold:
        # X in
        ser.write(b'I')
        print("Command: In")

    # Control U/D for up and down
    if (abs(centuv[1])) > threshold:
        if (centuv[1] > 0):
            # Down
            #ser.write(b'D')
            print("Command: Down")
        if (centuv[1] < 0):
            # Up
            #ser.write(b'U')
            print("Command: Up")
    else:
        print("No movement U/D")

    # Control L/R for pan
    if (abs(centuv[0])) > threshold:
        if (centuv[0] > 0):
            # Right
            #ser.write(b'R')
            print("Command: Right")
        if (centuv[0] < 0):
            # Left
            #ser.write(b'L')
            print("Command: Left")
    else:
        print("No movement L/R")


# ZED settings
def settings(key, cam, runtime, mat):
    if key == 115:  # for 's' key
        switch_camera_settings()
    elif key == 43:  # for '+' key
        current_value = cam.get_camera_settings(camera_settings)
        cam.set_camera_settings(camera_settings, current_value + step_camera_settings)
        print(str_camera_settings + ": " + str(current_value + step_camera_settings))
    elif key == 45:  # for '-' key
        current_value = cam.get_camera_settings(camera_settings)
        if current_value >= 1:
            cam.set_camera_settings(camera_settings, current_value - step_camera_settings)
            print(str_camera_settings + ": " + str(current_value - step_camera_settings))
    elif key == 114:  # for 'r' key
        cam.set_camera_settings(sl.PyCAMERA_SETTINGS.PyCAMERA_SETTINGS_BRIGHTNESS, -1, True)
        cam.set_camera_settings(sl.PyCAMERA_SETTINGS.PyCAMERA_SETTINGS_CONTRAST, -1, True)
        cam.set_camera_settings(sl.PyCAMERA_SETTINGS.PyCAMERA_SETTINGS_HUE, -1, True)
        cam.set_camera_settings(sl.PyCAMERA_SETTINGS.PyCAMERA_SETTINGS_SATURATION, -1, True)
        cam.set_camera_settings(sl.PyCAMERA_SETTINGS.PyCAMERA_SETTINGS_GAIN, -1, True)
        cam.set_camera_settings(sl.PyCAMERA_SETTINGS.PyCAMERA_SETTINGS_EXPOSURE, -1, True)
        cam.set_camera_settings(sl.PyCAMERA_SETTINGS.PyCAMERA_SETTINGS_WHITEBALANCE, -1, True)
        print("Camera settings: reset")
    elif key == 122:  # for 'z' key
        record(cam, runtime, mat)



# ------------------------------------------------------------------------------
# Main Function
if __name__ == '__main__':
    import argparse

    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Train Mask R-CNN to detect rings and robot arms.')
    parser.add_argument("command",
                        metavar="<command>",
                        help="'train' or 'splash'")
    parser.add_argument('--dataset', required=False,
                        metavar="/home/yq/hdd_files/samin/dsn2019/Suturing/ObjectTracking/data/train",
                        help='Directory of the surgery dataset')
    parser.add_argument('--weights', required=True,
                        metavar="/home/simon/logs/weights.h5",
                        help="Path to weights .h5 file or 'coco'")
    parser.add_argument('--logs', required=False,
                        default=DEFAULT_LOGS_DIR,
                        metavar="/path/to/logs/",
                        help='Logs and checkpoints directory (default=logs/)')
    parser.add_argument('--image', required=False,
                        metavar="path or URL to image",
                        help='Image to apply the color splash effect on')
    parser.add_argument('--video', required=False,
                        metavar="path or URL to video",
                        help='Video to apply the color splash effect on')
    parser.add_argument('--subset', required=False,
                        metavar="Dataset sub-directory",
                        help="Subset of dataset to run prediction on")
    args = parser.parse_args()

    # Configurations
    if args.command == "train":
        config = SurgeryConfig()
    else:
        class InferenceConfig(SurgeryConfig):
            # Set batch size to 1 since we'll be running inference on
            # one image at a time. Batch size = GPU_COUNT * IMAGES_PER_GPU
            GPU_COUNT = 1
            IMAGES_PER_GPU = 1
        config = InferenceConfig()
    config.display()

    # Create model
    model = modellib.MaskRCNN(mode="inference", config=config,model_dir=args.logs)

    # Select weights file to load
    if args.weights.lower() == "last":
        # Find last trained weights
        weights_path = model.find_last()[1]
        print (weights_path)
    elif args.weights.lower() == "imagenet":
        # Start from ImageNet trained weights
        weights_path = model.get_imagenet_weights()
    else:
        weights_path = args.weights

    # Load weights
    print("Loading weights ", weights_path)
    if args.weights.lower() == "coco":
        # Exclude the last layers because they require a matching
        # number of classes
        model.load_weights(weights_path, by_name=True, exclude=[
            "mrcnn_class_logits", "mrcnn_bbox_fc",
            "mrcnn_bbox", "mrcnn_mask"])
    else:
        model.load_weights(weights_path, by_name=True)

    # IBIS position
    # Array of float type with space for 6 floats
    position = multiprocessing.Array('d', 6)
    # Start logging position data from IBIS
    logIBISProcess = Process(target=logIBIS, args=(position,))
    logIBISProcess.start()

    # Set up ZED to get images
    print("Setting up ZED...")
    init = zcam.PyInitParameters()
    cam = zcam.PyZEDCamera()
    if not cam.is_opened():
        print("Opening ZED Camera...")
    status = cam.open(init)
    if status != tp.PyERROR_CODE.PySUCCESS:
        print(repr(status))
        exit()

    runtime = zcam.PyRuntimeParameters()
    mat = core.PyMat()

# ----------------------------------------------------------------------------------------------
    #detect_and_color_splash(model, args.image)

    key = ''
    while key != 113:   # for 'q' key
        # Stream video or images from the ZED Mini through the MRCNN
        # returns list of lists of objects and their coordinates

        # Grab images from the ZED
        #detect_and_color_splash(model, args.image) # -------------------------------------------
        err = cam.grab(runtime)
        if err == tp.PyERROR_CODE.PySUCCESS:
            print("Getting images")
            #start_time =time.time()
            # Retrieve left image
            cam.retrieve_image(mat, sl.PyVIEW.PyVIEW_LEFT)
            # cv2.imshow("ZED", mat.get_data())
            img_left = mat.get_data()[:,:,0:3]
            #img_left_new = np.expand_dims(img_left, axis=2)

            kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
            #img_left = cv2.filter2D(img_left, -1, kernel)
            # print (img_left_new.shape)

            # Retrieve right image
            cam.retrieve_image(mat,sl.PyVIEW.PyVIEW_RIGHT)
            # cv2.imshow("ZED2", mat.get_data())
            img_right = mat.get_data()[:,:,0:3]
            #img_right_new = np.expand_dims(img_right, axis=2)
            #img_right = cv2.filter2D(img_right, -1, kernel)
            #print ("time for reading images {}" .format(time.time()-start_time))

            # Detect on images
            #start_time =time.time()

            results_list_left = (model.detect([img_left]))
            results_list_right = (model.detect([img_right]))

            r = model.detect_old([img_left], verbose=1)[0]
            visualize.display_instances(img_left, r['rois'], r['masks'], r['class_ids'],
                                        class_names, r['scores'], making_image=True)
            r = model.detect_old([img_right], verbose=1)[0]
            visualize.display_instances(img_right, r['rois'], r['masks'], r['class_ids'],
                                        class_names, r['scores'], making_image=True)
    # ---------------------------------------------------------------------------------------------------------------------

            #print("autonomous_IBIS")
            print("MRCNN objects found: ")
            print(results_list_left)
            print(results_list_right)
            settings(key, cam, runtime, mat)

            left_image_object_list = []# return list from MRCNN
            right_image_object_list = []# return list from MRCNN

            # example data
            left_image_object_list = [["Left Grasper", (549.9, 387.7),  (547.9, 385.7)], ["Right Grasper", (965.5, 205.6),  (963.5, 203.6)], ["Red Block", (656, 332.8),  (654, 330.8)], ["Red Block", (1141.75, 488.75),  (1139.75, 486.75)], ["Green Block", (662.8, 387.8),  (660.8, 385.8)]]
            right_image_object_list = [["Left Grasper", (268.4, 384.1),  (266.4, 382.1)], ["Right Grasper", (720.7, 193.5),  (718.7, 191.5)], ["Red Block", (487.7, 338.7),  (485.7, 336.7)], ["Red Block", (937.1, 484.2),  (935.1, 482.2)], ["Green Block", (470.8, 387.5),  (468.8, 385.5)]]
            #print(type(left_image_object_list[0][1][0]))
            #print("control")

            if len(results_list_left) > 0:
                print("from ZED")
                left_image_object_list = sort_objects(results_list_left)
                right_image_object_list = sort_objects(results_list_right)
                print("sorted: ")
                print(left_image_object_list)
                print(right_image_object_list)

            #print(type(left_image_object_list[0][1][0]))
            #print(left_image_object_list)
            #print(right_image_object_list)

            try:
                # Read current position of IBIS
                IBISposition = position[:]  # position[:] has type list
                #print("IBIS position: {}" .format(IBISposition))
                #print(IBISposition)
                IBISposition = [54607.0, 1.45, 4.36, 0.94, 0.87, -0.49]
                print(IBISposition)

                # Determine centers of each object found for the left and right images
                left_image_object_centers = []
                right_image_object_centers = []

                # Iterate over all objects in the left image
                for object in range(len(left_image_object_list)):
                    # create a list for each object to contain the object and the pixel coordinates of its center
                    obj = [left_image_object_list[object][0]]
                    #print(left_image_object_list[object][1][0])
                    # calculate the center of each object
                    x_center = (left_image_object_list[object][1][0] + left_image_object_list[object][2][0])/2
                    y_center = (left_image_object_list[object][1][1] + left_image_object_list[object][2][1])/2
                    #print(x_center)
                    # create a tuple for the pixel coordinates of the object
                    obj_coord = (x_center, y_center)
                    #print(obj_coord)
                    # append the coordinates to the list for the object
                    obj.append(obj_coord)
                    #print(obj)
                    # append the object to the list of objects and their centers
                    left_image_object_centers.append(obj)
                    #print(left_image_object_centers)
                # Iterate over all objects in the right image
                for object in range(len(right_image_object_list)):
                    # create a list for each object to contain the object and the pixel coordinates of its center
                    obj = [right_image_object_list[object][0]]

                    # calculate the center of each object
                    x_center = (right_image_object_list[object][1][0] + right_image_object_list[object][2][0])/2
                    y_center = (right_image_object_list[object][1][1] + right_image_object_list[object][2][1])/2

                    # create a tuple for the pixel coordinates of the object
                    obj_coord = (x_center, y_center)

                    # append the coordinates to the list for the object
                    obj.append(obj_coord)

                    # append the object to the list of objects and their centers
                    right_image_object_centers.append(obj)

                #print(left_image_object_centers)
                #print(right_image_object_centers)

                # Determine location of each object relative to the base of the IBIS arm
                object_centers = []
                x_tot = 0
                y_tot = 0
                z_tot = 0
                n = len(left_image_object_centers)

                for object in range(len(left_image_object_centers)):
                    # create a list for each object to contain the object and the coordinates of its center
                    obj = [left_image_object_centers[object][0]]
                    obj_pos = object_position(IBISposition, left_image_object_centers[object][1], right_image_object_centers[object][1])
                    #print(type(obj_pos))
                    #print(obj_pos)

                    # Sums of x, y, z coords for taking the centroid
                    x_tot += obj_pos[0]
                    y_tot += obj_pos[1]
                    z_tot += obj_pos[2]
                    #print(x_tot, y_tot, z_tot)

                    obj.append(obj_pos.tolist())
                    object_centers.append(obj)

                #print('\n')
                print("object ceners from IBIS")
                print(object_centers)

                # Determine the centroid of the objects in 3D wrt the IBIS
                avg_x = x_tot/n
                avg_y = y_tot/n
                avg_z = z_tot/n

                # centroid
                centroid = [avg_x, avg_y, avg_z]
                print('centroid: ')
                print(centroid)

                # Determine motion
                project_to_cplane_and_control(IBISposition, object_centers, centroid)
                cv2.waitKey(1)
                #sleep(1)

            except:
                print("passed control loop")
                pass

        else:
            print ("entering else")
            key = cv2.waitKey(5)

    cv2.destroyAllWindows()
    cam.close()
    print("\nFINISH")

# end
