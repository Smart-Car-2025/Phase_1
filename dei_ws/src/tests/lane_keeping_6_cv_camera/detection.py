#!/usr/bin/python3

import numpy as np
import cv2 as cv
import os
from time import time, sleep

from scipy.__config__ import show

from helper_functions import *

LANE_KEEPER_PATH = "models/lane_keeper_small.onnx"
DISTANCE_POINT_AHEAD = 0.35
CAR_LENGTH = 0.4

STOP_LINE_ESTIMATOR_PATH = "models/stop_line_estimator.onnx"



class Detection:

    #init 
    def __init__(self) -> None:

        #lane following
        self.lane_keeper = cv.dnn.readNetFromONNX(LANE_KEEPER_PATH)
        self.lane_cnt = 0
        self.avg_lane_detection_time = 0

        #stop line detection
        self.stop_line_estimator = cv.dnn.readNetFromONNX(STOP_LINE_ESTIMATOR_PATH)
        self.est_dist_to_stop_line = 1.0
        self.avg_stop_line_detection_time = 0
        self.stop_line_cnt = 0

       
    def detect_lane(self, frame, show_ROI=True, faster=False):
        """
        Estimates:
        - the lateral error wrt the center of the lane (e2), 
        - the angular error around the yaw axis wrt a fixed point ahead (e3),
        - the ditance from the next stop line (1/dist)
        """
        start_time = time()
        IMG_SIZE = (32,32) #match with trainer
        #convert to gray
        frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        #keep the bottom 2/3 of the image
        frame = frame[int(frame.shape[0]/3):,:] #/3
        #blur
        # frame = cv.blur(frame, (15,15), 0) #worse than blur after 11,11 #with 15,15 is 1ms
        # frame = cv.resize(frame, (2*IMG_SIZE[0], 2*IMG_SIZE[1]))
        # frame = cv.blur(frame, (3,3), 0) #worse than blur after 11,11
        frame = cv.resize(frame, IMG_SIZE)
        frame = cv.blur(frame, (2,2), 0)  #7,7 both is best

        # add noise 1.5 ms 
        std = 50
        # std = np.random.randint(1, std)
        noisem = np.random.randint(0, std, frame.shape, dtype=np.uint8)
        frame = cv.subtract(frame, noisem)
        noisep = np.random.randint(0, std, frame.shape, dtype=np.uint8)
        frame = cv.add(frame, noisep)
        
        
        images = frame

        if faster:
            blob = cv.dnn.blobFromImage(images, 1.0, IMG_SIZE, 0, swapRB=True, crop=False) 
        else:
            frame_flip = cv.flip(frame, 1) 
            #stack the 2 images
            images = np.stack((frame, frame_flip), axis=0) 
            blob = cv.dnn.blobFromImages(images, 1.0, IMG_SIZE, 0, swapRB=True, crop=False) 
        # assert blob.shape == (2, 1, IMG_SIZE[1], IMG_SIZE[0]), f"blob shape: {blob.shape}"
        self.lane_keeper.setInput(blob)
        out = self.lane_keeper.forward()
        output = out[0]
        output_flipped = out[1] if not faster else None

        e2 = output[0]
        e3 = output[1]

        if not faster:
            e2_flipped = output_flipped[0]
            e3_flipped = output_flipped[1]

            e2 = (e2 - e2_flipped) / 2
            e3 = (e3 - e3_flipped) / 2

        #calculate estimated of thr point ahead to get visual feedback
        d = DISTANCE_POINT_AHEAD
        est_point_ahead = np.array([np.cos(e3)*d, np.sin(e3)*d])
        
        # print(f"lane_detection: {1000*(time()-start_time):.2f} ms")
        lane_detection_time = 1000*(time()-start_time)
        self.avg_lane_detection_time = (self.avg_lane_detection_time*self.lane_cnt + lane_detection_time)/(self.lane_cnt+1)
        self.lane_cnt += 1
        if show_ROI:
            cv.imshow('lane_detection', frame)
            cv.waitKey(1)
        return e2, e3, est_point_ahead

    def detect_stop_line(self, frame, show_ROI=True):
        """
        Estimates the distance to the next stop line
        """
        start_time = time()
        IMG_SIZE = (32,32) #match with trainer
        #convert to gray
        frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        #keep the bottom 2/3 of the image
        frame = frame[int(frame.shape[0]*(2/5)):,:]
        #blur
        # frame = cv.blur(frame, (15,15), 0) #worse than blur after 11,11 #with 15,15 is 1ms
        frame = cv.resize(frame, (2*IMG_SIZE[0], 2*IMG_SIZE[1]))
        frame = cv.blur(frame, (3,3), 0) #worse than blur after 11,11
        frame = cv.resize(frame, IMG_SIZE)
        frame = cv.blur(frame, (2,2), 0)  #7,7 both is best

        # # add noise 1.5 ms 
        std = 50
        # std = np.random.randint(1, std)
        noisem = np.random.randint(0, std, frame.shape, dtype=np.uint8)
        frame = cv.subtract(frame, noisem)
        noisep = np.random.randint(0, std, frame.shape, dtype=np.uint8)
        frame = cv.add(frame, noisep)

        blob = cv.dnn.blobFromImage(frame, 1.0, IMG_SIZE, 0, swapRB=True, crop=False)
        # assert blob.shape == (1, 1, IMG_SIZE[1], IMG_SIZE[0]), f"blob shape: {blob.shape}"
        self.stop_line_estimator.setInput(blob)
        output = self.stop_line_estimator.forward()
        dist = output[0]
        self.est_dist_to_stop_line = dist

        # return e2, e3, inv_dist, curv, est_point_ahead
        # print(f"lane_detection: {1000*(time()-start_time):.2f} ms")
        stop_line_detection_time = 1000*(time()-start_time)
        self.avg_stop_line_detection_time = (self.avg_stop_line_detection_time*self.lane_cnt + stop_line_detection_time)/(self.lane_cnt+1)
        self.lane_cnt += 1
        if show_ROI:
            cv.imshow('stop_line_detection', frame)
            cv.waitKey(1)
        print(f"stop_line_detection dist: {dist}, in {stop_line_detection_time:.2f} ms")
        return dist

