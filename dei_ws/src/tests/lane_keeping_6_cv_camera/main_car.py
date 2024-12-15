#!/usr/bin/python
# -*- coding: utf-8 -*-

#from this import d
import cv2 as cv
import rospy
import numpy as np
from time import sleep
from time import time
import os


from automobile_data import Automobile_Data     # I/O car manager
from controller3 import Controller              # Lane Keeping
from maneuvers import Maneuvers                 # Maneuvers
from helper_functions import *                  # helper functions
from detection import Detection                 # detection
from collections import deque
from traffic_light import TrafficLightDetector
import serial
import logging
 

# PARAMETERS
SAMPLE_TIME = 0.01      # [s]
WAIT_TIME_STOP = 3.0    # [s]
DESIRED_SPEED = -6.5   # [m/s]
OBSTACLE_DISTANCE = 0.20 # [m]
SLOWDOWN_DIST_VALUE = 0.2     # [m]
STOP_DIST_VALUE = 0.6         # [m]
SO_GOC_LAI = 4
SHOW_IMGS = True


# Pure pursuit controller parameters
k1 = 0 #4.0 gain error parallel to direction (speed)
k2 = 0.0 #2.0 perpedndicular error gain
k3 = 1.2 #1.5 yaw error gain 

#dt_ahea  = 0.5 # [s] how far into the future the curvature is estimated, feedforwarded to yaw controller
ff_curvature = 0.0 # feedforward gain

intersection = 0
intersection_state = None
intersection_start_time = None
is_handling_intersection = False

# Hàm x? lý giao l? không ch?n
intersection_actions = {
    1: [
        {"angle": 0, "speed": -6, "time": 0.8},
        {"angle": 25, "speed": -6, "time": 2.7},
    ],
    2: [
        {"angle": 0, "speed": -6, "time": 1.65},
        {"angle": -25, "speed": -6, "time": 1.6},
    ],
    3: [
        {"angle": 5, "speed": -6, "time": 1.8},
    ],
    4: [
        {"angle": 0, "speed": 0, "time": 3},
        {"angle": 0, "speed": -6, "time": 1.3},
        {"angle": -25, "speed": -6, "time": 1.3},
    ],
    5: [
        {"angle": 0, "speed": -6, "time": 1.73},
        {"angle": 25, "speed": -6, "time": 2.6},
    ],
    6: [
        {"angle": 0, "speed": -6, "time": 1.1},
        {"angle": 25, "speed": -6, "time": 2.7},
    ],
    7: [
        {"angle": 25, "speed": -6, "time": 0.4},
        {"angle": 0, "speed": -6, "time": 1.6},
    ],
    8: [
        {"angle": 0, "speed": -6, "time": 1.3},
        {"angle": 25, "speed": -6, "time": 2.2},
    ],
}






def handle_intersection(car, intersection):
    global intersection_state, intersection_start_time, is_handling_intersection
    actions = intersection_actions.get(intersection, [])

    if intersection_state is None:
        if actions:
            intersection_state = 0
            intersection_start_time = time()
            is_handling_intersection = True
            action = actions[intersection_state]
            car.drive(speed=action["speed"], angle=action["angle"])
            logging.info(f"Starting action {intersection_state + 1} at intersection {intersection}")
        return intersection

    if intersection_state < len(actions):
        elapsed_time = time() - intersection_start_time
        current_action = actions[intersection_state]
        if elapsed_time >= current_action["time"]:
            intersection_state += 1
            if intersection_state < len(actions):
                next_action = actions[intersection_state]
                intersection_start_time = time()
                car.drive(speed=next_action["speed"], angle=next_action["angle"])
                logging.info(f"Performing action {intersection_state + 1} at intersection {intersection}")
            else:
                logging.info(f"Completed all actions at intersection {intersection}")
                intersection_state = None
                is_handling_intersection = False
                return intersection
    return intersection


if __name__ == '__main__':

    #load camera with opencv
    cap = cv.VideoCapture(0)
    cap.set(cv.CAP_PROP_FRAME_WIDTH, 400)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, 400)
    cap.set(cv.CAP_PROP_FPS, 30)

    # init the car flow of data
    car = Automobile_Data(trig_control=True, trig_cam=False, trig_gps=False, trig_bno=True, trig_enc=True, trig_sonar=True, trig_sonar1=True)
    maneuver = Maneuvers()

    # initialize detector
    detect = Detection()
    traffic_lights = TrafficLightDetector()

    if SHOW_IMGS:
        cv.namedWindow('Frame preview', cv.WINDOW_NORMAL)
        cv.resizeWindow('Frame preview', 320, 240)
        cv.namedWindow('lane_detection', cv.WINDOW_NORMAL)
        cv.resizeWindow('lane_detection', 200, 200)

    # init controller
    controller = Controller(k1=k1, k2=k2, k3=k3, ff=ff_curvature, training=False)

    start_time_stamp = 0.0      # [s]
    angle_ref = 0.0             # [deg]

    # RESET THE CAR POSITION
    car.stop()
    print("Starting in 3 seconds...")
    sleep(3)
    car.drive_speed(speed=-5)
    GOC_LAI_TRUOC = deque(maxlen = SO_GOC_LAI)

    try:
        

        while not rospy.is_shutdown():
            start_time_stamp = time() * 1000.0

            # Get the image from the camera
            # frame = car.cv_image.copy()
            ret, frame = cap.read()
            if not ret:
                print("No image from camera")
                #frame = np.zeros((480, 640, 3), np.uint8)
                continue

            # -------------------- LANE KEEPING --------------------
            #Neural network control
            #response = uart.receive_data()
            #print(f"Response t? UART: {response}")
            
            processed_frame,dominant_color,color_pixel_counts= traffic_lights.detect_traffic_light(frame)

            lane_info = detect.detect_lane(frame, show_ROI=SHOW_IMGS)
            e2, e3, point_ahead = lane_info
            curv = 0.005
            speed_ref, angle_ref = controller.get_control(e2, e3, curv, DESIRED_SPEED)

            dist = detect.detect_stop_line(frame, show_ROI=SHOW_IMGS)
            #if response == 'STOP':

            distance = car.obstacle_ahead_median
            distance1 = car.obstacle_ahead_median1
            
            if is_handling_intersection:
                intersection = handle_intersection(car, intersection)
            elif distance < 0.48 and intersection == 1:
                pass
            elif distance < 0.48:
                car.stop()
                 
            elif dominant_color == 'Red':
                logging.info("Ðèn d? - D?ng xe")
                car.stop()
            elif dist < 0.3:
                intersection +=1
                intersection = handle_intersection(car, intersection)
            
            else:
                GOC_LAI_TRUOC.append(angle_ref)
                GOC_LAI_TRUNG_BINH = np.mean(GOC_LAI_TRUOC)
                #car.drive(speed=speed_ref, angle=np.rad2deg(-GOC_LAI_TRUNG_BINH))

            
            #cv.imshow('traffic_light',processed_frame)

            # -------------------- DEBUG --------------------
            os.system('cls' if os.name=='nt' else 'clear')
            print(f'distance: {distance}')
            print(f'distance1: {distance1}')
            
            print(f'intersection : {intersection}')

            

            if SHOW_IMGS:
                #project point ahead
                frame, proj = project_onto_frame(frame, car, point_ahead, False, color=(200, 200, 100))
                if proj is not None:
                    #convert proj to cv2 point
                    proj = (int(proj[0]), int(proj[1]))
                    #draw line from bottmo half to proj
                    cv.line(frame, (320,479), proj, (200, 200, 100), 2)

                cv.imshow('Frame preview', frame)
                key = cv.waitKey(1)
                if key == 27:
                    car.stop()
                    sleep(1.5)
                    cv.destroyAllWindows()
                    print("Shutting down...")
                    logging.info("Communication stopped by user")
                    exit(0)
                    uart.close()

    except rospy.ROSInterruptException:
        print('inside interrupt exeption')
        pass
      

