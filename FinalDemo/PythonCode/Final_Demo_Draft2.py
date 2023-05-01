import cv2 as cv
import numpy as np
from picamera import PiCamera
from time import sleep
import serial
import smbus
import struct
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
import subprocess

address = 0x04
bus = smbus.SMBus(1)

w = 640
h = 420
'''
print("Starting calibration")
cap = cv.VideoCapture(0)
cap.set(3, w) # set horizontal resolution
cap.set(4, h) # set vertical resolution
cap.set(5, 24)
'''
subprocess.run("v4l2-ctl -c auto_exposure=1 -c exposure_time_absolute=20",shell = True)
subprocess.run("v4l2-ctl -c brightness=85", shell =True)
subprocess.run("v4l2-ctl -c contrast=100", shell=True)


'''
sleep(2)
print("Ending calibration")
'''
def findId(idNum):
    foundMarker = False
    print("Starting calibration")
    cap = cv.VideoCapture(0)
    cap.set(3, w) # set horizontal resolution
    cap.set(4, h) # set vertical resolution
    cap.set(5, 10)
    #cap.set(cv.CAP_PROP_EXPOSURE, 1)
    #sleep(2)
    print("Ending calibration")
    
    
    print("Starting detection...")
    # Start video capture stream and initilize aruco dictionary
    dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
    
    parameters = cv.aruco.DetectorParameters_create()
    angle = 0.0
    while(foundMarker == False):
        ret, frame = cap.read()
        #camera.capture(frame,'bgr')
        #frame = frame.reshape((h, w , 3))
        
        gray = frame

        corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(gray, dictionary, parameters = parameters)
        cv.aruco.detectMarkers(gray, dictionary, corners, ids)
        cv.line(gray, (int(w/2),0), (int(w/2),h), (0,255,0),3)

        if ids is not None:
            cv.aruco.drawDetectedMarkers(frame, corners, ids)
            
            
            for i in range(0, len(ids)) :
                if (ids[i][0] == idNum) :
                    bus.write_byte_data(address, 0, 1)
                    foundMarker = True
        '''
        cv.imshow('Detect Image', frame)
        
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        '''
    
    # end while
    
    
    
    print("Ending detection...")

def angleFinder(idNum):
    
    cap = cv.VideoCapture(0)
    cap.set(3, w) # set horizontal resolution
    cap.set(4, h) # set vertical resolution
    cap.set(5, 10)
    #cap.set(cv.CAP_PROP_EXPOSURE, 1)
    #sleep(2)
    
    print("Starting angle...")
    # Start video capture stream and initilize aruco dictionary
    dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
    
    parameters = cv.aruco.DetectorParameters_create()
    angle = 0.0
    foundMarker = False
    
    #while(True):
    while(foundMarker == False):
        ret, frame = cap.read()
        
        gray = frame

        corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(gray, dictionary, parameters = parameters)
        cv.aruco.detectMarkers(gray, dictionary, corners, ids)
        cv.line(gray, (int(w/2),0), (int(w/2),h), (0,255,0),3)

        if ids is not None:
            
            for i in range(0,len(ids)) :
                if (ids[i][0] == idNum):
                    
                    foundMarker = True
                    
            
                    first = corners[i][0][0]
                    third = corners[i][0][2]
                        
                    middle_x = 0.5*(third[0] + first[0]) 
                    middle_y = 0.5*(third[1] + first[1])
                    
                    
                    center = 0.5 * w
                
                    angle = (27 * ( (center - middle_x) / center)) +7
                    print("Angle: %s" % angle)
                    fudge_angle = (angle + 54)
                    #cv.circle(gray,(int(middle_x), int(middle_y)), 10,(0,0,255),-1)
            
                    #cv.line(gray, (int(w/2), int(middle_y)), (int(middle_x),int(middle_y)),(0,0,255),3)
                    
                    send_angle = int(fudge_angle)
                    bus.write_byte_data(address, 1, send_angle)
        '''
        cv.imshow("Gray", gray)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        '''
        
    # end while

    

    
    print("Ending angle...")
    
'''
def angleFinder():
    
    #print("Starting calibration")
    cap = cv.VideoCapture(0)
    cap.set(3, w) # set horizontal resolution
    cap.set(4, h) # set vertical resolution
    cap.set(5, 10)
    cap.set(cv.CAP_PROP_EXPOSURE, 1)
    sleep(2)
    #lcd.clear()
    #print("Ending calibration")
    
    print("Starting angle...")
    # Start video capture stream and initilize aruco dictionary
    dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
    
    parameters = cv.aruco.DetectorParameters_create()
    angle = 0.0
    
    
    while(True):
        ret, frame = cap.read()
        
        gray = frame

        corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(gray, dictionary, parameters = parameters)
        cv.aruco.detectMarkers(gray, dictionary, corners, ids)
        cv.line(gray, (int(w/2),0), (int(w/2),h), (0,255,0),3)

        if ids is not None:
            
            first = corners[0][0][0]
            third = corners[0][0][2]
                
            middle_x = 0.5*(third[0] + first[0]) 
            middle_y = 0.5*(third[1] + first[1])
            
            
            center = 0.5 * w
        
            angle = (27 * ( (center - middle_x) / center)) + 4
            print("Angle * 100: %s" % angle)
            fudge_angle = (angle + 54)
            
            send_angle = int(fudge_angle)
            bus.write_byte_data(address, 1, send_angle)
            #lcd.clear()
            #lcd.message = "Fixing Angle"
            break
        
        
        #cv.imshow('Detect Image', frame)
        #if cv.waitKey(1) & 0xFF == ord('q'):
         #   break
    
    # end while
    cap.release()
    #cv.destroyAllWindows()
'''
    
def distanceFinder(idNum):
    cap = cv.VideoCapture(0)
    cap.set(3, w) # set horizontal resolution
    cap.set(4, h) # set vertical resolution
    cap.set(5, 24)
    #cap.set(cv.CAP_PROP_EXPOSURE, 1)
    #sleep(2)
    print("Starting distance...")
    # Start video capture stream and initilize aruco dictionary
    dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
    
    parameters = cv.aruco.DetectorParameters_create()
    
    distance = 0.0
    
    locatedMarker = False
    while(locatedMarker == False):
        
        ret, frame = cap.read()
        
        gray = frame

        corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(gray, dictionary, parameters = parameters)
        cv.aruco.detectMarkers(gray, dictionary, corners, ids)
        
        
        if ids is not None:
            for i in range (0,len(ids)):
                if(ids[i][0] == idNum):
                    locatedMarker = True
                    first = corners[i][0][0]
                    third = corners[i][0][2]
                    
                    v = third[1] - first[1] #camera coord image height in pixels
                    
                    #normal distance
                    distance = (2849.3 * (1/v) + 5.3856)-6.2 - 3.5
                    
                    #distance redesign
                    #distance = (3527.9 * (1/v) -30)
                    
                    #stop within 1 ft of marker
                    #distance = (2849.3 * (1/v) + 5.3856)-28
                    
                    send_dist = int(distance)
                    bus.write_byte_data(address, 2, send_dist)
                    
                    print(distance)
            
            
            
        '''
        cv.imshow('Detect Image', frame)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        '''
    
    # end while
    cap.release()
    cv.destroyAllWindows()
    
    print("Ending distance...")
    
    

for i in range(1,7):
    print("Finding marker %d" % i)
    findId(i)
    #sleep(2)
    #sleep(0.5)
    angleFinder(i)
    #sleep(1)
    #slee
    
    while True:
        done_msg = bus.read_byte(address)
        sleep(0.1)
        if done_msg == 8:
            print(done_msg)
            break
       
    distanceFinder(i)
    sleep(1)
    
    while True:
        done_msg = bus.read_byte(address)
        sleep(0.1)
        if done_msg == 8:
            print(done_msg)
            break
    sleep(4)
    bus.write_byte_data(address, 3, 1)
    

'''
while (True):
    angleFinder(1)
    angleFinder(1)
    angleFinder(1)
    print("done")
    sleep(50)
'''
    
bus.write_byte_data(address, 3, 0)

cv.destroyAllWindows()
cap.release()