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

# Initialize LCD display
#lcd_columns = 16
#lcd_rows = 2
#i2c = board.I2C()
#lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
#lcd.color = [100,0,0]

#camera = PiCamera()
'''
w = 1920
h = 1088
'''

w = 640
h = 420

'''
w = 1280
h = 840
'''

address = 0x04
bus = smbus.SMBus(1)

def continualCapture():
    loopNum = 1
    
    print("Starting calibration")
    cap = cv.VideoCapture(0)
    cap.set(3, w) # set horizontal resolution
    cap.set(4, h) # set vertical resolution
    cap.set(5, 10)
    cap.set(cv.CAP_PROP_EXPOSURE, 1)
    sleep(2)
    print("Ending calibration")
    
    print("Starting detection...")
    # Start video capture stream and initilize aruco dictionary
    dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
    
    parameters = cv.aruco.DetectorParameters_create()
    angle = 0.0
    while(True):
        ret, frame = cap.read()
        #camera.capture(frame,'bgr')
        #frame = frame.reshape((h, w , 3))
        
        gray = frame

        corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(gray, dictionary, parameters = parameters)
        cv.aruco.detectMarkers(gray, dictionary, corners, ids)
        cv.line(gray, (int(w/2),0), (int(w/2),h), (0,255,0),3)

        if ids is not None:
            
            bus.write_byte_data(address, 0, 1)
            #lcd.clear()
            #lcd.message = "Marker detected"
            break;
        
        #cv.imshow('Detect Image', frame)
        #if cv.waitKey(1) & 0xFF == ord('q'):
        #    break
    
    # end while
    cap.release()
    #cv.destroyAllWindows()
    
    print("Ending detection...")

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
    
    print("Ending angle...")
    

def distanceFinder():
    
    #print("Starting calibration")
    cap = cv.VideoCapture(0)
    cap.set(3, w) # set horizontal resolution
    cap.set(4, h) # set vertical resolution
    cap.set(5, 10)
    cap.set(cv.CAP_PROP_EXPOSURE, 1)
    sleep(2)
    #lcd.clear()
    #print("Ending calibration")
    
    print("Starting distance...")
    # Start video capture stream and initilize aruco dictionary
    dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)
    
    parameters = cv.aruco.DetectorParameters_create()
    angle = 0.0
    distance = 0.0
    f = 3.60 #focal length of camera in mm
    Y = 90 # world cood aruco marker height in mm
    mY = h / 2.740 # pixel density in y direction (px / mm) not used
    k = 172.5415 # similar triangles fudge factor
    
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
            
            v = third[1] - first[1] #camera coord image height in pixels
            
            #normal distance
            #distance = (2849.3 * (1/v) + 5.3856)-6.20
            
            #stop within 1 ft of marker
            distance = (2849.3 * (1/v) + 5.3856)-28
            
            send_dist = int(distance)
            bus.write_byte_data(address, 2, send_dist)
            #lcd.clear()
            #lcd.message = "Going Dist"
            
            #lcd.clear()
            #lcd.message = "D:" + str(distance)
            #print("v:\t%3.5f" % v)
            print(distance)
            break
            
            '''
            center = 0.5 * w
        
            angle = 27 * ( (center - middle_x) / center)
            print("Angle: %s" % angle)
            
            middle_x = int(middle_x)
            middle_y = int(middle_y)
            
            cv.circle(gray,(middle_x, middle_y), 10,(0,0,255),-1)
            
            cv.line(gray, (int(w/2), middle_y), (middle_x,middle_y),(0,0,255),3)
            '''
        
        #cv.imshow('Detect Image', frame)
        #if cv.waitKey(1) & 0xFF == ord('q'):
        #    break
    
    # end while
    cap.release()
    #cv.destroyAllWindows()
    
    print("Ending distance...")
    
        
continualCapture()
angleFinder()
distanceFinder()
