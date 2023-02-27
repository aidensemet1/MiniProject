# Below are all the libraries we used for this project
import numpy as np
import cv2 as cv
from picamera import PiCamera
from time import sleep
import serial
import smbus
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd    # library for LCD screen

arduino = serial.Serial("/dev/ttyACM0", 250000, timeout=2)  # sets up Serial communication between Pi and Ardunio
bus = smbus.SMBus(1)    

# initialize LCD
lcd_columns = 16
lcd_rows = 2
i2c = board.I2C()
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

# function that calibrates the camera settings
# funciton sets consitant white balance, iso
# shutter speed, exposure, and framerate
def cameraCalibration():
    with PiCamera() as camera:
        camera.framerate = 24
        sleep(2)
        camera.iso = 100
        sleep(2)
        camera.shutter_speed = camera.exposure_speed
        camera.exposure_mode = 'off'
        g = camera.awb_gains
        camera.awb_mode ='off'
        camera.awb_gains = g
        
# function that carries out detection and quadrant identification of markers
def continualCapture():
    # initialize video capture and aruco dictionary
    cap = cv.VideoCapture(0)
    dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)

    while(True):
        old_setpoint = ""
        ret, frame = cap.read() # get current frame and convert to grayscale
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
        h = gray.shape[0]
        w = gray.shape[1]
        
        parameters = cv.aruco.DetectorParameters_create()
            
        corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(gray, dictionary, parameters = parameters)
        cv.aruco.detectMarkers(gray, dictionary, corners, ids)
        if ids is not None:
             
             # following code identifies the middle of the detected marker
            first = corners[0][0][0]
            third = corners[0][0][2]
                
            middle_x = int( 0.5*(third[0] + first[0]) )
            middle_y = int( 0.5*(third[1] + first[1]) )
            
            # following code creates direction variables indicating
            # the top or bottom half of the frame
            # the left or right of the frame
            dir_x = middle_x - w/2
            dir_y = h/2 - middle_y
                
            if ((dir_x >= 0) and (dir_y >= 0)):             #if marker is in the 1st quadrant
                setpoint = "0"                              #setpoint = 0
                arduino.write(setpoint.encode("utf-8"))     #send 0 to Arduino
            elif ((dir_x < 0) and (dir_y >= 0)):            #if marker is in the 2nd quadrant
                setpoint = "pi/2"                           #setpoint = pi/2
                arduino.write(setpoint.encode("utf-8"))     #send pi/2 to Arduino
            elif ((dir_x < 0) and (dir_y < 0)):             #if marker is in the 3rd quadrant
                setpoint = "pi"                             #setpoint = pi
                arduino.write(str(setpoint).encode("utf-8"))    #send pi to Arduino
            elif ((dir_x >=0) and (dir_y <0)):              #if marker is in the 4th quadrant
                setpoint = "3pi/2"                          #setpoint = 3pi/2
                arduino.write(setpoint.encode("utf-8"))     #send 3pi/2 to Arduino
                    
            received_position = arduino.readline().decode("utf-8").strip()      # receive current wheel position from Ardunio
            new_position = ""
            
            #checks for a new setpoint or position
            #if the Pi receives a new setpoint or position, the LCD will update
            #if the setpoint or position is the same as the previous, the LCD will not update
            if (new_position != received_position or old_setpoint != setpoint):
                lcd.clear()                 #resets the LCD screen to blank
                lcd.color = [0,0,100]       #sets screen color to blue
                old_setpoint = setpoint
                lcd.message = "S: " + str(setpoint)
                new_position = recieved_position
                lcd.message = "\nP: " + str(new_position)
                
        #display current image and quit dection program if "q" is pressed"
        cv.imshow('output', gray)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        
    #end While and close all windows / release camera capture
    cap.release()
    cv.destroyAllWindows()
    print("Exiting function")
    
cameraCalibration()     #calls this function to calibrate the camera
continualCapture()      #calls this function to allow the camera to capture images
