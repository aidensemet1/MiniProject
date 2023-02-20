import numpy as np
import cv2 as cv
from picamera import PiCamera
from time import sleep
import serial
import time
import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

ser = serial.Serial("/dev/ttyAMC0", 9600)
aruco_setpoints = {0: 0, 1: 1.57, 2: 3.14, 3: 4.71}

lcd_columns = 16
lcd_rows = 2
i2c = board.I2C()
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

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
        

def continualCapture():
    cap = cv.VideoCapture(0)
    dictionary = cv.aruco.Dictionary_get(cv.aruco.DICT_6X6_250)

    while(True):
        
        ret, frame = cap.read()
        #print(frame.shape)
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        
        h = gray.shape[0]
        w = gray.shape[1]
        
        parameters = cv.aruco.DetectorParameters_create()
            
        corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(gray, dictionary, parameters = parameters)
        cv.aruco.detectMarkers(gray, dictionary, corners, ids)
        if ids is not None and ids[0] in aruco_setpoints:
            
            cv.aruco.drawDetectedMarkers(gray, corners, ids)
            
            for x in ids:
                
                first = corners[0][0][0]
                third = corners[0][0][2]
                
                middle_x = int( 0.5*(third[0] + first[0]) )
                middle_y = int( 0.5*(third[1] + first[1]) )
                cv.circle(gray,(middle_x, middle_y), 10,(0,0,255),-1)
                
                dir_x = middle_x - w/2
                dir_y = h/2 - middle_y
                
                #print([dir_x, dir_y])
                
                if ((dir_x >= 0) and (dir_y >= 0)):
                    print("First")
                    setpoint = aruco_setpoints[0]
                    ser.write(str(setpoint).encode())
                    lcd.clear()
                    lcd.color = [0,0,100]
                    time.sleep(1)
                    lcd.message = "Setpoint: 0"
                elif ((dir_x < 0) and (dir_y >=0)):
                    print("Second")
                    setpoint = aruco_setpoints[1]
                    ser.write(str(setpoint).encode())
                    lcd.clear()
                    lcd.color = [0,0,100]
                    time.sleep(1)
                    lcd.message = "Setpoint: pi/2"
                elif ((dir_x < 0) and (dir_y < 0)):
                    print ('Third')
                    setpoint = aruco_setpoints[2]
                    ser.write(str(setpoint).encode())
                    lcd.clear()
                    lcd.color = [0,0,100]
                    time.sleep(1)
                    lcd.message = "Setpoint: pi"
                elif ((dir_x >=0) and (dir_y <0)):
                    print("Fourth")
                    setpoint = aruco_setpoints[3]
                    ser.write(str(setpoint).encode())
                    lcd.clear()
                    lcd.color = [0,0,100]
                    time.sleep(1)
                    lcd.message = "Setpoint: 3pi/2"
                    
                    
        cv.line(gray, (0, int(h/2)),(w,int(h/2)), (0,255,0),3)
        cv.line(gray, (int(w/2),0), (int(w/2),h), (0,255,0),3)    
        cv.imshow('output', gray)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break
        
        recieved_position = ser.readline().decode().strip()
        position = float(received_postion)*(2*3.14)/1024
        print("Current position =", position)
        lcd.message = "\nPosition: " + str(position)

    #end While
    cap.release()
    cv.destroyAllWindows()
    print("Exiting function")



        
        
        
        
        
        

            


