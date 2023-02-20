import numpy as np
import cv2 as cv
from picamera import PiCamera
from time import sleep

#hello

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
        if (ids is not None):
                
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
                elif ((dir_x < 0) and (dir_y >=0)):
                    print("Second")
                    
                elif ((dir_x < 0) and (dir_y < 0)):
                    print ('Third')
                    
                elif ((dir_x >=0) and (dir_y <0)):
                    print("Fourth")
                
                            
        cv.line(gray, (0, int(h/2)),(w,int(h/2)), (0,255,0),3)
        cv.line(gray, (int(w/2),0), (int(w/2),h), (0,255,0),3)    
        cv.imshow('output', gray)
        if cv.waitKey(1) & 0xFF == ord('q'):
            break


    #end While
    cap.release()
    cv.destroyAllWindows()
    print("Exiting function")



        
        
        
        
        
        

            


