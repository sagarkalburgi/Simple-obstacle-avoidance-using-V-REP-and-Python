# -*- coding: utf-8 -*-
"""
Created on Fri Mar 23 18:13:07 2018

@author: Sagar Documents
"""

import time
import numpy as np
import matplotlib.pyplot as mlp
import pandas as pd

try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID!=-1:
    print ('Connected to remote API server')
    
else:
    print ('Failed connecting to remote API server')
    print ('Program ended')

def cam():
    # Editing the Image data
    global cam1
    global df
    global cam2
    
    errorCode,resolution,image = vrep.simxGetVisionSensorImage(clientID,cam_handle,0,vrep.simx_opmode_buffer)
    im = np.array(image, dtype = np.uint8)
    #im.shape
    im.resize([resolution[0],resolution[1],3])
    #im.shape
    cam1 = mlp.imshow(im,origin='lower')
    cam2 = np.append(cam2, cam1)
    df = pd.DataFrame(cam2)
    
    
# Initializing The Poineer Robot Motors
errorCode,left_motor_handle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking)
errorCode,right_motor_handle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking)

# Initializing The Cam- Vision Sensor and Taking a snap
errorCode,cam_handle = vrep.simxGetObjectHandle(clientID,'cam',vrep.simx_opmode_blocking)   
errorCode,resolution,image = vrep.simxGetVisionSensorImage(clientID,cam_handle,0,vrep.simx_opmode_streaming) 
   
# Initializing the Front Sensor
errorCode,sensor_left = vrep.simxGetObjectHandle(clientID,'Proximity_sensor',vrep.simx_opmode_blocking)
errorCode,sensor_right = vrep.simxGetObjectHandle(clientID,'Proximity_sensor0',vrep.simx_opmode_blocking)
    
# Setting the Sensor to Streaming State
errorCode,detectionStateLeft,detectedPointLeft,detectedObjectHandleLeft,detectedSurfaceNormalVectorLeft = vrep.simxReadProximitySensor(clientID,sensor_left,vrep.simx_opmode_streaming)
errorCode,detectionStateRight,detectedPointRight,detectedObjectHandleRight,detectedSurfaceNormalVectorRight = vrep.simxReadProximitySensor(clientID,sensor_right,vrep.simx_opmode_streaming)
 
# Saving the Detected Point distance in an array- Initializing this array
detectedpointLeft = np.array([])
detectedpointRight = np.array([])

    
# Setting the Run time Initializer
t = time.time()
global cam1
global df
global cam2
cam2 = ([])
while (time.time()-t)<150:            # Robot will Move for 60 seconds in this loop
        
   # Forward Motion
   errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,1,vrep.simx_opmode_streaming) 
   errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,1,vrep.simx_opmode_streaming) 
   
   # Setting the Sensors to buffer state
   errorCode,detectionStateLeft,detectedPointLeft,detectedObjectHandleLeft,detectedSurfaceNormalVectorLeft = vrep.simxReadProximitySensor(clientID,sensor_left,vrep.simx_opmode_buffer)
   errorCode,detectionStateRight,detectedPointRight,detectedObjectHandleRight,detectedSurfaceNormalVectorRight = vrep.simxReadProximitySensor(clientID,sensor_right,vrep.simx_opmode_buffer)
        
   # Rounding off the data recieved by the sensor upto 3 decimal points
   DPL = round(np.linalg.norm(detectedPointLeft),3)
   DPR = round(np.linalg.norm(detectedPointRight),3)
        
   #if DPL != 0.0 and DPL <= 1 and DPR != 0.0 and DPR <= 1.0:
            
            
   # Add the detected sensor data to the array
   #detectedpoint = np.append(detectedpoint,DP)
           
   if DPL <= 0.5 and DPL <= 1 and DPL != 0.0:
       errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,1,vrep.simx_opmode_streaming) 
       errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0,vrep.simx_opmode_streaming)
       time.sleep(0.001)
       print('Left', DPL)
       detectedpointLeft = np.append(detectedpointLeft,DPL)
       cam()
   elif DPR <= 0.5 and DPR <= 1 and DPR != 0.0:
       errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0,vrep.simx_opmode_streaming) 
       errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,1,vrep.simx_opmode_streaming)
       time.sleep(0.001)
       print('Right', DPR)
       detectedpointRight = np.append(detectedpointRight,DPR)
       cam()
   elif DPL <= 0.5 and DPL <= 1 and DPL != 0.0 and DPR <= 0.5 and DPR <= 1 and DPR != 0.0:
       errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,-1,vrep.simx_opmode_streaming) 
       errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,1,vrep.simx_opmode_streaming)
       time.sleep(0.001)
       print('Right', DPR)
       detectedpointRight = np.append(detectedpointRight,DPR)
   else:
       errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,1,vrep.simx_opmode_streaming) 
       errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,1,vrep.simx_opmode_streaming)
       time.sleep(0.1)
        
    
# Stop Motion
errorCode=vrep.simxSetJointTargetVelocity(clientID,left_motor_handle,0,vrep.simx_opmode_streaming) 
errorCode=vrep.simxSetJointTargetVelocity(clientID,right_motor_handle,0,vrep.simx_opmode_streaming) 
    