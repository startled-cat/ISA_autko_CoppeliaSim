# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19990)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!
import math
import numpy as np

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')


import time

def rotate(wheels, turning_speed, turning_time):
    stop_velocity = 0


    sim.simxSetJointTargetVelocity(clientID, wheels[0], turning_speed, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, wheels[2], turning_speed, sim.simx_opmode_oneshot)

    sim.simxSetJointTargetVelocity(clientID, wheels[1], -turning_speed, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, wheels[3], -turning_speed, sim.simx_opmode_oneshot)

    time.sleep(turning_time)

    sim.simxSetJointTargetVelocity(clientID, wheels[0], stop_velocity, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, wheels[2], stop_velocity, sim.simx_opmode_oneshot)

    sim.simxSetJointTargetVelocity(clientID, wheels[1], stop_velocity, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, wheels[3], stop_velocity, sim.simx_opmode_oneshot)

def set_wheels_force(wheels, left, right):
    sim.simxSetJointForce(clientID, wheels[0], left, sim.simx_opmode_oneshot)
    sim.simxSetJointForce(clientID, wheels[2], left, sim.simx_opmode_oneshot)

    sim.simxSetJointForce(clientID, wheels[1], right, sim.simx_opmode_oneshot)
    sim.simxSetJointForce(clientID, wheels[3], right, sim.simx_opmode_oneshot)

def set_wheels_velocity(wheels, left, right):
    sim.simxSetJointTargetVelocity(clientID, wheels[0], left, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, wheels[2], left, sim.simx_opmode_oneshot)

    sim.simxSetJointTargetVelocity(clientID, wheels[1], right, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, wheels[3], right, sim.simx_opmode_oneshot)

def getCarHorizontalAngle():
    return_value, eulerAngles = sim.simxGetObjectOrientation(clientID, carBody, -1, sim.simx_opmode_oneshot)
    angle = eulerAngles[2] * 180 / math.pi
    detect_object_quick()
    return angle

def rotateCarByDeg(degrees, speed):

    if degrees == 0:
        return

    if degrees < 0:
        print("rotate left")
        speed = -speed
        degrees = abs(degrees)
    else:
        print("rotate right")

    starting_angle = getCarHorizontalAngle()
    #start rotating
    set_wheels_velocity(wheels, speed, -speed)
    #print("starting to rotate ...")
    while(True):
        #print("rotated: " + str(starting_angle - getCarHorizontalAngle()))
        if (abs(starting_angle - getCarHorizontalAngle())) >= degrees :
            set_wheels_velocity(wheels, 0, 0)
            break

        if (abs(starting_angle - getCarHorizontalAngle())) >= (degrees-2) :
            set_wheels_velocity(wheels, speed/8, -speed/8)
            continue

        if (abs(starting_angle - getCarHorizontalAngle())) >= (degrees-5) :
            set_wheels_velocity(wheels, speed/4, -speed/4)
            continue
    #correct 1
    if (abs(starting_angle - getCarHorizontalAngle())) > degrees :
        #print("correcting ...")
        set_wheels_velocity(wheels, -speed/16, speed/16)
        while(True):
            #print("rotated: " + str(starting_angle - getCarHorizontalAngle()))
            if (abs(starting_angle - getCarHorizontalAngle())) <= degrees :
                set_wheels_velocity(wheels, 0, 0)
                break
    #correct 2
    if (abs(starting_angle - getCarHorizontalAngle())) < degrees :
        #print("correcting ...")
        set_wheels_velocity(wheels, speed/32, -speed/32)
        while(True):
            #print("rotated: " + str(starting_angle - getCarHorizontalAngle()))
            if (abs(starting_angle - getCarHorizontalAngle())) >= degrees :
                set_wheels_velocity(wheels, 0, 0)
                break

        
        #time.sleep(0.050)
    print("rotated car from {0}, to {1} deg, error={2}".format(starting_angle, getCarHorizontalAngle(), abs(starting_angle - getCarHorizontalAngle())-degrees ))

def rotateCarToDeg(degree, speed, error):
    print("rotating from {0}, to {1} ...".format(getCarHorizontalAngle(), degree))

    starting_angle = getCarHorizontalAngle()

    diff = degree - starting_angle
    #if diff > 180:
    #    diff -= 180
    rotateLeft = True
    # + left
    # - right
    if diff > 0:
        speed = -speed
        #degrees = abs(degrees)
    else:
        rotateLeft = False

    if ((degree < 0 and starting_angle > 0) or (degree > 0 and starting_angle < 0)) and (abs(starting_angle) >= 90 and abs(degree) >= 90):
        speed = -speed
        rotateLeft = not rotateLeft
    
    if rotateLeft:
        print("rotate left")
    else:
        print("rotate right")

    
    #start rotating
    set_wheels_velocity(wheels, speed, -speed)
    #print("starting to rotate ...")
    
    while(True):
        detect_object(leftSensorFront, 1)
        
        difference = degree - getCarHorizontalAngle()
        if difference > 180:
            difference -= 360
        #if difference < 10:
        #print("difference = {0}".format(difference))
        if rotateLeft:
            if difference <= error :
                set_wheels_velocity(wheels, 0, 0)
                break

            if difference <= 1:
                set_wheels_velocity(wheels, speed/16, -speed/16)
                continue

            if difference <= 5:
                set_wheels_velocity(wheels, speed/8, -speed/8)
                continue

            if difference <= 10 :
                set_wheels_velocity(wheels, speed/4, -speed/4)
                continue
        else:
            if difference >= -error :
                set_wheels_velocity(wheels, 0, 0)
                break

            if difference >= -1:
                set_wheels_velocity(wheels, speed/16, -speed/16)
                continue

            if difference >= -5:
                set_wheels_velocity(wheels, speed/8, -speed/8)
                continue

            if difference >= -10 :
                set_wheels_velocity(wheels, speed/4, -speed/4)
                continue
    
    print("rotated car from {0}, to {1}, target={2}".format(starting_angle, getCarHorizontalAngle(), degree ))

def getCarPosition():
    return_value, pos = sim.simxGetObjectPosition(clientID, carBody, -1, sim.simx_opmode_oneshot)
    detect_object_quick()
    return pos

def goForward(speed, distance, error):
    startingPosition = getCarPosition()

    set_wheels_velocity(wheels, speed, speed)
    #print("starting to move with speed {0}".format(speed))
    while(True):
        newPosition = getCarPosition()
        diff = [startingPosition[0] - newPosition[0], startingPosition[1] - newPosition[1]]
        d = math.sqrt(diff[0]*diff[0] + diff[1]*diff[1])
        #print("startingPosition= {0}, new = {1}, diff = {2}, d = {3}".format(startingPosition, newPosition, diff, d))
        if distance - d < error :
            set_wheels_velocity(wheels, 0, 0)
            #print("stopped, error = {0}".format(d - distance))
            break
        if distance - d < 0.01 :
            set_wheels_velocity(wheels, speed/4, speed/4)
            continue
        if distance - d < 0.1 :
            set_wheels_velocity(wheels, speed/2, speed/2)
            continue
        


def wheels_pos(wheels):
    return sim.simxGetJointPosition(clientID, wheels[0], sim.simx_opmode_oneshot)[1],sim.simxGetJointPosition(clientID, wheels[1], sim.simx_opmode_oneshot)[1],sim.simxGetJointPosition(clientID, wheels[2], sim.simx_opmode_oneshot)[1],sim.simxGetJointPosition(clientID, wheels[3], sim.simx_opmode_oneshot)[1]

def detect_object(sensorHolder, miliseconds):
    x = 0
    while(x<miliseconds):
        return_value, detectionState, detectionPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, leftSensorFront, sim.simx_opmode_buffer)
        
        if(detectionState==True and x%100==0):
            print(detectionPoint)
            print(detectionState)
            #sim.simxAddStatusbarMessage(clientID,'Detected something',sim.simx_opmode_oneshot)
        time.sleep(0.001)
        x+=1

def detect_object_quick():
    return_value, detectionState, detectionPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, leftSensorFront, sim.simx_opmode_buffer)
    if(detectionState==True):
        print(detectionPoint)
        print(detectionState)
        sim.simxAddStatusbarMessage(clientID,'Detected something',sim.simx_opmode_oneshot)

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19990,True,True,5000,5) # Connect to CoppeliaSim

while(clientID == -1):
    print("waiting for cimulation to start ...")
    time.sleep(1)
    clientID=sim.simxStart('127.0.0.1',19990,True,True,5000,5) # Connect to CoppeliaSim

if clientID!=-1:
    print ('Connected to remote API server')

    # Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'Hello from python!',sim.simx_opmode_oneshot)


    target_velocity = 1
    stop_velocity = 0

    turning_speed = 0.5
    turning_time = 1.80

    rotation_force = 100

    return_value, leftWheelFront = sim.simxGetObjectHandle(clientID, 'leftMotor', sim.simx_opmode_blocking)
    return_value, leftWheelBack = sim.simxGetObjectHandle(clientID, 'leftMotor_back', sim.simx_opmode_blocking)
    return_value, rightWheelFront = sim.simxGetObjectHandle(clientID, 'rightMotor', sim.simx_opmode_blocking)
    return_value, rightWheelBack = sim.simxGetObjectHandle(clientID, 'rightMotor_back', sim.simx_opmode_blocking)
    
    return_value, carBody = sim.simxGetObjectHandle(clientID, 'RobotBase', sim.simx_opmode_blocking)

    wheels = [leftWheelFront, rightWheelFront, leftWheelBack, rightWheelBack]

    return_value, leftSensorFront = sim.simxGetObjectHandle(clientID, 'Proximity_sensor_front_left', sim.simx_opmode_blocking)
    return_value, leftSensorBack = sim.simxGetObjectHandle(clientID, 'Proximity_sensor_back_left', sim.simx_opmode_blocking)
    return_value, rightSensorFront = sim.simxGetObjectHandle(clientID, 'Proximity_sensor_front_right', sim.simx_opmode_blocking)
    return_value, rightSensorBack = sim.simxGetObjectHandle(clientID, 'Proximity_sensor_back_right', sim.simx_opmode_blocking)

    return_value, detectionState, detectionPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, leftSensorFront, sim.simx_opmode_streaming)
    
    # while(True):
    #     rotateCarByDeg(90, turning_speed)
    #     time.sleep(0.1)
    #     set_wheels_velocity(wheels, 0.5, 0.5)
    #     detect_object(leftSensorFront, 2000)
    #     set_wheels_velocity(wheels, 0, 0)
    #     time.sleep(0.1)

    while(True):
        rotateCarToDeg(0, turning_speed, 0.1)
        time.sleep(1)
        goForward(5, 1, 0.001)
        time.sleep(1)

        rotateCarToDeg(-90, turning_speed, 0.1)
        time.sleep(1)
        goForward(5, 1, 0.001)
        time.sleep(1)

        rotateCarToDeg(-180, turning_speed, 0.1)
        time.sleep(1)
        goForward(5, 1, 0.001)
        time.sleep(1)

        rotateCarToDeg(90, turning_speed, 0.1)
        time.sleep(1)
        goForward(5, 1, 0.001)
        time.sleep(1)


        
        

    

    """
    rotate(wheels, turning_speed, turning_time)
    i = 0
    while(i < 1000):
        print(wheels_pos(wheels))
        i += 1

    #time.sleep(1)
    rotate(wheels, -turning_speed, turning_time)

    while(True):
        print(wheels_pos(wheels))
    """
    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')



