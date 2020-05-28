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


def detect_object(sensorHolder):
    return_value, detectionState, detectionPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, leftSensorFront, sim.simx_opmode_buffer)
    print(detectionPoint)
    print(detectionState)

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19990,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')

    # Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'Hello from python!',sim.simx_opmode_oneshot)


    
    return_value, leftSensorFront = sim.simxGetObjectHandle(clientID, 'Proximity_sensor_front_left', sim.simx_opmode_blocking)
    return_value, leftSensorBack = sim.simxGetObjectHandle(clientID, 'Proximity_sensor_back_left', sim.simx_opmode_blocking)
    return_value, rightSensorFront = sim.simxGetObjectHandle(clientID, 'Proximity_sensor_front_right', sim.simx_opmode_blocking)
    return_value, rightSensorBack = sim.simxGetObjectHandle(clientID, 'Proximity_sensor_back_right', sim.simx_opmode_blocking)

    return_value, detectionState, detectionPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(clientID, leftSensorFront, sim.simx_opmode_streaming)



    while(1):
        detect_object(leftSensorFront)
        time.sleep(1)


    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')

