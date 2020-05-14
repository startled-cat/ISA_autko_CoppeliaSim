import math
import numpy as np
import time
from map import Map


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


class Car:

    

    def __init__(self):
        print("dsdsdsd")
        self.target_velocity = 1
        self.stop_velocity = 0

        self.turning_speed = 0.5
        self.turning_time = 1.80

        self.rotation_force = 100
        self.map = Map(13, 13, [12, 8])
        self.clientID = 0

        self.sensors = []
        self.wheels = []


    def mapping_run(self):
        direction = [-1, 0]
        frontSensor = self.sensors[0]
        leftSensor = self.sensors[5]
        rightSensor = self.sensors[4]
        while True:
            
            direction = self.goForwardTillSensorDetectMap(5, frontSensor, leftSensor, rightSensor, 0.8, direction)
            print("rotate till sensor front clear")
            leftSensorDistance = self.sensorDistance(leftSensor)
            rightSensorDistance = self.sensorDistance(rightSensor)
            print("frontSensor = " + str(self.sensorDistance(frontSensor)))
            print("rightSensor = " + str(rightSensorDistance))
            print("leftSensor  = " + str(leftSensorDistance))
            #rotate left if right sensor detects
            if self.sensorDistance(rightSensor) < 0.001:
                while self.sensorDistance(frontSensor) < 0.5 and self.sensorDistance(frontSensor) > 0.001:
                    self.rotateCarByDeg(90, 2)
                    direction = self.set_direction(direction, 1)
            elif self.sensorDistance(leftSensor) < 0.001:
                while self.sensorDistance(frontSensor) < 0.5 and self.sensorDistance(frontSensor) > 0.001:
                    self.rotateCarByDeg(-90, 2)
                    direction = self.set_direction(direction, -1)
            else:
                while self.sensorDistance(frontSensor) < 0.5 and self.sensorDistance(frontSensor) > 0.001:
                    rotationError = 9999
                    while rotationError > 181:
                        rotationError = self.rotateCarByDegMap(90, 2)
                    direction = self.set_direction(direction, 1)

            #rotate right if left sensor detects
    


    def run(self):
        #go forward till front sensor close
        frontSensor = self.sensors[0]

        leftSensor = self.sensors[5]
        rightSensor = self.sensors[4]
        #frontSensor = sensors[2]
        while True:
            
            self.goForwardTillSensorDetect(5, frontSensor, 0.2)
            print("rotate till sensor front clear")
            print("frontSensor = " + str(self.sensorDistance(frontSensor)))
            print("rightSensor = " + str(self.sensorDistance(rightSensor)))
            print("leftSensor  = " + str(self.sensorDistance(leftSensor)))

            #rotate left if right sensor detects
            if self.sensorDistance(rightSensor) < 1 :
                while self.sensorDistance(frontSensor) < 0.5 and self.sensorDistance(frontSensor) > 0.001:
                    self.rotateCarByDeg(-10, 2)
            else: #$self.sensorDistance(leftSensor) < 1 :
                while self.sensorDistance(frontSensor) < 0.5 and self.sensorDistance(frontSensor) > 0.001:
                    self.rotateCarByDeg(10, 2)
            
            #rotate right if left sensor detects
            
                
            

    def square(self):
        while(True):
            self.rotateCarToDeg(0, self.turning_speed, 0.1)
            time.sleep(1)
            self.goForward(5, 1, 0.001)
            time.sleep(1)

            self.rotateCarToDeg(-90, self.turning_speed, 0.1)
            time.sleep(1)
            self.goForward(5, 1, 0.001)
            time.sleep(1)

            self.rotateCarToDeg(-180, self.turning_speed, 0.1)
            time.sleep(1)
            self.goForward(5, 1, 0.001)
            time.sleep(1)

            self.rotateCarToDeg(90, self.turning_speed, 0.1)
            time.sleep(1)
            self.goForward(5, 1, 0.001)
            time.sleep(1)

    # direction: previous direction, rotation: 1 - right, -1 - left, 0 - no rotation
    def set_direction(self, direction, rotation):
        if direction[0] == -1 and direction[1] == 0:
            if rotation == 1:
                direction = [0, 1]
            elif rotation == -1:
                direction = [0, -1]
        elif direction[0] == 1 and direction[1] == 0:
            if rotation == 1:
                direction = [0, -1]
            elif rotation == -1:
                direction = [0, 1]
        elif direction[0] == 0 and direction[1] == -1:
            if rotation == 1:
                direction = [-1, 0]
            elif rotation == -1:
                direction = [1, 0]
        elif direction[0] == 0 and direction[1] == 1:
            if rotation == 1:
                direction = [1, 0]
            elif rotation == -1:
                direction = [-1, 0]
        return direction


    def sensorDistance(self, sensor):
        return_value, detectionState, detectionPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(self.clientID, sensor, sim.simx_opmode_blocking)
        
        distance = math.sqrt(detectionPoint[0]*detectionPoint[0] + detectionPoint[1]*detectionPoint[1] + detectionPoint[2]*detectionPoint[2])
        return distance
        
        
    def rotate(self):
        sim.simxSetJointTargetVelocity(self.clientID, self.wheels[0], self.turning_speed, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.clientID, self.wheels[2], turning_speed, sim.simx_opmode_oneshot)

        sim.simxSetJointTargetVelocity(self.clientID, self.wheels[1], -self.turning_speed, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.clientID, self.wheels[3], -self.turning_speed, sim.simx_opmode_oneshot)

        time.sleep(self.turning_time)

        sim.simxSetJointTargetVelocity(self.clientID, self.wheels[0], self.stop_velocity, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.clientID, self.wheels[2], self.stop_velocity, sim.simx_opmode_oneshot)

        sim.simxSetJointTargetVelocity(self.clientID, self.wheels[1], self.stop_velocity, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.clientID, self.wheels[3], self.stop_velocity, sim.simx_opmode_oneshot)

    def set_wheels_force(self, left, right):
        sim.simxSetJointForce(self.clientID, self.wheels[0], left, sim.simx_opmode_oneshot)
        sim.simxSetJointForce(self.clientID, self.wheels[2], left, sim.simx_opmode_oneshot)

        sim.simxSetJointForce(self.clientID, self.wheels[1], right, sim.simx_opmode_oneshot)
        sim.simxSetJointForce(self.clientID, self.wheels[3], right, sim.simx_opmode_oneshot)

    def set_wheels_velocity(self, left, right):
        sim.simxSetJointTargetVelocity(self.clientID, self.wheels[0], left, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.clientID, self.wheels[2], left, sim.simx_opmode_oneshot)

        sim.simxSetJointTargetVelocity(self.clientID, self.wheels[1], right, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(self.clientID, self.wheels[3], right, sim.simx_opmode_oneshot)

    def getCarHorizontalAngle(self):
        return_value, eulerAngles = sim.simxGetObjectOrientation(self.clientID, self.carBody, -1, sim.simx_opmode_oneshot)
        angle = eulerAngles[2] * 180 / math.pi
        self.detect_object_quick()
        return angle

    def rotateCarByDegMap(self, degrees, speed):
	
        if degrees == 0:
            return

        if degrees < 0:
            print("rotate left")
            speed = -speed
            degrees = abs(degrees)
        else:
            print("rotate right")

        starting_angle = self.getCarHorizontalAngle()
        print(starting_angle)
        #start rotating
        self.set_wheels_velocity(speed, -speed)
        #print("starting to rotate ...")
        while(True):
            #print("rotated: " + str(abs(starting_angle - self.getCarHorizontalAngle())))
            if (abs(starting_angle - self.getCarHorizontalAngle())) >= degrees :
                self.set_wheels_velocity(0, 0)
                break

            if (abs(starting_angle - self.getCarHorizontalAngle())) >= (degrees-2) :
                self.set_wheels_velocity(speed/8, -speed/8)
                continue

            if (abs(starting_angle - self.getCarHorizontalAngle())) >= (degrees-5) :
                self.set_wheels_velocity(speed/4, -speed/4)
                continue
        #correct 1
        if (abs(starting_angle - self.getCarHorizontalAngle())) > degrees :
            #print("correcting ...")
            self.set_wheels_velocity(-speed/16, speed/16)
            while(True):
                #print("rotated: " + str(starting_angle - self.getCarHorizontalAngle()))
                if (abs(starting_angle - self.getCarHorizontalAngle())) <= degrees :
                    self.set_wheels_velocity(0, 0)
                    break
        #correct 2
        if (abs(starting_angle - self.getCarHorizontalAngle())) < degrees :
            #print("correcting ...")
            self.set_wheels_velocity(speed/32, -speed/32)
            while(True):
                #print("rotated: " + str(starting_angle - getCarHorizontalAngle()))
                if (abs(starting_angle - self.getCarHorizontalAngle())) >= degrees :
                    self.set_wheels_velocity(0, 0)
                    break
        return abs(starting_angle - self.getCarHorizontalAngle())-degrees

    def rotateCarByDeg(self, degrees, speed):

        if degrees == 0:
            return

        if degrees < 0:
            print("rotate left")
            speed = -speed
            degrees = abs(degrees)
        else:
            print("rotate right")

        starting_angle = self.getCarHorizontalAngle()
        print(starting_angle)
        #start rotating
        self.set_wheels_velocity(speed, -speed)
        #print("starting to rotate ...")
        while(True):
            #print("rotated: " + str(abs(starting_angle - self.getCarHorizontalAngle())))
            if (abs(starting_angle - self.getCarHorizontalAngle())) >= degrees :
                self.set_wheels_velocity(0, 0)
                break

            if (abs(starting_angle - self.getCarHorizontalAngle())) >= (degrees-2) :
                self.set_wheels_velocity(speed/8, -speed/8)
                continue

            if (abs(starting_angle - self.getCarHorizontalAngle())) >= (degrees-5) :
                self.set_wheels_velocity(speed/4, -speed/4)
                continue
        #correct 1
        if (abs(starting_angle - self.getCarHorizontalAngle())) > degrees :
            #print("correcting ...")
            self.set_wheels_velocity(-speed/16, speed/16)
            while(True):
                #print("rotated: " + str(starting_angle - self.getCarHorizontalAngle()))
                if (abs(starting_angle - self.getCarHorizontalAngle())) <= degrees :
                    self.set_wheels_velocity(0, 0)
                    break
        #correct 2
        if (abs(starting_angle - self.getCarHorizontalAngle())) < degrees :
            #print("correcting ...")
            self.set_wheels_velocity(speed/32, -speed/32)
            while(True):
                #print("rotated: " + str(starting_angle - getCarHorizontalAngle()))
                if (abs(starting_angle - self.getCarHorizontalAngle())) >= degrees :
                    self.set_wheels_velocity(0, 0)
                    break

            
            #time.sleep(0.050)
        print("rotated car from {0}, to {1} deg, error={2}".format(starting_angle, self.getCarHorizontalAngle(), abs(starting_angle - self.getCarHorizontalAngle())-degrees ))

    def rotateCarToDeg(self, degree, speed, error):
        print("rotating from {0}, to {1} ...".format(self.getCarHorizontalAngle(), degree))

        starting_angle = self.getCarHorizontalAngle()

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
        self.set_wheels_velocity(speed, -speed)
        #print("starting to rotate ...")
        
        while(True):
            self.detect_object(self.sensors[0], 1)
            
            difference = degree - self.getCarHorizontalAngle()
            if difference > 180:
                difference -= 360
            #if difference < 10:
            #print("difference = {0}".format(difference))
            if rotateLeft:
                if difference <= error :
                    self.set_wheels_velocity(0, 0)
                    break

                if difference <= 1:
                    self.set_wheels_velocity(speed/16, -speed/16)
                    continue

                if difference <= 5:
                    self.set_wheels_velocity(speed/8, -speed/8)
                    continue

                if difference <= 10 :
                    self.set_wheels_velocity(speed/4, -speed/4)
                    continue
            else:
                if difference >= -error :
                    self.set_wheels_velocity(0, 0)
                    break

                if difference >= -1:
                    self.set_wheels_velocity(speed/16, -speed/16)
                    continue

                if difference >= -5:
                    self.set_wheels_velocity(speed/8, -speed/8)
                    continue

                if difference >= -10 :
                    self.set_wheels_velocity(speed/4, -speed/4)
                    continue
        
        print("rotated car from {0}, to {1}, target={2}".format(starting_angle, self.getCarHorizontalAngle(), degree ))

    def getCarPosition(self):
        return_value, pos = sim.simxGetObjectPosition(self.clientID, self.carBody, -1, sim.simx_opmode_oneshot)
        #self.detect_object_quick()
        return pos

    def goForwardTillSensorDetect(self, speed, sensor, walldistance):
        
        while(True):
            distance = self.sensorDistance(sensor)
            if distance < walldistance and distance > 0.001 :
                print("detected wall")
                break
            else:
                self.set_wheels_velocity(speed, speed)
        self.set_wheels_velocity(0, 0)

    def goForwardTillSensorDetectMap(self, speed, frontSensor, leftSensor, rightSensor, walldistance, direction):
        while(True):
            frontDistance = self.sensorDistance(frontSensor)          
            if frontDistance < walldistance and frontDistance > 0.001 :
                self.map.set_forward_value(direction)
                break
            else:
                error = self.goForwardMap(5, 1, 0.001)
                if error > -0.001 and error < 0.001:
                    leftSensorDistance = self.sensorDistance(leftSensor)
                    rightSensorDistance = self.sensorDistance(rightSensor)
                    #print("Go Forward")  
                    self.map.set_values(direction, leftSensorDistance, rightSensorDistance)
                    returnCode = self.map.check_value(direction, frontDistance, leftSensorDistance, rightSensorDistance)
                    if returnCode == 1:
                        continue
                    if returnCode == 2:
                        print("returned 2")
                        rotationError = 9999
                        while rotationError > 181:
                            rotationError = self.rotateCarByDegMap(-90, 2)
                        direction = self.set_direction(direction, -1)
                        break
                    if returnCode == 3:
                        print("returned 3")
                        rotationError = 9999
                        while rotationError > 181:
                            rotationError = self.rotateCarByDegMap(90, 2)
                        direction = self.set_direction(direction, 1)
                        break
        self.set_wheels_velocity(0, 0)
        return direction

    def goForwardMap(self, speed, distance, error):
        startingPosition = self.getCarPosition()
        self.set_wheels_velocity(speed, speed)
        #print("starting to move with speed {0}".format(speed))
        while(True):
            newPosition = self.getCarPosition()
            diff = [startingPosition[0] - newPosition[0], startingPosition[1] - newPosition[1]]
            d = math.sqrt(diff[0]*diff[0] + diff[1]*diff[1])
            #print("startingPosition= {0}, new = {1}, diff = {2}, d = {3}".format(startingPosition, newPosition, diff, d))
            if distance - d < error :
                self.set_wheels_velocity(0, 0)
                print("stopped, error = {0}".format(d - distance))
                return d - distance
            if distance - d < 0.01 :
                self.set_wheels_velocity(speed/4, speed/4)
                continue
            if distance - d < 0.1 :
                self.set_wheels_velocity(speed/2, speed/2)
                continue

    def goForward(self, speed, distance, error):
        startingPosition = self.getCarPosition()
        self.set_wheels_velocity(speed, speed)
        #print("starting to move with speed {0}".format(speed))
        while(True):
            newPosition = self.getCarPosition()
            diff = [startingPosition[0] - newPosition[0], startingPosition[1] - newPosition[1]]
            d = math.sqrt(diff[0]*diff[0] + diff[1]*diff[1])
            #print("startingPosition= {0}, new = {1}, diff = {2}, d = {3}".format(startingPosition, newPosition, diff, d))
            if distance - d < error :
                self.set_wheels_velocity(0, 0)
                print("stopped, error = {0}".format(d - distance))
                break
            if distance - d < 0.01 :
                self.set_wheels_velocity(speed/4, speed/4)
                continue
            if distance - d < 0.1 :
                self.set_wheels_velocity(speed/2, speed/2)
                continue
            


    def wheels_pos(self):
        return sim.simxGetJointPosition(self.clientID, self.wheels[0], sim.simx_opmode_oneshot)[1],
        sim.simxGetJointPosition(self.clientID, self.wheels[1], sim.simx_opmode_oneshot)[1],
        sim.simxGetJointPosition(self.clientID, self.wheels[2], sim.simx_opmode_oneshot)[1],
        sim.simxGetJointPosition(self.clientID, self.wheels[3], sim.simx_opmode_oneshot)[1]

    def detect_object(self, sensorHolder, miliseconds):
        x = 0
        while(x<miliseconds):
            return_value, detectionState, detectionPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(self.clientID, self.sensors[0], sim.simx_opmode_buffer)
            
            if(detectionState==True and x%100==0):
                print(detectionPoint)
                print(detectionState)
                #sim.simxAddStatusbarMessage(clientID,'Detected something',sim.simx_opmode_oneshot)
            time.sleep(0.001)
            x+=1

    def detect_object_quick(self):
        return_value, detectionState, detectionPoint, detectedObjectHandle, detectedSurfaceNormalVector = sim.simxReadProximitySensor(self.clientID, self.sensors[0], sim.simx_opmode_buffer)
        if(detectionState==True):
            print(detectionPoint)
            print(detectionState)
            sim.simxAddStatusbarMessage(self.clientID,'Detected something',sim.simx_opmode_oneshot)
