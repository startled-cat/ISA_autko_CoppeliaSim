import math
import numpy as np
import time
from map import Map
from direction import Direction
from celltype import CellType
try:
    from api import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')


class Car:

    def __init__(self, sensors, wheels, carBody, clientID):
        #print("dsdsdsd")
        self.target_velocity = 1
        self.stop_velocity = 0

        self.turning_speed = 0.5
        self.turning_time = 1.80

        self.rotation_force = 100
        #self.map = Map(13, 13, [12, 8])#old labirynth01
        self.map = Map(10, 10, [8, 6])#new labirynth02
        #self.clientID = 0

        self.sensors = sensors
        self.wheels = wheels
        self.clientID = clientID
        self.carBody = carBody

        self.cell_size = 1
        self.cell_size_half = 0.5

        self.rotation_speed = 2
        self.rotation_error = 0.1

        self.frontSensor = self.sensors[0]
        self.leftSensor = self.sensors[5]
        self.rightSensor = self.sensors[4]

        self.direction = self.get_car_direction()

    def random_test(self):

        i = 0
        while i < 4:
            self.rotate_right()
            print(str(self.getCarHorizontalAngle()) + " > " + str(self.get_car_direction()))
            i += 1
            time.sleep(1)

        i = 0
        while i < 4:
            self.rotate_left()
            print(str(self.getCarHorizontalAngle()) + " > " + str(self.get_car_direction()))
            i += 1
            time.sleep(1)
        
        
        #print("======================")
        #time.sleep(3)

    def sensor_test(self):
        frontSensor = self.sensors[0]
        leftSensor = self.sensors[5]
        rightSensor = self.sensors[4]
        while True:
            print("right = " + str(self.sensorDistance(rightSensor)))
            print("left  = " + str(self.sensorDistance(leftSensor)))
            print("front = " + str(self.sensorDistance(frontSensor)))
            time.sleep(1)

    def is_correct(self, measured_distance):
        if measured_distance > 0 :
            return True
        return False

    def mapping_run(self):
        direction = [-1, 0]#Direction.North.value
        # frontSensor = self.sensors[0]
        # leftSensor = self.sensors[5]
        # rightSensor = self.sensors[4]
        while True:
            
            direction = self.goForwardTillSensorDetectMap(5, 0.8, direction)
            print("decide where to go : rotate till sensor front clear")
            leftSensorDistance = self.sensorDistance(self.leftSensor)
            rightSensorDistance = self.sensorDistance(self.rightSensor)
            frontSensorDistance = self.sensorDistance(self.frontSensor)
            print("frontSensor = " + str(frontSensorDistance))
            print("rightSensor = " + str(rightSensorDistance))
            print("leftSensor  = " + str(leftSensorDistance))
            #DECIDE WHERE TO GO: 
            self.get_car_direction()
            forward = self.map.check(self.direction, Direction.FORWARD)
            left = self.map.check(self.direction, Direction.LEFT)
            right = self.map.check(self.direction, Direction.RIGHT)

            if left == CellType.BLOCKED and right == CellType.BLOCKED and forward == CellType.BLOCKED:
                #gotta go back
                self.rotate_180()
                continue

            if forward == CellType.CLEAR :
                #must go forward
                continue

            if forward == CellType.DISCOVERED :
                #been forward, try left or roght, worst case scenario go straight
                if left == CellType.CLEAR:
                    #never been there, go left
                    self.rotate_left()
                    continue
                if right == CellType.CLEAR:
                    #never been there, go right
                    self.rotate_right()
                    continue

                if left == CellType.BLOCKED and right == CellType.BLOCKED:
                    #already been forward, but no other way
                    continue

                if left == CellType.BLOCKED:
                    #never been there, go left
                    self.rotate_right()
                    continue
                if right == CellType.BLOCKED:
                    #never been there, go right
                    self.rotate_left()
                    continue
                continue

            if forward == CellType.BLOCKED:
                if left == CellType.CLEAR:
                    #never been there, go left
                    self.rotate_left()
                    continue
                if right == CellType.CLEAR:
                    #never been there, go right
                    self.rotate_right()
                    continue

                if left == CellType.DISCOVERED:
                    #never been there, go left
                    self.rotate_left()
                    continue
                if right == CellType.DISCOVERED:
                    #never been there, go right
                    self.rotate_right()
                    continue
            #should not come to this point
            while True:
                print("i dont know what do do, hlep human!")
                print("left = " + str(left))
                print("right = " + str(right))
                print("forward = " + str(forward))
                time.sleep(3600)
            '''
            # #rotate left if right sensor detects
            if rightSensorDistance > 0:#right sensor detected sth; rotate left
                while frontSensorDistance < (self.cell_size_half+0.25) and self.is_correct(frontSensorDistance):#while front sensor detects
                    self.rotate_left()
                    direction = self.set_direction(direction, -1)
                    frontSensorDistance = self.sensorDistance(self.frontSensor)
            elif leftSensorDistance > 0:#left sensor detected sth; rotate right
                while frontSensorDistance < (self.cell_size_half+0.25) and self.is_correct(frontSensorDistance):
                    self.rotate_right()
                    direction = self.set_direction(direction, 1)
                    frontSensorDistance = self.sensorDistance(self.frontSensor)
            else:#both right and left sensors detected (concluded)
                while frontSensorDistance < (self.cell_size_half+0.25) and self.is_correct(frontSensorDistance):
                    #rotationError = 9999
                    #while rotationError > 181:
                    #    rotationError = self.rotateCarByDegMap(90, 2)
                    direction = self.set_direction(direction, 1)
                    frontSensorDistance = self.sensorDistance(self.frontSensor)
            '''
            #rotate right if left sensor detects
    
    def rotate_left(self):
        current_car_direction = self.get_car_direction()

        if current_car_direction == Direction.NORTH:
            self.rotate_car_to_direction(Direction.WEST)
            self.direction = Direction.WEST.value

        if current_car_direction == Direction.WEST:
            self.rotate_car_to_direction(Direction.SOUTH)
            self.direction = Direction.SOUTH.value

        if current_car_direction == Direction.SOUTH:
            self.rotate_car_to_direction(Direction.EAST)
            self.direction = Direction.EAST.value

        if current_car_direction == Direction.EAST:
            self.rotate_car_to_direction(Direction.NORTH)
            self.direction = Direction.NORTH.value

    def rotate_right(self):
        current_car_direction = self.get_car_direction()

        if current_car_direction == Direction.NORTH:
            self.rotate_car_to_direction(Direction.EAST)
            self.direction = Direction.EAST.value

        if current_car_direction == Direction.WEST:
            self.rotate_car_to_direction(Direction.NORTH)
            self.direction = Direction.NORTH.value

        if current_car_direction == Direction.SOUTH:
            self.rotate_car_to_direction(Direction.WEST)
            self.direction = Direction.WEST.value

        if current_car_direction == Direction.EAST:
            self.rotate_car_to_direction(Direction.SOUTH)
            self.direction = Direction.SOUTH.value
        
    def rotate_180(self):
        current_car_direction = self.get_car_direction()

        if current_car_direction == Direction.NORTH:
            self.rotate_car_to_direction(Direction.SOUTH)
            self.direction = Direction.SOUTH.value

        if current_car_direction == Direction.WEST:
            self.rotate_car_to_direction(Direction.EAST)
            self.direction = Direction.EAST.value

        if current_car_direction == Direction.SOUTH:
            self.rotate_car_to_direction(Direction.NORTH)
            self.direction = Direction.NORTH.value

        if current_car_direction == Direction.EAST:
            self.rotate_car_to_direction(Direction.WEST)
            self.direction = Direction.WEST.value

    def rotate_car_to_direction(self, direction):

        if direction == Direction.NORTH:
            self.rotateCarToDeg(0, self.rotation_speed, self.rotation_error)
            self.direction = Direction.NORTH.value

        if direction == Direction.WEST:
            self.rotateCarToDeg(90, self.rotation_speed, self.rotation_error)
            self.direction = Direction.WEST.value

        if direction == Direction.SOUTH:
            self.rotateCarToDeg(180, self.rotation_speed, self.rotation_error)
            self.direction = Direction.SOUTH.value

        if direction == Direction.EAST:
            self.rotateCarToDeg(-90, self.rotation_speed, self.rotation_error)
            self.direction = Direction.EAST.value

    def get_car_direction(self):
        angle = self.getCarHorizontalAngle()
        self.direction = Direction.get_direction_from_angle(angle).value
        return Direction.get_direction_from_angle(angle)

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
        #print("detectionState = " + str(detectionState))
        if detectionState == False:
            distance = -1
        else:
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
        #self.detect_object_quick()
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
            #print("rotate left")
            speed = -speed
            degrees = abs(degrees)
        else:
            pass
            #print("rotate right")

        starting_angle = self.getCarHorizontalAngle()
        #print(starting_angle)
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
        #print("rotated car from {0}, to {1} deg, error={2}".format(starting_angle, self.getCarHorizontalAngle(), abs(starting_angle - self.getCarHorizontalAngle())-degrees ))

    def rotateCarToDeg(self, degree, speed, error):
        #print("rotateCarToDeg: rotating from {0}, to {1} ...".format(self.getCarHorizontalAngle(), degree))

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
        
        # if rotateLeft:
        #     print("rotateCarToDeg: rotate left")
        # else:
        #     print("rotateCarToDeg: rotate right")

        
        #start rotating
        self.set_wheels_velocity(speed, -speed)
        #print("starting to rotate ...")
        
        while(True):
            #self.detect_object(self.sensors[0], 1)
            
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
        
        #print("rotateCarToDeg: rotated car from {0}, to {1}, target={2}".format(starting_angle, self.getCarHorizontalAngle(), degree ))

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

    def goForwardTillSensorDetectMap(self, speed, walldistance, direction):
        while(True):
            self.get_car_direction()
            self.map.set_values(self.direction, self.sensorDistance(self.leftSensor), self.sensorDistance(self.rightSensor), self.sensorDistance(self.frontSensor))
            error = self.goForwardMap(5, self.cell_size, 0.001)# car actually travel after 2nd time, dunno why xd
            if error > -0.001 and error < 0.001:# if actually moved
                leftSensorDistance = self.sensorDistance(self.leftSensor)
                rightSensorDistance = self.sensorDistance(self.rightSensor)
                frontDistance = self.sensorDistance(self.frontSensor) 
                #print("Go Forward")  
                self.get_car_direction()
                self.map.update_position(self.direction)
                self.map.set_values(self.direction, leftSensorDistance, rightSensorDistance, frontDistance)
                break
                    
                    
        self.set_wheels_velocity(0, 0)
        return direction

    def goForwardMap(self, speed, distance_to_travel, error):
        startingPosition = self.getCarPosition()
        self.set_wheels_velocity(speed, speed)
        #print("starting to move with speed {0}".format(speed))
        while(True):
            newPosition = self.getCarPosition()
            diff = [startingPosition[0] - newPosition[0], startingPosition[1] - newPosition[1]]
            traveled_distance = math.sqrt(diff[0]*diff[0] + diff[1]*diff[1])
            #print(" ... startingPosition= {0}, new = {1}, diff = {2}, d = {3}".format(startingPosition, newPosition, diff, traveled_distance))
            #print(" ... distance_to_travel - traveled_distance = " + str(distance_to_travel - traveled_distance))
            if distance_to_travel - traveled_distance < error :
                self.set_wheels_velocity(0, 0)
                #print("stopped, error = {0}".format(traveled_distance - distance_to_travel))
                return traveled_distance - distance_to_travel
            if distance_to_travel - traveled_distance < 0.01 :
                self.set_wheels_velocity(speed/4, speed/4)
                continue
            if distance_to_travel - traveled_distance < 0.1 :
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
