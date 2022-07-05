from re import T # <- raus?
import ev3dev.ev3 as ev3

import ev3dev.core as ev3s
from odometry import Odometry
import time
import random #Nur für's rumfahren...

class Robot_Control:
    def __init__(self, motorA, motorB, sensorA, sensorB, dBV = 300, dWV = 500):
        self.dBV = dBV                   
        self.dWV = dWV 
        self.ticPerDeg = 2.336
        self.medianBW = (self.dBV + self.dWV) / 2   #Median of black line and white floor, should equal edge of path
        ###
        self.od_module = Odometry()
        ###
        self.motorA: ev3.LargeMotor = motorA
        self.motorB: ev3.LargeMotor = motorB
        self.motorA.reset()
        self.motorB.reset()
        self.motorsChangeSpeed(0) #<- evtl. weg?
        self.motorSpeed = 0
        ###
        self.sensorA = sensorA
        self.sensorA.mode = 'RGB-RAW'
        self.sensorB = sensorB
        self.sensorB.mode = 'US-DIST-CM'
        self.lastErrors = [0, 0]
        self.pathScan = [False, False, False, False]
        self.sensorLastReadingSec = 0
        ###
        self.colorRcorrection = 1
        self.colorGcorrection = 1
        self.colorBcorrection = 1

    """Sensor functions"""
    def sensorGetLightIntensity(self):
        return sum(list(self.sensorA.bin_data("hhh"))) #fragen was das hhh soll

    def sensorSeesColor(self, color): #Würde ich lassen, geht so
        rgb = self.sensorGetCalibratedRGB()
        if color == "r":
            if rgb[2] * 4 < rgb[0]:
                return True
            return False
        elif color == "b":
            if rgb[0] * 1.6 < rgb[2] and rgb[1] * 1.4 > rgb[2]:
                return True
            return False

    def sensorGetCalibratedRGB(self):
        correctedRGB = list(self.sensorA.bin_data("hhh"))
        correctedRGB[0] = correctedRGB[0] * self.colorRcorrection
        correctedRGB[1] = correctedRGB[1] * self.colorGcorrection
        correctedRGB[2] = correctedRGB[2] * self.colorBcorrection
        return correctedRGB

    def calibrateOnWhite(self):
        median = self.sensorGetLightIntensity() / 3
        rawSensorInput = self.sensorA.bin_data("hhh")
        self.colorRcorrection = median / rawSensorInput[0] #geändert auf abfrage aus tupel
        self.colorGcorrection = median / rawSensorInput[1]
        self.colorBcorrection = median / rawSensorInput[2]

    def sensorScanForPaths(self, viewDirection = 0):
        self.motorsRawMovement(self.ticPerDeg * 360, -self.ticPerDeg * 360, 200) #Speed hier hochsetzen soweit geht!
        while "running" in self.motorA.state:

            if self.sensorGetLightIntensity() <  self.medianBW:
                self.pathScan[round(self.motorA.position / self.ticPerDeg * 90)] #<--- Neue Funktion

            # if self.motorA.position < self.ticPerDeg * 360:
            #     if self.motorA.position > self.ticPerDeg * 45 and self.motorA.position < self.ticPerDeg * 135 and self.sensorGetLightIntensity() <  self.medianBW:
            #         self.pathScan[1] = True
            #     elif self.motorA.position > self.ticPerDeg * 135 and self.motorA.position < self.ticPerDeg * 225 and self.sensorGetLightIntensity() <  self.medianBW:
            #         self.pathScan[2] = True
            #     elif self.motorA.position > self.ticPerDeg * 225 and self.motorA.position < self.ticPerDeg * 315 and self.sensorGetLightIntensity() <  self.medianBW:
            #         self.pathScan[3] = True
            #     elif self.motorA.position > self.ticPerDeg * 315 and self.sensorGetLightIntensity() <  self.medianBW:
            #         self.pathScan[0] = True

        self.motorA.wait_until_not_moving()
        print(*self.pathScan) #Nur zum testen

    """Motor functions"""
    def motorsRawMovement(self, distanceL, distanceR, speed):
        self.motorA.reset()
        self.motorB.reset()
        self.motorA.stop_action = "brake"
        self.motorB.stop_action = "brake"
        self.motorA.position_sp = distanceL
        self.motorB.position_sp = distanceR
        self.motorA.speed_sp = speed
        self.motorB.speed_sp = speed
        self.motorA.command = "run-to-rel-pos"
        self.motorB.command = "run-to-rel-pos"

    def motorsMove_waitTillFinish(self, distanceL, distanceR, speed):
        self.motorsRawMovement(distanceL, distanceR, speed)
        self.motorA.wait_until_not_moving()
        self.motorB.wait_until_not_moving()

    def motorsTurnToMedian(self):
        light = self.sensorGetLightIntensity()
        offset = (self.medianBW - light) / self.medianBW * 15
        self.motorsMove_waitTillFinish(-offset, offset, 30)

    def motorsTurnToPath(self, degToExpectedPath):
        self.motorsRawMovement(degToExpectedPath * self.ticPerDeg, -degToExpectedPath * self.ticPerDeg, self.motorSpeed)
        while "running" in self.motorA.state:
            if self.sensorGetLightIntensity() <  self.medianBW and self.motorA.position > degToExpectedPath * self.ticPerDeg * 0.75:
                self.motorsTurnToMedian()
                break

    def motorsChangeSpeed(self, speed):
        self.motorA.speed_sp = speed
        self.motorB.speed_sp = speed
        self.motorA.command = "run-forever"
        self.motorB.command = "run-forever"
        self.motorSpeed = speed

    """Navigation functions"""
    def eventWatcher(self): #Muss noch, vlt Totholz...
        #alle Abbruchbedingungen
        pass

    def isKnot(self):    
        if self.sensorSeesColor("r") or self.sensorSeesColor("b"):
            return True
        return False

    def centerOnKnot(self):
        self.motorsMove_waitTillFinish(230, 230, self.motorSpeed)
        self.od_module.calculateCorrectionLog()

        randA = random.randint(1,4)
        self.motorsTurnToPath(90 * randA)
        while self.sensorGetLightIntensity() > self.medianBW:
            randB = random.randint(1,4)
            self.motorsTurnToPath(90 * randB)
            self.od_module.position[2] += 90 * randB

    def isMeteor(self):
        if self.sensorB.value() < 200:
            self.motorA.STOP_ACTION_BRAKE
            self.motorB.STOP_ACTION_BRAKE
            ev3s.Sound.play("/home/robot/src/Bruh_Sound_Effect.wav")
            return True
        return False

    """PD prime functions"""
    def followEdge(self, speed):
        self.motorsChangeSpeed(speed)
        usSensorTimeout = 0
        while not self.pickedUp():      
            self.doPathCorrection()
            if self.isKnot():
                self.centerOnKnot()
            if usSensorTimeout >= 3 and self.isMeteor():
                self.motorsTurnToPath(180)
                usSensorTimeout = 0
            usSensorTimeout += 1

    def caclculateError(self):
        return self.sensorGetLightIntensity() - self.medianBW
    
    def calculateKp(self):
        return self.motorSpeed / (self.dWV - self.medianBW) * abs(self.medianBW - self.sensorGetLightIntensity()) / self.medianBW * 1.4 #Später mit testen weiter verfeinern

    def calculateKd(self):
        kd = self.lastErrors
        self.lastErrors = [self.caclculateError()] + self.lastErrors[:-1]
        return sum(kd) * 0.01

    def calculateTurn(self):
        return self.calculateKp() * self.caclculateError() + self.calculateKd()

    def doPathCorrection(self):
        self.od_module.logPath(self.motorA.position, self.motorB.position)
        self.tuneColorRange()
        self.motorA.speed_sp = (self.motorSpeed + self.calculateTurn()) * (1.05 - abs(self.medianBW - self.sensorGetLightIntensity()) / self.medianBW * 0.999)
        #self.motorA.speed_sp = self.motorSpeed + self.calculateTurn() * abs(self.medianBW - self.sensorGetLightIntensity()) / self.medianBW            
        self.motorB.speed_sp = (self.motorSpeed - self.calculateTurn()) * (1.05 - abs(self.medianBW - self.sensorGetLightIntensity()) / self.medianBW * 0.999)
        #self.motorA.speed_sp = self.motorSpeed + self.calculateTurn() * abs(self.medianBW - self.sensorGetLightIntensity()) / self.medianBW            
        self.motorA.command = "run-forever"
        self.motorB.command = "run-forever"

    def tuneColorRange(self):
        currLight = int(self.sensorGetLightIntensity())
        if currLight > self.dWV:
            self.dWV = currLight
            self.calibrateOnWhite()
        elif currLight < self.dBV:
            self.dBV = currLight

    """Testing/Debugging/Utilities"""
    def pickedUp(self):
        if self.dBV < 10: #Vorher < 60
            #ev3s.Sound.play("/home/robot/src/Gefahr.wav")
            return True
        return False
