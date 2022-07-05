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
        self.motorsChangeSpeed(0)
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
    #Get light-intensity from color-sensor/ see RGB as grayscale-value
    def sensorGetLightIntensity(self):
        return sum(list(self.sensorA.bin_data("hhh"))) #fragen was das hhh soll

    def sensorGetRawRed(self):
        red = self.sensorA.bin_data("hhh")
        return red[0]

    def sensorGetRawGreen(self):
        green = self.sensorA.bin_data("hhh")
        return green[1]

    def sensorGetRawBlue(self):
        blue = self.sensorA.bin_data("hhh")
        return blue[2]

    def sensorSeesColor(self, color):
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

    def sensorScanForPaths(self):
        self.motorsRawMovement(self.ticPerDeg * 360, -self.ticPerDeg * 360, 200)
        while "running" in self.motorA.state:
            if self.motorA.position < self.ticPerDeg * 360:
                if self.motorA.position > self.ticPerDeg * 45 and self.motorA.position < self.ticPerDeg * 135 and self.sensorGetLightIntensity() <  self.medianBW:
                    self.pathScan[1] = True
                elif self.motorA.position > self.ticPerDeg * 135 and self.motorA.position < self.ticPerDeg * 225 and self.sensorGetLightIntensity() <  self.medianBW:
                    self.pathScan[2] = True
                elif self.motorA.position > self.ticPerDeg * 225 and self.motorA.position < self.ticPerDeg * 315 and self.sensorGetLightIntensity() <  self.medianBW:
                    self.pathScan[3] = True
                elif self.motorA.position > self.ticPerDeg * 315 and self.sensorGetLightIntensity() <  self.medianBW:
                    self.pathScan[0] = True

        self.motorA.wait_until_not_moving()
        print(*self.pathScan)

    """Motor functions"""
    def motorsRawMovement(self, distanceL, distanceR, speed):

        self.motorA.reset()
        self.motorB.reset()

        #Für gelogte Strecken ...TillFinish nehmen

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

        #self.od_module.logPath(distanceL, distanceR)
        #self.od_module.logPath(distanceL * 2, distanceR * 2)

    def motorsTurnToMedian(self):
        light = self.sensorGetLightIntensity()
        offset = (self.medianBW - light) / self.medianBW * 15
        self.motorsMove_waitTillFinish(-offset, offset, 30)

    def motorsTurnToPath(self, degToExpectedPath):

        self.motorsRawMovement(degToExpectedPath * self.ticPerDeg, -degToExpectedPath * self.ticPerDeg, self.motorSpeed)

        while "running" in self.motorA.state:
            if self.sensorGetLightIntensity() <  self.medianBW and self.motorA.position > degToExpectedPath * self.ticPerDeg * 0.75:

                #self.od_module.logPath(self.motorA.position, self.motorB.position)
                #self.od_module.logPath(self.motorA.position * 2, self.motorB.position * 2)

                self.motorsTurnToMedian()
                #self.motorA.wait_until_not_moving()
                break

    ##---------------------------------------------------------------##

    def motorsChangeSpeed(self, speed):
        self.motorA.speed_sp = speed
        self.motorB.speed_sp = speed
        self.motorA.command = "run-forever"
        self.motorB.command = "run-forever"
        self.motorSpeed = speed

    def motorsDriveDistance(self, leftMotor, rightMotor, scanForPath = False):

        # self.motorA.reset()
        # self.motorA.stop_action = "brake"
        # self.motorA.position_sp = leftMotor
        # self.motorA.speed_sp = 50
        # self.motorA.command = "run-to-rel-pos"

        # self.motorB.reset()
        # self.motorB.stop_action = "brake"
        # self.motorB.position_sp = rightMotor
        # self.motorB.speed_sp = 50
        # self.motorB.command = "run-to-rel-pos"

        self.motorsRawMovement(leftMotor, rightMotor, 50)

        while scanForPath:
            if self.motorA.position < 800:
                if self.motorA.position > 150 and self.motorA.position < 250:
                    self.pathScan[1] = True
                elif self.motorA.position > 350 and self.motorA.position < 450:
                    self.pathScan[2] = True
                elif self.motorA.position > 550 and self.motorA.position < 650:
                    self.pathScan[3] = True
                elif self.motorA.position > 750 and self.motorA.position < 850:
                    self.pathScan[0] = True

        self.motorA.wait_until_not_moving()
        print(*self.pathScan)

    def motorsTurn(self, deg):

        self.motorA.reset()
        self.motorA.stop_action = "brake"
        self.motorA.position_sp = 206 * deg / 90 + 20
        self.motorA.speed_sp = self.motorSpeed
        self.motorA.command = "run-to-rel-pos"

        self.motorB.reset()
        self.motorB.stop_action = "brake"
        self.motorB.position_sp = -206 * deg / 90 - 20
        self.motorB.speed_sp = self.motorSpeed
        self.motorB.command = "run-to-rel-pos"

        while "running" in self.motorA.state:
            if self.sensorGetLightIntensity() <  self.medianBW and self.motorA.position > 206 * deg / 90 * 0.7:

                # self.motorA.reset()
                # self.motorA.stop_action = "brake"
                # self.motorA.position_sp = -5
                # self.motorA.speed_sp = self.motorSpeed
                # self.motorA.command = "run-to-rel-pos"

                # self.motorB.reset()
                # self.motorB.stop_action = "brake"
                # self.motorB.position_sp = 5
                # self.motorB.speed_sp = self.motorSpeed
                # self.motorB.command = "run-to-rel-pos"

                # self.motorA.wait_until_not_moving()
                self.motorsTurnToMedian()
                break #vlt weg?, hm eher nicht...

    def determineDistancePerTic(self, testDistance = 100):

        benchListA = []
        print("Place explorer perpendicular to flat obstacle, test starts in ten seconds...")
        time.sleep(10)
        while self.sensorB.value() > 2000:
            print("searching for obstacle")
            ###self.motorsDriveDistance(100, 100)
            self.motorsMove_waitTillFinish(100, 100, 50)
            time.sleep(1)

        print(f"Obstacle found in {self.sensorB.value() / 10}cm distance!")

        while self.sensorB.value() > 100:
            startMessurement = self.sensorB.value()
            self.motorsMove_waitTillFinish(testDistance, testDistance, 50)
            time.sleep(1)
            print(f"Distanz: {(startMessurement - self.sensorB.value())}mm")
            benchListA.append(startMessurement - self.sensorB.value())

        if len(benchListA) != 0 and testDistance != 0:
            dPT = sum(benchListA) / len(benchListA) / testDistance #<- davor?
            ###dPT = testDistance / sum(benchListA) / len(benchListA)
        else:
            return 0
        print(f"Based on this test, the travelled distance for 100 tics should be [{int(dPT * 100)} mm] or [{dPT} mm] per tic")
        benchListB = []
        i = 0
        while i < 10:

            self.motorA.reset()
            self.motorA.stop_action = "brake"
            self.motorA.position_sp = 206 * 4
            self.motorA.speed_sp = self.motorSpeed
            self.motorA.command = "run-to-rel-pos"

            self.motorB.reset()
            self.motorB.stop_action = "brake"
            self.motorB.position_sp = -206 * 4
            self.motorB.speed_sp = self.motorSpeed
            self.motorB.command = "run-to-rel-pos"

            while "running" in self.motorA.state:
                if self.sensorGetLightIntensity() <  self.medianBW and self.motorA.position > 206 * 4 * 0.7:
                    benchListB.append(self.motorA.position / 3.142)
                    self.motorA.wait_until_not_moving()
                    break #vlt weg?, hm eher nicht...
            axel = sum(benchListB) / len(benchListB)
        print(f"Based on this second test, the explorers axel should be {axel * dPT} mm wide")

    """Navigation functions"""
    def eventWatcher(self): #Muss noch
        #alle Abbruchbedingungen
        pass

    def isKnot(self):
        
        if self.sensorSeesColor("r") or self.sensorSeesColor("b"):
            return True
        return False

    def centerOnKnot(self):

        # self.motorA.reset()
        # self.motorA.stop_action = "brake"
        # self.motorA.position_sp = 230
        # self.motorA.speed_sp = self.motorSpeed
        # self.motorA.command = "run-to-rel-pos"

        # self.motorB.reset()
        # self.motorB.stop_action = "brake"
        # self.motorB.position_sp = 230
        # self.motorB.speed_sp = self.motorSpeed
        # self.motorB.command = "run-to-rel-pos"

        # self.motorA.wait_until_not_moving()

        self.motorsMove_waitTillFinish(230, 230, self.motorSpeed)

        self.od_module.calculateCorrectionLog()

        randA = random.randint(1,4)
        ##self.motorsTurn(90 * randA)
        self.motorsTurnToPath(90 * randA)
        #self.od_module.position[2] += 90 * randA

        while self.sensorGetLightIntensity() > self.medianBW:
            randB = random.randint(1,4)
            ##self.motorsTurn(90 * randB)
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
                #self.motorsTurn(180) #könnte noch durch turntopath ersetzt werden falls es funktioniert...
                self.motorsTurnToPath(180)
                usSensorTimeout = 0
            usSensorTimeout += 1

    def caclculateError(self):
        return self.sensorGetLightIntensity() - self.medianBW
    
    def calculateKp(self):
        return self.motorSpeed / (self.dWV - self.medianBW) * abs(self.medianBW - self.sensorGetLightIntensity()) / self.medianBW * 1.4 #0.1 + ... am Anfang weg

    def calculateKd(self):
        kd = self.lastErrors
        self.lastErrors = [self.caclculateError()] + self.lastErrors[:-1]
        #print(*self.lastErrors) #kucken ob geht
        return sum(kd) * 0.01

    def calculateTurn(self):
        return self.calculateKp() * self.caclculateError() + self.calculateKd() #/ self.dT() #Neu ab Plus, vlt sogar ohne /

    def doPathCorrection(self): #Namen geändert
        #print(f"Links: {self.motorA.position} | Rechts: {self.motorB.position} | Turn: {self.motorA.position - self.motorB.position}")
        self.od_module.logPath(self.motorA.position, self.motorB.position)
        self.tuneColorRange()
        self.motorA.speed_sp = (self.motorSpeed + self.calculateTurn()) * (1.05 - abs(self.medianBW - self.sensorGetLightIntensity()) / self.medianBW * 0.999)
        #self.motorA.speed_sp = self.motorSpeed + self.calculateTurn() * abs(self.medianBW - self.sensorGetLightIntensity()) / self.medianBW            
        self.motorB.speed_sp = (self.motorSpeed - self.calculateTurn()) * (1.05 - abs(self.medianBW - self.sensorGetLightIntensity()) / self.medianBW * 0.999)
        #self.motorA.speed_sp = self.motorSpeed + self.calculateTurn() * abs(self.medianBW - self.sensorGetLightIntensity()) / self.medianBW            
        self.motorA.command = "run-forever"
        self.motorB.command = "run-forever"

    def tuneColorRange(self): #geändert currLight für alles
        currLight = int(self.sensorGetLightIntensity())
        if currLight > self.dWV:
            self.dWV = currLight
            self.calibrateOnWhite() #lassen, kalibriert weiß
        elif currLight < self.dBV:
            self.dBV = currLight

    """Testing/Debugging/Utilities"""
    def pickedUp(self):
        if self.dBV < 10: #Vorher < 60
            #ev3s.Sound.play("/home/robot/src/Gefahr.wav")
            return True
        return False
