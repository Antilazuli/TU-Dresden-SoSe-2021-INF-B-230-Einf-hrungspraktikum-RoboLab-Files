from re import T
import ev3dev.ev3 as ev3

import ev3dev.core as ev3s
from odometry import Odometry
import time
import random #Nur für's rumfahren...

class PDControler:
    def __init__(self, motorA, motorB, sensorA, sensorB):
        self.dBV = 300                              #Default black value as seen in
        self.dWV = 500                              #Default white value as seen in 
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
        self.sensorLastReadingSec = 0
        ###
        self.colorRcorrection = 1
        self.colorGcorrection = 1
        self.colorBcorrection = 1

    """Sensor functions"""
    #Get light-intensity from color-sensor/ see RGB as grayscale-value
    def sensorGetLightIntensity(self):
        return sum(list(self.sensorA.bin_data("hhh")))

    def setBlack(self):
        self.dBV = self.sensorGetLightIntensity()
        self.medianBW = (self.dBV + self.dWV) // 2

    def setWhite(self):
        self.dWV = self.sensorGetLightIntensity()
        self.medianBW = (self.dBV + self.dWV) // 2

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
        self.colorRcorrection = median / self.sensorGetRawRed() #Liste aus Raw RGB und dann var[0] usw hier
        self.colorGcorrection = median / rawSensorInput[1] #Wie hier
        self.colorBcorrection = median / self.sensorGetRawBlue()

    """Motor functions"""
    def motorsChangeSpeed(self, speed):

        self.motorA.speed_sp = speed
        self.motorB.speed_sp = speed
        self.motorA.command = "run-forever"
        self.motorB.command = "run-forever"
        self.motorSpeed = speed

    def driveDistance(self, leftMotor, rightMotor)

    def motorsTurn(self, deg): #Muss noch für 45 Grad Kurven angepasst werden

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

                self.motorA.reset()
                self.motorA.stop_action = "brake"
                self.motorA.position_sp = -5
                self.motorA.speed_sp = self.motorSpeed
                self.motorA.command = "run-to-rel-pos"

                self.motorB.reset()
                self.motorB.stop_action = "brake"
                self.motorB.position_sp = 5
                self.motorB.speed_sp = self.motorSpeed
                self.motorB.command = "run-to-rel-pos"

                self.motorA.wait_until_not_moving()
                break #vlt weg?, hm eher nicht...

    def determineDistancePerTick(self, testDistance):
        benchList = []
        print("Place explorer perpendicular to flat obstacle, test starts in ten seconds...")
        time.sleep(10)
        while self.sensorB.value() > 2000:

        while self.sensorB.value() < 50:


    """Navigation functions"""
    def eventWatcher(self): #Muss noch
        #alle Abbruchbedingungen
        pass

    def isKnot(self):
        
        if self.sensorSeesColor("r") or self.sensorSeesColor("b"):
            return True
        return False

    def centerOnKnot(self):

        self.motorA.reset()
        self.motorA.stop_action = "brake"
        self.motorA.position_sp = 230
        self.motorA.speed_sp = self.motorSpeed
        self.motorA.command = "run-to-rel-pos"

        self.motorB.reset()
        self.motorB.stop_action = "brake"
        self.motorB.position_sp = 230
        self.motorB.speed_sp = self.motorSpeed
        self.motorB.command = "run-to-rel-pos"

        self.motorA.wait_until_not_moving()

        #self.od_module.calculateCorrectionLog()

        self.motorsTurn(90 * random.randint(1,4))

        while self.sensorGetLightIntensity() > self.medianBW:  
            #hn = random.randint(1, 3)
            self.motorsTurn(90 * random.randint(1, 4))

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
                self.motorsTurn(180)
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
        return sum(kd) * 0.0005

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

    def dT(self): #Wird evtl nicht gebraucht?
        if self.sensorLastReadingSec == 0:
            self.sensorLastReadingSec = time.time()
            return 1
        else:
            dT = time.time() - self.sensorLastReadingSec
            self.sensorLastReadingSec = time.time()
            return dT

    """Testing/Debugging/Utilities"""
    def pickedUp(self):
        if self.dBV < 60: #Vorher > 50d
            print(self.dBV)
            ev3s.Sound.play("/home/robot/src/Gefahr.wav")
            return True
        return False
