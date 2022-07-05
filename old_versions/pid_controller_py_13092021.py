import ev3dev.ev3 as ev3
import ev3dev.core as ev3s
import time
import random

class PDControler:
    def __init__(self, motorA, motorB, sensorA):
        self.dBV = 300                              #Default black value as seen in
        self.dWV = 500                              #Default white value as seen in 
        self.medianBW = (self.dBV + self.dWV) / 2   #Median of black line and white floor, should equal edge of path
        ###
        self.motorA = motorA
        self.motorB = motorB
        self.motorA.reset()
        self.motorB.reset()
        self.motorsChangeSpeed(0)
        self.motorSpeed = 0
        ###
        self.sensorA = sensorA
        self.sensorA.mode = 'RGB-RAW'
        self.lastErrors = [0, 0, 0]
        self.sensorLastReadingSec = 0
        ###
        self.colorRcorrection = 1
        self.colorGcorrection = 1
        self.colorBcorrection = 1

        self.i = 0
        self.hub = 0

    """Sensor functions"""
    #Get light-intensity from color-sensor/ see RGB as grayscale-value
    def sensorGetLightIntensity(self): 
        #print(sum(list(self.sensorA.bin_data("hhh"))))
        return sum(list(self.sensorA.bin_data("hhh")))

    def setBlack(self):
        self.dBV = self.sensorGetLightIntensity()
        self.medianBW = (self.dBV + self.dWV) / 2

    def setWhite(self):
        self.dWV = self.sensorGetLightIntensity()
        self.medianBW = (self.dBV + self.dWV) / 2

    def sensorGetRawRed(self):
        red = self.sensorA.bin_data("hhh")
        return red[0]

    def sensorGetRawGreen(self):
        green = self.sensorA.bin_data("hhh")
        return green[1]

    def sensorGetRawBlue(self):
        blue = self.sensorA.bin_data("hhh")
        return blue[2]

    def sensorSeesBlack(self, tolerance):
        if self.sensorGetLightIntensity() < self.dBV * (1 + 0.01 * tolerance):
            return True
        return False

    def sensorSeesWhite(self, tolerance):
        if self.sensorGetLightIntensity() > self.dWV * (1 - 0.01 * tolerance):
            return True
        return False
    
    def sensorSeesColor(self, color):
        if color == "r":
            rgb = self.sensorGetCalibratedRGB()
            if rgb[2] * 4 < rgb[0]:
                return True
            return False
        elif color == "b":
            rgb = self.sensorGetCalibratedRGB()
            if rgb[0] * 1.6 < rgb[2] and rgb[1] * 1.4 > rgb[2]:
                return True
            return False

    def sensorGetCalibratedRGB(self):
        correctedRGB = list(self.sensorA.bin_data("hhh"))
        correctedRGB[0] = correctedRGB[0] * self.colorRcorrection
        correctedRGB[1] = correctedRGB[1] * self.colorGcorrection
        correctedRGB[2] = correctedRGB[2] * self.colorBcorrection
       
        if self.i >= 150:
            #print(f"{int(correctedRGB[0])}, {int(correctedRGB[1])}, {int(correctedRGB[2])}")
            self.i = 0
        self.i += 1
        return correctedRGB

    def calibrateOnWhite(self):
        median = self.sensorGetLightIntensity() / 3
        self.colorRcorrection = median / self.sensorGetRawRed()
        self.colorGcorrection = median / self.sensorGetRawGreen()
        self.colorBcorrection = median / self.sensorGetRawBlue()

    """Motor functions"""
    def motorsChangeSpeed(self, speed):
        self.motorA.speed_sp = speed
        self.motorB.speed_sp = speed
        self.motorA.command = "run-forever"
        self.motorB.command = "run-forever"
        self.motorSpeed = speed

    def motorsTurn(self, deg):
        print(f"tunring: {deg} degrees")
        self.motorA.reset()
        self.motorA.stop_action = "brake"
        self.motorA.position_sp = 206 * deg / 90 - 10
        self.motorA.speed_sp = self.motorSpeed
        self.motorA.command = "run-to-rel-pos"

        self.motorB.reset()
        self.motorB.stop_action = "brake"
        self.motorB.position_sp = -206 * deg / 90 - 10
        self.motorB.speed_sp = self.motorSpeed
        self.motorB.command = "run-to-rel-pos"

        self.motorA.wait_until_not_moving()

    """Navigation functions"""
    def adjustSteering(self):
        pass #See PD prime functions -> setMotorsTurn

    def eventWatcher(self):
        pass

    def followPath(self, speed):
        self.motorsChangeSpeed(speed)

    def isKnot(self):
        
        if self.sensorSeesColor("r"):
            print("Base is Red")
            self.hub = 0
        elif self.sensorSeesColor("b"):
            print("Base is Blue")
            self.hub = 1
        else:
            pass
        
        if self.sensorSeesColor("r") or self.sensorSeesColor("b"):
            #print("Knot found!")
            self.centerOnKnot()

    def centerOnKnot(self):
        test = self.sensorGetCalibratedRGB()
        #print(*test)

        self.motorA.reset()
        self.motorA.stop_action = "brake"
        self.motorA.position_sp = 222
        self.motorA.speed_sp = self.motorSpeed
        self.motorA.command = "run-to-rel-pos"

        self.motorB.reset()
        self.motorB.stop_action = "brake"
        self.motorB.position_sp = 222
        self.motorB.speed_sp = self.motorSpeed
        self.motorB.command = "run-to-rel-pos"

        #print("Stopping!")

        time.sleep(2)
        #print("Knot found, turning...")
        if self.hub == 1:
            ev3.Sound.speak('Blue vented')
        else:
            ev3.Sound.speak('Red is kinda sus')
        
        #dev#
        bruh = random.randint(0,4)
        self.motorsTurn(90)

        time.sleep(5)

        if self.sensorGetLightIntensity() > self.medianBW:         
            self.motorA.reset()
            self.motorB.reset()
            self.motorsTurn(90)
            print("Not Track, turning again")
            print(self.medianBW)
            print(self.sensorGetLightIntensity())
        else:
            self.motorsChangeSpeed(self.motorSpeed)
            print("New track found!")
            pass

        print(bruh)


    """PD prime functions"""
    def caclculateError(self):
        return self.sensorGetLightIntensity() - self.medianBW
    
    def calculateKp(self):
        return self.motorSpeed / (self.dWV - self.medianBW) * 0.75

    def calculateKd(self):
        if self.lastErrors[0] == 0:
            return 0
        else:
            return (sum(self.caclculateError() + self.lastErrors[:-1])) * 1000

    def calculateTurn(self):
        return self.calculateKp() * self.caclculateError() + self.calculateKd() #/ self.dT() #Neu ab Plus, vlt sogar ohne /

    def setMotorsTurn(self):
        self.tuneColorRange()
        self.motorA.speed_sp = self.motorSpeed + self.calculateTurn()
        self.motorB.speed_sp = self.motorSpeed - self.calculateTurn()
        self.motorA.command = "run-forever"
        self.motorB.command = "run-forever"

    def tuneColorRange(self):
        if int(self.sensorGetLightIntensity()) > self.dWV:
            self.dWV = self.sensorGetLightIntensity()
            self.calibrateOnWhite()
            #print(f"(R: {self.colorRcorrection}, G: {self.colorGcorrection}, B: {self.colorBcorrection})")
        elif int(self.sensorGetLightIntensity()) < self.dBV:
            self.dBV = self.sensorGetLightIntensity()

    def dT(self):
        if self.sensorLastReadingSec == 0:
            self.sensorLastReadingSec = time.time()
            return 0
        else:
            dT = time.time() - self.sensorLastReadingSec
            self.sensorLastReadingSec = time.time()
            return dT

    """Testing/Debugging/Utilities"""
    def pickedUp(self):
        if self.dBV > 50:
            return False
        ev3s.Sound.play("/home/robot/src/Gefahr.wav")
        return True
