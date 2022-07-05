import ev3dev.ev3 as ev3
import ev3dev.core as ev3s
from odometry import Odometry

class Robot_Control:
    def __init__(self, motorA, motorB, sensorA, sensorB, speed = 0, dBV = 300, dWV = 500):
        self.dBV = dBV                   
        self.dWV = dWV 
        self.ticPerDeg = 2.336
        self.medianBW = (self.dBV + self.dWV) / 2
        self.od_module = Odometry()
        self.motorA: ev3.LargeMotor = motorA
        self.motorB: ev3.LargeMotor = motorB
        self.motorA.reset()
        self.motorB.reset()
        self.motorsChangeSpeed(speed)
        self.sensorA = sensorA
        self.sensorA.mode = 'RGB-RAW'
        self.sensorB = sensorB
        self.sensorB.mode = 'US-DIST-CM'
        self.lastErrors = [0, 0]
        self.pathScan = [False, False, False, False]
        self.sensorLastReadingSec = 0
        self.colorRcorrection = 1
        self.colorGcorrection = 1
        self.colorBcorrection = 1

    ###Sensor functions###

    def sensorIsKnot(self):
        """Checks if the color sensor is currently facing a knot and returns a boolean accordingly.
        """
        if self.sensorIsColor("r") or self.sensorIsColor("b"):
            ev3.Sound.beep()
            return True
        return False

    def sensorIsMeteor(self):
        """Checks if an obstacle is in front of the ultrasonic sensor. If so,
        the robot gives an acoustic warning and returns to the last knot
        while marking the boolean variable 'pathWasBlocked' in its odometry module as True.
        """
        if self.sensorB.value() < 200:
            self.motorA.STOP_ACTION_BRAKE
            self.motorB.STOP_ACTION_BRAKE
            #ev3s.Sound.play("/home/robot/src/Bruh_Sound_Effect.wav")
            self.motorsTurnToPath(180)
            self.od_module.pathWasBlocked = True
            self.od_module.addViewDirection(180)
            return True
        return False

    def sensorIsPickedUp(self):
        """Stop's the robot when the sensor sees a value close to zero
        which is the case if the robot is picked up and the sensor, therefore, sees nothing at all...
        """
        if self.dBV < 60:
            #ev3s.Sound.play("/home/robot/src/Gefahr.wav")
            return True
        return False

    def sensorCalibrateEdge(self):
        """Updates both white and black values if the sensor sees a darker or lighter Value
        to get a better median and estimate where exactly the edge is.
        """
        currLight = int(self.sensorGetBrightness())
        if currLight > self.dWV:
            self.dWV = currLight
            self.sensorCalibrateOnWhite()
        elif currLight < self.dBV:
            self.dBV = currLight

    def sensorGetBrightness(self):
        """Returns the sum of the current RGB values of the color sensor,
        which should equal the overall brightness the sensor sees at the time of the method call.
        """
        return sum(list(self.sensorA.bin_data("hhh")))

    def sensorIsColor(self, color):
        """Only takes "r" or "b" (Red or Blue) as parameters.
        Returns True if the sensor is over a knot in the given color at the time of the method call.
        """
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
        """Returns list with the color-corrected RGB values the correction factors are global attributes
        and can be changed manually or via the 'sensorCalibrateOnWhite' method."""
        correctedRGB = list(self.sensorA.bin_data("hhh"))
        correctedRGB[0] = correctedRGB[0] * self.colorRcorrection
        correctedRGB[1] = correctedRGB[1] * self.colorGcorrection
        correctedRGB[2] = correctedRGB[2] * self.colorBcorrection
        return correctedRGB

    def sensorCalibrateOnWhite(self):
        """Should be called with the sensor being pointed at a white surface,
        changes the global color correction variables so that all color values should be the same
        when the 'sensorGetCalibratedRGB' method is called while on a white surface.
        """
        median = self.sensorGetBrightness() / 3
        rawSensorInput = self.sensorA.bin_data("hhh")
        self.colorRcorrection = median / rawSensorInput[0]
        self.colorGcorrection = median / rawSensorInput[1]
        self.colorBcorrection = median / rawSensorInput[2]

    def sensorScanForPaths(self,turning_speed = 400):
        """Makes the robot do a 360-degree turn while scanning for paths.
        Overrides the global 'pathScan' bool-list with the new values.
        WARNING: high speed values can make the robot overshoot and therefore end up away from its starting position.
        """
        self.pathScan = [False, False, False, False]
        self.motorsRawMovement(self.ticPerDeg * 360, -self.ticPerDeg * 360, turning_speed)
        while "running" in self.motorA.state:
            if self.sensorGetBrightness() <  self.medianBW:
                self.pathScan[round((self.motorA.position / self.ticPerDeg) / 90) % 4] = True
        self.motorA.wait_until_not_moving()

    ###Motor functions###

    def motorsCenterOnKnot(self):
        """Lets the robot drive a specific distance to center on a potential knot.
        The driven Distance is highly dependant on the robot design!
        """
        self.motorsMoveWaitTillFinished(230, 230, self.motorSpeed)
        self.od_module.calculateCorrectionLog()

    def motorsTurnToMedian(self):
        """Let the robot determine how much off he is from the edge of the path and correct his position accordingly,
        only works on small offsets and only while not moving. Shouldn't be used as part of the PD algorithms!
        """
        light = self.sensorGetBrightness()
        offset = (self.medianBW - light) / self.medianBW * 15
        self.motorsMoveWaitTillFinished(-offset, offset, 30)

    def motorsChangeSpeed(self, speed):
        """Changes the speed of both motors without stoping them, updates the global 'motorsSpeed' variable.
        """
        self.motorA.speed_sp = speed
        self.motorB.speed_sp = speed
        self.motorA.command = "run-forever"
        self.motorB.command = "run-forever"
        self.motorSpeed = speed

    def motorsTurnToPath(self, degToExpectedPath):
        """Let's the robot turn in the given direction. When the movement is about 70% complete,
        the robot starts looking for a path and terminates his movement early once he found one, centering on the wanted path.
        """
        self.od_module.addViewDirection(degToExpectedPath)
        self.motorsRawMovement(degToExpectedPath * self.ticPerDeg, -degToExpectedPath * self.ticPerDeg, self.motorSpeed)
        while "running" in self.motorA.state:
            if self.sensorGetBrightness() <  self.medianBW and self.motorA.position > degToExpectedPath * self.ticPerDeg * 0.75:
                self.motorsTurnToMedian()
                break

    def motorsRawMovement(self, distanceL, distanceR, speed):
        """Lets the motors turn the given tics with the given speed,
        WARNING: doesn't wait until the movement is finished before leaving the method call!
        """
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

    def motorsMoveWaitTillFinished(self, distanceL, distanceR, speed):
        """Lets the motors turn the given tics with the given speed,
        waits until the movement is finished before leaving the method call.
        """
        self.motorsRawMovement(distanceL, distanceR, speed)
        self.motorA.wait_until_not_moving()
        self.motorB.wait_until_not_moving()

    ###Navigation functions###

    def navGoToStart(self):
        """The robot follows the current path to the first knot while not logging odometry data.
        """
        self.motorsChangeSpeed(self.motorSpeed)
        while not self.sensorIsKnot():
            self.pdSetPathCorrection()
        self.od_module.rawLog.clear()
        self.motorsMoveWaitTillFinished(230, 230, self.motorSpeed)
        print("Ready!")

    def navFollowEdge(self):
        """Lets the robot follow the line until a knot is reached or an obstacle is detected
        in which case it turns around and travels back to the last point. After arriving
        at a knot this method terminates.
        """
        self.motorsChangeSpeed(self.motorSpeed)
        usSensorTimeout = 0
        while not self.sensorIsPickedUp() and not self.sensorIsKnot():      
            self.pdSetPathCorrection()
            if usSensorTimeout >= 20:
                self.sensorIsMeteor()
                usSensorTimeout = 0
            usSensorTimeout += 1
        self.motorsCenterOnKnot()

    ###PD functions###

    def pdCalcKp(self):
        """Returns the calculated KP of the PD-Controller,
        for further information search for documentation about 'PD-Controllers.'
        """
        return self.motorSpeed / (self.dWV - self.medianBW) * abs(self.medianBW - self.sensorGetBrightness()) / self.medianBW * 1.4

    def pdCalcKd(self):
        """Returns the calculated KD of the PD-Controller,
        for further information search for documentation about 'PD-Controllers.'
        """
        kd = self.lastErrors
        self.lastErrors = [self.pdCalcError()] + self.lastErrors[:-1]
        return sum(kd) * 0.007

    def pdCalcError(self):
        """Returns the current absolute offset of the robot from the edge.
        """
        return self.sensorGetBrightness() - self.medianBW

    def pdCalcCorrection(self):
        """Calculates the needed correction based on the current offset from the edge, is part of the PD-Controller.
        """
        return self.pdCalcKp() * self.pdCalcError() + self.pdCalcKd()

    def pdSetPathCorrection(self):
        """Changes the speed of both motors to counter the current error and stay on the path.
        """
        self.od_module.logPath(self.motorA.position, self.motorB.position)
        self.sensorCalibrateEdge()
        self.motorA.speed_sp = (self.motorSpeed + self.pdCalcCorrection()) * (1.05 - abs(self.medianBW - self.sensorGetBrightness()) / self.medianBW * 0.999)
        self.motorB.speed_sp = (self.motorSpeed - self.pdCalcCorrection()) * (1.05 - abs(self.medianBW - self.sensorGetBrightness()) / self.medianBW * 0.999)
        self.motorA.command = "run-forever"
        self.motorB.command = "run-forever"

    #             /´/\\\
    #            /´/  \\\
    #           /´/    \\\
    #          /´///////\\\
    #         /´/        \\\
    #        /´/          \\\
    #       /´/            \\\
    #      |||||Antilazuli|||||