# !/usr/bin/env python3
import math
class Odometry:
    def __init__(self):
        self.trackWidth = 135
        self.distancePerTic = 0.488692
        self.rawLog = []
        self.correctionLog = []
        self.position = [0, 0, 0]
        self.cleanedPosition = [0, 0, 0]
        self.pathWasBlocked = False

    def logPath(self, motorADistance, motorBDistance):
        self.rawLog.append((motorADistance, motorBDistance))

    def roundNumber(self, xVar, yVar, alphaVar):
        number = round(xVar / 600) * 500, round(yVar / 600) * 500, (round(math.degrees(alphaVar) / 90) * 90) % 360
        self.cleanedPosition[0] += number[0] / 500
        self.cleanedPosition[1] += number[1] / 500
        self.cleanedPosition[2] = (number[2]) % 360
        self.position[0] = self.cleanedPosition[0] * 500
        self.position[1] = self.cleanedPosition[1] * 500

    def calculateCorrectionLog(self):
        self.position[0] = 0
        self.position[1] = 0
        self.position[2] = math.radians(self.cleanedPosition[2]) % 360
        if self.pathWasBlocked:
            self.rawLog.clear()
            self.addViewDirection(180)
            self.pathWasBlocked = False
        for i in range(1, len(self.rawLog)):
            dleft = (self.rawLog[i][0] - self.rawLog[i - 1][0]) * self.distancePerTic
            dright = (self.rawLog[i][1] - self.rawLog[i - 1][1]) * self.distancePerTic
            beta = (dleft - dright) / (self.trackWidth * 2)
            if beta != 0:
                s = ((dleft + dright) * math.sin(beta)) / (beta * 2)
            else:
                s = (dleft + dright) / 2
            self.position[0] += math.sin(self.position[2] + beta) * s
            self.position[1] += math.cos(self.position[2] + beta) * s
            self.position[2] = (self.position[2] + beta * 2) % (2 * math.pi)

        self.roundNumber(self.position[0], self.position[1], self.position[2])
        
    def addViewDirection(self, deg):
        self.cleanedPosition[2] = (self.cleanedPosition[2] + deg) % 360
        print(f"New View Direction: {self.cleanedPosition[2]}")