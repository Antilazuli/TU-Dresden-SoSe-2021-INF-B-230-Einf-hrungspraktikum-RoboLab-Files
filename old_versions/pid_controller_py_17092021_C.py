# !/usr/bin/env python3
import math
import time
class Odometry:
    def __init__(self):
        self.trackWidth = 146.1
        self.distancePerTic = 0.488692 # Muss ermittelt werden
        self.rawLog = []
        self.correctionLog = []
        self.position = [0, 0, 0]
        ###

    def logPath(self, motorADistance, motorBDistance):
        self.rawLog.append((motorADistance, motorBDistance))

    def roundNumber(self, xVar, yVar, alphaVar):
        number = round((xVar + 50) / 500) * 500, round((yVar + 50) / 500) * 500, round(math.degrees(alphaVar) / 90) * 90

        self.position[0] = number[0]
        self.position[1] = number[1]
        self.position[2] = number[2]

    def calculateCorrectionLog(self):
        self.position[2] = math.radians(self.position[2])
        for i in range(1, len(self.rawLog)):
            dleft = (self.rawLog[i][0] - self.rawLog[i - 1][0]) * self.distancePerTic
            dright = (self.rawLog[i][1] - self.rawLog[i - 1][1]) * self.distancePerTic

            beta = (dleft - dright) / (self.trackWidth * 2)
            if beta != 0:
                s = ((dleft + dright) * math.sin(beta)) / (beta * 2) #"""self.distancePerTic""" 
            else:
                s = (self.rawLog[i][0] - self.rawLog[i - 1][0]) * self.distancePerTic

            self.position[0] -= math.sin(self.position[2] + beta) * s
            self.position[1] += math.cos(self.position[2] + beta) * s
            self.position[2] = (self.position[2] + beta * 2) // (2 * math.pi)

        self.roundNumber(self.position[0], self.position[1], self.position[2])

            #print(f"s: {self.correctionLog[i - 1][0]}, beta: {self.correctionLog[i - 1][1]}")
        print(f"X: {self.position[0]}, Y: {self.position[1]}, Alpha: {self.position[2]}")
        #print(*self.position)
        #print(*self.rawLog[-10:])
        