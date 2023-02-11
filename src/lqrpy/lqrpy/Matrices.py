#!/usr/bin/env python3
import numpy as np
from math import cos,sin
class FrameTranformation:
    def defineNedAngles(self,angles):
        self.roll = angles[0]
        self.pitch = angles[1]
        self.yaw = angles[2]


    def getLinVelocityTransToBody(self):
        self.Rnb = np.vstack(
            ([cos(self.yaw)*cos(self.pitch),  -sin(self.yaw)*cos(self.roll)+cos(self.yaw)*sin(self.roll)*sin(self.pitch),   sin(self.yaw)*sin(self.roll)+cos(self.yaw)*cos(self.roll)*sin(self.pitch) ],
            [sin(self.yaw)*cos(self.pitch),   cos(self.yaw)*cos(self.roll) + sin(self.roll)*sin(self.pitch)*sin(self.yaw), -cos(self.yaw)*sin(self.roll)+sin(self.pitch)*sin(self.yaw)*cos(self.roll)],
            [-sin(self.pitch),           cos(self.pitch)*sin(self.roll),                                cos(self.pitch)*cos(self.roll)])
        )
        self.Rbn = self.Rnb.transpose()

    def getAngTransToBody(self):
        self.Tbn = np.vstack(
            ([1,     0,           -sin(self.pitch)      ],
            [0,  cos(self.roll),  cos(self.pitch)*sin(self.roll) ],
            [0,  -sin(self.roll), cos(self.pitch)*cos(self.roll) ])
        )
    
    def getTranMat (self):
     
        self.Jeuler = np.vstack((np.hstack((self.Rbn,np.zeros((3,3)))) ,np.hstack((np.zeros((3,3)),self.Tbn))  ))
    
    def Transform(self,velocityVectorNed):
        self.getLinVelocityTransToBody()
        self.getAngTransToBody()
        self.getTranMat()
        velocityVectorBody = np.matmul(self.Jeuler,velocityVectorNed.transpose())
        return velocityVectorBody
    
    def PosTransform(self,posVector):
        self.getLinVelocityTransToBody()
        posTransform = np.matmul(self.Rbn,posVector.transpose())
        return posTransform