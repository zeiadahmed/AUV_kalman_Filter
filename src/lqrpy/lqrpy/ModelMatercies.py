import numpy as np
from lqrpy.LQR import LQR
import os


class ModelMatercies:
    # modeldata is filename where the matercies MRA and MRB are saved
    def __init__(self):
        modelData = ["/home/islams/AUVControl/src/lqrpy/lqrpy/MRB.txt", "/home/islams/AUVControl/src/lqrpy/lqrpy/MA.txt", "/home/islams/AUVControl/src/lqrpy/lqrpy/linearDamping.txt",
                     "/home/islams/AUVControl/src/lqrpy/lqrpy/QuadraticDamping.txt"]
        self.MRB = self.loadMassMatrix(modelData[0])
        self.MA = self.loadMassMatrix(modelData[1])
        self.LD = self.LinearDampingMatrix(modelData[2])
        self.QD = self.QuadraticDampingMatrix(modelData[3])

    def loadMassMatrix(self, filename):
        f = open(file=filename, mode="r")
        for i in range(0, 6):
            row = f.readline()
            row = np.fromstring(string=row, dtype=np.float32, sep=" ")
            if(i == 0):
                M = np.array(row, np.float32)
            else:
                M = np.vstack((M, row))
        return M

    def skew(self, vect):
        return np.array([
            [0, -vect[2],   vect[1]],
            [vect[2],   0,  -vect[0]],
            [-vect[1], vect[0],   0]
        ])

    def CentriptalMatirx(self, M, v):
        M11 = M[0:3, 0:3]
        M12 = M[0:3, 3:6]
        M21 = M[3:6, 0:3]
        M22 = M[3:6, 3:6]
        v1 = v[0:3].transpose()
        v2 = v[3:6].transpose()
        C11 = np.zeros(shape=(3, 3))
        C12 = -1*self.skew(np.matmul(M11, v1)+np.matmul(M12, v2))
        C21 = -1*self.skew(np.matmul(M11, v1)+np.matmul(M12, v2))
        C22 = -1*self.skew(np.matmul(M21, v1)+np.matmul(M22, v2))
        c1 = np.hstack((C11, C12))
        c2 = np.hstack((C21, C22))
        C = np.vstack((c1, c2))
        return C

    def LinearDampingMatrix(self, filename):
        f = open(filename, "r")
        D = np.fromstring(string=f.readline(), dtype=np.float32, sep=" ")
        return D

    def QuadraticDampingMatrix(self, filename):
        f = open(filename, "r")
        D = np.fromstring(string=f.readline(), dtype=np.float32, sep=" ")
        return D

    def CalcDamping(self, linearMat, QuadMat, v):
        D = np.zeros(shape=(6, 6))
        D[0, 0] = linearMat[0] + QuadMat[0] * v[0]
        D[1, 1] = linearMat[1] + QuadMat[1] * v[1]
        D[2, 2] = linearMat[2] + QuadMat[2] * v[2]
        D[3, 3] = linearMat[3] + QuadMat[3] * v[3]
        D[4, 4] = linearMat[4] + QuadMat[3] * v[4]
        D[5, 5] = linearMat[5] + QuadMat[3] * v[5]
        return D

    def calcAMatrix(self, v):
        A11 = np.zeros(shape=(6, 6))
        A12 = np.eye(N=6)
        A21 = A11
        M = self.MA+self.MRB
        C = self.CentriptalMatirx(self.MA, v) + \
            self.CentriptalMatirx(self.MRB, v)

        D = self.CalcDamping(self.LD, self.QD, v)

        Minv = np.linalg.inv(M)
        A22 = np.matmul(Minv, (C+self.LD))
        A = np.vstack((np.hstack((A11, A12)), np.hstack((A21, A22))))
        return A

    def calcBMatrix(self):
        M = self.MA+self.MRB
        Minv = np.linalg.inv(M)
        B = np.vstack((np.zeros((6, 6)), Minv))
        return B


m = ModelMatercies()
v = np.array([2, 2, 0, 0, 0, 0])
lqr = LQR()
lqr.CalculateLQR(m.calcAMatrix(
    v), m.calcBMatrix(), np.eye(12, 12), np.eye(6, 6))
print(lqr.getLQRGain().shape)
# print(lqr.checkControlability(m.calcAMatrix(v), m.calcBMatrix()))
