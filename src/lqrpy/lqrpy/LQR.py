import numpy as np
import control
class LQR:
    def CalculateLQR(self,A,B,Q,R):
        self.K,self.S,self.E=control.lqr(A,B,Q,R)
    def getLQRGain(self):
        return self.K
    def getEigens(self):
        return self.E
    def getOptimalRicattiSol(self):
        return self.S
    def checkControlability(self,A,B):
        Conrolability=control.ctrb(A,B)
        print(Conrolability.shape)
        # if(np.linalg.det(Conrolability)==0):
        #     return "un controllable"
        # else:
        #     return "controllable"
