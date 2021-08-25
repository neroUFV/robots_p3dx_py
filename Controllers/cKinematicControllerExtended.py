import numpy as np
import math


def cKinematicControllerExtended(p3dx, pgains=np.array([[0.2, 0.2, 1]])):

    # Computing the pose error:
    p3dx.pPos.Xtil = p3dx.pPos.Xd - p3dx.pPos.X

    # Extended Kinematic model
    vx = p3dx.pPos.Xd[6] + pgains[0]*p3dx.pPos.Xtil[0]
    vy = p3dx.pPos.Xd[7] + pgains[1]*p3dx.pPos.Xtil[1]

    if abs(p3dx.pPar.alpha) < math.pi/2 and p3dx.pPar.a > 0:
        vw = ((-1)*math.sin(p3dx.pPos.X[5])*vx + math.cos(p3dx.pPos.X[5]*vy) / (p3dx.pPar.a * math.cos(p3dx.pPar.alpha)))
    else:
        p3dx.pPos.Xd[5] = math.atan2(vy, vx)
        p3dx.pPos.Xd[11] = (p3dx.pPos.Xd[5] - p3dx.pPos.Xda[5]) / 0.1
        p3dx.pPos.Xtil[5] = p3dx.pPos.Xd[6] - p3dx.pPos.X[5]

        if abs(p3dx.pPos.Xtil[5]) > math.pi:
            if p3dx.pPos.Xtil[5] > 0:
                p3dx.pPos.Xtil[5] = (-2)*math.pi + p3dx.pPos.Xd[5] - p3dx.pPos.X[5]
            else:
                p3dx.pPos.Xtil[5] = 2*math.pi + p3dx.pPos.Xd[5] - p3dx.pPos.X[5]

        vw = 0*(p3dx.pPos.Xd[11]) + pgains[2]*p3dx.pPos.Xtil[5]

    p3dx.pSC.Ud[1] = vw
    p3dx.pSC.Ud[0] = vx*math.cos(p3dx.pPos.X[5]) + vy*math.sin(p3dx.pPos.X[5]) + p3dx.pPar.a*math.sin(p3dx.pPar.alpha)*vw

    # Saturation of the control signal, based on the P3DX robot's datasheet:
    if abs(p3dx.pSC.Ur[0]) > 0.75:
        p3dx.pSC.Ur[1] = np.sign(p3dx.pSC.Ur[1])*1
