import numpy as np
import math


def cKinematicController(p3dx, pgains):
    # Control gains
    # pgains = np.array([[1.5, 1, 1.5, 1])    # Ganhos Daniel

    pgains = np.array([[0.35, 0.35, 0.8, 0.8]])

    Kp1 = np.array([[pgains[0], 0], [0, pgains[1]]])
    Kp2 = np.array([[pgains[2], 0], [0, pgains[3]]])

    K = np.array([[math.cos(p3dx.pPos.X[5]), (-1)*p3dx.pPar.a*math.sin(p3dx.pPos.X[5])],
                  [math.sin(p3dx.pPos.X[5]), p3dx.pPar.a*math.cos(p3dx.pPos.X[5])]])

    p3dx.pPos.Xtil = p3dx.pPos.Xd - p3dx.pPos.X

    p3dx.pSC.Ur = np.linalg.solve(K, (p3dx.pPos.Xd[6:7] + np.dot(Kp1, np.tanh(np.dot(Kp2, p3dx.pPos.Xtil[0:1])))))

    # Saturação do sinal de controle, baseado na folha de dados do Pioneer 3DX:
    if abs(p3dx.pSC.Ur[0]) > 0.75:
        p3dx.pSC.Ur[0] = np.sign(p3dx.pSC.Ur[0]*0.75)
    elif abs(p3dx.pSC.Ur[1]) > 1:
        p3dx.pSC.Ur[1] = np.sign(p3dx.pSC.Ur[1]*1)

    p3dx.pSC.Ud = p3dx.pSC.Ur
