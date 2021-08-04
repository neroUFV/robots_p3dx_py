import numpy as np


class Pioneer3DX:
    def __init__(self, pCAD, pPar, pID, pPos, pSC, pFlag, pData, pCom):

        # Properties or Parameters:
        self.pCAD = pCAD
        self.pPar = pPar
        self.pID = pID

        # Control variables:
        self.pPos = pPos
        self.pSC = pSC
        self.pFlag = pFlag

        # Navigation Data and Communication:
        self.pData = pData
        self.pCom = pCom

    def iControlVariables(self):

        # Robot pose:

        self.pPos.X = np.zeros(12, 1)   # Current pose (point of control)
        self.pPos.Xa = np.zeros(12, 1)  # Past pose

        self.pPos.Xc = np.zeros(12, 1)  # Current pose (center of the robot)
        self.pPos.Xp = np.zeros(12, 1)  # Current pose (computed by the robot)

        self.pPos.Xd = np.zeros(12, 1)  # Desired pose
        self.pPos.Xda = np.zeros(12, 1)  # Past desired pose

        self.pPos.Xr = np.zeros(12, 1)  # Reference pose
        self.pPos.Xra = np.zeros(12, 1)  # Past reference pose

        # First time derivative:

        self.pPos.dX = np.zeros(12, 1)  # Current pose
        self.pPos.dXd = np.zeros(12, 1)  # Desired pose
        self.pPos.dXr = np.zeros(12, 1)  # Reference pose

        # Pose error:

        self.pPos.Xtil = self.pPos.Xd - self.pPos.X

        # Sensor data:

        self.pPos.Xso = np.zeros(12, 1)  # Initial sensor data
        self.pPos.Xs = np.zeros(12, 1)  # Current sensor data
        self.pPos.Xsa = np.zeros(12, 1)  # Past sensor data
        self.pPos.dXs = np.zeros(12, 1)  # First time derivative of sensor data

        # Sensorial fusion data:

        self.pPos.Xf = np.zeros(12, 1)

        # GPS data: Latitude, Longitude e Altitude:

        self.pPos.Xg = np.zeros(12, 1)

        # Signals of Control:

        # Linear and Angular Velocity:

        self.pSC.U = np.array([[0], [0]])  # Current
        self.pSC.Ua = np.array([[0], [0]])  # Past
        self.pSC.Ud = np.array([[0], [0]])  # Desired
        self.pSC.Uda = np.array([[0], [0]])  # Past desired
        self.pSC.Ur = np.array([[0], [0]])  # Reference
        self.pSC.Kinematics_control = 0

        # Linear and Angular Acceleration:

        self.pSC.dU = np.array([[0], [0]])  # Current
        self.pSC.dUd = np.array([[0], [0]])  # Desired
