import numpy as np
import time
import math


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

    def iFlags(self):
        # Flags:
        self.pFlag.Connected = 0
        self.pFlag.JoyON = 0
        self.pFlag.GPS = 0
        self.pFlag.EmergencyStop = 0

    def iParameters(self):
        self.pPar.Model = 'P3DX'    # Robot model

        # Sample time:
        self.pPar.Ts = 0.1    # For numerical integration
        self.pPar.ti = time.time()    # Flag time

        # Dynamic Model Parameters
        self.pPar.g = 9.8    # [kg.m/s^2] Gravitational acceleration

        # [kg]
        self.pPar.m = 0.429    # 0.442

        # [m and rad]
        self.pPar.a = 0.15    # Point of control
        self.pPar.alpha = 0    # Angle of control

        # [Identified Parameters]
        # Reference:
        # Martins, F.N., & Brandão, A.S.(2018).
        # Motion Control and Velocity - Based Dynamic Compensation for Mobile Robots.
        # In Applications of Mobile Robots.IntechOpen.
        # DOI: http://dx.doi.org/10.5772/intechopen.79397

        self.pPar.theta = np.array([[0.5338], [0.2168], [-0.0134], [0.9560], [-0.0843], [1.0590]])

    def get_x(self):
        return 0

    def get_y(self):
        return 0

    def get_h(self):
        return 0

    def get_vel(self):
        return 0

    def get_rotvel(self):
        return 0

    def rGetSensorData(self):
        self.pPos.Xa = self.pPos.X
        if self.pFlag.Connected == 1:
            # MobileSim or Real P3DX
            # Robot pose from ARIA
            self.pPos.Xc[0] = (self.get_x())/1000    # x
            self.pPos.Xc[1] = (self.get_y())/1000    # y
            self.pPos.Xc[5] = (self.get_h())/1000    # psi

            # Robot velocities
            self.pSC.Ua = self.pSC.U
            self.pSC.U[0] = (self.get_vel()) / 1000    # linear
            self.pSC.U[1] = (self.get_rotvel()) / (180*math.pi)    # angular

            K1 = np.array([math.cos(self.pPos.Xc[5]), 0], [math.sin(self.pPos.Xc[5]), 0])
            K2 = np.array([math.cos(self.pPos.Xc[5]), (-1)*self.pPar.a*math.sin(self.pPos.Xc[5])], [math.sin(self.pPos.Xc[5]), self.pPar.a*math.cos(self.pPos.Xc[5])])

            self.pPos.Xc[6] = K1 * self.pSC.U
            self.pPos.Xc[7] = K1 * self.pSC.U
            self.pPos.Xc[11] = self.pSC.U[1]

            # Pose of the Control point
            self.pPos.X[0] = self.pPos.Xc[0] + np.array([self.pPar.a*math.cos(self.pPos.Xc[5])], [self.pPar.a*math.sin(self.pPos.Xc[5])], [0])
            self.pPos.X[1] = self.pPos.Xc[1] + np.array([self.pPar.a * math.cos(self.pPos.Xc[5])], [self.pPar.a * math.sin(self.pPos.Xc[5])], [0])
            self.pPos.X[2] = self.pPos.Xc[2] + np.array([self.pPar.a * math.cos(self.pPos.Xc[5])], [self.pPar.a * math.sin(self.pPos.Xc[5])], [0])

            self.pPos.X[3] = self.pPos.Xc[3]
            self.pPos.X[4] = self.pPos.Xc[4]
            self.pPos.X[5] = self.pPos.Xc[5]

            self.pPos.X[7] = K2 * self.pSC.U
            self.pPos.X[8] = K2 * self.pSC.U

            self.pPos.X[12] = self.pSC.U[2]

        else:
            self.pPos.Xc[0] = self.pPos.X[0] - np.array([self.pPar.a * math.cos(self.pPos.X[6])], [self.pPar.a * math.sin(self.pPos.X[6])], [0])