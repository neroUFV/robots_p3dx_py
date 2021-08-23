import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import time
import math


class Pioneer3DX:
    def __init__(self, pFlag=0, pID=0):
        # nargin: quantos argumentos foram passados (*kwargs)
        # Tirar parametros de definicao da classe e chamar as funções

        # Properties or Parameters:
        self.pID = pID
        self.pPar = pPar()

        # Control variables:
        self.pPos = pPos()
        self.pSC = pSC()
        self.pFlag = pFlag

        # Simulation:
        triangle_points = np.array([[0, 2 * (np.sqrt(1 ** 2 - 0.5 ** 2)) / 3],
                                    [-0.5, -(np.sqrt(1 ** 2 - 0.5 ** 2)) / 3],
                                    [0.5, -(np.sqrt(1 ** 2 - 0.5 ** 2)) / 3]])

        self.shape = Polygon(triangle_points, closed=False)

        self.iParameters()
        self.iControlVariables()

    def iControlVariables(self):
        # Robot pose:
        self.pPos.X = np.zeros((12, 1))   # Current pose (point of control)
        self.pPos.Xa = np.zeros((12, 1))  # Past pose

        self.pPos.Xc = np.zeros((12, 1))  # Current pose (center of the robot)
        # self.pPos.Xp = np.zeros((12, 1))  # Current pose (computed by the robot)

        self.pPos.Xd = np.zeros((12, 1))  # Desired pose
        self.pPos.Xda = np.zeros((12, 1))  # Past desired pose

        self.pPos.Xr = np.zeros((12, 1))  # Reference pose
        self.pPos.Xra = np.zeros((12, 1))  # Past reference pose

        # First time derivative:
        self.pPos.dX = np.zeros((12, 1))  # Current pose
        self.pPos.dXd = np.zeros((12, 1))  # Desired pose
        self.pPos.dXr = np.zeros((12, 1))  # Reference pose

        # Pose error:
        self.pPos.Xtil = self.pPos.Xd - self.pPos.X

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

        # Signals of Control:

        # Linear and Angular Velocity:
        self.pSC.U = np.array([[0], [0]])  # Current
        self.pSC.Ua = np.array([[0], [0]])  # Past
        self.pSC.Ud = np.array([[0], [0]])  # Desired
        self.pSC.Uda = np.array([[0], [0]])  # Past desired
        self.pSC.Ur = np.array([[0], [0]])  # Reference
        # self.pSC.Kinematics_control = 0

        # Linear and Angular Acceleration:
        self.pSC.dU = np.array([[0], [0]])  # Current
        self.pSC.dUd = np.array([[0], [0]])  # Desired

    def iParameters(self):
        self.pPar.Model = 'P3DX'    # Robot model

        # Sample time:
        self.pPar.Ts = 0.1    # For numerical integration
        # self.pPar.ti = time.time()    # Flag time

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
        # X(0)
        return self.pPos.X[0]

    def get_y(self):
        # X(1)
        return self.pPos.X[1]

    def get_h(self):
        # X(3)
        return self.pPos.X[3]

    def get_vel(self):
        # Vel. Linear norma p.pPos.X 7,8,9
        return math.sqrt((self.pPos.X[6])**2 + (self.pPos.X[7])**2 + (self.pPos.X[8])**2)

    def get_rotvel(self):
        # Vel. Angular norma p.pPos.X 10,11,12
        return math.sqrt((self.pPos.X[9])**2 + (self.pPos.X[10])**2 + (self.pPos.X[11])**2)

    def set_vel(self, vel):
        # Definir a velocidade angular por p.pSC.U(2)
        self.pSC.U[0] = vel

    def set_rotvel(self, rotvel):
        # Definir a velocidade angular por p.pSC.U(2)
        self.pSC.U[1] = rotvel

    def set_pose(self, pose):
        # P.pPos.X como parâmetro, P.pPos.X é um estado
        self.pPos.X = pose

    def rGetSensorData(self):
        self.pPos.Xa = self.pPos.X

        # Tava nesse if: "if self.pFlag.Connected == 1:"
        # MobileSim or Real P3DX
        # Robot pose from ARIA
        self.pPos.Xc[0] = (self.get_x())/1000    # x
        self.pPos.Xc[1] = (self.get_y())/1000    # y
        self.pPos.Xc[5] = (self.get_h())/1000    # psi

        # Robot velocities
        self.pSC.Ua = self.pSC.U
        self.pSC.U[0] = (self.get_vel()) / 1000    # linear
        self.pSC.U[1] = (self.get_rotvel()) / (180*math.pi)    # angular

        K1 = np.array([[math.cos(self.pPos.Xc[5]), 0], [math.sin(self.pPos.Xc[5]), 0]])
        K2 = np.array([[math.cos(self.pPos.Xc[5]), (-1)*self.pPar.a*math.sin(self.pPos.Xc[5])], [math.sin(self.pPos.Xc[5]), self.pPar.a*math.cos(self.pPos.Xc[5])]])

        self.pPos.Xc[6] = K1 * self.pSC.U
        self.pPos.Xc[7] = K1 * self.pSC.U
        self.pPos.Xc[11] = self.pSC.U[1]

        # Pose of the Control point (matriz de rotação em torno de z) r.set_pose
        self.pPos.X[0] = self.pPos.Xc[0] + np.array([[self.pPar.a*math.cos(self.pPos.Xc[5])], [self.pPar.a*math.sin(self.pPos.Xc[5])], [0]])
        self.pPos.X[1] = self.pPos.Xc[1] + np.array([[self.pPar.a * math.cos(self.pPos.Xc[5])], [self.pPar.a * math.sin(self.pPos.Xc[5])], [0]])
        self.pPos.X[2] = self.pPos.Xc[2] + np.array([[self.pPar.a * math.cos(self.pPos.Xc[5])], [self.pPar.a * math.sin(self.pPos.Xc[5])], [0]])

        self.pPos.X[3] = self.pPos.Xc[3]
        self.pPos.X[4] = self.pPos.Xc[4]
        self.pPos.X[5] = self.pPos.Xc[5]

        self.pPos.X[7] = K2 * self.pSC.U
        self.pPos.X[8] = K2 * self.pSC.U

        self.pPos.X[12] = self.pSC.U[2]

        # Tava em um else:
        self.pPos.Xc[0] = self.pPos.X[0] - np.array([[self.pPar.a * math.cos(self.pPos.X[6])], [self.pPar.a * math.sin(self.pPos.X[6])], [0]])

    def rSendControlSignals(self):
        # Tava dentro de um if: "if self.pFlag.Connected == 1:"
        # Experiment Mode: MobileSim or P3DX
        # self.set_vel((self.pSC.Ud[0])*1000)    # Linear velocity
        # self.set_rotvel((self.pSC.Ud[1])/(math.pi*180))    # Angular velocity

        # Stand-by mode:
        self.pSC.Ud = np.array([[0],[0]])

        # Tava dentro de um else:
        # Simulation Mode
        self.pSC.U = self.pSC.Ud
        self.sKinematicModel()    # Modify it by the dynamic model

    def rSetPose(self, Xo):
        # nargin? fazer o nargin aqui #xo ponto inicial onde o robo esta nesse momento
        # (matriz de rotação em torno de z) r.set_pose
        self.pPos.X[0] = self.pPos.Xc[0] + np.array([[math.cos(self.pPos.X[5]), (-1)*math.sin(self.pPos.X[5]), 0, 0], [math.sin(self.pPos.X[5]), math.cos(self.pPos.X[5]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])*np.array([[self.pPar.a*math.cos(self.pPar.alpha)], [self.pPar.a*math.sin(self.pPar.alpha)], [0], [0]])
        self.pPos.X[1] = self.pPos.Xc[1] + np.array([[math.cos(self.pPos.X[5]), (-1) * math.sin(self.pPos.X[5]), 0, 0],
                                                     [math.sin(self.pPos.X[5]), math.cos(self.pPos.X[5]), 0, 0],
                                                     [0, 0, 1, 0], [0, 0, 0, 1]]) * np.array(
            [[self.pPar.a * math.cos(self.pPar.alpha)], [self.pPar.a * math.sin(self.pPar.alpha)], [0], [0]])
        self.pPos.X[2] = self.pPos.Xc[2] + np.array([[math.cos(self.pPos.X[5]), (-1) * math.sin(self.pPos.X[5]), 0, 0],
                                                     [math.sin(self.pPos.X[5]), math.cos(self.pPos.X[5]), 0, 0],
                                                     [0, 0, 1, 0], [0, 0, 0, 1]]) * np.array(
            [[self.pPar.a * math.cos(self.pPar.alpha)], [self.pPar.a * math.sin(self.pPar.alpha)], [0], [0]])
        self.pPos.X[5] = self.pPos.Xc[5] + np.array([[math.cos(self.pPos.X[5]), (-1) * math.sin(self.pPos.X[5]), 0, 0],
                                                     [math.sin(self.pPos.X[5]), math.cos(self.pPos.X[5]), 0, 0],
                                                     [0, 0, 1, 0], [0, 0, 0, 1]]) * np.array(
            [[self.pPar.a * math.cos(self.pPar.alpha)], [self.pPar.a * math.sin(self.pPar.alpha)], [0], [0]])

        self.pPos.Xa = self.pPos.X

        # Tava dentro de um if: "if self.pFlag.Conected == 1:"
        # The position is given in milimeters and the heading in degrees:
        # self.set_pose(self.pPos.Xc[0]*1000, self.pPos.Xc[1]*1000, self.pPos.Xc[5]*(180/math.pi))
        self.set_pose()

    def sInvKinematicModel(self, dXr):
        # Verify vector length:
        l = len(dXr)

        # Inverse Kinematic Matrix (2D)
        if l == 2:
            Kinv = np.array([[math.cos(self.pPos.X[5]), math.sin(self.pPos.X[6])], [(-1) * math.sin(self.pPos.X[5]) / self.pPar.a, math.cos(self.pPos.X[5]) / self.pPar.a]])

        # Inverse Kinematic Matrix (3D)
        elif l == 3:
            Kinv = np.array([[math.cos(self.pPos.X[5]), math.sin(self.pPos.X[6]), 0], [(-1) * math.sin(self.pPos.X[5]) / self.pPar.a, math.cos(self.pPos.X[5]) / self.pPar.a, 0], [0, 0, 0], [0, 0, 0]])

        else:
            print('Invalid vector length (please verify dXr).')
            Kinv = 0

        self.pSC.Ur = Kinv*dXr

    def sKinematicModel(self):
        K = np.array([[math.cos(self.pPos.X[6]), (-1)*self.pPar.a*math.sin(self.pPos.X[5] + self.pPar.alpha)], [math.sin(self.pPos.X[5]), self.pPar.a*math.cos(self.pPos.X[5] + self.pPar.alpha)], [0, 1]])

        # Current position
        self.pPos.X[0] = self.pPos.X[0] + K*np.array([[self.pSC.U[0]], [self.pSC.U[1]]])*self.pPar.Ts
        self.pPos.X[1] = self.pPos.X[1] + K * np.array([[self.pSC.U[0]], [self.pSC.U[1]]]) * self.pPar.Ts
        self.pPos.X[5] = self.pPos.X[5] + K * np.array([[self.pSC.U[0]], [self.pSC.U[1]]]) * self.pPar.Ts

        # first-time derivative of the current position
        self.pPos.X[6] = K*np.array([[self.pSC.U[0]], [self.pSC.U[1]]])
        self.pPos.X[7] = K * np.array([[self.pSC.U[0]], [self.pSC.U[1]]])
        self.pPos.X[11] = K * np.array([[self.pSC.U[0]], [self.pSC.U[1]]])

        # Angle limitation per quadrant:
        for i in range(3, 6):
            if abs(self.pPos.X[i]) > math.pi:
                if self.pPos.X[i] < 0:
                    self.pPos.X[i] = self.pPos.X[i] + 2*math.pi
                else:
                    self.pPos.X[i] = self.pPos.X[i] - 2*math.pi

        # Pose of the robot's center:
        self.pPos.Xc[0] = self.pPos.X[0] - np.array([[math.cos(self.pPos.X[5]), (-1)*math.sin(self.pPos.X[5]), 0], [math.sin(self.pPos.X[5]), math.cos(self.pPos.X[5]), 0], [0, 0, 1]])*np.array([[self.pPar.a*math.cos(self.pPar.alpha)], [self.pPar.a*math.sin(self.pPar.alpha)], [0]])


    def Simulate(self):
        fig_1 = plt.figure(1, figsize=[5, 5])
        self.shape.set_color('r')
        grid_1 = fig_1.add_subplot()
        grid_1.add_patch(self.shape)
        plt.xlabel("x")
        plt.ylabel("y")
        plt.axis([-5, 5, -5, 5])
        plt.grid()
        plt.title("Pioneer P3DX - Simulator")
        plt.show()


class pPar:
    pass


class pPos:
    pass


class pSC:
    pass
