""" matrix.py: Matrix class for matrix operations. """
# Built for STEWART FORCE>..

import math
from math import *

import random
import sys
import numpy as np

__version__ = "0.48"
print('Stewart Platform API v0.48\n');


def rotation_z(angleZ):
    sinZ = sin(angleZ)
    cosZ = cos(angleZ)

    rot = np.array([[cosZ, -sinZ, 0], [sinZ, cosZ, 0], [0, 0, 1]])
    return rot

# eqn 2
def rotation(angleX, angleY, angleZ):
    cosX = cos(angleX);
    sinX = sin(angleX);
    cosY = cos(angleY);
    sinY = sin(angleY);
    cosZ = cos(angleZ);
    sinZ = sin(angleZ);
    m_sinX = -sinX;
    m_sinY = -sinY;
    m_sinZ = -sinZ;

    rot = np.array([[cosY * cosZ,           m_sinX * m_sinY * cosZ + cosX * sinZ,           cosX * m_sinY * cosZ + sinX * sinZ,         0],
                    [cosY * m_sinZ,         m_sinX * m_sinY * m_sinZ + cosX * cosZ,         cosX * m_sinY * m_sinZ + sinX * cosZ,       0],
                    [sinY,                  m_sinX * cosY,                                  cosX * cosY,                                0],
                    [0, 0, 0, 1]])
    return rot


def translation(x, y, z):
    trans = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [x, y, z, 1]])
    return trans

def calculate_motor_positions(DXL_ID, angle):
    
    pi = math.pi
    scale_factor = 2048 / pi

    angle_code = 2048

    if DXL_ID % 2 == 0:
        angle_code = 2048 + (angle * scale_factor)
    else: 
        angle_code = 2048 - (angle * scale_factor)

    angle_code %= 4096
    angle_code = round(angle_code)
    
    return angle_code


class MatrixError(Exception):
    """ An exception class for Matrix """
    pass


class Matrix(object):
    """ A simple Python matrix class with
    basic operations and operator overloading """

    def __init__(self, m, n, init=True):
        if init:
            self.rows = [[0] * n for x in range(m)]
        else:
            self.rows = []
        self.m = m
        self.n = n

    def __getitem__(self, idx):
        return self.rows[idx]

    def __setitem__(self, idx, item):
        self.rows[idx] = item

    def __str__(self):
        s = '\n'.join([' '.join([str(item) for item in row]) for row in self.rows])
        return s + '\n'

    def __repr__(self):
        s = str(self.rows)
        rank = str(self.getRank())
        rep = "Matrix: \"%s\", rank: \"%s\"" % (s, rank)
        return rep

    def reset(self):
        """ Reset the matrix data """
        self.rows = [[] for x in range(self.m)]
        return self;

    def transpose(self):
        """ Transpose the matrix. Changes the current matrix """

        self.m, self.n = self.n, self.m
        self.rows = [list(item) for item in zip(*self.rows)]
        return self;

    def getTranspose(self):
        """ Return a transpose of the matrix without
        modifying the matrix itself """

        m, n = self.n, self.m
        mat = Matrix(m, n)
        mat.rows = [list(item) for item in zip(*self.rows)]

        return mat

    def getRank(self):
        return (self.m, self.n)

    def __eq__(self, mat):
        """ Test equality """

        return (mat.rows == self.rows)

    def __add__(self, mat):
        """ Add a matrix to this matrix and
        return the new matrix. Doesn't modify
        the current matrix """

        if self.getRank() != mat.getRank():
            raise MatrixError("Trying to add matrixes of varying rank!")

        ret = Matrix(self.m, self.n)

        for x in range(self.m):
            row = [sum(item) for item in zip(self.rows[x], mat[x])]
            ret[x] = row

        return ret

    def __sub__(self, mat):
        """ Subtract a matrix from this matrix and
        return the new matrix. Doesn't modify
        the current matrix """

        if self.getRank() != mat.getRank():
            raise MatrixError("Trying to add matrixes of varying rank!")

        ret = Matrix(self.m, self.n)

        for x in range(self.m):
            row = [item[0] - item[1] for item in zip(self.rows[x], mat[x])]
            ret[x] = row

        return ret

    def __mul__(self, mat):
        """ Multiple a matrix with this matrix and
        return the new matrix. Doesn't modify
        the current matrix """

        matm, matn = mat.getRank()

        if (self.n != mat.m):
            raise MatrixError("Matrices cannot be multipled!")

        mat_t = mat.getTranspose()
        mulmat = Matrix(self.m, matn)

        for x in range(self.m):
            for y in range(mat_t.m):
                mulmat[x][y] = sum([item[0] * item[1] for item in zip(self.rows[x], mat_t[y])])

        return mulmat

    def __iadd__(self, mat):
        """ Add a matrix to this matrix.
        This modifies the current matrix """

        # Calls __add__
        tempmat = self + mat
        self.rows = tempmat.rows[:]
        return self

    def __isub__(self, mat):
        """ Add a matrix to this matrix.
        This modifies the current matrix """

        # Calls __sub__
        tempmat = self - mat
        self.rows = tempmat.rows[:]
        return self

    def __imul__(self, mat):
        """ Add a matrix to this matrix.
        This modifies the current matrix """

        # Possibly not a proper operation
        # since this changes the current matrix
        # rank as well...

        # Calls __mul__
        tempmat = self * mat
        self.rows = tempmat.rows[:]
        self.m, self.n = tempmat.getRank()
        return self

    def save(self, filename):
        open(filename, 'w').write(str(self))

    @classmethod
    def _makeMatrix(cls, rows):

        m = len(rows)
        n = len(rows[0])
        # Validity check
        if any([len(row) != n for row in rows[1:]]):
            raise MatrixError("inconsistent row length")
        mat = Matrix(m, n, init=False)
        mat.rows = rows

        return mat

    @classmethod
    def makeRandom(cls, m, n, low=0, high=10):
        """ Make a random matrix with elements in range (low-high) """

        obj = Matrix(m, n, init=False)
        for x in range(m):
            obj.rows.append([random.randrange(low, high) for i in range(obj.n)])

        return obj

    @classmethod
    def makeZero(cls, m, n):
        """ Make a zero-matrix of rank (mxn) """

        rows = [[0] * n for x in range(m)]
        return cls.fromList(rows)

    @classmethod
    def makeDiagonal(cls, m, n):
        """ Make a diagonal matrix (mxn) """
        if m != n:
            raise MatrixError("Only sqaure matrices can be made diagonal")

        rows = cls.makeZero(m, n)
        for i in range(m): rows[i][i] = 1
        return cls.fromList(rows)

    @classmethod
    def makeId(cls, m):
        """ Make identity matrix of rank (mxm) """

        rows = [[0] * m for x in range(m)]
        idx = 0

        for row in rows:
            row[idx] = 1
            idx += 1

        return cls.fromList(rows)

    @classmethod
    def readStdin(cls):
        """ Read a matrix from standard input """

        print('Enter matrix row by row. Type "q" to quit')
        rows = []
        while True:
            line = sys.stdin.readline().strip()
            if line == 'q': break

            row = [int(x) for x in line.split()]
            rows.append(row)

        return cls._makeMatrix(rows)

    @classmethod
    def readGrid(cls, fname):
        """ Read a matrix from a file """

        rows = []
        for line in open(fname).readlines():
            row = [int(x) for x in line.split()]
            rows.append(row)

        return cls._makeMatrix(rows)

    @classmethod
    def fromList(cls, listoflists):
        """ Create a matrix by directly passing a list
        of lists """

        # E.g: Matrix.fromList([[1 2 3], [4,5,6], [7,8,9]])

        rows = listoflists[:]
        return cls._makeMatrix(rows)
    

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

import sys
sys.path.append('/media/shashank/New Volume1/Aduza_IISc_package/Robotis/DynamixelSDK-master/python/src')

from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import time                                    # time for delay


# Control table address
ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_TORQUE_MAX         = 34

# Data Byte Length
LEN_MX_GOAL_POSITION       = 4
LEN_MX_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
DXL3_ID                     = 3                 # Dynamixel#1 ID: 3
DXL4_ID                     = 4                 # Dynamixel#2 ID: 4
DXL5_ID                     = 5                 # Dynamixel#1 ID: 5
DXL6_ID                     = 6                 # Dynamixel#2 ID: 6
BAUDRATE                    = 1000000           # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
TORQUE_MAX                  = 150              # Max Torque Value
DXL_MINIMUM_POSITION_VALUE  = 0                 # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4000              # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

from math import *
import numpy as np

def rad(deg):
    return deg / 180.0 * pi


def deg(rad):
    return rad * 180.0 / pi


class Robot:
    motorH = 102.30
    HOME_Z = 295.92
    RELATIVE = 'rel'
    ABSOLUTE = 'abs'
    x, y, z, roll, pitch, yaw = 0, 0, HOME_Z, 0, 0, 0

    LINK_1 = 80.0
    LINK_2 = 220.0

    DEFAULT_LINK_2_CORR = np.array([0, 0, 0, 0, 0, 0])

    # hs = platform
    platform = np.array([[23.56, 141.78, 0, 1],
                         [-23.56, 141.78, 0, 1],
                         [-134.57, -50.49, 0, 1],
                         [-110.77, -91.59, 0, 1],
                         [110.77, -91.59, 0, 1],
                         [134.57, -50.49, 0, 1]])
    # sh = base
    base = np.array([[58.66, 144.14, 0, 1],
                     [-58.66, 144.14, 0, 1],
                     [-154.15, -21.27, 0, 1],
                     [-95.5, -122.86, 0, 1],
                     [95.5, -122.86, 0, 1],
                     [154.15, -21.27, 0, 1]])

    def __init__(self, port=None, LINK_1=80.0, LINK_2=220.0, link_1_correction=np.array([0, 0, 0, 0, 0, 0]),
                 link_2_correction=DEFAULT_LINK_2_CORR):
        #self.actuators = Actuators(port)

        self.LINK_1 = LINK_1
        self.LINK_2 = LINK_2

        self.link_1_correction = link_1_correction
        self.link_2_correction = link_2_correction

        self.tool_x = self.tool_y = self.tool_z = 0
        self.tool_roll = self.tool_pitch = self.tool_yaw = 0


    def reset(self):
        self.goto()

    def reset_cycloidal(self):
        self.move_cycloidal()

    #def gripper(self, state=False):
     #   self.actuators.set_goal_position_single(7, (130 if state else 95)) #130 close, 95#open

    def tool(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
        self.tool_x, self.tool_y, self.tool_z = x, y, z
        self.tool_roll, self.tool_pitch, self.tool_yaw = roll, pitch, yaw

    valx = float(input("Enter x co-ordinate: "))
    valy= float(input("Enter y co-ordinate: "))
    valz = float(input("Enter z co-ordinate: "))
    valroll = float(input("Enter roll: "))
    valpitch = float(input("Enter pitch: "))
    valyaw = float(input("Enter yaw: "))

    def goto(self, x=valx, y=valy, z=HOME_Z+valz, roll=valroll, pitch=valpitch, yaw=valyaw):
        self.x, self.y, self.z, self.roll, self.pitch, self.yaw = x, y, z, roll, pitch, yaw
        # Update Geometry (as per as stewart dimensions)
        link1 = np.array([self.LINK_1 + self.link_1_correction[i] for i in range(6)])
        link2 = np.array([self.LINK_2 + self.link_2_correction[i] for i in range(6)])

        print("Link_1 =",link1)
        print("Link_2 =",link2)

        hs = self.platform
        sh = self.base

        # Applying transformation
        hs = np.matmul(hs, translation(-self.tool_x, -self.tool_y, -self.tool_z))    #ln 401 * ln 42, [6*4]*[4*4]=[6*4 matrix]
        print("Translation_-tool = ",'\n',translation(-self.tool_x, -self.tool_y, -self.tool_z))
        print("HS1",'\n',hs)
        print('\n')

        hs = np.matmul(hs, rotation(rad(self.roll), rad(self.pitch), rad(self.yaw))) #ln 401 * ln 24
        print("Rotation = ",'\n',rotation(rad(self.roll), rad(self.pitch), rad(self.yaw)))
        print("HS2",'\n',hs)
        print('\n')

        hs = np.matmul(hs, translation(self.tool_x, self.tool_y, self.tool_z))
        print("Translation_tool = ",'\n',translation(self.tool_x, self.tool_y, self.tool_z))
        print("HS3",'\n',hs)
        print('\n')

        hs = np.matmul(hs, translation(self.x, self.y, self.z - self.motorH))
        print("Translation_motorH = ",'\n',translation(self.x, self.y, self.z - self.motorH))
        print("HS4",'\n',hs)
        print('\n')

        # iKin
        l1o = np.array([[hs[0][0] - sh[0][0]], [hs[0][1] - sh[0][1]], [hs[0][2] - sh[0][2]]])
        l2o = np.array([[hs[1][0] - sh[1][0]], [hs[1][1] - sh[1][1]], [hs[1][2] - sh[1][2]]])
        l3o = np.array([[hs[2][0] - sh[2][0]], [hs[2][1] - sh[2][1]], [hs[2][2] - sh[2][2]]])
        l4o = np.array([[hs[3][0] - sh[3][0]], [hs[3][1] - sh[3][1]], [hs[3][2] - sh[3][2]]])
        l5o = np.array([[hs[4][0] - sh[4][0]], [hs[4][1] - sh[4][1]], [hs[4][2] - sh[4][2]]])
        l6o = np.array([[hs[5][0] - sh[5][0]], [hs[5][1] - sh[5][1]], [hs[5][2] - sh[5][2]]])

        print('\n')
        print("1@Pi-Bi , l1o =",'\n',l1o)
        print("2@Pi-Bi , l2o=",'\n',l2o)
        print("3@Pi-Bi , l3o=",'\n',l3o)
        print("4@Pi-Bi , l4o=",'\n',l4o)
        print("5@Pi-Bi , l5o=",'\n',l5o)
        print("6@Pi-Bi , l6o=",'\n',l6o)


        # eqn 3
        '''why .transpose because each row in l1o vector represents x,y,z 
        each column in roational matrix represents x-projection on X,Y,Z axis
        so, when transofrmed , matrix multiplication will be in correct form ''' 

        # Apply rotations using transposed rotation matrices for efficient dot product (column as direction)
        l1 = np.matmul(rotation_z(rad(-150)).transpose(), l1o) #(.transpose is used because Li is expressed in terms of Bi i.e. inverse)
        
        print("Z-axis rotation =",'\n',rotation_z(rad(-150)))
        print("transposed matrix = ",'\n',rotation_z(rad(-150)).transpose())

        l2 = np.matmul(rotation_z(rad(150)).transpose(), l2o)         
        l3 = np.matmul(rotation_z(rad(-30)).transpose(), l3o)
        l4 = np.matmul(rotation_z(rad(-90)).transpose(), l4o)
        l5 = np.matmul(rotation_z(rad(90)).transpose(), l5o)
        l6 = np.matmul(rotation_z(rad(30)).transpose(), l6o)

        # l11 = np.matmul(rotation_z(rad(-150)), l1o) # matmul without tranformation
        
        print('\n')
        print("l1 =",'\n',l1)
        print("l2 =",'\n',l2)
        print("l3 =",'\n',l3)
        print("l4 =",'\n',l4)
        print("l5 =",'\n',l5)
        print("l6 =",'\n',l6)

        # print("l11 = ",'\n',l11)


        # eqn 4 ie., θ3
        thetav1 = asin(l1[0][0] / link2[0])
        thetav2 = asin(l2[0][0] / link2[1])
        thetav3 = asin(l3[0][0] / link2[2])
        thetav4 = asin(l4[0][0] / link2[3])
        thetav5 = asin(l5[0][0] / link2[4])
        thetav6 = asin(l6[0][0] / link2[5])

        '''print("θ31 =",thetav1)
        print("θ32 =",thetav2)
        print("θ33 =",thetav3)
        print("θ34 =",thetav4)
        print("θ35 =",thetav5)
        print("θ36 =",thetav6)'''

        # eqn 6
        theta21 = acos((l1[1][0] ** 2 + l1[2][0] ** 2 - link1[0] ** 2 - (link2[0] * cos(thetav1)) ** 2) / (
                    2 * link1[0] * link2[0] * cos(thetav1)));
        theta22 = acos((l2[1][0] ** 2 + l2[2][0] ** 2 - link1[1] ** 2 - (link2[1] * cos(thetav2)) ** 2) / (
                    2 * link1[1] * link2[1] * cos(thetav2)));
        theta23 = acos((l3[1][0] ** 2 + l3[2][0] ** 2 - link1[2] ** 2 - (link2[2] * cos(thetav3)) ** 2) / (
                    2 * link1[2] * link2[2] * cos(thetav3)));
        theta24 = acos((l4[1][0] ** 2 + l4[2][0] ** 2 - link1[3] ** 2 - (link2[3] * cos(thetav4)) ** 2) / (
                    2 * link1[3] * link2[3] * cos(thetav4)));
        theta25 = acos((l5[1][0] ** 2 + l5[2][0] ** 2 - link1[4] ** 2 - (link2[4] * cos(thetav5)) ** 2) / (
                    2 * link1[4] * link2[4] * cos(thetav5)));
        theta26 = acos((l6[1][0] ** 2 + l6[2][0] ** 2 - link1[5] ** 2 - (link2[5] * cos(thetav6)) ** 2) / (
                    2 * link1[5] * link2[5] * cos(thetav6)));
        
        '''print('\n')
        print("θ21 =",theta21)
        print("θ22 =",theta22)
        print("θ23 =",theta23)
        print("θ24 =",theta24)
        print("θ25 =",theta25)
        print("θ26 =",theta26)'''

        # eqn 8a
        A1 = sqrt(
            (link1[0] + link2[0] * cos(thetav1) * cos(theta21)) ** 2 + (link2[0] * cos(thetav1) * sin(theta21)) ** 2);
        A2 = sqrt(
            (link1[1] + link2[1] * cos(thetav2) * cos(theta22)) ** 2 + (link2[1] * cos(thetav2) * sin(theta22)) ** 2);
        A3 = sqrt(
            (link1[2] + link2[2] * cos(thetav3) * cos(theta23)) ** 2 + (link2[2] * cos(thetav3) * sin(theta23)) ** 2);
        A4 = sqrt(
            (link1[3] + link2[3] * cos(thetav4) * cos(theta24)) ** 2 + (link2[3] * cos(thetav4) * sin(theta24)) ** 2);
        A5 = sqrt(
            (link1[4] + link2[4] * cos(thetav5) * cos(theta25)) ** 2 + (link2[4] * cos(thetav5) * sin(theta25)) ** 2);
        A6 = sqrt(
            (link1[5] + link2[5] * cos(thetav6) * cos(theta26)) ** 2 + (link2[5] * cos(thetav6) * sin(theta26)) ** 2);

        # eqn 8b
        psi1 = atan2((link2[0] * cos(thetav1) * sin(theta21)), (link1[0] + link2[0] * cos(thetav1) * cos(theta21)));
        psi2 = atan2((link2[1] * cos(thetav2) * sin(theta22)), (link1[1] + link2[1] * cos(thetav2) * cos(theta22)));
        psi3 = atan2((link2[2] * cos(thetav3) * sin(theta23)), (link1[2] + link2[2] * cos(thetav3) * cos(theta23)));
        psi4 = atan2((link2[3] * cos(thetav4) * sin(theta24)), (link1[3] + link2[3] * cos(thetav4) * cos(theta24)));
        psi5 = atan2((link2[4] * cos(thetav5) * sin(theta25)), (link1[4] + link2[4] * cos(thetav5) * cos(theta25)));
        psi6 = atan2((link2[5] * cos(thetav6) * sin(theta26)), (link1[5] + link2[5] * cos(thetav6) * cos(theta26)));

        # eqn 9
        self.theta11 = acos(l1[1][0] / A1) - psi1;
        self.theta12 = acos(l2[1][0] / A2) - psi2;
        self.theta13 = acos(l3[1][0] / A3) - psi3;
        self.theta14 = acos(l4[1][0] / A4) - psi4;
        self.theta15 = acos(l5[1][0] / A5) - psi5;
        self.theta16 = acos(l6[1][0] / A6) - psi6;

        print('\n')
        print("θ11 =",self.theta11)
        print("θ12 =",self.theta12)
        print("θ13 =",self.theta13)
        print("θ14 =",self.theta14)
        print("θ15 =",self.theta15)
        print("θ16 =",self.theta16)

        '''print('\n')
        print("θ31 =",thetav1,'\t',"θ21 =",theta21,'\t',"θ11 =",self.theta11)
        print("θ32 =",thetav2,'\t',"θ22 =",theta22,'\t',"θ12 =",self.theta12)
        print("θ33 =",thetav3,'\t',"θ23 =",theta23,'\t',"θ13 =",self.theta13)
        print("θ34 =",thetav4,'\t',"θ24 =",theta24,'\t',"θ14 =",self.theta14)
        print("θ35 =",thetav5,'\t',"θ25 =",theta25,'\t',"θ15 =",self.theta15)
        print("θ36 =",thetav6,'\t',"θ26 =",theta26,'\t',"θ16 =",self.theta16)'''

        motor1_position = calculate_motor_positions(DXL1_ID, self.theta11)
        motor2_position = calculate_motor_positions(DXL2_ID, self.theta12)
        motor3_position = calculate_motor_positions(DXL3_ID, self.theta13)
        motor4_position = calculate_motor_positions(DXL4_ID, self.theta14)
        motor5_position = calculate_motor_positions(DXL5_ID, self.theta15)
        motor6_position = calculate_motor_positions(DXL6_ID, self.theta16)

        print('\n')
        print("Motor1 =",motor1_position)
        print("Motor2 =",motor2_position)
        print("Motor3 =",motor3_position)
        print("Motor4 =",motor4_position)
        print("Motor5 =",motor5_position)
        print("Motor6 =",motor6_position)

        dxl1_goal_position = motor1_position
        dxl2_goal_position = motor2_position
        dxl3_goal_position = motor3_position
        dxl4_goal_position = motor4_position
        dxl5_goal_position = motor5_position
        dxl6_goal_position = motor6_position

        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        portHandler = PortHandler(DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Initialize GroupSyncWrite instance
        groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)

        # Open port
        if portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()


        # Set port baudrate
        if portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()


        # Enable Dynamixel#1 Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL1_ID)

        # Enable Dynamixel#2 Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL2_ID)

        # Enable Dynamixel#3 Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL3_ID)

        # Enable Dynamixel#4 Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL4_ID)

        # Enable Dynamixel#5 Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL5_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL6_ID)

        # Enable Dynamixel#6 Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL6_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel#%d has been successfully connected" % DXL6_ID)

        

        # Limit Dynamixel#1 Torque
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_MAX, TORQUE_MAX)
        # Limit Dynamixel#2 Torque
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_MAX, TORQUE_MAX)
        # Limit Dynamixel#3 Torque
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_MAX, TORQUE_MAX)
        # Limit Dynamixel#4 Torque
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL4_ID, ADDR_MX_TORQUE_MAX, TORQUE_MAX)
        # Limit Dynamixel#5 Torque
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL5_ID, ADDR_MX_TORQUE_MAX, TORQUE_MAX)
        # Limit Dynamixel#6 Torque
        dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL6_ID, ADDR_MX_TORQUE_MAX, TORQUE_MAX)
        


        param1_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl1_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl1_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl1_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl1_goal_position))]
        param2_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl2_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl2_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl2_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl2_goal_position))]
        param3_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl3_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl3_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl3_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl3_goal_position))]
        param4_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl4_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl4_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl4_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl4_goal_position))]
        param5_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl5_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl5_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl5_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl5_goal_position))]
        param6_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl6_goal_position)), DXL_HIBYTE(DXL_LOWORD(dxl6_goal_position)), DXL_LOBYTE(DXL_HIWORD(dxl6_goal_position)), DXL_HIBYTE(DXL_HIWORD(dxl6_goal_position))]

        # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param1_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
            quit()

        # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param2_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
            quit()

        # Add Dynamixel#3 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, param3_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL3_ID)
            quit()
        
        # Add Dynamixel#4 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL4_ID, param4_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL4_ID)
            quit()

        # Add Dynamixel#5 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL5_ID, param5_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL5_ID)
            quit()

        # Add Dynamixel#6 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL6_ID, param6_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL6_ID)
            quit()


        # Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()

        while 1:
            # Read Dynamixel#1 present position
            dxl1_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))

            # Read Dynamixel#2 present position
            dxl2_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))

            # Read Dynamixel#3 present position
            dxl3_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_MX_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))

            # Read Dynamixel#4 present position
            dxl4_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL4_ID, ADDR_MX_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))

            # Read Dynamixel#5 present position
            
            dxl5_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL5_ID, ADDR_MX_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            
            # Read Dynamixel#6 present position
            dxl6_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL6_ID, ADDR_MX_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))

            print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL1_ID, dxl1_goal_position, dxl1_present_position, DXL2_ID, dxl2_goal_position, dxl2_present_position))
            print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL3_ID, dxl3_goal_position, dxl3_present_position, DXL4_ID, dxl4_goal_position, dxl4_present_position))
            print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL5_ID, dxl5_goal_position, dxl5_present_position, DXL6_ID, dxl6_goal_position, dxl6_present_position))

            if not ((abs(dxl1_goal_position - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) and (abs(dxl2_goal_position - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD) and (abs(dxl3_goal_position - dxl3_present_position) > DXL_MOVING_STATUS_THRESHOLD) and (abs(dxl4_goal_position - dxl4_present_position) > DXL_MOVING_STATUS_THRESHOLD) and (abs(dxl5_goal_position - dxl5_present_position) > DXL_MOVING_STATUS_THRESHOLD) and (abs(dxl6_goal_position - dxl6_present_position) > DXL_MOVING_STATUS_THRESHOLD)):
                break



    def move(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
        self.goto(self.x+x, self.y+y, self.z+z, self.roll+roll, self.pitch+pitch, self.yaw+yaw)

    def move_relative(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0, T=2, steps=100):
        dx, dy, dz, droll, dpitch, dyaw = x / steps, y / steps, z / steps, roll / steps, pitch / steps, yaw / steps
        for i in (range(steps)):
            self.move(x=dx, y=dy, z=dz, roll=droll, pitch=dpitch, yaw=dyaw)
            #wait(T / steps)



robot = Robot()

robot.goto()
