from __future__ import print_function

import datetime

from numpy import linalg,array,amax,amin,add,divide,multiply,subtract,power,Inf,matrix,tile,bmat,random,arange,ones,zeros,eye,squeeze
import smbus
import math
import time
import sys
import rclpy
from rclpy.node import Node

class HMC5883L:
    __scales = {
        0.88: [0, 0.73],
        1.30: [1, 0.92],
        1.90: [2, 1.22],
        2.50: [3, 1.52],
        4.00: [4, 2.27],
        4.70: [5, 2.56],
        5.60: [6, 3.03],
        8.10: [7, 4.35],
    }

    def __init__(self, port=0, address=0x1E, gauss=1.3):
        self.bus = smbus.SMBus(port)
        self.address = address

        (reg, self.__scale) = self.__scales[gauss]
        self.bus.write_byte_data(self.address, 0x00, 0x98)  # 1 Average, 75 Hz, normal measurement
        self.bus.write_byte_data(self.address, 0x01, reg << 5)  # Scale
        self.bus.write_byte_data(self.address, 0x02, 0x00)  # Continuous measurement

        self.offsets = {'cx': 0.0,
                        'cy': 0.0,
                        'cz': 0.0}
        self.calibration_matrix = matrix([[1.0, 0.0, 0.0],
                                          [0.0, 1.0, 0.0],
                                          [0.0, 0.0, 1.0]])

    def twos_complement(self, val, len):
        # Convert twos compliment to integer
        if (val & (1 << len - 1)):
            val = val - (1 << len)
        return val

    def _convert(self, data, offset):
        val = self.twos_complement(data[offset] << 8 | data[offset+1], 16)
        if val == -4096: return None
        return round(val * self.__scale, 4)

    def read_data(self):
        data = self.bus.read_i2c_block_data(self.address, 0x03, 6)
        x = self._convert(data, 0)
        y = self._convert(data, 4)
        z = self._convert(data, 2)

        p = matrix([x - self.offsets['cx'], y - self.offsets['cy'], z - self.offsets['cz']]) * self.calibration_matrix
        return p[0, 0], p[0, 1], p[0, 2]

    def sample(self, num_of_calibration_measurements, print_status=False):
        max_line_length = 0
        mag_samples = []

        if print_status:
            print("Calibrating compass offsets & matrices, please hold still...")

        while (len(mag_samples) < num_of_calibration_measurements) and (not rclpy.ok()):
            (x, y, z) = self.read_data()
            mag_samples.append([x, y, z])

            if print_status:
                s = "[{}/{}] cx:{:0.4f}  cy:{:0.4f}  cz:{:0.4f}".format(len(mag_samples),
                                                                        num_of_calibration_measurements,
                                                                        x,
                                                                        y,
                                                                        z)
                max_line_length = len(s) if len(s) > max_line_length else max_line_length
                num_of_space = max_line_length - len(s)

                print(s + ' ' * num_of_space, end='\r')

        if print_status:
            print('\nDone.')

        return mag_samples[50:]

    def calibrate(self, num_of_calibration_measurements=5000, print_status=False, timestamp=datetime.datetime.now()):
        mag_samples = self.sample(num_of_calibration_measurements, print_status)
        self.save_calibration_data(mag_samples, timestamp)

        if (not rclpy.ok()):
            xyz = matrix(mag_samples)

            # compute the vectors [ x^2 y^2 z^2 2*x*y 2*y*z 2*x*z x y z 1] for every sample
            # the result for the x*y y*z and x*z components should be divided by 2
            xyz2 = power(xyz, 2)
            xy = multiply(xyz[:, 0], xyz[:, 1])
            xz = multiply(xyz[:, 0], xyz[:, 2])
            yz = multiply(xyz[:, 1], xyz[:, 2])

            # build the data matrix
            A = bmat('xyz2 xy xz yz xyz')

            b = 1.0 * ones((xyz.shape[0], 1))

            # solve the system Ax = b
            q, res, rank, sing = linalg.lstsq(A, b)

            # build scaled ellipsoid quadric matrix (in homogeneous coordinates)
            A = matrix([[q[0][0], 0.5 * q[3][0], 0.5 * q[4][0], 0.5 * q[6][0]],
                        [0.5 * q[3][0], q[1][0], 0.5 * q[5][0], 0.5 * q[7][0]],
                        [0.5 * q[4][0], 0.5 * q[5][0], q[2][0], 0.5 * q[8][0]],
                        [0.5 * q[6][0], 0.5 * q[7][0], 0.5 * q[8][0], -1]])

            # build scaled ellipsoid quadric matrix (in regular coordinates)
            Q = matrix([[q[0][0], 0.5 * q[3][0], 0.5 * q[4][0]],
                        [0.5 * q[3][0], q[1][0], 0.5 * q[5][0]],
                        [0.5 * q[4][0], 0.5 * q[5][0], q[2][0]]])

            # obtain the centroid of the ellipsoid
            x0 = linalg.inv(-1.0 * Q) * matrix([0.5 * q[6][0], 0.5 * q[7][0], 0.5 * q[8][0]]).T

            # translate the ellipsoid in homogeneous coordinates to the center
            T_x0 = matrix(eye(4))
            T_x0[0, 3] = x0[0]; T_x0[1, 3] = x0[1]; T_x0[2, 3] = x0[2];
            A = T_x0.T * A * T_x0

            # rescale the ellipsoid quadric matrix (in regular coordinates)
            Q = Q * (-1.0 / A[3, 3])

            # take the cholesky decomposition of Q. this will be the matrix to transform
            # points from the ellipsoid to a sphere, after correcting for the offset x0
            L = eye(3)
            try:
                L = linalg.cholesky(Q).transpose()
            except Exception as e:
                print(str(e))
                L = eye(3)

            self.offsets = {'cx': float(x0[0, 0]),
                            'cy': float(x0[1, 0]),
                            'cz': float(x0[2, 0])}

            self.calibration_matrix = L * 500

            # print("Magnetometer offset:\n {}".format(self.offsets))
            # print("Magnetometer Calibration Matrix:\n {}".format(self.calibration_matrix))

        return not rclpy.ok()

    @staticmethod
    def visualise(data):
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D

        ax = plt.axes(projection='3d')

        xdata = list()
        ydata = list()
        zdata = list()

        for point in data:
            xdata.append(point[0])
            ydata.append(point[1])
            zdata.append(point[2])

        ax.scatter3D(xdata, ydata, zdata, c=zdata, cmap='Greens')

        plt.show()

    @staticmethod
    def save_calibration_data(data, timestamp):
        import json

        d_data = dict()

        for i, point in enumerate(data):
            d_data[str(i)] = [point[0], point[1], point[2]]

        filename = '/home/nano/catkin_ws/src/gy-86/imu_calibration/mag_data/{}-raw-compass-data.json'.format(timestamp)
        with open(filename, 'w') as outfile:
            json.dump(d_data, outfile, indent=4)

    @staticmethod
    def save_calibration_config_data(offsets, calibration_matrix, timestamp):
        import json

        offsets_list = [0.0, 0.0, 0.0]
        for i in range(3):
            offsets_list[i] = float(offsets[i, 0])

        calibration_matrix_list = [[1.0, 0.0, 0.0],
                                   [0.0, 1.0, 0.0],
                                   [0.0, 0.0, 1.0]]
        for r in range(3):
            for c in range(3):
                calibration_matrix_list[r][c] = float(calibration_matrix[r, c])

        d_data = {'offsets': offsets_list,
                  'calibration_matrix': calibration_matrix_list}

        with open('/home/nano/catkin_ws/src/gy-86/calibration_datas/{}-compass-config-data.json'.format(timestamp), 'w') as outfile:
            json.dump(d_data, outfile, indent=4)
        return


    def __str__(self):
        (x, y, z) = self.read_data()
        return "Axis X: " + str(x) + "\n" + \
               "Axis Y: " + str(y) + "\n" + \
               "Axis Z: " + str(z) + "\n" + \
               "Declination: " + str(self.degrees(self.declination())) + "\n" + \
               "Heading: " + str(self.heading()) + "\n"

