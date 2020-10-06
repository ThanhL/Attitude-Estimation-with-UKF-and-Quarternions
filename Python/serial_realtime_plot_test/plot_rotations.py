"""
Plot rotations from IMU in realtime
"""
import argparse
import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from drawnow import *

import threading
import time

### Packet that is being sent over serial
# The data packets over serial are being sent as imu_data_t structs (Refer to IMU_9DOF.h)
# This means that to unpack these data packets, the packet datatype that is being sent
# is 9 floats corresponding to: 
# - acceleration data (acc_x,acc_y,acc_z),
# - gyroscope data (gyro_x, gyro_y, gyro_z)
# - magnetometer data (mag_x, mag_y, mag_z)
IMU_PKT_DATATYPE = 'fffffffff'
IMU_PKT_SIZE = struct.calcsize(IMU_PKT_DATATYPE)


### History Logger
acc_x_hist, acc_y_hist, acc_z_hist = [], [], []
gyro_x_hist, gyro_y_hist, gyro_z_hist = [], [], []
mag_x_hist, mag_y_hist, mag_z_hist = [], [], []

### Reading data from port
def animate(i):
    global axs
    plt.cla()
    # plt.plot(acc_x_hist)


    axs[0].plot(acc_x_hist)
    axs[1].plot(acc_y_hist)
    axs[2].plot(acc_z_hist)

def handle_imu_data(imu_data_pkt):
    global acc_x_hist, acc_y_hist, acc_z_hist
    global gyro_x_hist, gyro_y_hist, gyro_z_hist
    global mag_x_hist, mag_y_hist, mag_z_hist

    # Unpack data
    imu_data_tuple = struct.unpack('fffffffff', imu_data_pkt)
    acc_x, acc_y, acc_z = imu_data_tuple[0:3]
    gyro_x, gyro_y, gyro_z = imu_data_tuple[3:6]
    mag_x, mag_y, mag_z = imu_data_tuple[6:9]


    # Append data
    acc_x_hist.append(acc_x)
    acc_y_hist.append(acc_y)
    acc_z_hist.append(acc_z)

    gyro_x_hist.append(gyro_x)
    gyro_y_hist.append(gyro_y)
    gyro_z_hist.append(gyro_z)

    mag_x_hist.append(mag_x)
    mag_y_hist.append(mag_y)
    mag_z_hist.append(mag_z)


    # Constrain data size
    acc_x_hist = acc_x_hist[-20:]
    acc_y_hist = acc_y_hist[-20:]
    acc_z_hist = acc_z_hist[-20:]

    gyro_x_hist = gyro_x_hist[-20:]
    gyro_y_hist = gyro_y_hist[-20:]
    gyro_z_hist = gyro_z_hist[-20:]

    mag_x_hist = mag_x_hist[-20:]
    mag_y_hist = mag_y_hist[-20:]
    mag_z_hist = mag_z_hist[-20:]


    # Compute complimentary angles
    acceleration = np.array([acc_x, acc_y, acc_z])
    gyroscope = np.array([gyro_x, gyro_y, gyro_z])

    # print("acceleration: \t", acceleration)
    # print("gyro: \t", gyroscope)

    # print("(acc_x, acc_y, acc_z): \t\t", (acc_x, acc_y, acc_z))
    # print("(gyro_x, gyro_y, gyro_z): \t", (gyro_x, gyro_y, gyro_z))
    # print("(mag_x, mag_y, mag_z): \t\t", (mag_x, mag_y, mag_z))
 


    comp_filter.compute_complimentary_filter(acceleration, gyroscope)
    comp_filter.debug_roll_pitch()
    print()



def read_from_imu_port(ser):
    while True:

        # Grab data
        imu_data_pkt = ser.read(IMU_PKT_SIZE)

        # Handle the data
        handle_imu_data(imu_data_pkt)

### Good ol main
def main():
    ### Argparse Setup
    parser = argparse.ArgumentParser(description='Realtime IMU Raw Data values plotter')
    parser.add_argument('--baudrate', dest='baudrate', default=115200,
                        help='baudrate of serial COM port')
    parser.add_argument('--port', dest='com_port', required=True,
                        help='COM port for teensy/arduino')

    args = parser.parse_args()
    baudrate = args.baudrate
    com_port = args.com_port

    ### Serial Initialize
    ser = serial.Serial()
    ser.port = com_port             # Arduino/Teensy serial port
    ser.baudrate = baudrate
    ser.timeout = 10                # Timeout for reading
    ser.open()                      # Initialize the serial

    if ser.is_open:
        print("[!] Serial port opened! Serial Configuration: \n", ser)


    ### Grab Data from serial (threading)
    thread = threading.Thread(target=read_from_imu_port, args=(ser,))
    thread.start()

    # fig, axs = plt.subplots(3)
    # ani = animation.FuncAnimation(fig, animate, interval=100)
    # plt.show()


if __name__ == "__main__":
    main()