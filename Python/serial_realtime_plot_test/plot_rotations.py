"""
Plot rotations from IMU in realtime
"""
import argparse
import serial
import struct

### Packet that is being sent over serial
# The data packets over serial are being sent as imu_data_t structs (Refer to IMU_9DOF.h)
# This means that to unpack these data packets, the packet datatype that is being sent
# is 9 floats corresponding to: 
# - acceleration data (acc_x,acc_y,acc_z),
# - gyroscope data (gyro_x, gyro_y, gyro_z)
# - magnetometer data (mag_x, mag_y, mag_z)
IMU_PKT_DATATYPE = 'fffffffff'
IMU_PKT_SIZE = struct.calcsize(IMU_PKT_DATATYPE)


### Plotting
def animate(acc_x_hist, acc_y_hist, acc_hist_z):
    return



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


    ### Grab Data from serial
    while True:
        # Grab data
        imu_data_pkt = ser.read(IMU_PKT_SIZE)

        # Unpack data
        imu_data_tuple = struct.unpack('fffffffff', imu_data_pkt)
        acc_x, acc_y, acc_z = imu_data_tuple[0:3]
        gyro_x, gyro_y, gyro_z = imu_data_tuple[3:6]
        mag_x, mag_y, mag_z = imu_data_tuple[6:9]

        print("(acc_x, acc_y, acc_z): \t\t", (acc_x, acc_y, acc_z))
        print("(gyro_x, gyro_y, gyro_z): \t", (gyro_x, gyro_y, gyro_z))
        print("(mag_x, mag_y, mag_z): \t\t", (mag_x, mag_y, mag_z))
        print()

if __name__ == "__main__":
    main()