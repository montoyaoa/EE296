from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import csv
import numpy as np
import scipy.integrate as integrate
import math
import serial
import time

VELOCITY_MAGNITUDE = 1
INITIAL_POSITION = [0, 0, 0]

FILE_NAME = '04262201_130700.csv'

PRESSURE_INDEX = 5
QUAT_W_INDEX = 11
QUAT_X_INDEX = 12
QUAT_Y_INDEX = 13
QUAT_Z_INDEX = 14
EULER_X_INDEX = 15
EULER_Y_INDEX = 16
EULER_Z_INDEX = 17
# VELOCITY_INDEX =

#change as needed
PORT = '/dev/cu.usbserial-1410'
BAUD_RATE = 115200
READ_SERIAL_PORT = False

def get_velocity_vector(magnitude, angle_x, angle_y, angle_z):
    v_xy = magnitude * math.cos(angle_y*math.pi/180)
    v_x = v_xy * math.cos(angle_x*math.pi/180)
    v_y = v_xy * math.sin(angle_x*math.pi/180)
    v_z = magnitude * math.sin(angle_y*math.pi/180)
    return [v_x, v_y, v_z]

def get_velocity_x_component(magnitude, angle_x, angle_y):
    v_xy = magnitude * math.cos(angle_y*math.pi/180)
    v_x = v_xy * math.cos(angle_x*math.pi/180)
    return v_x

def get_velocity_y_component(magnitude, angle_x, angle_y):
    v_xy = magnitude * math.cos(angle_y*math.pi/180)
    v_y = v_xy * math.sin(angle_x*math.pi/180)
    return v_y

def get_velocity_z_component(magnitude, angle_y):
    v_xy = magnitude * math.cos(angle_y*math.pi/180)
    v_z = magnitude * math.sin(angle_y*math.pi/180)
    return v_z

def get_next_position(current_pos, v_x, v_y, v_z, integrate_velocity=False):
    if not(integrate_velocity):
        return [current_pos[0] + v_x, 
                current_pos[1] + v_y, 
                current_pos[2] + v_z]
    else:
        r_x = integrate.quad(lambda t:v_x, 0, 1)
        r_y = integrate.quad(lambda t:v_y, 0, 1)
        r_z = integrate.quad(lambda t:v_z, 0, 1)
        return [current_pos[0] + r_x, 
                current_pos[1] + r_y, 
                current_pos[2] + r_z]

def get_start_time_from_csv_file_name(fn):
    ht = fn.split('_')
    ht = ht[1].split('.')
    return ht[0]

def ht_to_gmt(t):
    hour = int(t[0:1])
    min = t[2:3]
    sec = t[4:5]
    hour += 10
    if (hour >= 24):
        hour -= 24
    t = [string(hour), min, sec]
    return ':'.join(t)

def get_total_row_val(fn):
    with open(fn) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        next(csv_reader) #skip header row
        row_count = 0
        for row in csv_reader:
            row_count += 1
    return row_count

# def read_csv(fn, p, qw, qx, qy, qz, ex, ey, ez, v):
def read_csv(fn, ex, ey, ez):
    with open(fn) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        next(csv_reader) #skip header row
        row_count = 0
        for row in csv_reader:
            if row_count == 0:
                # assign values in each col to appropriate array
                # # p = row[PRESSURE_INDEX]
                # qw = row[QUAT_W_INDEX]
                # qx = row[QUAT_X_INDEX]
                # qy = row[QUAT_Y_INDEX]
                # qz = row[QUAT_Z_INDEX]
                # v = row[VELOCITY_INDEX]
                ex[0] = float(row[EULER_X_INDEX])
                ey[0] = float(row[EULER_Y_INDEX])
                ez[0] = float(row[EULER_Z_INDEX])
            else:
                ex = np.append(ex, float(row[EULER_X_INDEX]))
                ey = np.append(ey, float(row[EULER_X_INDEX]))
                ez = np.append(ez, float(row[EULER_X_INDEX]))
            row_count += 1
    # return [p, qw, qx, qy, qz, ex, ey, ez, v]
    return [ex, ey, ez]

def get_z_position(p, z):
    for x in p:
        #offset = 105 not included
        z = np.append((((x * 0.06103515625)/0.04382512987)/14.69595)*33)
    return z

#convert quaterion to angle
def get_angles_from_quat(qw, qx, qy, qz):
    x = atan2(2.0 * (qx*qy + qz*qw), (qx**2 - qy**2 - qz**2 + qw**2))
    y = asin(-2.0 * (qx*qz - qy*qw) / (qx**2 + qy**2 + qz**2 + qw**2))
    z = atan2(2.0 * (qy*qz + qx*qw), (-qx**2 - qy**2 + qz**2 + qw**2))
    return [x, y, z]

def main():
    current_pos = INITIAL_POSITION
    x_positions = np.array([0])
    y_positions = np.array([0])
    z_positions = np.array([0])
    euler_x_angles = np.array([0], dtype = 'f')
    euler_y_angles = np.array([0], dtype = 'f')
    euler_z_angles = np.array([0], dtype = 'f')
    velocity_x_components = np.array([0])
    velocity_y_components = np.array([0])
    velocity_z_components = np.array([0])

    if (READ_SERIAL_PORT):
        ser = serial.Serial(PORT, BAUD_RATE)
        time.sleep(1)

        is_calibrated = False

        while not(is_calibrated):
            while ser.in_waiting == 0:
                pass

            data = str(ser.readline(), 'utf-8')
            splitData = data.split(',')
            accel_calibration_status = float(splitData[0])
            gyro_calibration_status = float(splitData[1])
            mag_calibration_status = float(splitData[2])
            system_calibration_status = float(splitData[3])
            line_num = int(splitData[12])

            if (accel_calibration_status <= 3.0 and 
                gyro_calibration_status == 3.0 and
                mag_calibration_status == 3.0 and 
                system_calibration_status == 3.0):
                    is_calibrated = True;
            else:
                print(
                    line_num, " ",
                    "Calibration status:",
                    "Accel =", accel_calibration_status,
                    "Gyro =", gyro_calibration_status,
                    "Mag =", mag_calibration_status,
                    "Sys =", system_calibration_status)

        time_delay = 3
        time_delay_start = time.time()

        for i in range(time_delay):
            while time.time() < time_delay_start + 1:
                d = str(ser.readline(), 'utf-8')
            print(i, " second")
            time_delay_start = time.time()

        time_duration = 15
        time_start = time.time()

        time_counter = 0

        # while True:
        while time.time() < time_start + time_duration:
            while ser.in_waiting == 0:
                pass

            data = str(ser.readline(), 'utf-8')
            splitData = data.split(',')

            if (len(euler_x_angles) == 1 and euler_x_angles[0] == 0):
                euler_x_angles[0] = float(splitData[8])
                euler_y_angles[0] = float(splitData[9])
                euler_z_angles[0] = float(splitData[10])
            else:
                euler_x_angles = np.append(euler_x_angles, splitData[8])
                euler_y_angles = np.append(euler_y_angles, splitData[9])
                euler_z_angles = np.append(euler_z_angles, splitData[10])
            
            line_num = int(splitData[12])

            # if time.time() - time_counter == 1:
            #   VELOCITY_MAGNITUDE += 1
            #   time_counter = time.time() 
    else:
        total_rows = get_total_row_val(FILE_NAME)

        # for i in np.arange(0, total_rows, 1):
        euler_x_angles, euler_y_angles, euler_z_angles = read_csv(
            FILE_NAME,
            euler_x_angles,
            euler_y_angles,
            euler_z_angles)


    for i in np.arange(0, len(euler_x_angles), 1):
        if i == 0:
            velocity_x_components[i] = get_velocity_x_component(
                VELOCITY_MAGNITUDE, 
                euler_x_angles[i],
                euler_y_angles[i])
            velocity_y_components[i] = get_velocity_y_component(
                VELOCITY_MAGNITUDE, 
                euler_x_angles[i],
                euler_y_angles[i])
            velocity_z_components[i] = get_velocity_z_component(
                VELOCITY_MAGNITUDE, 
                euler_y_angles[i])
        else:
            velocity_x_components = np.append(
                velocity_x_components, 
                get_velocity_x_component(
                    VELOCITY_MAGNITUDE, 
                    euler_x_angles[i],
                    euler_y_angles[i]))
            velocity_y_components = np.append(
                velocity_y_components, 
                get_velocity_y_component(
                    VELOCITY_MAGNITUDE, 
                    euler_x_angles[i],
                    euler_y_angles[i]))
            velocity_z_components = np.append(
                velocity_z_components, 
                get_velocity_z_component(
                    VELOCITY_MAGNITUDE, 
                    euler_y_angles[i]))

    for j in np.arange(0, len(velocity_x_components), 1):
        x_positions = np.append(x_positions, current_pos[0])
        y_positions = np.append(y_positions, current_pos[1])
        z_positions = np.append(z_positions, current_pos[2])

        current_pos = get_next_position(
            current_pos, 
            velocity_x_components[j], 
            velocity_y_components[j], 
            velocity_z_components[j])

        # print(current_pos)
        # print(line_num, " ", current_pos)

        # if current_pos == INITIAL_POSITION:
        #     current_pos = get_next_position(
        #         INITIAL_POSITION, v_x, v_y, v_z)
        # else:
        #     current_pos = get_next_position(
        #         current_pos, v_x, v_y, v_z)

    map = plt.figure(figsize=(16, 8))
    map_ax = Axes3D(map)
    # map_ax = map.add_subplot(121)

    # map_ax = map.add_subplot(122, projection='3d')
    map_ax.scatter3D(
        x_positions[0], y_positions[0], z_positions[0], marker='s')
    # map_ax.scatter3D(x_positions[1:], y_positions[1:], z_positions[1:])
    map_ax.plot(x_positions[1:], y_positions[1:], z_positions[1:])
    map_ax.autoscale(enable=True, axis='both', tight=True)
    # map_ax.set_zlim3d([-100.0, 0])
    map_ax.set_xlabel('X')
    map_ax.set_ylabel('Y')
    map_ax.set_zlabel('Z')
    plt.show()

if __name__ == '__main__':
    main()