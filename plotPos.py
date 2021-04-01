from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import numpy as np
import scipy.integrate as integrate
import math
import serial
import time

VELOCITY_MAGNITUDE = 1
INITIAL_POSITION = [0, 0, 0]

PORT = '/dev/cu.usbserial-1410'
BAUD_RATE = 115200

def get_velocity_vector(magnitude, angle_x, angle_y, angle_z):
	v_xy = magnitude * math.cos(angle_y*math.pi/180)
	v_x = v_xy * math.cos(angle_x*math.pi/180)
	v_y = v_xy * math.sin(angle_x*math.pi/180)
	v_z = magnitude * math.sin(angle_y*math.pi/180)
	return [v_x, v_y, v_z]

def get_next_position_vector(current_pos, v_x, v_y, v_z, 
		integrate_velocity=False):
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

def main():
	ser = serial.Serial(PORT, BAUD_RATE)
	time.sleep(1)

	current_pos = INITIAL_POSITION
	x_positions = np.array([0])
	y_positions = np.array([0])
	z_positions = np.array([0])

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

	map = plt.figure()
	map_ax = Axes3D(map)
	map_ax.autoscale(enable=True, axis='both', tight=True)

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
		x = float(splitData[8])
		y = float(splitData[9])
		z = float(splitData[10])
		line_num = int(splitData[12])

		# if time.time() - time_counter == 1:
		# 	VELOCITY_MAGNITUDE += 1
		# 	time_counter = time.time() 

		v_x, v_y, v_z = get_velocity_vector(VELOCITY_MAGNITUDE, x, -y, z)

		if current_pos == INITIAL_POSITION:
			current_pos = get_next_position_vector(
				INITIAL_POSITION, v_x, v_y, v_z)
		else:
			current_pos = get_next_position_vector(
				current_pos, v_x, v_y, v_z)
		# print(current_pos)
		print(line_num, " ", current_pos)

		x_positions = np.append(x_positions, current_pos[0])
		y_positions = np.append(y_positions, current_pos[1])
		z_positions = np.append(z_positions, current_pos[2])

	map_ax.scatter3D(x_positions, y_positions, z_positions)
	map_ax.autoscale(enable=True, axis='both', tight=True)
	map_ax.set_zlim3d([-50.0, 0.0])
	map_ax.set_xlabel('X')
	map_ax.set_ylabel('Y')
	map_ax.set_zlabel('Z')
	plt.show()

if __name__ == '__main__':
	main()