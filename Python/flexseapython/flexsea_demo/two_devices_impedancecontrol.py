import os, sys
from time import sleep

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)
from fxUtil import *

# Control gain constants
kp = 100
ki = 32
K = 100
B = 0

def printDevice(actPackState):
	print('State time: ', actPackState.timestamp)
	print('Accel X: ', actPackState.accelx, ', Accel Y: ', actPackState.accely, ' Accel Z: ', actPackState.accelz)
	print('Gyro X: ', actPackState.gyrox, ', Gyro Y: ', actPackState.gyroy, ' Gyro Z: ', actPackState.gyroz)
	print('Motor angle: ', actPackState.encoderAngle, ', Motor voltage: ', actPackState.motorVoltage, flush=True)



def fxTwoDeviceImpedanceControl(port0, port1, baudRate, expTime = 11, time_step = 0.1, transition_time = 1, delta = 4000):

	devId0 = fxOpen(port0, baudRate, 0)
	devId1 = fxOpen(port1, baudRate, 0)

	fxStartStreaming(devId0, 200, True)
	fxStartStreaming(devId1, 200, True)

	sleep(0.2)

	actPackState0 = fxReadDevice(devId0)
	actPackState1 = fxReadDevice(devId1)

	initialAngle0 = actPackState0.encoderAngle
	initialAngle1 = actPackState1.encoderAngle

	fxSetGains(devId0, kp, ki, 0, K, B)
	fxSetGains(devId1, kp, ki, 0, K, B)
	
	fxSendMotorCommand(devId0, FxImpedance, initialAngle0)
	fxSendMotorCommand(devId1, FxImpedance, initialAngle1)

    # setting angle waypoints
	positions0 = [initialAngle0, initialAngle0+delta, initialAngle0, initialAngle0+delta, initialAngle0]
	current_pos0 = -1
	num_pos0 = 5

	positions1 = [initialAngle1, initialAngle1+delta, initialAngle1, initialAngle1, initialAngle1+delta]
	current_pos1 = -1
	num_pos1 = 5

	# setting loop duration and transition rate
	num_time_steps = int(expTime/time_step)
	transition_steps = int(transition_time/time_step)

	for i in range(num_time_steps):
		if i % transition_steps == 0:
			current_pos0 = (current_pos0+1)%num_pos0
			current_pos1 = (current_pos1+1)%num_pos1

			fxSendMotorCommand(devId0, FxImpedance, positions0[current_pos0])
			fxSendMotorCommand(devId1, FxImpedance, positions1[current_pos1])
		
		printDevice(actPackState0)
		printDevice(actPackState1)

		sleep(time_step)

	# while(True):
	# 	sleep(0.2)
	# 	os.system('cls')
	# 	preamble = "Holding position with impedance control, two devices: "
	# 	print(preamble)

	# 	actPackState0 = fxReadDevice(devId0)
	# 	actPackState1 = fxReadDevice(devId1)
		
	# 	printDevice(actPackState0)
	# 	printDevice(actPackState1)


	print('Turning off impedance control...')
	fxStopStreaming(devId0)
	fxClose(devId0)
	fxStopStreaming(devId1)
	fxClose(devId1)

if __name__ == '__main__':
	baudRate = sys.argv[1]
	ports = sys.argv[2:4]
	try:
		fxTwoDevicePositionControl(ports[0], ports[1], baudRate)
	except Exception as e:
		print("broke: " + str(e))

