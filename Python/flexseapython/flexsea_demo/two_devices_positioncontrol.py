import os, sys
from time import sleep

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)
from fxUtil import *

def printDevice(actPackState):
	print('State time: ', actPackState.timestamp)
	print('Accel X: ', actPackState.accelx, ', Accel Y: ', actPackState.accely, ' Accel Z: ', actPackState.accelz)
	print('Gyro X: ', actPackState.gyrox, ', Gyro Y: ', actPackState.gyroy, ' Gyro Z: ', actPackState.gyroz)
	print('Motor angle: ', actPackState.encoderAngle, ', Motor voltage: ', actPackState.motorVoltage, flush=True)



def fxTwoDevicePositionControl(port0, port1, baudRate, exp_time = 10, time_step = 0.1, resolution = 100):

	devId0 = fxOpen(port0, baudRate, 0)
	devId1 = fxOpen(port1, baudRate, 0)

	fxStartStreaming(devId0, resolution, True)
	fxStartStreaming(devId1, resolution, True)
	sleep(0.2)

	actPackState0 = fxReadDevice(devId0)
	actPackState1 = fxReadDevice(devId1)

	initialAngle0 = actPackState0.encoderAngle
	initialAngle1 = actPackState1.encoderAngle

	fxSetGains(devId0, 50, 3, 0, 0, 0)
	fxSetGains(devId1, 50, 3, 0, 0, 0)
	
	fxSendMotorCommand(devId0, FxPosition, initialAngle0)
	fxSendMotorCommand(devId1, FxPosition, initialAngle1)

	num_time_steps = int(exp_time/time_step)
	try:
		for i in range(num_time_steps):
			sleep(time_step)
			os.system('cls')
			preamble = "Holding position, two devices: "
			print(preamble)
	
			actPackState0 = fxReadDevice(devId0)
			actPackState1 = fxReadDevice(devId1)
			
			printDevice(actPackState0)
			printDevice(actPackState1)

	except:
		pass

	print('Turning off position control...')
	
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
