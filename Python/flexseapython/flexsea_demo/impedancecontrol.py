import os, sys
from time import sleep

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)
from fxUtil import *

kp = 100
ki = 32
K = 325
B = 0
B_Increments = 50

def fxImpedanceControl(port, baudRate, exptime = 10, time_step = 0.1, resolution = 100):
	devId = fxOpen(port, baudRate, 0)
	fxStartStreaming(devId, resolution, True)
	sleep(0.1)

	actPackState = fxReadDevice(devId)
	# printDevice(actPackState)
	initialAngle = actPackState.encoderAngle

	# set gains
	global B
	fxSetGains(devId, kp, ki, 0, K, B)

	fxSendMotorCommand(devId, FxImpedance, initialAngle)
	sleep(0.1)

	num_time_steps = int(exptime/time_step)

	for i in range(num_time_steps):
		B = B + B_Increments
		fxSetGains(devId, kp, ki, 0, K, B)
		fxSendMotorCommand(devId, FxImpedance, initialAngle)

		sleep(time_step)
		preamble = "Holding position: {}...".format(initialAngle)
		print(preamble)
		print("with controller: K=", K, ", B=", B)

		actPackState = fxReadDevice(devId)
		# printDevice(actPackState)
		currentAngle = actPackState.encoderAngle
		
		print("Measured delta is: ", currentAngle - initialAngle, flush=True)

	fxStopStreaming(devId)
	fxClose(devId)

	return True

if __name__ == '__main__':
	baudRate = sys.argv[1]
	ports = sys.argv[2:3]
	try:
		fxOpenControl(baudRate)
	except Exception as e:
		print("Broke... ")
		print(str(e))
