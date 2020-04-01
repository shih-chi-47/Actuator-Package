import os, sys
from time import sleep, time, strftime
import matplotlib as mpl
mpl.use('WebAgg')
mpl.rcParams['webagg.address'] = "0.0.0.0"
mpl.rcParams['webagg.open_in_browser'] = 0

import matplotlib.pyplot as plt


pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)
from fxUtil import *

# Control gain constants
kp = 100
ki = 32
K = 325
B = 0
B_Increments = 200

kp = 0
ki = 0
K_Increments = 10
B_Increments = 100

def fxImpedanceControl(port, baudRate, expTime = 20, time_step = 0.02, delta = 7500, transition_time = 1, resolution = 500):

	devId = fxOpen(port, baudRate, 0) 
	fxStartStreaming(devId, resolution, True)
	
	result = True
	
	data = fxReadDevice(devId)
	initialAngle = data.encoderAngle
	
	timeout = 100
	timeoutCount = 0
	transition_steps = int(transition_time / time_step)
			
	# Initialize lists - matplotlib
	requests = []
	measurements = []
	times = []
	i = 0
	t0 = 0

	fxSendMotorCommand(devId, FxImpedance, initialAngle)
	# Set gains
	global B
	global K
	fxSetGains(devId, kp, ki, 0, K, B)

	# Select transition rate and positions
	currentPos = 0
	num_time_steps = int(expTime/time_step)
	positions = [initialAngle,initialAngle + delta]
	sleep(0.4)
	
	# Record start time of experiment
	t0 = time()
	
	# Run demo
	print(result)
	B = -B_Increments # We do that to make sure we start at 0
	B = 0 

	for i in range(num_time_steps):
		data = fxReadDevice(devId)
		measuredPos = data.encoderAngle
		if i % transition_steps == 0:
			B = B + B_Increments	# Increments every cycle
			K = K + K_Increments
			
			fxSetGains(devId, kp, ki, 0, K, B)
			delta = abs(positions[currentPos] - measuredPos)
			result &= delta < resolution
			currentPos = (currentPos + 1) % 2
			fxSendMotorCommand(devId, FxImpedance, positions[currentPos])
		sleep(time_step)
		preamble = "Holding position: {}...".format(positions[currentPos])
		print(preamble, " Achieve: ", fxReadDevice(devId).encoderAngle)	
		# Plotting:
		measurements.append(measuredPos)
		times.append(time() - t0)
		requests.append(positions[currentPos])

	# Close device and do device cleanup
	fxClose(devId)
	
	# Plot before we exit:
	print("=== Plotting with matplotlib ===")
	title = "Impedance Control Demo"
	plt.plot(times, requests, color = 'b', label = 'Desired position')
	plt.plot(times, measurements, color = 'r', label = 'Measured position')
	plt.xlabel("Time (s)")
	plt.ylabel("Encoder position")
	plt.title(title)
	plt.legend(loc='upper right')
	plt.show()
	
	return result

if __name__ == '__main__':
	baudRate = sys.argv[1]
	ports = sys.argv[2:3]
	try:
		fxPositionControl(ports, baudRate)
	except Exception as e:
		print("broke: " + str(e))
		pass
