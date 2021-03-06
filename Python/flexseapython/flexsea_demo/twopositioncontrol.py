import os, sys
from time import sleep, time, strftime
# import matplotlib
# import matplotlib.pyplot as plt
# matplotlib.use('WebAgg')

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)
from fxUtil import *

def printDevice(actPackState):
	print('State time: ', actPackState.timestamp)
	print('Accel X: ', actPackState.accelx, ', Accel Y: ', actPackState.accely, ' Accel Z: ', actPackState.accelz)
	print('Gyro X: ', actPackState.gyrox, ', Gyro Y: ', actPackState.gyroy, ' Gyro Z: ', actPackState.gyroz)
	print('Motor angle: ', actPackState.encoderAngle, ', Motor voltage: ', actPackState.motorVoltage)

def fxTwoPositionControl(port, baudRate, expTime = 5, time_step = 0.1, delta = 10000, transition_time = 1.5, resolution = 100):
    # open device
    devId = fxOpen(port, baudRate, 0)
    fxStartStreaming(devId, resolution, True)
    sleep(0.1)

    # setting initial angle and angle waypoints
    actPackState = fxReadDevice(devId)
    printDevice(actPackState)
    initialAngle = actPackState.encoderAngle

    # setting angle waypoints
    positions = [initialAngle, initialAngle + delta]
    current_pos = 0
    num_pos = 2

    # setting loop duration and transition rate
    num_time_steps = int(expTime/time_step)
    transition_steps = int(transition_time/time_step)

    # setting gains (devId, kp, ki, kd, K, B)
    fxSetGains(devId, 50, 3, 0, 0, 0)

    # setting position control at initial position
    fxSendMotorCommand(devId, FxPosition, initialAngle)

	# matplotlib - initialize lists
    # requests = []
    # measurements = []
    # times = []
    # i = 0
    # t0 = 0

    sleep(0.4)
    t0 = time()

    # start two position control
    for i in range(num_time_steps):
        data = fxReadDevice(devId)
        measuredPos = data.encoderAngle

        if i % transition_steps == 0:
            current_pos = (current_pos + 1) % num_pos
            fxSendMotorCommand(devId, FxPosition, positions[current_pos])
        sleep(time_step)

        preamble = "Holding position: {}...".format(positions[current_pos])
        print(preamble)

        # plotting
        # times.append(time() - t0)
        # requests.append(positions[current_pos])
        # measurements.append(measuredPos)

    # close device and do device cleanup
    fxStopStreaming(devId)
    close_check = fxClose(devId)

    # Plot before exit:
    # title = "Two Position Control Demo"
    # plt.plot(times, requests, color = 'b', label = 'Desired position')
    # plt.plot(times, measurements, color = 'r', label = 'Measured position')
    # plt.xlabel("Time (s)")
    # plt.ylabel("Encoder position")
    # plt.title(title)
    # plt.legend(loc='upper right')
    # plt.show()

    return close_check

if __name__ == '__main__':
	baudRate = sys.argv[1]
	ports = sys.argv[2:3]
	try:
		fxPositionControl(ports, baudRate)
	except Exception as e:
		print("broke: " + str(e))
		pass
