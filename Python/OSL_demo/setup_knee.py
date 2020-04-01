from time import sleep


import os, sys

# add path-to-flexseapython!
# pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# sys.path.append(pardir)

# import Actuator-Package
from flexseapython.pyFlexsea import *
from flexseapython.fxUtil import *

from neurobionics import *
# from logger~~~~~


def mainfunc(port, baudRate, resolution = 100):

	# Initialize FlexSEA
	devId = fxOpen(port, baudRate)
	fxStartStreaming(devId, resolution, False)
	sleep(0.1)

	# checkBatteries
	checkBatteries(1, devId)

	# Get joint
	joint = 'knee'

	# # Home and map (if needed)
	print('Start homing routine: ')

	vHoming 	= 2500	# Voltage for homing mechanism, mV
	homingRate 	= 0.1
	posMaxMotor, posMaxJoint, posMinMotor, posMinJoint = homeAndMap(vHoming,homingRate,devId)

	actPackState = fxReadDevice(devId)
	print('\nTemperature: %.2f C\n' % actPackState.batteryTemp)

	# # Turn off motor motion
	fxSendMotorCommand(devId, FxCurrent, 0)
	sleep(1)

	# ## print data from homeAndMap
	print('Finish homing routine! for ', joint)
	print('Maximun Position (Motor / Joint): ', posMaxMotor, posMaxJoint)
	print('Minimun Position (Motor / Joint): ', posMinMotor, posMinJoint)

	fxSendMotorCommand(devId, FxCurrent, 0)
	sleep(1)

	# Trun off motor connection
	fxStopStreaming(devId)
	fxClose(devId)
	sleep(1)

if __name__ == '__main__':
	## connection setting
	# baudRate = 230400
	# ports = sys.argv[1:2]   # first argument of the script

	scriptPath = os.path.dirname(os.path.abspath(__file__))
	fpath = scriptPath + '/flexseapython/com.txt'
	ports, baudRate = loadPortsFromFile(fpath)
	print('Loaded ports: ' + str(ports))
	print('Using baud rate: ' + str(baudRate))

	try:
		mainfunc(ports[0], int(baudRate))
	except Exception as e:
		print("borke: " + str(e))