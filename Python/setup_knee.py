from time import sleep


import os, sys

# add path-to-flexseapython!
# pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)

# import Actuator-Package
from flexseapython.pyFlexsea import *
from flexseapython.fxUtil import *

from neurobionics import *


def mainfunc(port, baudRate, resolution = 100):

    # Initialize FlexSEA
    devId = fxOpen(port, baudRate)
    fxStartStreaming(devId, resolution, True)
    sleep(0.1)

    # checkBatteries
    checkBatteries(1, devId)

    # Get joint
    joint = 'knee'

    # Home and map (if needed)
    home 		= str(input('\nHome/Map the prosthesis? (y/n)\n***Press (y) if this this hasn\'t been done since assembly***\n'))
	vHoming 	= 2500	# Voltage for homing mechanism, mV
	homingRate 	= 0.1
	if home == 'y':
		homeAndMap(home,vHoming,homingRate,devId,joint,True)
	else:
		homeAndMap(home,vHoming,homingRate,devId,joint,False)

	actPackState = fxReadDevice(devId)
	print('\nTemperature: %.2f C' % actPackState.batteryTemp)

    # Turn off motor motion
    fxSendMotorCommand(devId, FxCurrent, 0)
    sleep(1)

    ## print data log from homeAndMap
    print_data_log()

    # Trun off motor connection
    fxClose(devId)


def print_data_log():
    print('***CHECK ENCODER MAP BELOW***')
	print('[Joint Angle (deg), Motor Angle (count)]')
	# Read data into arrays
	jointDeg 	= []
	motorCount 	= []
	f = open('encMap_' + joint + '.txt','r')
	data = f.read().splitlines()
	for x in range(len(data)-1):
		dataSplit = data[x].split(' ')
		jointDeg.append(float(dataSplit[0]))
		motorCount.append(float(dataSplit[1]))
		print(jointDeg[-1],motorCount[-1])

	# Convert to numpy arrays
	jointDeg	= np.array(jointDeg)
	motorCount	= np.array(motorCount)
	print('***CHECK ENCODER MAP ABOVE***')

	print('\n\nRoM: %5.2f deg'	% (jointDeg[0]-jointDeg[-1]))



if __name__ == '__main__':
    ## connection setting
    # baudRate = 230400
    # ports = sys.argv[1:2]   # first argument of the script

    scriptPath = os.path.dirname(os.path.abspath(__file__))
	fpath = scriptPath + '/com.txt'
	ports, baudRate = loadPortsFromFile(fpath)
	print('Loaded ports: ' + str(ports))
	print('Using baud rate: ' + str(baudRate))

    try:
        mainfunc(ports[0], int(baudRate))
    except Exception as e:
        print("borke: " + str(e))