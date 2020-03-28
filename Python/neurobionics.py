
from DataLogger import dataLogger



#################### SETUP ####################

# Constants
degToCount		= 2**14/360								# 1 degree is 45.5111 ticks, AS5047P 14-bit encoder (2^14/360)
countToDeg		= 360/2**14								# 1/degToCount
diffCoeffs 		= [25/12,-4.0,3.0,-4/3,1/4]				# Coefficients for 5-point backward difference derivative estimation
diffCoeffs 		= [1.0,-1.0]							# Coefficients for 2-point backward difference derivative estimation
llToPP 			= np.sqrt(3) 							# Line-line current to phase-phase
ppToLL 	 		= 1/llToPP 								# Phase-phase current to line-line
lbsTokg			= 0.453592 								# Pounds to kg

Sk = 2**9       # Bit shift by 9 (stiffness)
Sb = 2**6       # Bit shift by 6 (damping)
mA = 1000      	# mA/A conversion
ms = 1000      	# ms/s conversion
kt = 0.096     	# Motor constant





def checkBatteries(count,devId):
	# Check battery voltage
    actPackState = fxReadDevice(devId)

    if actPackState.batteryVoltage <= 34000:
        print('\n***BATTERY VOLTAGE LOW" %.3f V***\n' %actPackState.batteryVoltage/1000)
    elif:
        print('\nVoltage: %.3f V' %actPackState.batteryVoltage/1000)


def homeAndMap(home,vHoming,homingRate,devId,filename,saveDataLogger=False):
	# Purpose: goes through homing routine and exports a text file of joint degree and motor counts. This should be called whenever the motor is first turned on 
	posMinMotor, posMinJoint, temp	= homing(home,-vHoming,homingRate,devId,filename,saveDataLogger)
	posMaxMotor, posMaxJoint, dl 	= homing(home,vHoming,homingRate,devId,filename,saveDataLogger)
	
	if saveDataLogger:
		dl.myData.pop()
		dl.myData.pop()
		dl.myData.pop()
		dl.myData.pop()
		dl.myData.pop()

		dl.writeOut()


def homing(home,vHoming,homingRate,devId,filename='', saveDataLogger=False):
	# Purpose: moves joint until hitting the hardstop

	actPackState = fxReadDevice(devId)

    posFinalMotor 	= actPackState.encoderAngle
    posFinalJoint 	= actPackState.ankleAngle
	print("posFinalJoint",posFinalJoint )

	dl = []
	if saveDataLogger:
		dl 				= dataLogger('encMap_' + filename + '.txt')
		dl.clearData()

	if home == 'y':
		try:
			print('Homing...')

            # start rotating joint
            setMotorCommand(devId, FxVoltage, vHoming)
            
			actPackState = fxReadDevice(devId)
            posCurr = actPackState.encoderAngle
            sleep(0.5)

			print(posCurr)

			if saveDataLogger:
                # Add data to file
                d1.appendData([fxReadDevice(devId).ankleAngle*countToDeg] + [posCurr]) 

			keepGoing 	= True
				
			# Homing routine
			while keepGoing:
				sleep(homingRate)

                # Get previous position
				posPrev 	= posCurr					

                # Get current position
				actPackState = fxReadDevice(devId)
                posCurr = actPackState.encoderAngle		
				print(posCurr)
                
                # Get difference
				posDiff 	= posCurr - posPrev			
				
				if saveDataLogger:
                    # Add data to file
					dl.appendData([fxReadDevice(devId).ankleAngle*countToDeg] + [posCurr])	

				if -degToCount/2 <= posDiff <= degToCount/2:
					print('posdiff: ', posDiff)
					print('degToCount: ', degToCount)

                    # Set motor voltage to XXX mV
                    setMotorCommand(devId, FxVoltage, 0)

                    # Send/receive controller/sensor data				
                    actPackState = fxReadDevice(devId)

                    posFinalMotor 	= actPackState.encoderAngle
                    posFinalJoint 	= actPackState.ankleAngle

					keepGoing 		= False
					print('Homing successful.\n')

		except KeyboardInterrupt:
			print('\n***Homing routine stopped by user***\n')
	else:
		print('\n***Did not run homing routine***\n')

	return posFinalMotor, posFinalJoint, dl
