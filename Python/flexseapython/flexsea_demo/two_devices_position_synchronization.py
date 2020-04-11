import os, sys
from time import sleep

import traceback

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)
from fxUtil import *

def printDevice(actPackState):
    print('State time: ', actPackState.timestamp)
    print('Accel X: ', actPackState.accelx, ', Accel Y: ', actPackState.accely, ' Accel Z: ', actPackState.accelz)
    print('Gyro X: ', actPackState.gyrox, ', Gyro Y: ', actPackState.gyroy, ' Gyro Z: ', actPackState.gyroz)
    print('Motor angle: ', actPackState.encoderAngle, ', Motor voltage: ', actPackState.motorVoltage, flush=True)


def fxSync2Device(leaderPort, followerPort, baudRate):

    devId0 = fxOpen(leaderPort, baudRate, 0)
    devId1 = fxOpen(followerPort, baudRate, 0)

    fxStartStreaming(devId0, 200, True)
    fxStartStreaming(devId1, 200, True)

    sleep(0.2)

    actPackState0 = fxReadDevice(devId0)	
    actPackState1 = fxReadDevice(devId1)

    for i in range(3):
        initialAngle0 = actPackState0.encoderAngle
        initialAngle1 = actPackState1.encoderAngle

        print(initialAngle0, initialAngle1)

    kp = 50
    kd = 0

    sleep(0.5)

    fxSetGains(devId0, kp, kd, 0, 0, 0)
    fxSendMotorCommand(devId0, FxPosition, initialAngle0)

    fxSetGains(devId1, kp, kd, 0, 0, 0)
    fxSendMotorCommand(devId1, FxPosition, initialAngle1)

    # sleep(5)

    count = 0
    try:
        while(True):
            sleep(0.05)

            Data0 = fxReadDevice(devId0)
            Data1 = fxReadDevice(devId1)

            angle0 = Data0.encoderAngle
            angle1 = Data1.encoderAngle
            
            diff0 = angle0 - initialAngle0
            diff1 = angle1 - initialAngle1
            
            fxSendMotorCommand(devId0, FxPosition, initialAngle0 + diff1)
            fxSendMotorCommand(devId1, FxPosition, initialAngle1 + diff0)

            
            print("device {} following device {}".format(devId1, devId0))
            
            # printDevice(Data0)
            # printDevice(Data1)

            print('dev0: ', angle0, initialAngle0)
            print('dev1: ', angle1, initialAngle1)
            print(diff0, diff1)

    except Exception as e:
        print(traceback.format_exc())
        print('Turning off position control...')
        fxClose(devId0)
        fxClose(devId1)

    print('Turning off position control...')
    fxClose(devId0)
    fxClose(devId1)

if __name__ == '__main__':
    baudRate = sys.argv[1]
    ports = sys.argv[2:4]
    try:
        fxLeaderFollower(ports[0], ports[1], baudRate)
    except Exception as e:
        print("broke: " + str(e))
        pass
