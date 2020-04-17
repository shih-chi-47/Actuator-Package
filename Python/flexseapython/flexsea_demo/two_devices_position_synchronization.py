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
    fxSendMotorCommand(devId0, FxCurrent, 0)

    fxSetGains(devId1, kp, kd, 0, 0, 0)
    fxSendMotorCommand(devId1, FxCurrent, 0)

    # sleep(5)
    
    e0 = 0
    e0_pre = 0
    e0d = 0

    e1 = 0 
    e1_pre = 0
    e1d = 0
    
    G = 0.02
    K = G*1
    B = G*0.8
    B = 0

    tau_lim = 2000

    friction = 650
    # friction = 0

    deadband = 10

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

            e0 = -diff0 + diff1
            e1 = -diff1 + diff0

            e0d = e0 - e0_pre
            e1d = e1 - e1_pre

            tau0 = K*e0 + B*e0d
            tau1 = K*e1 + B*e1d

            if(tau0 >deadband):
                tau0 = tau0 + friction
            elif(tau0 < -deadband):
                tau0 = tau0 - friction
            if(tau1 > deadband):
                tau1 = tau1 + friction
            elif(tau1 < -deadband):
                tau1 = tau1 - friction


            tau0 = min(max(tau0, -tau_lim), tau_lim)
            tau1 = min(max(tau1, -tau_lim), tau_lim)
            
            # fxSendMotorCommand(devId0, FxPosition, tau0)
            # fxSendMotorCommand(devId1, FxPosition, tau1)

            fxSendMotorCommand(devId0, FxCurrent, tau0)
            fxSendMotorCommand(devId1, FxCurrent, tau1)

            
            print("device {} following device {}".format(devId1, devId0))
            
            # printDevice(Data0)
            # printDevice(Data1)


            print('diff: ',diff0, diff1)
            print('error:', e0, e1)
            print('tau:  ', tau0, tau1)
            print('\n')

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
