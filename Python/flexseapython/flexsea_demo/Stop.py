import os, sys
from time import sleep

import traceback

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)
from fxUtil import *

def STOP(leaderPort, followerPort, baudRate):

    devId0 = fxOpen(leaderPort, baudRate, 0)
    devId1 = fxOpen(followerPort, baudRate, 0)

    fxStartStreaming(devId0, 200, True)
    fxStartStreaming(devId1, 200, True)

    # sleep(0.2)

    fxSetGains(devId0, 0, 0, 0, 0, 0)
    fxSendMotorCommand(devId0, FxCurrent, 0)

    fxSetGains(devId1, 0, 0, 0, 0, 0)
    fxSendMotorCommand(devId1, FxCurrent, 0)

    fxStopStreaming(devId0)
    fxClose(devId0)

    fxStopStreaming(devId1)
    fxClose(devId1)

    sleep(1)
