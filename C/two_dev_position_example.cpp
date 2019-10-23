#include "two_dev_position_example.h"
#include "read_all_example.h"

#include <chrono>
#include <thread>
#include <iostream>
#include "utils.h"
#include "Exo.h"
#include "device_wrapper.h"

using namespace std;
using namespace std::literals::chrono_literals;

void runTwoDevicePositionControl(int devId0, int devId1, bool* shouldQuit)
{
	FxError errCode[2];

	ExoState readData[2];
	
	//
	// Start streaming the data
	//
	errCode[0] = fxStartStreaming(devId0, true);
	errCode[1] = fxStartStreaming(devId1, true);
	if(errCode[0] != ESuccess || errCode[1] != ESuccess)
	{
		cout << "Streaming failed ..." << endl;
		return;
	}

	//
	// Find the initial angles
	//
	int initialAngle[2] = {-1, -1};
	
	errCode[0] = fxReadDevice(devId0, &readData[0]);
	errCode[1] = fxReadDevice(devId1, &readData[1]);
	if(errCode[0] != ESuccess || errCode[1] != ESuccess)
	{
		cout << "Reading failed ..." << endl;
		return;
	}

	initialAngle[0] = readData[0]._execute._motor_data._motor_angle;
	initialAngle[1] = readData[1]._execute._motor_data._motor_angle;

	cout << "Initial angles are: " << initialAngle[0] << ", " << initialAngle[1] << endl;

	//
	// Set position controller for both devices
	//
	fxSetGains(devId0, 50, 3, 0, 0);
	fxSetGains(devId1, 50, 3, 0, 0);
	
	fxSendMotorCommand(devId0, EPosition, initialAngle[0]);
	fxSendMotorCommand(devId1, EPosition, initialAngle[1]);

	while(! *shouldQuit)
	{
		this_thread::sleep_for(50ms);
		clearScreen();
		
		errCode[0] = fxReadDevice(devId0, &readData[0]);
		errCode[1] = fxReadDevice(devId1, &readData[1]);
		if(errCode[0] != ESuccess || errCode[1] != ESuccess)
		{
			cout << "Reading failed ..." << endl;
			return;
		}

		cout << "Holding position, two devices: " << endl;
		displayState(readData[0]);
		displayState(readData[1]);
	}

	return;
}
