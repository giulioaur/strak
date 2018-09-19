#pragma once
#include <process.h> 
#include "Dwmapi.h"
#include <Iphlpapi.h>
#include <Wlanapi.h>
#include "errorCodes.hpp"
#include "raspberry.hpp"

using namespace std;

namespace strak{
PROCESS_INFORMATION pi;

int init_raspberry(string pathToScripts) {
	STARTUPINFO si;

	ZeroMemory(&si, sizeof(si));
	si.cb = sizeof(si);
	ZeroMemory(&pi, sizeof(pi));

	// Some cool black magic.
	string command = "putty -ssh pi@192.168.137.32 -pw raspberry -m \"" + pathToScripts + "openCam.txt\"";
	TCHAR *param = new TCHAR[command.size() + 1];
	param[command.size()] = 0;
	std::copy(command.begin(), command.end(), param);

	// Create the process for handling the camera application in the raspberry.
	if (!CreateProcess(NULL,   // No module name (use command line)
		param,        // Command line
		NULL,           // Process handle not inheritable
		NULL,           // Thread handle not inheritable
		FALSE,          // Set handle inheritance to FALSE
		0,              // No creation flags
		NULL,           // Use parent's environment block
		NULL,           // Use parent's starting directory 
		&si,            // Pointer to STARTUPINFO structure
		&pi)           // Pointer to PROCESS_INFORMATION structure
		)
	{
		return TrackingError::NO_RASP;
	}

	return TrackingError::OK;
}

int end_raspberry(string pathToScripts) {
	// Terminate ssh session and close camera.
	TerminateProcess(pi.hProcess, 0);
	CloseHandle(pi.hProcess);
	CloseHandle(pi.hThread);
	string command = "putty -ssh pi@192.168.137.32 -pw raspberry -m \"" + pathToScripts + "closeCam.txt\"";
	system(command.data());

	return 0;
}
}