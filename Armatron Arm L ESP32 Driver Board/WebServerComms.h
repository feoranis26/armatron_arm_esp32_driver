// WebServerComms.h

#ifndef _WEBSERVERCOMMS_h
#define _WEBSERVERCOMMS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "ARMATRON_Arm_L.h"
#include <AsyncWebServer_ESP32_ENC.h>

class WebServerComms {
public:
	static void Init(ARMATRON_Arm_L* a);
	static AsyncWebServer* server;
	static AsyncEventSource* events;
	static ARMATRON_Arm_L* arm;
};

#endif

