// WebSocketComms.h

#ifndef _WEBSOCKETCOMMS_h
#define _WEBSOCKETCOMMS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "ARMATRON_Arm_L.h"
#include <AsyncWebServer_ESP32_ENC.h>

class WebSocketComms {
public:
	void Init(AsyncWebServer* server, ARMATRON_Arm_L* a);

	AsyncWebSocket* CommandSrv;
	AsyncWebSocket* DataSrv;
	ARMATRON_Arm_L* arm;

private:
	static void on_event(AsyncWebSocket* s, AsyncWebSocketClient* c, AwsEventType t, void* arg, uint8_t* data, size_t len);
	static void on_arm_event(ArmEvent t);
	static void task(void* param);

	void on_cmd_message(AsyncWebSocketClient* c, String message);
	void on_cmd_message_recv_cmd(AsyncWebSocketClient* c, DynamicJsonDocument* message);
	void send_data();
	void send_command_event(String evt_type);
	void send_status_response(AsyncWebSocketClient* c, String status);

	static WebSocketComms* this_;
};

extern WebSocketComms WS_Comms;

#endif

