#include <AsyncUDP.h>
#define CONFIG_ASYNC_TCP_RUNNING_CORE 0

#include <AsyncTCP.h>
#include <AsyncWebServer_ESP32_ENC.h>
#include <WiFi.h>
#include <FS.h>
#include <LittleFS.h>
#include <ArduinoJson.hpp>
#include <ArduinoJson.h>
#include <ESP_FlexyStepper.h>
#include <CircularBuffer.h>
#include <ESPmDNS.h>
#include "soc/rtc_wdt.h"

#include "Actuator.h"
#include "ARMATRON_Arm_L.h"
#include "SerialComm.h"
#include "WebSocketComms.h"
#include "WebServerComms.h"

#define ENC_INT 36
#define ENC_CS 12

#define SPI_SCK 22
#define SPI_MOSI 17
#define SPI_MISO 16

ARMATRON_Arm_L* arm = new ARMATRON_Arm_L();
SerialComm* s_comm = new SerialComm();

void setup() {
    Serial.begin(115200);
    Serial.println("Starting up...");
    LittleFS.begin(true);

    //rtc_wdt_protect_off();
    //rtc_wdt_disable();


    bool ok = ETH.begin(SPI_MISO, SPI_MOSI, SPI_SCK, ENC_CS, ENC_INT, 8, 1);

    if(!ok)
        Serial.printf("Ethernet error!\n");

    //WiFi.begin("ARMATRON_NETWORK", "16777216");

    //while (ETH.localIP() == IPAddress(0, 0, 0, 0))
    //    delay(100);


    //WiFi.waitForConnectResult();
    //Serial.printf("WiFi IP: %s\n", WiFi.localIP().toString().c_str());

    Serial.printf("ETH IP: %s\n", ETH.localIP().toString().c_str());

    disableCore0WDT();
    disableCore1WDT();

    arm->Init();
    //s_comm->Init(arm);
    WebServerComms::Init(arm);
    WS_Comms.Init(WebServerComms::server, arm);

    Serial.println("OK");

    MDNS.begin("armatron_controller");
    /*
    arm->Elbow->Home();
    arm->Elbow->WaitUntilIdle();
    delay(2000);

    arm->Shoulder->Home();
    arm->Shoulder->WaitUntilIdle();*/

    Serial.setTimeout(10);

    pinMode(23, OUTPUT);
    digitalWrite(23, LOW);
}

void loop() {
    delay(1000);
    Serial.printf("Free heap    :   %d\n", ESP.getFreeHeap());
    Serial.printf("Block        :   %d\n", heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    Serial.printf("State        :   %d\n", arm->ArmState);
    //rm->Elbow->RunSpeed(0.1);
    //arm->Aux->RunSpeed(0.1);
}
