#include "WebServerComms.h"

ARMATRON_Arm_L* WebServerComms::arm;
AsyncWebServer* WebServerComms::server;
AsyncEventSource* WebServerComms::events;

void WebServerComms::Init(ARMATRON_Arm_L* a) {
    arm = a;

    server = new AsyncWebServer(80);
    events = new AsyncEventSource("/events");

    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, PUT");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");

    server->on("/goto", [](AsyncWebServerRequest* r) {
        if (!r->hasParam("base") || !r->hasParam("shoulder") || !r->hasParam("elbow"))
        {
            r->send(400, "text/html", "Invalid request");
            return;
        }

        float base = atof(r->getParam("base")->value().c_str());
        float shoulder = atof(r->getParam("shoulder")->value().c_str());
        float elbow = atof(r->getParam("elbow")->value().c_str());

        arm->Base->GoToRadians(base);
        arm->Shoulder->GoToRadians(shoulder);
        arm->Elbow->GoToRadians(elbow);

        r->send(200, "text/html", "ok");
        });

    server->on("/goto_simultaneus", [](AsyncWebServerRequest* r) {
        if (!r->hasParam("base") || !r->hasParam("shoulder") || !r->hasParam("elbow") || !r->hasParam("time"))
        {
            r->send(400, "text/html", "Invalid request");
            return;
        }

        float base = atof(r->getParam("base")->value().c_str());
        float shoulder = atof(r->getParam("shoulder")->value().c_str());
        float elbow = atof(r->getParam("elbow")->value().c_str());
        float time = atof(r->getParam("time")->value().c_str());

        //arm->SynchronousMove(ArmAngles(base, shoulder, elbow, 0), time);

        r->send(200, "text/html", "ok");
        });

    server->on("/base/home", [](AsyncWebServerRequest* r) {
        arm->Base->Home();
        r->send(200, "text/html", "ok");
        });

    server->on("/shoulder/home", [](AsyncWebServerRequest* r) {
        arm->Shoulder->Home();
        r->send(200, "text/html", "ok");
        });

    server->on("/elbow/home", [](AsyncWebServerRequest* r) {
        arm->Elbow->Home();
        r->send(200, "text/html", "ok");
        });

    server->on("/setConfig", [](AsyncWebServerRequest* r) {
        if (!r->hasParam("key"))
        {
            r->send(400, "text/html", "Invalid request");
            return;
        }

        int actuator = r->hasParam("actuator") ? atoi(r->getParam("actuator")->value().c_str()) : -1;

        String key = r->getParam("key")->value();
        double value = r->hasParam("value") ? atof(r->getParam("value")->value().c_str()) : NAN;

        arm->SetConfig(actuator, key, value);

        String config_serialized;
        serializeJson(arm->ArmConfig, config_serialized);

        r->send(200, "application/json", config_serialized);
        });

    server->on("/setMotors", [](AsyncWebServerRequest* r) {
        if (!r->hasParam("enabled"))
        {
            r->send(400, "text/html", "Invalid request");
            return;
        }

        bool enabled = r->getParam("enabled")->value() == "true";
        digitalWrite(23, !enabled);

        r->send(200, "text/html", "ok");
        });

    server->on(
        "/execute",
        HTTP_POST,
        [](AsyncWebServerRequest* request) {},
        NULL,
        [](AsyncWebServerRequest* r, uint8_t* data, size_t len, size_t index, size_t total) {
            if (index != 0)
            {
                Serial.println("Received chunked upload!");
                return;
            }

            Serial.printf("Current core: %d\n", xPortGetCoreID());

            String data_string((const char*)data, len);

            Serial.println("Received data:" + data_string);
            DynamicJsonDocument d(16384);
            deserializeJson(d, data_string);

            int initial_size = arm->buffer.size();

            DynamicJsonDocument resp(256);
            resp["status"] = "ok";

            for (JsonObject obj : d["commands"].as<JsonArray>()) {
                JsonArray angles_array = obj["a"].as<JsonArray>();
                ArmAngles angles(angles_array[0], angles_array[1], angles_array[2], angles_array[3]);
                ArmCommand* cmd;
                switch (obj["type"].as<ArmCommandType>()) {
                case ARM_CMD_GOTO:
                    cmd = ArmCommand_Goto::FromJson(&obj);
                    break;

                case ARM_CMD_GOTO_SYNCH:
                    cmd = ArmCommand_GotoSynch::FromJson(&obj);
                    break;

                case ARM_CMD_RUN_SPEED:
                    cmd = ArmCommand_RunSpeed::FromJson(&obj);
                    break;
                }

                if (!arm->QueueCommand(cmd)) {
                    resp["status"] = "buffer_full";
                    resp["comitted"] = arm->buffer.size() - initial_size;
                    break;
                }
            }

            resp["buffered"] = arm->buffer.size();

            String resp_str;
            serializeJson(resp, resp_str);

            r->send(200, "application/json", resp_str);
        });

    server->addHandler(events);
    server->begin();
}