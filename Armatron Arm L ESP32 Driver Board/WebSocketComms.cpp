#include "WebSocketComms.h"

WebSocketComms* WebSocketComms::this_;

void WebSocketComms::Init(AsyncWebServer* server, ARMATRON_Arm_L* a)
{
    CommandSrv = new AsyncWebSocket("/cmd");
    server->addHandler(CommandSrv);
    CommandSrv->onEvent(on_event);

    DataSrv = new AsyncWebSocket("/data");
    server->addHandler(DataSrv);

    arm = a;
    this_ = this;

    arm->OnEvent(on_arm_event);

    xTaskCreate(task, "ws_update", 4096, NULL, 0, NULL);
}

void WebSocketComms::on_event(AsyncWebSocket* s, AsyncWebSocketClient* c, AwsEventType t, void* arg, uint8_t* data, size_t len)
{
    switch (t) {

    case WS_EVT_DATA:
        this_->on_cmd_message(c, String((char*)data, len));
        break;

    case WS_EVT_CONNECT:
        ESP_LOGI("WebSkt", "Client connected : %s.", c->remoteIP().toString().c_str());
        break;

    case WS_EVT_DISCONNECT:
        ESP_LOGI("WebSkt", "Client disconnected : %s.", c->remoteIP().toString().c_str());
        break;
    }
}

void WebSocketComms::on_arm_event(ArmEvent t)
{
    Serial.printf("Arm event!");
    switch (t) {
    case ARM_EVT_COMMAND_START:
        this_->send_command_event("cmd_start");
        break;

    case ARM_EVT_COMMAND_END:
        this_->send_command_event("cmd_end");
        break;
    }
}

void WebSocketComms::task(void* param)
{
    while (true) {
        delay(100);
        this_->send_data();
    }
}

void WebSocketComms::on_cmd_message(AsyncWebSocketClient* c, String message)
{
    ESP_LOGI("WebSkt", "Received message from %s : %s.", c->remoteIP().toString().c_str(), message.c_str());

    DynamicJsonDocument d(16384);
    deserializeJson(d, message);

    if (d["action"] == "execute")
        on_cmd_message_recv_cmd(c, &d);
    else if (d["action"] == "abort") {
        send_status_response(c, arm->CurrentCommand != nullptr ? "ok" : "no_cmd_to_abort");
        arm->AbortCommand();
    }
}

void WebSocketComms::on_cmd_message_recv_cmd(AsyncWebSocketClient* c, DynamicJsonDocument* dp)
{
    int initial_size = arm->buffer.size();

    DynamicJsonDocument& d = *dp;

    DynamicJsonDocument resp(256);
    resp["type"] = "cmd_feedback";
    resp["status"] = "ok";

    bool send = true;

    for (JsonObject obj : d["commands"].as<JsonArray>()) {
        ArmCommand* cmd;
        switch (obj["type"].as<ArmCommandType>()) {
        case ARM_CMD_GOTO:
            cmd = ArmCommand_Goto::FromJson(&obj);
            break;

        case ARM_CMD_GOTO_SYNCH:
            cmd = ArmCommand_GotoSynch::FromJson(&obj);
            break;

        case ARM_CMD_HOME:
            cmd = ArmCommand_Home::FromJson(&obj);
            break;

        case ARM_CMD_RUN_SPEED:
            cmd = ArmCommand_RunSpeed::FromJson(&obj);
            send = false;
            break;
        }

        if (!arm->QueueCommand(cmd)) {
            resp["status"] = "buffer_full";
            resp["comitted"] = arm->buffer.size() - initial_size;
            break;
        }
    }

    resp["buffered"] = arm->buffer.size();

    if (!send)
        return;

    String resp_str;
    serializeJson(resp, resp_str);
    c->text(resp_str);
}

void WebSocketComms::send_data() {
    DynamicJsonDocument response(8192);

    response["state"] = arm->ArmState;
    response["buffered"] = arm->buffer.size();

    JsonArray angles_array = response.createNestedArray("joints");

    angles_array.add(arm->Base->GetCurrentRadians());
    angles_array.add(arm->Shoulder->GetCurrentRadians());
    angles_array.add(arm->Elbow->GetCurrentRadians());
    angles_array.add(arm->Aux->GetCurrentRadians());

    JsonArray actuators_array = response.createNestedArray("joints");


    Actuator* actuators[] = { arm->Base, arm->Shoulder, arm->Elbow, arm->Aux };

    for (int i = 0; i < 4; i++) {

        Actuator* actuator = actuators[i];

        JsonObject obj = actuators_array.createNestedObject();
        obj["state"] = actuator->GetState();
        obj["current"] = actuator->GetCurrentRadians();
        obj["target"] = actuator->GetStepper()->getTargetPositionInRevolutions() * TWO_PI;
        obj["speed"] = actuator->GetStepper()->getCurrentVelocityInRevolutionsPerSecond() * TWO_PI;
    }

    String response_str;
    serializeJson(response, response_str);
    DataSrv->textAll(response_str);
}

void WebSocketComms::send_command_event(String evt_type)
{
    if (arm->CurrentCommand->type == ARM_CMD_RUN_SPEED)
        return;

    DynamicJsonDocument resp(1024);
    resp["type"] = "event";
    resp["event_type"] = evt_type;
    JsonObject cmd_obj = resp.createNestedObject("command");
    arm->CurrentCommand->ToJson(&cmd_obj);

    String response_str;
    serializeJson(resp, response_str);
    CommandSrv->textAll(response_str);
}

void WebSocketComms::send_status_response(AsyncWebSocketClient* c, String status)
{
    DynamicJsonDocument resp(256);
    resp["type"] = "cmd_feedback";
    resp["status"] = status;

    String resp_str;
    serializeJson(resp, resp_str);
    c->text(resp_str);
}

WebSocketComms WS_Comms;
