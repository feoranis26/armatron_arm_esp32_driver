// 
// 
// 


#include "ARMATRON_Arm_L.h"

ARMATRON_Arm_L::ARMATRON_Arm_L() : ArmConfig(8192) {

}

void ARMATRON_Arm_L::Init() {
    SetupActuators();
}

void ARMATRON_Arm_L::SetupActuators()
{
    /*
    //Shoulder
    ShoulderConfig["stepMultipler"] = 16;
    ShoulderConfig["stepsPerRev"] = 3600;

    ShoulderConfig["maxSpeed"] = 0.125;
    ShoulderConfig["maxAcc"] = 0.0625;

    ShoulderConfig["homeSpeedFast"] = 0.0375;
    ShoulderConfig["homeSpeedSlow"] = 0.003125;

    ShoulderConfig["backlashRads"] = 0.0;
    ShoulderConfig["homeRadians"] = -0.0125 + HALF_PI;
    //ShoulderConfig["preHomeRadians"] = HALF_PI / 2;

    //Elbow
    ElbowConfig["backlashRads"] = 0.00625;
    ElbowConfig["homeRadians"] = 0.0125 - PI;
    ElbowConfig["stepsPerRev"] = 4200;

    BaseConfig["homeRadians"] = 0;

    */

    ArmConfig.createNestedObject("Base");
    ArmConfig.createNestedObject("Shoulder");
    ArmConfig.createNestedObject("Elbow");
    ArmConfig.createNestedObject("Aux");
    ArmConfig.createNestedObject("Global");

    Base = new Actuator(25, 26, 27);
    Shoulder = new Actuator(35, 13, 4);
    Elbow = new Actuator(18, 32, 33);
    Aux = new Actuator(34, 21, 19);

    ReadConfig();

    Base->Init();
    Shoulder->Init();
    Elbow->Init();
    Aux->Init();

    xTaskCreatePinnedToCore(motion_task, "arm_step", 8192, this, 5, NULL, 1);
    xTaskCreate(update_task, "arm_update", 8192, this, 1, NULL);
}

void ARMATRON_Arm_L::SetConfig(int Actuator, String key, double value)
{
    Serial.println("Setting config " + key + ": " + String(value));

    String actuator_str = "Global";

    if (Actuator == 0)
        actuator_str = "Base";
    else if (Actuator == 1)
        actuator_str = "Shoulder";
    else if (Actuator == 2)
        actuator_str = "Elbow";
    else if (Actuator == 3)
        actuator_str = "Aux";

    if (isnan(value))
        ArmConfig[actuator_str].remove(key);
    else
        ArmConfig[actuator_str].as<JsonObject>()[key] = value;

    Base->SetConfig(ArmConfig["Base"]);
    Shoulder->SetConfig(ArmConfig["Shoulder"]);
    Elbow->SetConfig(ArmConfig["Elbow"]);
    Aux->SetConfig(ArmConfig["Aux"]);

    SaveConfig();
}

void ARMATRON_Arm_L::SaveConfig()
{
    String config_serialized;
    serializeJson(ArmConfig, config_serialized);

    File config_file = LittleFS.open("/arm_config.json", "w", true);
    config_file.print(config_serialized);
    config_file.close();

    Serial.println("Saved config: " + config_serialized);
}

void ARMATRON_Arm_L::ReadConfig()
{
    File config_file = LittleFS.open("/arm_config.json", "r", true);
    String config_serialized = config_file.readString();
    config_file.close();

    Serial.println("Read config: " + config_serialized);

    ArmConfig = DynamicJsonDocument(8192);

    deserializeJson(ArmConfig, config_serialized);

    Base->SetConfig(ArmConfig["Global"]);
    Shoulder->SetConfig(ArmConfig["Global"]);
    Elbow->SetConfig(ArmConfig["Global"]);
    Aux->SetConfig(ArmConfig["Global"]);

    Base->SetConfig(ArmConfig["Base"]);
    Shoulder->SetConfig(ArmConfig["Shoulder"]);
    Elbow->SetConfig(ArmConfig["Elbow"]);
    Aux->SetConfig(ArmConfig["Aux"]);
}

void ARMATRON_Arm_L::OnEvent(void(*event_func)(ArmEvent type))
{
    event_f = event_func;
}

void ARMATRON_Arm_L::Move(ArmAngles target) {
    Base->GoToRadians(target.base);
    Shoulder->GoToRadians(target.shoulder);
    Elbow->GoToRadians(target.elbow);
    Aux->GoToRadians(target.aux);
}

void ARMATRON_Arm_L::SynchronousMove(ArmAngles target, double seconds) {
    double currentBase = Base->GetCurrentRadians();
    double currentShoulder = Shoulder->GetCurrentRadians();
    double currentElbow = Elbow->GetCurrentRadians();
    double currentAux = Aux->GetCurrentRadians();

    double distBase = target.base - currentBase;
    double distShoulder = target.shoulder - currentShoulder;
    double distElbow = target.elbow - currentElbow;
    double distAux = target.aux - currentAux;

    Base->CancelBacklashForDirection(distBase > 0);
    Shoulder->CancelBacklashForDirection(distShoulder > 0);
    Elbow->CancelBacklashForDirection(distElbow > 0);
    Aux->CancelBacklashForDirection(distAux > 0);

    Base->GoToRadians(currentBase);
    Shoulder->GoToRadians(currentShoulder);
    Elbow->GoToRadians(currentElbow);
    Aux->GoToRadians(currentAux);

    Base->WaitUntilIdle();
    Shoulder->WaitUntilIdle();
    Elbow->WaitUntilIdle();
    Aux->WaitUntilIdle();

    distBase = abs(target.base - Base->GetCurrentRadians());
    distShoulder = abs(target.shoulder - Shoulder->GetCurrentRadians());
    distElbow = abs(target.elbow - Elbow->GetCurrentRadians());
    distAux = abs(target.aux - Aux->GetCurrentRadians());

    double distMax = max(distBase, max(distShoulder, max(distElbow, distAux)));
    double speedMax = distMax / seconds;

    if (speedMax > ArmConfig["Global"].as<JsonObject>()["maxSpeed"])
        seconds *= speedMax / ArmConfig["Global"].as<JsonObject>()["maxSpeed"].as<double>();

    double speedBase = distBase / seconds;
    double speedShoulder = distShoulder / seconds;
    double speedElbow = distElbow / seconds;
    double speedAux = distAux / seconds;

    Serial.println("Fixed speed move speeds: " + String(speedBase) + " " + String(speedShoulder) + " " + String(speedElbow) + " " + String(speedAux));

    Base->StartFixedSpeedMove(target.base, speedBase);
    Shoulder->StartFixedSpeedMove(target.shoulder, speedShoulder);
    Elbow->StartFixedSpeedMove(target.elbow, speedElbow);
    Aux->StartFixedSpeedMove(target.aux, speedAux);
}

void ARMATRON_Arm_L::Stop()
{
    Base->Stop();
    Shoulder->Stop();
    Elbow->Stop();
    Aux->Stop();
}

void ARMATRON_Arm_L::ExecuteNextCommand() {
    if (CurrentCommand != nullptr) {
        event_f(ARM_EVT_COMMAND_END);

        delete CurrentCommand;
        CurrentCommand = nullptr;
    }

    if (!buffer.isEmpty()) {
        CurrentCommand = buffer.pop();
        event_f(ARM_EVT_COMMAND_START);

        CurrentCommand->Execute(this);

        delay(0);
    }
}

void ARMATRON_Arm_L::motion_task(void* param) {
    ARMATRON_Arm_L* c = (ARMATRON_Arm_L*)param;

    ESP_FlexyStepper* stepper1 = c->Base->GetStepper();
    ESP_FlexyStepper* stepper2 = c->Shoulder->GetStepper();
    ESP_FlexyStepper* stepper3 = c->Elbow->GetStepper();
    ESP_FlexyStepper* stepper4 = c->Aux->GetStepper();

    //portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
    //taskENTER_CRITICAL(&myMutex);

    Serial.println("arm stepper motion task started, spinning.");

    while (true) {
        stepper1->processMovement();
        stepper2->processMovement();
        stepper3->processMovement();
        stepper4->processMovement();
        esp_task_wdt_reset();
    }
}

void ARMATRON_Arm_L::update_task(void* param) {
    ARMATRON_Arm_L* c = (ARMATRON_Arm_L*)param;

    Actuator* stepper1 = c->Base;
    Actuator* stepper2 = c->Shoulder;
    Actuator* stepper3 = c->Elbow;
    Actuator* stepper4 = c->Aux;

    Serial.println("arm update task started, spinning.");

    while (true) {
        delay(10);

        if (stepper1->GetState() == ERROR || stepper2->GetState() == ERROR || stepper3->GetState() == ERROR || stepper4->GetState() == ERROR)
            c->ArmState = ERROR;
        else if (stepper1->GetState() == HOMING || stepper2->GetState() == HOMING || stepper3->GetState() == HOMING || stepper4->GetState() == HOMING)
            c->ArmState = HOMING;
        else if (stepper1->GetState() == MOVING || stepper2->GetState() == MOVING || stepper3->GetState() == MOVING || stepper4->GetState() == MOVING)
            c->ArmState = MOVING;
        else if (stepper1->GetState() == RUN_SPEED || stepper2->GetState() == RUN_SPEED || stepper3->GetState() == RUN_SPEED || stepper4->GetState() == RUN_SPEED)
            c->ArmState = RUN_SPEED;
        else
            c->ArmState = IDLE;

        if (c->ArmState == IDLE || c->ArmState == RUN_SPEED)
            c->ExecuteNextCommand();
    }
}

bool ARMATRON_Arm_L::QueueCommand(ArmCommand* c) {
    if (!buffer.isFull()) {
        buffer.unshift(c);
        return true;
    }

    return false;
}

void ARMATRON_Arm_L::AbortCommand()
{
    if (CurrentCommand == nullptr)
        return;

    if (buffer.isEmpty())
        Stop();
    else
        ExecuteNextCommand();
}

ArmCommand_RunSpeed::ArmCommand_RunSpeed(ArmAngles s, bool obey_limits) : speeds(s)
{
    type = ARM_CMD_RUN_SPEED;
    limits = obey_limits;
}

ArmCommand_RunSpeed* ArmCommand_RunSpeed::FromJson(JsonObject* obj_p)
{
    JsonObject& obj = *obj_p;
    JsonArray angles_array = obj["speeds"].as<JsonArray>();
    ArmAngles angles(angles_array[0], angles_array[1], angles_array[2], angles_array[3]);

    return new ArmCommand_RunSpeed(angles, obj["obey_limits"].as<bool>());
}

void ArmCommand_RunSpeed::ToJson(JsonObject* obj)
{
    ArmCommand::ToJson(obj);
    JsonArray angles_array = obj->createNestedArray("speeds");

    angles_array.add(speeds.base);
    angles_array.add(speeds.shoulder);
    angles_array.add(speeds.elbow);
    angles_array.add(speeds.aux);

    (*obj)["obey_limits"] = limits;
}

void ArmCommand_RunSpeed::Execute(ARMATRON_Arm_L* arm)
{
    arm->Base->RunSpeed(speeds.base, limits);
    arm->Shoulder->RunSpeed(speeds.shoulder, limits);
    arm->Elbow->RunSpeed(speeds.elbow, limits);
    arm->Aux->RunSpeed(speeds.aux, limits);
}

ArmCommand_GotoSynch::ArmCommand_GotoSynch(ArmAngles tgt, float t) : target(tgt) {
    type = ARM_CMD_GOTO_SYNCH;
    time = t;
}

ArmCommand_GotoSynch* ArmCommand_GotoSynch::FromJson(JsonObject* obj_p) {
    JsonObject& obj = *obj_p;
    JsonArray angles_array = obj["angles"].as<JsonArray>();
    ArmAngles angles(angles_array[0], angles_array[1], angles_array[2], angles_array[3]);

    return new ArmCommand_GotoSynch(angles, obj["time"]);
}

void ArmCommand_GotoSynch::ToJson(JsonObject* obj)
{
    ArmCommand::ToJson(obj);
    JsonArray angles_array = obj->createNestedArray("angles");

    angles_array.add(target.base);
    angles_array.add(target.shoulder);
    angles_array.add(target.elbow);
    angles_array.add(target.aux);

    (*obj)["time"] = time;
}

void ArmCommand_GotoSynch::Execute(ARMATRON_Arm_L* arm)
{
    arm->SynchronousMove(target, time);
}

ArmCommand_Goto::ArmCommand_Goto(ArmAngles tgt) : target(tgt) {
    type = ARM_CMD_GOTO;
}

ArmCommand_Goto* ArmCommand_Goto::FromJson(JsonObject* obj_p) {
    JsonObject& obj = *obj_p;
    JsonArray angles_array = obj["angles"].as<JsonArray>();
    ArmAngles angles(angles_array[0], angles_array[1], angles_array[2], angles_array[3]);

    return new ArmCommand_Goto(angles);
}

void ArmCommand_Goto::ToJson(JsonObject* obj) {
    ArmCommand::ToJson(obj);
    JsonArray angles_array = obj->createNestedArray("angles");

    angles_array.add(target.base);
    angles_array.add(target.shoulder);
    angles_array.add(target.elbow);
    angles_array.add(target.aux);
}

void ArmCommand_Goto::Execute(ARMATRON_Arm_L* arm) {
    arm->Move(target);
}

ArmCommand_Home::ArmCommand_Home(int actuator)
{
    Actuator_ID = actuator;
    type = ARM_CMD_HOME;
}

ArmCommand_Home* ArmCommand_Home::FromJson(JsonObject* obj_p)
{
    JsonObject& obj = *obj_p;
    return new ArmCommand_Home(obj["actuator"].as<int>());
}

void ArmCommand_Home::ToJson(JsonObject* obj)
{
    ArmCommand::ToJson(obj);
    (*obj)["actuator"] = Actuator_ID;
}

void ArmCommand_Home::Execute(ARMATRON_Arm_L* arm)
{
    switch (Actuator_ID) {
    case 0:
        arm->Base->Home();
        break;

    case 1:
        arm->Shoulder->Home();
        break;

    case 2:
        arm->Elbow->Home();
        break;

    case 3:
        arm->Aux->Home();
        break;
    }
}
