// ArmController.h

#ifndef _ARMCONTROLLER_h
#define _ARMCONTROLLER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Actuator.h"
#include "LittleFS.h"
#include "FS.h"
#include "CircularBuffer.h"
#include "FreeRTOS/portmacro.h"
#include "FreeRTOS/task.h"
#include "esp_task_wdt.h"

class ARMATRON_Arm_L;

enum ArmEvent {
    ARM_EVT_COMMAND_START,
    ARM_EVT_COMMAND_END
};

enum ArmCommandType {
    ARM_CMD_GOTO,
    ARM_CMD_GOTO_SYNCH,
    ARM_CMD_HOME,
    ARM_CMD_SET_ENABLED,
    ARM_CMD_RUN_SPEED
};

struct ArmAngles {
    double base;
    double shoulder;
    double elbow;
    double aux;

    ArmAngles(double b, double s, double e, double a) {
        base = b;
        shoulder = s;
        elbow = e;
        aux = a;
    }
};

class ArmCommand {
public:
    ArmCommandType type;

    virtual void ToJson(JsonObject* obj) {
        (*obj)["type"] = type;
    }

    virtual void Execute(ARMATRON_Arm_L* arm) {};
};

class ArmCommand_Goto : public ArmCommand {
public:
    ArmCommand_Goto(ArmAngles tgt);

    static ArmCommand_Goto* FromJson(JsonObject* obj_p);
    void ToJson(JsonObject* obj) override;

    void Execute(ARMATRON_Arm_L* arm) override;

    ArmAngles target;
};

class ArmCommand_GotoSynch : public ArmCommand {
public:
    ArmCommand_GotoSynch(ArmAngles tgt, float t);

    static ArmCommand_GotoSynch* FromJson(JsonObject* obj_p);
    void ToJson(JsonObject* obj) override;

    void Execute(ARMATRON_Arm_L* arm) override;

    ArmAngles target;
    float time;
};

class ArmCommand_RunSpeed : public ArmCommand {
public:
    ArmCommand_RunSpeed(ArmAngles s, bool obey_limits = true);

    static ArmCommand_RunSpeed* FromJson(JsonObject* obj_p);
    void ToJson(JsonObject* obj);

    void Execute(ARMATRON_Arm_L* arm) override;

    ~ArmCommand_RunSpeed() {
        Serial.println("destructor runspeed");
    }

    ArmAngles speeds;

    bool limits = false;
};

class ArmCommand_Home : public ArmCommand {
public:
    ArmCommand_Home(int actuator);

    static ArmCommand_Home* FromJson(JsonObject* obj_p);
    void ToJson(JsonObject* obj) override;

    void Execute(ARMATRON_Arm_L* arm) override;

    int Actuator_ID;
};

class ARMATRON_Arm_L {
public:
    ARMATRON_Arm_L();

    void Init();

    void SetupActuators();
    void SetConfig(int Actuator, String key, double value);
    void SaveConfig();
    void ReadConfig();
    void OnEvent(void(*event_func)(ArmEvent type));

    bool QueueCommand(ArmCommand* cmd);
    void AbortCommand();

    void Move(ArmAngles target);
    void SynchronousMove(ArmAngles target, double time);
    void Stop();

    Actuator* Base;
    Actuator* Shoulder;
    Actuator* Elbow;
    Actuator* Aux;

    MotionState ArmState = IDLE;

    CircularBuffer<ArmCommand*, 64> buffer;

    ArmCommand* CurrentCommand = nullptr;

    DynamicJsonDocument ArmConfig;

private:
    void ExecuteNextCommand();

    static void motion_task(void* param);
    static void update_task(void* param);

    void(*event_f)(ArmEvent type);
};

#endif

