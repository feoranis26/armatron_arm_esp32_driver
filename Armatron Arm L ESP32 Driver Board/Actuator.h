#ifndef Actuator_h
#define Actuator_h

//ext step 4
//ext dir 13
//ext sensor 35

//2 dir 32
//2 step 33
//2 sensor 18

//3 dir 21
//3 step 19
//3 sensor 34

#include <ArduinoJson.h>
#include <ESP_FlexyStepper.h>

enum MotionState {
    UNKNOWN_POS,
    IDLE,
    MOVING,
    HOMING,
    ERROR,
    RUN_SPEED
};

class Actuator {
public:
    Actuator(uint8_t sensor, uint8_t dir, uint8_t step, DynamicJsonDocument* cnf = nullptr);

    void Init();

    void SetConfig(JsonObject cnf);

    void GoToRadians(double rads, bool wait = false);
    void RunSpeed(double speed, bool limits = true);
    void WaitUntilIdle();
    void StartFixedSpeedMove(double rads, double speed);

    void SetMaxSpeed(double speed);

    ESP_FlexyStepper* GetStepper();

    double GetCurrentRadians();

    MotionState GetState();

    void Home();
    bool HasBeenHomed();

    void CancelBacklashForDirection(bool positive);

    void Stop();

private:
    uint8_t pin_sensor;
    uint8_t pin_dir;
    uint8_t pin_step;

    ESP_FlexyStepper* stepper;

    DynamicJsonDocument config;

    MotionState state = UNKNOWN_POS;

    bool hasBeenHomed = false;

    bool lastMovePositive = false;
    bool backlashCancel = true;
    bool runSpeed = false;
    bool runSpeedLimits = true;

    static DynamicJsonDocument GetDefaultConfig() {
        DynamicJsonDocument cnf(2048);

        cnf["reversed"] = false;

        cnf["maxSpeed"] = 0.0625; //0.25
        cnf["maxAcc"] = 0.125; //0.25

        cnf["homeSpeedFast"] = 0.0625;
        cnf["homeSpeedSlow"] = 0.00625;
        cnf["homeRadians"] = 0;
        cnf["preHomeRadians"] = 0.1;

        cnf["stepMultiplier"] = 32;

        cnf["backlashRads"] = 0.00325;

        cnf["stepsPerRev"] = 4366.0;

        return cnf;
    }

    static void home_task(void* param) {
        Actuator* actuator = (Actuator*)param;
        ESP_FlexyStepper* stepper = actuator->GetStepper();
        Serial.println("Homing actuator");

        actuator->state = HOMING;
        actuator->backlashCancel = false;

        if (actuator->home_action()) {
            actuator->hasBeenHomed = true;
            actuator->state = IDLE;
            Serial.println("home ok");
        }
        else {
            actuator->state = ERROR;
            Serial.println("home error");
        }

        actuator->restoreConfigSpeeds();
        actuator->backlashCancel = true;

        vTaskDelete(NULL);
        delay(0);
    }

    bool home_action() {
        restoreConfigSpeeds();
        stepper->setTargetPositionRelativeInRevolutions(config["preHomeRadians"].as<double>() / TWO_PI);
        while (abs(stepper->getDistanceToTargetSigned()) > 0)
            delay(20);

        stepper->setSpeedInRevolutionsPerSecond(config["homeSpeedFast"]);

        stepper->setTargetPositionRelativeInRevolutions(-1);
        bool sensorState = digitalRead(pin_sensor);

        while ((digitalRead(pin_sensor) == sensorState) && (abs(stepper->getDistanceToTargetSigned()) > 0))
            delay(20);

        delay(50);

        if (digitalRead(pin_sensor) == sensorState)
            return false;

        while (!digitalRead(pin_sensor))
            delay(20);

        stepper->setCurrentPositionInRevolutions(0);
        stepper->setSpeedInRevolutionsPerSecond(config["homeSpeedSlow"]);
        stepper->setTargetPositionRelativeInRevolutions(0.1);

        delay(10);

        while (abs(stepper->getCurrentVelocityInRevolutionsPerSecond()) < 0)
            delay(10);

        while (digitalRead(pin_sensor) && (abs(stepper->getDistanceToTargetSigned()) > 0))
            delay(10);

        if (digitalRead(pin_sensor))
            return false;

        long pos = stepper->getCurrentPositionInSteps();
        stepper->setTargetPositionInSteps(pos);

        while (stepper->getCurrentVelocityInStepsPerSecond() > 0)
            delay(20);

        stepper->setCurrentPositionInRevolutions(config["homeRadians"].as<double>() / (2 * PI));
        stepper->setTargetPositionInSteps(0);

        restoreConfigSpeeds();
        backlashCancel = true;

        while (abs(stepper->getDistanceToTargetSigned()) > 0)
            delay(20);

        return true;
    }

    void restoreConfigSpeeds() {
        stepper->setAccelerationInRevolutionsPerSecondPerSecond(config["maxAcc"].as<double>());
        stepper->setDecelerationInRevolutionsPerSecondPerSecond(config["maxAcc"].as<double>());
        stepper->setSpeedInRevolutionsPerSecond(config["maxSpeed"].as<double>());
    }

    void processAntiBacklash() {
        if (lastMovePositive && stepper->getCurrentVelocityInRevolutionsPerSecond() < 0)
        {
            lastMovePositive = false;
            stepper->setCurrentPositionInRevolutions(stepper->getCurrentPositionInRevolutions() + config["backlashRads"].as<double>());

            Serial.println("Reversed dir now going negative");
        }
        else if (!lastMovePositive && stepper->getCurrentVelocityInRevolutionsPerSecond() > 0)
        {
            lastMovePositive = true;
            stepper->setCurrentPositionInRevolutions(stepper->getCurrentPositionInRevolutions() - config["backlashRads"].as<double>());
            Serial.println("Reversed dir now going positive");
        }
    }

    static Actuator* actuators[16];
    static int numActuators;

    static void update_task(void* param) {
        Actuator* actuator = (Actuator*)param;
        ESP_FlexyStepper* stepper = actuator->GetStepper();
        Serial.println("actuator service started, spinning.");

        while (true) {
            if (actuator->runSpeed)
                actuator->state = RUN_SPEED;
            else if (actuator->state == IDLE && (stepper->getCurrentVelocityInRevolutionsPerSecond() != 0 || stepper->getDistanceToTargetSigned() != 0))
                actuator->state = MOVING;
            else if (actuator->state == MOVING && !(stepper->getCurrentVelocityInRevolutionsPerSecond() != 0 || stepper->getDistanceToTargetSigned() != 0))
                actuator->state = IDLE;

            if (actuator->backlashCancel)
                actuator->processAntiBacklash();

            if (actuator->runSpeed && !actuator->runSpeedLimits)
                actuator->stepper->setCurrentPositionInSteps(0);

            delay(10);
        }
    }

    static void motion_task(void* param) {
        Actuator* actuator = (Actuator*)param;
        ESP_FlexyStepper* stepper = actuator->GetStepper();
        Serial.println("actuator motion task started, spinning.");

        while (true) {
            //actuator->ProcessMovement();
            delayMicroseconds(150);

            stepper->processMovement();
        }
    }
};


#endif