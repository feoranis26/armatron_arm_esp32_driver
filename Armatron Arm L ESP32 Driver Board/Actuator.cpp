#include "Actuator.h"

Actuator::Actuator(uint8_t sensor, uint8_t dir, uint8_t step, DynamicJsonDocument* cnf) : config(1024) {
    pin_sensor = sensor;
    pin_dir = dir;
    pin_step = step;

    stepper = new ESP_FlexyStepper();

    config = GetDefaultConfig();

    if (cnf != nullptr)
        for (JsonPair kvp : (*cnf).as<JsonObject>())
            config[kvp.key()] = kvp.value();
}

void Actuator::SetConfig(JsonObject cnf) {
    for (JsonPair kvp : cnf)
        config[kvp.key()] = kvp.value();

    double current_rads = GetCurrentRadians();

    stepper->setStepsPerRevolution(config["stepsPerRev"].as<double>() * config["stepMultiplier"].as<double>());

    stepper->setAccelerationInRevolutionsPerSecondPerSecond(config["maxAcc"].as<double>());
    stepper->setDecelerationInRevolutionsPerSecondPerSecond(config["maxAcc"].as<double>());
    stepper->setSpeedInRevolutionsPerSecond(config["maxSpeed"].as<double>());

    stepper->setCurrentPositionInRevolutions(current_rads);
}

void Actuator::Init() {
    Serial.println("initializing actuator");// , pins s : " + String(pin_step) + ", d : " + String(pin_dir) + ", sensor : " + String(pin_sensor));
    pinMode(pin_sensor, INPUT);

    stepper->connectToPins(pin_step, pin_dir);
    stepper->setStepsPerRevolution(config["stepsPerRev"].as<double>() * config["stepMultiplier"].as<double>());

    stepper->setAccelerationInRevolutionsPerSecondPerSecond(config["maxAcc"].as<double>());
    stepper->setDecelerationInRevolutionsPerSecondPerSecond(config["maxAcc"].as<double>());
    stepper->setSpeedInRevolutionsPerSecond(config["maxSpeed"].as<double>());

    stepper->setCurrentPositionInRevolutions(0);

    //Serial.println("starting stepper service");
    //stepper->startAsService(0);
    //xTaskCreatePinnedToCore(motion_task, "ActuatorMtnTask", 4096, this, config["core"].as<int>(), NULL, 1);
    xTaskCreate(update_task, "actr_update", 4096, this, 0, NULL);
    Serial.println("ok");
}

void Actuator::GoToRadians(double rads, bool wait) {
    restoreConfigSpeeds();

    rads = min(config["upperLimit"].as<double>(), max(rads, config["lowerLimit"].as<double>()));

    stepper->setTargetPositionInRevolutions(rads / (2.0 * PI));
    state = MOVING;
    runSpeed = false;

    if (wait)
        WaitUntilIdle();
}

void Actuator::RunSpeed(double speed, bool limits) {
    if (speed == 0) {
        Stop();
        runSpeed = false;
        return;
    }

    stepper->setSpeedInRevolutionsPerSecond(abs(speed) / (2.0 * PI));
    stepper->setAccelerationInRevolutionsPerSecondPerSecond(1);
    stepper->setDecelerationInRevolutionsPerSecondPerSecond(1);

    Serial.println(speed);

    runSpeedLimits = limits;

    Serial.printf("Run speed limits: %d\n", runSpeedLimits);


    if (!config.containsKey("upperLimit") || !config.containsKey("lowerLimit"))
        runSpeedLimits = false;


    Serial.printf("Run speed limits: %d\n", runSpeedLimits);

    if (speed > 0)
        stepper->setTargetPositionInRevolutions(runSpeedLimits ? config["upperLimit"].as<double>() / TWO_PI : 1);
    else
        stepper->setTargetPositionInRevolutions(runSpeedLimits ? config["lowerLimit"].as<double>() / TWO_PI : -1);

    state = RUN_SPEED;
    runSpeed = true;
    hasBeenHomed = false;
}

void Actuator::WaitUntilIdle() {
    if (state == UNKNOWN_POS)
        return;

    while (state != IDLE)
        delay(10);
}

void Actuator::StartFixedSpeedMove(double rads, double speed) {
    stepper->setSpeedInRevolutionsPerSecond(speed / (2 * PI));
    stepper->setAccelerationInRevolutionsPerSecondPerSecond(1);
    stepper->setDecelerationInRevolutionsPerSecondPerSecond(1);

    stepper->setTargetPositionInRevolutions(rads / (2 * PI));
    state = MOVING;
}

void Actuator::SetMaxSpeed(double speed) {
    config["maxSpeed"] = speed;
    restoreConfigSpeeds();
}

ESP_FlexyStepper* Actuator::GetStepper() {
    return stepper;
}

double Actuator::GetCurrentRadians() {
    return stepper->getCurrentPositionInRevolutions() * 2.0 * PI;
}

MotionState Actuator::GetState() {
    return state;
}

void Actuator::Home() {
    if ((state != HOMING) && (state != MOVING)) {
        xTaskCreate(home_task, "home_task", 2048, this, 0, NULL);
        state = HOMING;
    }
}

bool Actuator::HasBeenHomed() {
    return hasBeenHomed;
}

void Actuator::CancelBacklashForDirection(bool positive) {
    Serial.println("direction: " + String(positive));
    Serial.println("last dir was: " + String(lastMovePositive));
    if (lastMovePositive && !positive)
    {
        lastMovePositive = false;
        stepper->setCurrentPositionInRevolutions(stepper->getCurrentPositionInRevolutions() + config["backlashRads"].as<double>());

        Serial.println("Reversed dir now going negative");
    }
    else if (!lastMovePositive && positive)
    {
        lastMovePositive = true;
        stepper->setCurrentPositionInRevolutions(stepper->getCurrentPositionInRevolutions() - config["backlashRads"].as<double>());
        Serial.println("Reversed dir now going positive");
    }
}

void Actuator::Stop() {
    stepper->setTargetPositionToStop();
}
