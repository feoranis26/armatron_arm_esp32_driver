#include "SerialComm.h"
#define packet_size 64

String padded(String data, int length) {
    char* p = (char*)malloc(length + 1);
    memcpy(p, data.c_str(), data.length());

    for (int i = data.length(); i < length; i++)
        p[i] = '#';

    p[length] = 0;

    String s(p, length);
    free(p);
    return s;
}

void SerialComm::serial_send(String data) {
    String toSend = padded(padded(String(data.length()), 4) + data, 63) + ";";

    Serial.print(toSend);
}

void SerialComm::Init(ARMATRON_Arm_L* a) {
    arm = a;
    xTaskCreate(task, "serial_t", 8192, this, 0, NULL);
}

void SerialComm::received(String read) {
    //Serial.println("READ:" + read);
    int numSplices = 0;
    int idx = read.indexOf(' ');
    while (idx != -1) {
        idx = read.indexOf(' ');

        String token = read.substring(0, idx);
        //Serial.println(token);
        spliced[numSplices++] = token;
        read = read.substring(idx + 1);
    }

    if (numSplices == 5 && spliced[0] == "go") {
        /*arm->Base->GoToRadians(atof(spliced[1].c_str()));
        arm->Shoulder->GoToRadians(atof(spliced[2].c_str()));
        arm->Elbow->GoToRadians(atof(spliced[3].c_str()));
        arm->Aux->GoToRadians(atof(spliced[4].c_str()));*/
    }
    else if (numSplices == 6 && spliced[0] == "go_timed") {
        ArmAngles angles(atof(spliced[1].c_str()), atof(spliced[2].c_str()), atof(spliced[3].c_str()), atof(spliced[4].c_str()));
        //arm->SynchronousMove(angles, atof(spliced[5].c_str()));
    }

    if (spliced[0] == "home") {
        for (int i = 1; i < numSplices; i++) {
            if (spliced[i] == "elbow")
            {
                arm->Elbow->Home();
                arm->Elbow->WaitUntilIdle();
            }
            else if (spliced[i] == "shoulder") {
                arm->Shoulder->Home();
                arm->Shoulder->WaitUntilIdle();
            }
            else if (spliced[i] == "base") {
                arm->Base->Home();
                arm->Base->WaitUntilIdle();
            }
        }
    }

    if (spliced[0] == "disable") {
        digitalWrite(23, HIGH);
    }

    if (spliced[0] == "enable") {
        digitalWrite(23, LOW);
    }

    if (spliced[0] == "set_config") {
        int num = atoi(spliced[1].c_str());
        double val = atof(spliced[3].c_str());

        if (spliced[3] == "null")
            val = NAN;

        arm->SetConfig(num, spliced[2], val);
    }

    if (spliced[0] == "write_config") {
        DynamicJsonDocument doc(8192);
        String data = spliced[1];
        Serial.println("Rewriting config: " + data);
        deserializeJson(doc, data);

        arm->ArmConfig.set(doc);
        arm->SaveConfig();
    }

    if (spliced[0] == "restart")
        ESP.restart();
}

void SerialComm::parse() {
    while (Serial.available())
        buffer += (char)Serial.read();

    while (buffer.length() >= packet_size)
    {
        //Serial.println("Buffer: \"" + buffer + "\"");

        int index = 0;
        for (; index < min((uint)2 * packet_size, buffer.length()); index++)
            if (buffer[index] == ';')
                break;

        //Serial.println("Index: " + String(index));
        if (index <= packet_size) {
            String split = buffer.substring(0, index);
            //Serial.println(split);
            int i = 0;
            for (; i < 4; i++)
                if (split[i] == '#')
                    break;

            int length = atoi(split.substring(0, i).c_str());
            String data = split.substring(4, length + 4);
            //Serial.println(data);
            received(data);
            buffer = buffer.substring(index + 1);
        }
        else if (index > packet_size && index < 2 * packet_size)
        {
            buffer = buffer.substring(index - (packet_size - 1), index + 1);
        }
        else {
            buffer = buffer.substring(packet_size);
        }
    }
}

void SerialComm::send() {
    if (arm->ArmState == IDLE)
        serial_send("state: IDLE;");
    else if (arm->ArmState == MOVING)
        serial_send("state: MOVING;");
    else if(arm->ArmState == ERROR)
        serial_send("state: ERROR;");
    else if(arm->ArmState == HOMING)
        serial_send("state: HOMING;");

    serial_send("base:  " + String(arm->Base->GetCurrentRadians()));
    serial_send("shoulder:  " + String(arm->Shoulder->GetCurrentRadians()));
    serial_send("elbow:  " + String(arm->Elbow->GetCurrentRadians()));
    serial_send("aux:  " + String(arm->Aux->GetCurrentRadians()));
}

void SerialComm::task(void* p) {
    SerialComm* t = (SerialComm*)p;

    while (true) {
        t->parse();
        t->send();
        delay(250);
    }
}