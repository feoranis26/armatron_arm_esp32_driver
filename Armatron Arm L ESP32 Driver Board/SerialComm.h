#ifndef SerialComm_h
#define SerialComm_h

#include "arduino.h"
#include "Actuator.h"
#include "ARMATRON_Arm_L.h"

class SerialComm {
public:
    void Init(ARMATRON_Arm_L* a);
private:
    void parse();
    void send();
    void serial_send(String data);
    void received(String read);
    static void task(void* p);

    ARMATRON_Arm_L* arm;
    String spliced[32];
    String buffer;

};
#endif // !SerialComm_h