#include <Arduino.h>
//#include <HardwareSerial.h>
#include <src/LiquidCrystal.h>

#include "aDSEngine.h"

#include <HardwareSerial0.cpp>

aDSEngine *engine;

void setup()
{
#if 0
    pinMode(PWM_CHANNEL1_PIN, OUTPUT);
    analogWrite(PWM_CHANNEL1_PIN, 0);

    pinMode(PWM_CHANNEL2_PIN, OUTPUT);
    analogWrite(PWM_CHANNEL2_PIN, 0);
#endif // 0

    Serial.begin(57600);

    engine = new aDSEngine();
    engine->setup();
}

void loop()
{
    engine->run();
}
