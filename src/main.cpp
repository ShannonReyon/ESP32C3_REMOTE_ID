#include <Arduino.h>

uint32_t counter = 0;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("XIAO ESP32-C3 Remote ID Project");
    Serial.println("v0.1 Serial heartbeat test");
}

void loop()
{
    Serial.print("Heartbeat: ");
    Serial.println(counter++);

    delay(1000);
}