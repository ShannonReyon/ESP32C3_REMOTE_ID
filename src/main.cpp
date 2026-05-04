#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEAdvertising.h>

BLEAdvertising *pAdvertising;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("v0.2 BLE name advertising");

    // Initialize BLE
    BLEDevice::init("XIAO-RID-TEST");

    // Get advertising object
    pAdvertising = BLEDevice::getAdvertising();

    // Start advertising
    pAdvertising->start();

    Serial.println("BLE advertising started");
}

void loop()
{

    delay(1000);
}