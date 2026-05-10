#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEAdvertising.h>

BLEAdvertising *pAdvertising;
uint8_t counter = 0;

void updateAdvertisement()
{
    BLEAdvertisementData advData;

    advData.setName("RID-TEST");

    // Manufacturer data format:
    // [0..1] Company ID, little-endian dummy value 0x1234
    // [2]    Protocol version
    // [3]    Message type
    // [4]    Counter
    // [5]    Test/status byte
    uint8_t mfgData[6];

    mfgData[0] = 0x34;
    mfgData[1] = 0x12;
    mfgData[2] = 0x01;
    mfgData[3] = 0x01;
    mfgData[4] = counter;
    mfgData[5] = 0xA5;

    advData.setManufacturerData(std::string((char *)mfgData, sizeof(mfgData)));

    pAdvertising->stop();
    pAdvertising->setAdvertisementData(advData);
    pAdvertising->start();

    Serial.print("Broadcasting manufacturer data: ");
    for (size_t i = 0; i < sizeof(mfgData); i++)
    {
        if (mfgData[i] < 0x10) Serial.print("0");
        Serial.print(mfgData[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("v0.3 BLE custom manufacturer data broadcast");

    BLEDevice::init("RID-TEST");
    pAdvertising = BLEDevice::getAdvertising();

    updateAdvertisement();
}

void loop()
{
    delay(1000);

    counter++;
    updateAdvertisement();
}