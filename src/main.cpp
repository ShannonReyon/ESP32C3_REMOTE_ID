#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEAdvertising.h>

BLEAdvertising *pAdvertising;

// Example drone data for v0.4 custom payload test
uint32_t droneID = 1001;
double latitude = 52.2749000;
double longitude = 10.5234000;
uint16_t altitude_m = 120;
float speed_mps = 5.25;
float heading_deg = 90.50;

// Status flags
bool armed = true;
bool flying = true;
bool gpsFix = true;
bool emergencyState = false;
bool lowBattery = false;
bool failsafe = false;

uint8_t sequenceCounter = 0;

void writeUint16LE(uint8_t *buffer, int index, uint16_t value)
{
    buffer[index] = value & 0xFF;
    buffer[index + 1] = (value >> 8) & 0xFF;
}

void writeUint32LE(uint8_t *buffer, int index, uint32_t value)
{
    buffer[index] = value & 0xFF;
    buffer[index + 1] = (value >> 8) & 0xFF;
    buffer[index + 2] = (value >> 16) & 0xFF;
    buffer[index + 3] = (value >> 24) & 0xFF;
}

void writeInt32LE(uint8_t *buffer, int index, int32_t value)
{
    buffer[index] = value & 0xFF;
    buffer[index + 1] = (value >> 8) & 0xFF;
    buffer[index + 2] = (value >> 16) & 0xFF;
    buffer[index + 3] = (value >> 24) & 0xFF;
}

uint8_t buildStatusFlags()
{
    uint8_t flags = 0;

    if (armed) flags |= (1 << 0);
    if (flying) flags |= (1 << 1);
    if (gpsFix) flags |= (1 << 2);
    if (emergencyState) flags |= (1 << 3);
    if (lowBattery) flags |= (1 << 4);
    if (failsafe) flags |= (1 << 5);

    return flags;
}

void updateAdvertisement()
{
    BLEAdvertisementData advData;

    // Important: do not add a BLE name here.
    // The 24-byte manufacturer payload is too large to reliably fit together with a name.

    // Manufacturer data total = 24 bytes:
    // [0..1]   Company ID, dummy value 0x1234, little-endian
    // [2]      Protocol Version
    // [3]      Message Type
    // [4..7]   Drone ID
    // [8..11]  Latitude, int32, degrees * 1e7
    // [12..15] Longitude, int32, degrees * 1e7
    // [16..17] Altitude, uint16, meters
    // [18..19] Speed, uint16, m/s * 100
    // [20..21] Heading, uint16, degrees * 100
    // [22]     Status flags
    // [23]     Sequence counter
    uint8_t mfgData[24] = {0};

    mfgData[0] = 0x34;
    mfgData[1] = 0x12;
    mfgData[2] = 0x01;
    mfgData[3] = 0x01;

    int32_t lat_encoded = (int32_t)(latitude * 1e7);
    int32_t lon_encoded = (int32_t)(longitude * 1e7);
    uint16_t speed_encoded = (uint16_t)(speed_mps * 100.0f);
    uint16_t heading_encoded = (uint16_t)(heading_deg * 100.0f);
    uint8_t statusFlags = buildStatusFlags();

    writeUint32LE(mfgData, 4, droneID);
    writeInt32LE(mfgData, 8, lat_encoded);
    writeInt32LE(mfgData, 12, lon_encoded);
    writeUint16LE(mfgData, 16, altitude_m);
    writeUint16LE(mfgData, 18, speed_encoded);
    writeUint16LE(mfgData, 20, heading_encoded);
    mfgData[22] = statusFlags;
    mfgData[23] = sequenceCounter;

    advData.setManufacturerData(std::string((char *)mfgData, sizeof(mfgData)));

    pAdvertising->stop();
    pAdvertising->setAdvertisementData(advData);
    pAdvertising->start();

    Serial.println("Broadcasting v0.4 custom drone payload");
    Serial.print("Drone ID: "); Serial.println(droneID);
    Serial.print("Latitude: "); Serial.println(latitude, 7);
    Serial.print("Longitude: "); Serial.println(longitude, 7);
    Serial.print("Altitude: "); Serial.print(altitude_m); Serial.println(" m");
    Serial.print("Speed: "); Serial.print(speed_mps); Serial.println(" m/s");
    Serial.print("Heading: "); Serial.print(heading_deg); Serial.println(" deg");
    Serial.print("Status flags: 0x"); Serial.println(statusFlags, HEX);
    Serial.print("Sequence: "); Serial.println(sequenceCounter);
    Serial.println();
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("v0.4 Full custom drone payload broadcast");

    BLEDevice::init("");
    pAdvertising = BLEDevice::getAdvertising();

    updateAdvertisement();
}

void loop()
{
    delay(1000);

    // Simulate small movement east and heading change
    longitude += 0.0000100;
    heading_deg += 2.0;
    if (heading_deg >= 360.0)
    {
        heading_deg -= 360.0;
    }

    sequenceCounter++;
    updateAdvertisement();
}