#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEAdvertising.h>

extern "C" {
#include "opendroneid.h"
}

BLEAdvertising *pAdvertising;

ODID_BasicID_data basicID;
ODID_Location_data locationData;
ODID_System_data systemData;

uint8_t odidCounter = 0;

void buildBasicID()
{
    memset(&basicID, 0, sizeof(basicID));

    // Basic ID identifies the drone.
    basicID.IDType = ODID_IDTYPE_SERIAL_NUMBER;
    basicID.UAType = ODID_UATYPE_HELICOPTER_OR_MULTIROTOR;
    strncpy((char *)basicID.UASID, "TESTDRONE123", ODID_ID_SIZE);
}

void buildLocation()
{
    memset(&locationData, 0, sizeof(locationData));

    // Fixed test location for v0.5.
    // Later, GNSS will replace these values.
    locationData.Status = ODID_STATUS_AIRBORNE;
    locationData.Direction = 90.0;
    locationData.SpeedHorizontal = 5.0;
    locationData.SpeedVertical = 0.0;
    locationData.Latitude = 52.2749;
    locationData.Longitude = 10.5234;
    locationData.AltitudeBaro = 100.0;
    locationData.AltitudeGeo = 100.0;
    locationData.HeightType = ODID_HEIGHT_REF_OVER_TAKEOFF;
    locationData.Height = 100.0;
    locationData.HorizAccuracy = ODID_HOR_ACC_10_METER;
    locationData.VertAccuracy = ODID_VER_ACC_10_METER;
    locationData.BaroAccuracy = ODID_VER_ACC_10_METER;
    locationData.SpeedAccuracy = ODID_SPEED_ACC_3_METERS_PER_SECOND;
    locationData.TSAccuracy = ODID_TIME_ACC_1_0_SECOND;
    locationData.TimeStamp = 0;
}

void buildSystem()
{
    memset(&systemData, 0, sizeof(systemData));

    // For this fixed test, we use the same coordinates as operator/takeoff location.
    systemData.OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_TAKEOFF;
    systemData.ClassificationType = ODID_CLASSIFICATION_TYPE_EU;
    systemData.OperatorLatitude = 52.2749;
    systemData.OperatorLongitude = 10.5234;
    systemData.AreaCount = 1;
    systemData.AreaRadius = 0;
    systemData.AreaCeiling = 120;
    systemData.AreaFloor = 0;
    systemData.OperatorAltitudeGeo = 100.0;
    systemData.Timestamp = 0;
}

void advertiseOpenDroneIDMessage(uint8_t *encodedMessage)
{
    uint8_t adv[31] = {0};

    // BLE Service Data advertisement for OpenDroneID:
    // [Length][AD Type][UUID LSB][UUID MSB][ODID App Code][Counter][25-byte ODID message]
    // 0x1E = 30 bytes after the length byte.
    // 0x16 = Service Data - 16-bit UUID.
    // 0xFFFA = ASTM Remote ID / OpenDroneID service UUID, little-endian as FA FF.
    // 0x0D = OpenDroneID application code.
    adv[0] = 0x1E;
    adv[1] = 0x16;
    adv[2] = 0xFA;
    adv[3] = 0xFF;
    adv[4] = 0x0D;
    adv[5] = odidCounter++;

    memcpy(&adv[6], encodedMessage, ODID_MESSAGE_SIZE);

    BLEAdvertisementData advData;
    advData.addData(std::string((char *)adv, 6 + ODID_MESSAGE_SIZE));

    pAdvertising->stop();
    pAdvertising->setAdvertisementData(advData);
    pAdvertising->start();
}

void broadcastNextMessage()
{
    uint8_t message[ODID_MESSAGE_SIZE] = {0};
    uint8_t selector = odidCounter % 3;

    if (selector == 0)
    {
        ODID_BasicID_encoded encoded{};
        encodeBasicIDMessage(&encoded, &basicID);
        memcpy(message, &encoded, ODID_MESSAGE_SIZE);
        Serial.println("Broadcasting OpenDroneID BasicID");
    }
    else if (selector == 1)
    {
        ODID_Location_encoded encoded{};
        encodeLocationMessage(&encoded, &locationData);
        memcpy(message, &encoded, ODID_MESSAGE_SIZE);
        Serial.println("Broadcasting OpenDroneID Location");
    }
    else
    {
        ODID_System_encoded encoded{};
        encodeSystemMessage(&encoded, &systemData);
        memcpy(message, &encoded, ODID_MESSAGE_SIZE);
        Serial.println("Broadcasting OpenDroneID System");
    }

    advertiseOpenDroneIDMessage(message);
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("v0.5 Fixed OpenDroneID BLE broadcaster");

    BLEDevice::init("");
    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinInterval(0xA0); // 100 ms
    pAdvertising->setMaxInterval(0xF0); // 150 ms

    buildBasicID();
    buildLocation();
    buildSystem();
}

void loop()
{
    broadcastNextMessage();
    delay(150);
}