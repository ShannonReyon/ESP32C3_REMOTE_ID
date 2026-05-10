#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEAdvertising.h>

extern "C" {
#include "opendroneid.h"
}

// =========================================================
// v0.7 GNSS + OpenDroneID BLE integration
// Board: Seeed Studio XIAO ESP32-C3
// GNSS: u-blox M9N using UBX-NAV-PVT
// =========================================================

// Wiring:
//   M9N TX  -> XIAO D7  (ESP32-C3 RX)
//   M9N RX  -> XIAO D6  (ESP32-C3 TX, optional)
//   M9N GND -> XIAO GND
//   M9N VCC -> XIAO 3V3 or suitable external 3.3 V supply

HardwareSerial GNSS(1);

constexpr uint32_t GNSS_BAUD = 230400;
constexpr int GNSS_RX_PIN = D7;
constexpr int GNSS_TX_PIN = D6;

// UBX-NAV-PVT constants
constexpr uint8_t UBX_SYNC_1 = 0xB5;
constexpr uint8_t UBX_SYNC_2 = 0x62;
constexpr uint8_t UBX_CLASS_NAV = 0x01;
constexpr uint8_t UBX_ID_NAV_PVT = 0x07;
constexpr uint16_t NAV_PVT_PAYLOAD_LEN = 92;

// BLE/OpenDroneID
BLEAdvertising *pAdvertising;

ODID_BasicID_data basicID;
ODID_Location_data locationData;
ODID_System_data systemData;

uint8_t odidCounter = 0;
constexpr uint32_t ODID_BROADCAST_INTERVAL_MS = 150;
uint32_t lastBroadcastMs = 0;

struct UtcTime
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    bool validDate;
    bool validTime;
    bool fullyResolved;
};

struct GnssData
{
    double latitude = 52.2749;
    double longitude = 10.5234;
    double height_m = 100.0;
    double height_msl_m = 100.0;
    double horizontal_accuracy_m = 0.0;
    double vertical_accuracy_m = 0.0;
    double ground_speed_mps = 0.0;
    double heading_deg = 0.0;
    uint8_t fixType = 0;
    uint8_t numSatellites = 0;
    bool gnssFixOk = false;
    UtcTime utc{};
    bool utcValid = false;
    uint32_t lastUpdateMs = 0;
};

uint8_t payload[NAV_PVT_PAYLOAD_LEN];
GnssData gnss;

uint16_t readU16LE(const uint8_t *buffer, uint16_t index)
{
    return static_cast<uint16_t>(buffer[index]) |
           (static_cast<uint16_t>(buffer[index + 1]) << 8);
}

uint32_t readU32LE(const uint8_t *buffer, uint16_t index)
{
    return static_cast<uint32_t>(buffer[index]) |
           (static_cast<uint32_t>(buffer[index + 1]) << 8) |
           (static_cast<uint32_t>(buffer[index + 2]) << 16) |
           (static_cast<uint32_t>(buffer[index + 3]) << 24);
}

int32_t readI32LE(const uint8_t *buffer, uint16_t index)
{
    return static_cast<int32_t>(
        static_cast<uint32_t>(buffer[index]) |
        (static_cast<uint32_t>(buffer[index + 1]) << 8) |
        (static_cast<uint32_t>(buffer[index + 2]) << 16) |
        (static_cast<uint32_t>(buffer[index + 3]) << 24));
}

bool parseUtc(const uint8_t *p, UtcTime &utc)
{
    utc.year = readU16LE(p, 4);
    utc.month = p[6];
    utc.day = p[7];
    utc.hour = p[8];
    utc.minute = p[9];
    utc.second = p[10];

    const uint8_t valid = p[11];
    utc.validDate = valid & 0x01;
    utc.validTime = valid & 0x02;
    utc.fullyResolved = valid & 0x04;

    return utc.validDate && utc.validTime;
}

void parseNavPvt(const uint8_t *p, GnssData &data)
{
    const int32_t lon = readI32LE(p, 24);
    const int32_t lat = readI32LE(p, 28);
    const int32_t height = readI32LE(p, 32);
    const int32_t heightMsl = readI32LE(p, 36);
    const uint32_t hAcc = readU32LE(p, 40);
    const uint32_t vAcc = readU32LE(p, 44);
    const int32_t gSpeed = readI32LE(p, 60);
    const int32_t headMot = readI32LE(p, 64);

    data.fixType = p[20];
    const uint8_t flags = p[21];
    data.gnssFixOk = flags & 0x01;
    data.numSatellites = p[23];

    data.longitude = lon * 1e-7;
    data.latitude = lat * 1e-7;
    data.height_m = height / 1000.0;
    data.height_msl_m = heightMsl / 1000.0;
    data.horizontal_accuracy_m = hAcc / 1000.0;
    data.vertical_accuracy_m = vAcc / 1000.0;
    data.ground_speed_mps = gSpeed / 1000.0;
    data.heading_deg = headMot * 1e-5;

    if (data.heading_deg < 0.0) data.heading_deg += 360.0;
    if (data.heading_deg >= 360.0) data.heading_deg -= 360.0;

    data.utcValid = parseUtc(p, data.utc);
    data.lastUpdateMs = millis();
}

bool readUbxNavPvtPacket()
{
    static enum
    {
        WAIT_SYNC_1,
        WAIT_SYNC_2,
        READ_CLASS,
        READ_ID,
        READ_LEN_1,
        READ_LEN_2,
        READ_PAYLOAD,
        READ_CK_A,
        READ_CK_B
    } state = WAIT_SYNC_1;

    static uint8_t msgClass = 0;
    static uint8_t msgId = 0;
    static uint16_t payloadLen = 0;
    static uint16_t payloadIndex = 0;
    static uint8_t ckA = 0;
    static uint8_t ckB = 0;
    static uint8_t receivedCkA = 0;

    while (GNSS.available())
    {
        const uint8_t b = GNSS.read();

        switch (state)
        {
        case WAIT_SYNC_1:
            if (b == UBX_SYNC_1) state = WAIT_SYNC_2;
            break;

        case WAIT_SYNC_2:
            if (b == UBX_SYNC_2)
            {
                ckA = 0;
                ckB = 0;
                state = READ_CLASS;
            }
            else
            {
                state = WAIT_SYNC_1;
            }
            break;

        case READ_CLASS:
            msgClass = b;
            ckA += b;
            ckB += ckA;
            state = READ_ID;
            break;

        case READ_ID:
            msgId = b;
            ckA += b;
            ckB += ckA;
            state = READ_LEN_1;
            break;

        case READ_LEN_1:
            payloadLen = b;
            ckA += b;
            ckB += ckA;
            state = READ_LEN_2;
            break;

        case READ_LEN_2:
            payloadLen |= static_cast<uint16_t>(b) << 8;
            ckA += b;
            ckB += ckA;
            payloadIndex = 0;

            if (payloadLen > NAV_PVT_PAYLOAD_LEN)
            {
                state = WAIT_SYNC_1;
            }
            else if (payloadLen == 0)
            {
                state = READ_CK_A;
            }
            else
            {
                state = READ_PAYLOAD;
            }
            break;

        case READ_PAYLOAD:
            payload[payloadIndex++] = b;
            ckA += b;
            ckB += ckA;

            if (payloadIndex >= payloadLen) state = READ_CK_A;
            break;

        case READ_CK_A:
            receivedCkA = b;
            state = READ_CK_B;
            break;

        case READ_CK_B:
        {
            const uint8_t receivedCkB = b;
            const bool checksumOk = (receivedCkA == ckA) && (receivedCkB == ckB);

            if (checksumOk &&
                msgClass == UBX_CLASS_NAV &&
                msgId == UBX_ID_NAV_PVT &&
                payloadLen == NAV_PVT_PAYLOAD_LEN)
            {
                state = WAIT_SYNC_1;
                return true;
            }

            state = WAIT_SYNC_1;
            break;
        }
        }
    }

    return false;
}

void buildBasicID()
{
    memset(&basicID, 0, sizeof(basicID));
    basicID.IDType = ODID_IDTYPE_SERIAL_NUMBER;
    basicID.UAType = ODID_UATYPE_HELICOPTER_OR_MULTIROTOR;
    strncpy((char *)basicID.UASID, "TESTDRONE123", ODID_ID_SIZE);
}

ODID_Horizontal_accuracy estimateHorizontalAccuracy(double hAcc_m)
{
    if (hAcc_m <= 1.0) return ODID_HOR_ACC_1_METER;
    if (hAcc_m <= 3.0) return ODID_HOR_ACC_3_METER;
    if (hAcc_m <= 10.0) return ODID_HOR_ACC_10_METER;
    if (hAcc_m <= 30.0) return ODID_HOR_ACC_30_METER;
    return ODID_HOR_ACC_UNKNOWN;
}

ODID_Vertical_accuracy estimateVerticalAccuracy(double vAcc_m)
{
    if (vAcc_m <= 1.0) return ODID_VER_ACC_1_METER;
    if (vAcc_m <= 3.0) return ODID_VER_ACC_3_METER;
    if (vAcc_m <= 10.0) return ODID_VER_ACC_10_METER;
    if (vAcc_m <= 45.0) return ODID_VER_ACC_45_METER;
    return ODID_VER_ACC_UNKNOWN;
}

void updateLocationFromGnss()
{
    memset(&locationData, 0, sizeof(locationData));

    const bool freshGnss = (millis() - gnss.lastUpdateMs) < 3000;
    const bool validPosition = freshGnss && gnss.gnssFixOk && gnss.fixType >= 2;

    locationData.Status = validPosition ? ODID_STATUS_AIRBORNE : ODID_STATUS_UNDECLARED;
    locationData.Direction = gnss.heading_deg;
    locationData.SpeedHorizontal = gnss.ground_speed_mps;
    locationData.SpeedVertical = 0.0;
    locationData.Latitude = gnss.latitude;
    locationData.Longitude = gnss.longitude;
    locationData.AltitudeBaro = gnss.height_msl_m;
    locationData.AltitudeGeo = gnss.height_m;
    locationData.HeightType = ODID_HEIGHT_REF_OVER_TAKEOFF;
    locationData.Height = gnss.height_msl_m;
    locationData.HorizAccuracy = estimateHorizontalAccuracy(gnss.horizontal_accuracy_m);
    locationData.VertAccuracy = estimateVerticalAccuracy(gnss.vertical_accuracy_m);
    locationData.BaroAccuracy = estimateVerticalAccuracy(gnss.vertical_accuracy_m);
    locationData.SpeedAccuracy = ODID_SPEED_ACC_3_METERS_PER_SECOND;
    locationData.TSAccuracy = ODID_TIME_ACC_1_0_SECOND;
    locationData.TimeStamp = gnss.utcValid ? (gnss.utc.minute * 60 + gnss.utc.second) : 0;
}

void updateSystemFromGnss()
{
    memset(&systemData, 0, sizeof(systemData));

    // Prototype choice: use current GNSS position as operator/takeoff location.
    // Later this should become the real operator/takeoff position.
    systemData.OperatorLocationType = ODID_OPERATOR_LOCATION_TYPE_TAKEOFF;
    systemData.ClassificationType = ODID_CLASSIFICATION_TYPE_EU;
    systemData.OperatorLatitude = gnss.latitude;
    systemData.OperatorLongitude = gnss.longitude;
    systemData.AreaCount = 1;
    systemData.AreaRadius = 0;
    systemData.AreaCeiling = gnss.height_msl_m + 20.0;
    systemData.AreaFloor = 0;
    systemData.OperatorAltitudeGeo = gnss.height_m;
    systemData.Timestamp = gnss.utcValid ? (gnss.utc.minute * 60 + gnss.utc.second) : 0;
}

void updateOpenDroneIDFromGnss()
{
    updateLocationFromGnss();
    updateSystemFromGnss();
}

void advertiseOpenDroneIDMessage(uint8_t *encodedMessage)
{
    uint8_t adv[31] = {0};

    // [Length][AD Type][UUID LSB][UUID MSB][ODID App Code][Counter][25-byte ODID message]
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
    const uint8_t selector = odidCounter % 3;

    if (selector == 0)
    {
        ODID_BasicID_encoded encoded{};
        encodeBasicIDMessage(&encoded, &basicID);
        memcpy(message, &encoded, ODID_MESSAGE_SIZE);
        Serial.println("Broadcasting BasicID");
    }
    else if (selector == 1)
    {
        ODID_Location_encoded encoded{};
        encodeLocationMessage(&encoded, &locationData);
        memcpy(message, &encoded, ODID_MESSAGE_SIZE);
        Serial.println("Broadcasting Location");
    }
    else
    {
        ODID_System_encoded encoded{};
        encodeSystemMessage(&encoded, &systemData);
        memcpy(message, &encoded, ODID_MESSAGE_SIZE);
        Serial.println("Broadcasting System");
    }

    advertiseOpenDroneIDMessage(message);
}

void printGnssDebug()
{
    static uint32_t lastPrintMs = 0;
    if (millis() - lastPrintMs < 1000) return;
    lastPrintMs = millis();

    Serial.println("--------------------------------------------------");
    if (gnss.utcValid)
    {
        Serial.printf("UTC: %04u-%02u-%02u %02u:%02u:%02u\n",
                      gnss.utc.year,
                      gnss.utc.month,
                      gnss.utc.day,
                      gnss.utc.hour,
                      gnss.utc.minute,
                      gnss.utc.second);
    }
    else
    {
        Serial.println("UTC: not valid yet");
    }

    Serial.printf("Fix: type=%u gnssFixOk=%u satellites=%u freshAge=%lu ms\n",
                  gnss.fixType,
                  gnss.gnssFixOk,
                  gnss.numSatellites,
                  static_cast<unsigned long>(millis() - gnss.lastUpdateMs));

    Serial.printf("Position: Lat=%.7f, Lon=%.7f\n", gnss.latitude, gnss.longitude);
    Serial.printf("Altitude: Ellipsoid=%.2f m, MSL=%.2f m\n", gnss.height_m, gnss.height_msl_m);
    Serial.printf("Accuracy: H=%.2f m, V=%.2f m\n", gnss.horizontal_accuracy_m, gnss.vertical_accuracy_m);
    Serial.printf("Motion: speed=%.2f m/s, heading=%.2f deg\n", gnss.ground_speed_mps, gnss.heading_deg);
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("v0.7 GNSS + OpenDroneID BLE integration");
    Serial.printf("GNSS baud: %lu\n", static_cast<unsigned long>(GNSS_BAUD));
    Serial.printf("GNSS RX pin: %d, TX pin: %d\n", GNSS_RX_PIN, GNSS_TX_PIN);

    GNSS.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);

    BLEDevice::init("");
    pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinInterval(0xA0);
    pAdvertising->setMaxInterval(0xF0);

    buildBasicID();
    updateOpenDroneIDFromGnss();
}

void loop()
{
    while (readUbxNavPvtPacket())
    {
        parseNavPvt(payload, gnss);
        updateOpenDroneIDFromGnss();
    }

    if (millis() - lastBroadcastMs >= ODID_BROADCAST_INTERVAL_MS)
    {
        lastBroadcastMs = millis();
        broadcastNextMessage();
    }

    printGnssDebug();
}