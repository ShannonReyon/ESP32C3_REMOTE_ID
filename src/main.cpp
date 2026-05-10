#include <Arduino.h>

// =========================================================
// v0.6 Standalone GNSS UBX-NAV-PVT parser
// Board: Seeed Studio XIAO ESP32-C3
// GNSS: u-blox M9N
//
// Wiring:
//   M9N TX  -> XIAO D7  (ESP32-C3 RX)
//   M9N RX  -> XIAO D6  (ESP32-C3 TX, optional for this receive-only test)
//   M9N GND -> XIAO GND
//   M9N VCC -> XIAO 3V3 or suitable external 3.3 V supply
// =========================================================

HardwareSerial GNSS(1);

constexpr uint32_t GNSS_BAUD = 230400;
constexpr int GNSS_RX_PIN = D7;
constexpr int GNSS_TX_PIN = D6;

// UBX frame format:
// Sync1 Sync2 Class ID Length_L Length_H Payload... CK_A CK_B
constexpr uint8_t UBX_SYNC_1 = 0xB5;
constexpr uint8_t UBX_SYNC_2 = 0x62;
constexpr uint8_t UBX_CLASS_NAV = 0x01;
constexpr uint8_t UBX_ID_NAV_PVT = 0x07;
constexpr uint16_t NAV_PVT_PAYLOAD_LEN = 92;

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
    double latitude;
    double longitude;
    double height_m;
    double height_msl_m;
    double horizontal_accuracy_m;
    double vertical_accuracy_m;
    double ground_speed_mps;
    double heading_deg;
    uint8_t fixType;
    uint8_t numSatellites;
    bool gnssFixOk;
    UtcTime utc;
    bool utcValid;
};

uint8_t payload[NAV_PVT_PAYLOAD_LEN];

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
    // UBX-NAV-PVT payload offsets:
    // 4: year, 6: month, 7: day, 8: hour, 9: min, 10: sec, 11: valid flags
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
    // Important UBX-NAV-PVT offsets:
    // 20: fixType
    // 21: flags
    // 23: numSV
    // 24: lon [deg * 1e-7]
    // 28: lat [deg * 1e-7]
    // 32: height above ellipsoid [mm]
    // 36: height above mean sea level [mm]
    // 40: horizontal accuracy [mm]
    // 44: vertical accuracy [mm]
    // 60: ground speed [mm/s]
    // 64: heading of motion [deg * 1e-5]

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

    if (data.heading_deg < 0.0)
    {
        data.heading_deg += 360.0;
    }
    if (data.heading_deg >= 360.0)
    {
        data.heading_deg -= 360.0;
    }

    data.utcValid = parseUtc(p, data.utc);
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
            if (b == UBX_SYNC_1)
            {
                state = WAIT_SYNC_2;
            }
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

            if (payloadIndex >= payloadLen)
            {
                state = READ_CK_A;
            }
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

void printGnssData(const GnssData &data)
{
    Serial.println("--------------------------------------------------");

    if (data.utcValid)
    {
        Serial.printf("UTC: %04u-%02u-%02u %02u:%02u:%02u | validDate=%u validTime=%u fullyResolved=%u\n",
                      data.utc.year,
                      data.utc.month,
                      data.utc.day,
                      data.utc.hour,
                      data.utc.minute,
                      data.utc.second,
                      data.utc.validDate,
                      data.utc.validTime,
                      data.utc.fullyResolved);
    }
    else
    {
        Serial.println("UTC: not valid yet");
    }

    Serial.printf("Fix: type=%u gnssFixOk=%u satellites=%u\n",
                  data.fixType,
                  data.gnssFixOk,
                  data.numSatellites);

    Serial.printf("Position: Lat=%.7f, Lon=%.7f\n",
                  data.latitude,
                  data.longitude);

    Serial.printf("Height: Ellipsoid=%.2f m, MSL=%.2f m\n",
                  data.height_m,
                  data.height_msl_m);

    Serial.printf("Accuracy: H=%.2f m, V=%.2f m\n",
                  data.horizontal_accuracy_m,
                  data.vertical_accuracy_m);

    Serial.printf("Motion: GroundSpeed=%.2f m/s, Heading=%.2f deg\n",
                  data.ground_speed_mps,
                  data.heading_deg);
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("v0.6 Standalone GNSS UBX-NAV-PVT parser");
    Serial.printf("GNSS baud: %lu\n", static_cast<unsigned long>(GNSS_BAUD));
    Serial.printf("GNSS RX pin: %d, TX pin: %d\n", GNSS_RX_PIN, GNSS_TX_PIN);

    GNSS.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
}

void loop()
{
    if (readUbxNavPvtPacket())
    {
        GnssData data{};
        parseNavPvt(payload, data);
        printGnssData(data);
    }
}