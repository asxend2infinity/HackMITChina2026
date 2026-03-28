#include <Arduino.h>
#include <Wire.h>
#include <SPIFFS.h>
#include <U8g2lib.h>
#include <NimBLEDevice.h>
#include <math.h>

// =======================================================
// OLEDs
// Screen 1: HW I2C SDA=8, SCL=9
// Screen 2: SW I2C SDA=6, SCL=7
// =======================================================
static const uint8_t OLED_ADDR_7BIT = 0x3C;

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2_screen1(U8G2_R0, U8X8_PIN_NONE);

U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2_screen2(
    U8G2_R2,
    /* clock=*/7,
    /* data=*/6,
    /* reset=*/U8X8_PIN_NONE
);

// =======================================================
// Config file
// =======================================================
static const char* SENSOR_CONFIG_FILE = "/sensor_addresses.txt";

// =======================================================
// framebuffer horizontal mirror helpers
// =======================================================
static inline void mirrorHorizU8g2(U8G2 &u) {
    uint8_t *buf = u.getBufferPtr();
    const int w = u.getDisplayWidth();
    const int h = u.getDisplayHeight();
    const int pages = h / 8;

    for (int p = 0; p < pages; ++p) {
        int row = p * w;
        for (int x = 0; x < w / 2; ++x) {
            uint8_t tmp = buf[row + x];
            buf[row + x] = buf[row + (w - 1 - x)];
            buf[row + (w - 1 - x)] = tmp;
        }
    }
}

static inline void sendBufferMirrored(U8G2 &u) {
    mirrorHorizU8g2(u);
    u.sendBuffer();
    mirrorHorizU8g2(u);
}

// =======================================================
// Cycling HUD values
// =======================================================
String spdVal  = "0.0";
String distVal = "0.00";
String gearVal = "N/A";

String hrVal   = "N/A";
String cadVal  = "N/A";
String pwrVal  = "N/A";

volatile float totalDist_km = 0.0f;

unsigned long lastSpeedUpdate = 0;
unsigned long lastGearUpdate  = 0;
unsigned long lastHrUpdate    = 0;
unsigned long lastCadUpdate   = 0;
unsigned long lastPwrUpdate   = 0;

const unsigned long TIMEOUT_MS = 2000;

// =======================================================
// Cycling BLE constants/devices
// =======================================================
static const char* DI2_SVC_UUID   = "000018ef-5348-494d-414e-4f5f424c4500";
static const char* DI2_CHAR_UUID  = "00002ac1-5348-494d-414e-4f5f424c4500";

NimBLEAdvertisedDevice* di2Device = nullptr;
NimBLEClient*           di2Client = nullptr;

uint8_t lastGearPayload[32];
size_t  lastGearLen = 0;
bool    hasLastGear = false;

NimBLEAdvertisedDevice* hrDevice = nullptr;
NimBLEClient*           hrClient = nullptr;

NimBLEAdvertisedDevice* cadDevice = nullptr;
NimBLEClient*           cadClient = nullptr;

bool     hasLastCrank   = false;
uint16_t lastCrankRevs  = 0;
uint16_t lastCrankTime  = 0;

NimBLEAdvertisedDevice* spdDevice = nullptr;
NimBLEClient*           spdClient = nullptr;

bool     hasLastWheel   = false;
uint32_t lastWheelRevs  = 0;
uint16_t lastWheelTime  = 0;

NimBLEAdvertisedDevice* pmDevice = nullptr;
NimBLEClient*           pmClient = nullptr;

static const float WHEEL_CIRCUM_MM = 2105.0f;

// =======================================================
// Address targets loaded from config file
// =======================================================
String witTargetAddr[4];
String hrTargetAddr  = "";
String cadTargetAddr = "";
String spdTargetAddr = "";
String pwrTargetAddr = "";
String di2TargetAddr = "";

// =======================================================
// WIT BLE
// =======================================================
static const char* WIT_SERVICE_UUID = "0000ffe5-0000-1000-8000-00805f9a34fb";
static const char* WIT_NOTIFY_UUID  = "0000ffe4-0000-1000-8000-00805f9a34fb";

static const uint32_t WIT_DATA_TIMEOUT_MS = 2000;
static const uint32_t WIT_RETRY_PERIOD_MS = 3000;
static const uint32_t WIT_CONNECT_GAP_MS  = 300;

// =======================================================
// WIT sensor state
// Mapping:
// S1 = thigh
// S2 = calf
// S3 = arm
// S4 = back
// All use X angles
// =======================================================
struct SensorSlot {
    String targetAddr;
    NimBLEClient* client;
    float xAngle;
    float yAngle;
    uint32_t lastUpdate;
    String status;
};

SensorSlot sensors[4];

// =======================================================
// Posture parameters
// Formulas:
// knee = 180 + S2 - S1
// body = 180 - S1 + S4
// arm  = S3 - S4
// =======================================================
float KNEE_MAX_ALLOWED       = 150.0f;
float BODY_THIGH_MIN_ALLOWED = 70.0f;
float BODY_THIGH_MAX_ALLOWED = 110.0f;
float ARM_BODY_MIN_ALLOWED   = 75.0f;
float ARM_BODY_MAX_ALLOWED   = 95.0f;

// =======================================================
// Warning messages
// =======================================================
const char* MSG_KNEE_TOO_STRAIGHT = "KNEE TOO STRAIGHT";
const char* MSG_LEAN_FORWARD      = "LEAN FORWARD";
const char* MSG_LEAN_BACK         = "LEAN BACK";
const char* MSG_TUCK_ARMS_IN      = "TUCK ARMS IN";
const char* MSG_REACH_OUT         = "REACH OUT";

// =======================================================
// Posture / alert state
// =======================================================
String postureMessage = "";
String lastPostureMessage = "";
bool postureAlertActive = false;
uint32_t postureAlertLastSeenMs = 0;
static const uint32_t POSTURE_ALERT_HOLD_MS = 1200;

// Derived angles
float kneeAngle = 0.0f;
float bodyThighAngle = 0.0f;
float armBodyAngle = 0.0f;

// =======================================================
// Serial state cache
// =======================================================
String lastPrintedWitStatus[4];
bool lastPrintedDi2Connected = false;
bool lastPrintedHrConnected  = false;
bool lastPrintedCadConnected = false;
bool lastPrintedSpdConnected = false;
bool lastPrintedPwrConnected = false;
String lastPrintedAlert = "";
uint32_t lastPeriodicStatusMs = 0;

// =======================================================
// Forward declarations for WIT notify callbacks
// =======================================================
void notify0(NimBLERemoteCharacteristic*, uint8_t*, size_t, bool);
void notify1(NimBLERemoteCharacteristic*, uint8_t*, size_t, bool);
void notify2(NimBLERemoteCharacteristic*, uint8_t*, size_t, bool);
void notify3(NimBLERemoteCharacteristic*, uint8_t*, size_t, bool);

// =======================================================
// Helpers
// =======================================================
static inline bool di2Connected() { return di2Client && di2Client->isConnected(); }
static inline bool hrConnected()  { return hrClient  && hrClient->isConnected(); }
static inline bool cadConnected() { return cadClient && cadClient->isConnected(); }
static inline bool spdConnected() { return spdClient && spdClient->isConnected(); }
static inline bool pwrConnected() { return pmClient  && pmClient->isConnected(); }

static String normalizeAddr(const String& s) {
    String t = s;
    t.trim();
    t.toLowerCase();
    return t;
}

static bool witConnected(int idx) {
    return sensors[idx].client && sensors[idx].client->isConnected();
}

static bool witFresh(int idx) {
    return witConnected(idx) && (millis() - sensors[idx].lastUpdate <= WIT_DATA_TIMEOUT_MS);
}

static String decodeGear(const uint8_t* data, size_t len) {
    if (len < 6) return "N/A";
    uint8_t front = data[3];
    uint8_t rear  = data[5];
    if (front < 1 || front > 2) return "N/A";
    if (rear  < 1 || rear  > 12) return "N/A";
    return String(front) + "/" + String(rear);
}

static void printBanner(const char* title) {
    Serial.println();
    Serial.println("==================================================");
    Serial.println(title);
    Serial.println("==================================================");
}

static float wrap360(float a) {
    while (a >= 360.0f) a -= 360.0f;
    while (a < 0.0f) a += 360.0f;
    return a;
}

static float wrap180Signed(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

static float sensorAxisDeg(int idx) {
    return sensors[idx].xAngle;
}

static void setPostureAlert(const String& msg) {
    postureAlertActive = true;
    postureMessage = msg;
    lastPostureMessage = msg;
    postureAlertLastSeenMs = millis();
}

// =======================================================
// Cleanup helpers
// =======================================================
static void cleanupDi2Client() {
    if (di2Client) {
        if (di2Client->isConnected()) di2Client->disconnect();
        NimBLEDevice::deleteClient(di2Client);
        di2Client = nullptr;
    }
}

static void cleanupHRClient() {
    if (hrClient) {
        if (hrClient->isConnected()) hrClient->disconnect();
        NimBLEDevice::deleteClient(hrClient);
        hrClient = nullptr;
    }
}

static void cleanupCadClient() {
    if (cadClient) {
        if (cadClient->isConnected()) cadClient->disconnect();
        NimBLEDevice::deleteClient(cadClient);
        cadClient = nullptr;
    }
}

static void cleanupSpdClient() {
    if (spdClient) {
        if (spdClient->isConnected()) spdClient->disconnect();
        NimBLEDevice::deleteClient(spdClient);
        spdClient = nullptr;
    }
}

static void cleanupPwrClient() {
    if (pmClient) {
        if (pmClient->isConnected()) pmClient->disconnect();
        NimBLEDevice::deleteClient(pmClient);
        pmClient = nullptr;
    }
}

static void cleanupWitClient(int idx) {
    if (sensors[idx].client) {
        if (sensors[idx].client->isConnected()) sensors[idx].client->disconnect();
        NimBLEDevice::deleteClient(sensors[idx].client);
        sensors[idx].client = nullptr;
    }
}

// =======================================================
// HUD text helpers
// =======================================================
static const uint8_t X1 = 0;
static const uint8_t X2 = 44;
static const uint8_t X3 = 88;

static const uint8_t Y_HDR = 18;
static const uint8_t Y_VAL = 31;
static const uint8_t Y2_HDR = 13;
static const uint8_t Y2_VAL = 26;

static String fmtSPD()  { return spdConnected() ? (spdVal + "kph") : String("N/A"); }
static String fmtDIST() { return distVal; }
static String fmtGEAR() { return (di2Connected() && hasLastGear) ? gearVal : String("N/A"); }

static String fmtHR()   { return hrConnected()  ? hrVal  : String("N/A"); }
static String fmtCAD()  { return cadConnected() ? cadVal : String("N/A"); }
static String fmtPWR()  { return pwrConnected() ? pwrVal : String("N/A"); }

// =======================================================
// Serial reporting
// =======================================================
static void printCyclingConnStatus() {
    bool di2 = di2Connected();
    bool hr  = hrConnected();
    bool cad = cadConnected();
    bool spd = spdConnected();
    bool pwr = pwrConnected();

    if (di2 != lastPrintedDi2Connected) {
        Serial.print("[HUD] DI2: ");
        Serial.println(di2 ? "CONNECTED" : "DISCONNECTED");
        lastPrintedDi2Connected = di2;
    }
    if (hr != lastPrintedHrConnected) {
        Serial.print("[HUD] HR: ");
        Serial.println(hr ? "CONNECTED" : "DISCONNECTED");
        lastPrintedHrConnected = hr;
    }
    if (cad != lastPrintedCadConnected) {
        Serial.print("[HUD] CAD: ");
        Serial.println(cad ? "CONNECTED" : "DISCONNECTED");
        lastPrintedCadConnected = cad;
    }
    if (spd != lastPrintedSpdConnected) {
        Serial.print("[HUD] SPD: ");
        Serial.println(spd ? "CONNECTED" : "DISCONNECTED");
        lastPrintedSpdConnected = spd;
    }
    if (pwr != lastPrintedPwrConnected) {
        Serial.print("[HUD] PWR: ");
        Serial.println(pwr ? "CONNECTED" : "DISCONNECTED");
        lastPrintedPwrConnected = pwr;
    }
}

static void printWitStatusChanges() {
    for (int i = 0; i < 4; i++) {
        String nowStatus = sensors[i].status;
        if (nowStatus != lastPrintedWitStatus[i]) {
            Serial.print("[WIT] S");
            Serial.print(i + 1);
            Serial.print(": ");

            if (nowStatus == "DT") {
                Serial.println("CONNECTED");
            } else if (nowStatus == "TRY") {
                Serial.println("CONNECTING");
            } else if (nowStatus == "CF") {
                Serial.println("CONNECT FAILED");
            } else if (nowStatus == "DS") {
                Serial.println("DISCONNECTED");
            } else if (nowStatus == "WT") {
                Serial.println("CONNECTED, WAITING FOR DATA");
            } else if (nowStatus == "RDY") {
                Serial.println("READY");
            } else if (nowStatus == "FILE") {
                Serial.println("ADDRESS FILE ERROR");
            } else {
                Serial.print("STATUS=");
                Serial.println(nowStatus);
            }

            lastPrintedWitStatus[i] = nowStatus;
        }
    }
}

static void printAlertChanges() {
    String nowAlert = postureAlertActive ? postureMessage : String("NONE");
    if (nowAlert != lastPrintedAlert) {
        Serial.print("[POSTURE] ALERT -> ");
        Serial.println(nowAlert);
        lastPrintedAlert = nowAlert;
    }
}

static void printPeriodicStatusSnapshot() {
    Serial.println();
    Serial.println("----------- CONNECTION STATUS -----------");

    Serial.print("[HUD] DI2=");
    Serial.println(di2Connected() ? "CONNECTED" : "DISCONNECTED");

    Serial.print("[HUD] HR=");
    Serial.println(hrConnected() ? "CONNECTED" : "DISCONNECTED");

    Serial.print("[HUD] CAD=");
    Serial.println(cadConnected() ? "CONNECTED" : "DISCONNECTED");

    Serial.print("[HUD] SPD=");
    Serial.println(spdConnected() ? "CONNECTED" : "DISCONNECTED");

    Serial.print("[HUD] PWR=");
    Serial.println(pwrConnected() ? "CONNECTED" : "DISCONNECTED");

    for (int i = 0; i < 4; i++) {
        Serial.print("[WIT] S");
        Serial.print(i + 1);
        Serial.print("=");
        Serial.println(witConnected(i) ? "CONNECTED" : "DISCONNECTED");
    }

    for (int i = 0; i < 4; i++) {
        Serial.print("[WIT DEG] S");
        Serial.print(i + 1);
        Serial.print(" X=");
        Serial.print(sensors[i].xAngle, 2);
        Serial.print(" Y=");
        Serial.print(sensors[i].yAngle, 2);
        Serial.print(" fresh=");
        Serial.println(witFresh(i) ? "YES" : "NO");
    }

    if (witFresh(0) && witFresh(1)) {
        Serial.print("[POSTURE] kneeAngle(180+S2-S1)=");
        Serial.println(kneeAngle, 1);
    }

    if (witFresh(0) && witFresh(3)) {
        Serial.print("[POSTURE] bodyAngle(180-S1+S4)=");
        Serial.println(bodyThighAngle, 1);
    }

    if (witFresh(2) && witFresh(3)) {
        Serial.print("[POSTURE] armAngle(S3-S4)=");
        Serial.println(armBodyAngle, 1);
    }

    Serial.println("----------------------------------------");
}

// =======================================================
// Cycling notify callbacks
// =======================================================
void di2Notify(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
    if (len > sizeof(lastGearPayload)) len = sizeof(lastGearPayload);
    memcpy(lastGearPayload, data, len);
    lastGearLen = len;
    hasLastGear = true;

    gearVal = decodeGear(data, len);
    lastGearUpdate = millis();
}

void hrNotify(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
    if (len >= 2) {
        hrVal = String((int)data[1]);
        lastHrUpdate = millis();
    }
}

void cadNotify(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
    if (len < 1) return;

    uint8_t flags = data[0];
    size_t index = 1;

    bool wheelPresent = (flags & 0x01) != 0;
    bool crankPresent = (flags & 0x02) != 0;

    if (wheelPresent) {
        if (len < index + 6) return;
        index += 6;
    }
    if (!crankPresent) return;
    if (len < index + 4) return;

    uint16_t revs  = (uint16_t)data[index] | ((uint16_t)data[index + 1] << 8);
    uint16_t t1024 = (uint16_t)data[index + 2] | ((uint16_t)data[index + 3] << 8);

    if (!hasLastCrank) {
        hasLastCrank = true;
        lastCrankRevs = revs;
        lastCrankTime = t1024;
        return;
    }

    uint16_t deltaRevs = (revs >= lastCrankRevs)
        ? (revs - lastCrankRevs)
        : (uint16_t)((uint32_t)revs + 65536u - lastCrankRevs);

    uint16_t deltaTime = (t1024 >= lastCrankTime)
        ? (t1024 - lastCrankTime)
        : (uint16_t)((uint32_t)t1024 + 65536u - lastCrankTime);

    lastCrankRevs = revs;
    lastCrankTime = t1024;

    if (deltaTime == 0 || deltaRevs == 0) return;

    float dt_sec = (float)deltaTime / 1024.0f;
    float rpm = ((float)deltaRevs / dt_sec) * 60.0f;
    if (rpm < 0) rpm = 0;
    if (rpm > 250) rpm = 250;

    cadVal = String((int)(rpm + 0.5f));
    lastCadUpdate = millis();
}

void spdNotify(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
    if (len < 7) return;

    uint8_t flags = data[0];
    bool wheelPresent = (flags & 0x01) != 0;
    if (!wheelPresent) return;

    uint32_t revs =
        (uint32_t)data[1] |
        ((uint32_t)data[2] << 8) |
        ((uint32_t)data[3] << 16) |
        ((uint32_t)data[4] << 24);

    uint16_t t1024 =
        (uint16_t)data[5] |
        ((uint16_t)data[6] << 8);

    if (!hasLastWheel) {
        hasLastWheel = true;
        lastWheelRevs = revs;
        lastWheelTime = t1024;
        return;
    }

    uint32_t deltaRevs = (revs >= lastWheelRevs)
        ? (revs - lastWheelRevs)
        : ((0xFFFFFFFFu - lastWheelRevs) + 1u + revs);

    uint16_t deltaTime = (t1024 >= lastWheelTime)
        ? (t1024 - lastWheelTime)
        : (uint16_t)((uint32_t)t1024 + 65536u - lastWheelTime);

    lastWheelRevs = revs;
    lastWheelTime = t1024;

    if (deltaTime == 0 || deltaRevs == 0) return;

    float dt_sec = (float)deltaTime / 1024.0f;

    float distance_m = (float)deltaRevs * (WHEEL_CIRCUM_MM / 1000.0f);
    float speed_mps  = distance_m / dt_sec;
    float speed_kmh  = speed_mps * 3.6f;

    if (speed_kmh < 0) speed_kmh = 0;
    if (speed_kmh > 120) speed_kmh = 120;

    spdVal = String(speed_kmh, 1);

    totalDist_km += (float)deltaRevs * (WHEEL_CIRCUM_MM / 1000000.0f);
    distVal = String(totalDist_km, 2);

    lastSpeedUpdate = millis();
}

void powerNotify(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
    if (len < 4) return;
    int16_t instPower = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
    if (instPower < 0) instPower = 0;
    if (instPower > 3000) instPower = 3000;
    pwrVal = String((int)instPower);
    lastPwrUpdate = millis();
}

// =======================================================
// WIT angle parsing
// =======================================================
static void parseAnglePacket(int idx, uint8_t* data, size_t len) {
    if (len >= 20 && data[0] == 0x55 && data[1] == 0x61) {
        int16_t xRaw = (int16_t)((data[15] << 8) | data[14]);
        int16_t yRaw = (int16_t)((data[17] << 8) | data[16]);

        sensors[idx].xAngle = xRaw / 32768.0f * 180.0f;
        sensors[idx].yAngle = yRaw / 32768.0f * 180.0f;
        sensors[idx].lastUpdate = millis();
        sensors[idx].status = "DT";
        return;
    }

    for (size_t i = 0; i + 10 < len; i++) {
        if (data[i] == 0x55 && data[i + 1] == 0x53) {
            int16_t xRaw = (int16_t)((data[i + 3] << 8) | data[i + 2]);
            int16_t yRaw = (int16_t)((data[i + 5] << 8) | data[i + 4]);

            sensors[idx].xAngle = xRaw / 32768.0f * 180.0f;
            sensors[idx].yAngle = yRaw / 32768.0f * 180.0f;
            sensors[idx].lastUpdate = millis();
            sensors[idx].status = "DT";
            return;
        }
    }
}

// =======================================================
// WIT notify callbacks
// =======================================================
void notify0(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) { parseAnglePacket(0, data, len); }
void notify1(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) { parseAnglePacket(1, data, len); }
void notify2(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) { parseAnglePacket(2, data, len); }
void notify3(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) { parseAnglePacket(3, data, len); }

// =======================================================
// Config loader
// Format:
// WIT1=xx:xx:xx:xx:xx:xx
// WIT2=...
// WIT3=...
// WIT4=...
// HR=...
// CAD=...
// SPD=...
// PWR=...
// DI2=...
// =======================================================
static bool loadAddressesFromSPIFFS() {
    if (!SPIFFS.begin(true)) {
        Serial.println("[SYS] SPIFFS mount failed");
        return false;
    }

    File file = SPIFFS.open(SENSOR_CONFIG_FILE, "r");
    if (!file) {
        Serial.println("[SYS] Failed to open /sensor_addresses.txt");
        return false;
    }

    for (int i = 0; i < 4; i++) {
        witTargetAddr[i] = "";
        sensors[i].targetAddr = "";
    }

    hrTargetAddr  = "";
    cadTargetAddr = "";
    spdTargetAddr = "";
    pwrTargetAddr = "";
    di2TargetAddr = "";

    while (file.available()) {
        String line = file.readStringUntil('\n');
        line.trim();

        if (line.length() == 0) continue;
        if (line.startsWith("#")) continue;

        int eq = line.indexOf('=');
        if (eq < 0) continue;

        String key = line.substring(0, eq);
        String val = line.substring(eq + 1);

        key.trim();
        val.trim();
        val = normalizeAddr(val);
        key.toUpperCase();

        if (key == "WIT1") {
            witTargetAddr[0] = val;
            sensors[0].targetAddr = val;
        } else if (key == "WIT2") {
            witTargetAddr[1] = val;
            sensors[1].targetAddr = val;
        } else if (key == "WIT3") {
            witTargetAddr[2] = val;
            sensors[2].targetAddr = val;
        } else if (key == "WIT4") {
            witTargetAddr[3] = val;
            sensors[3].targetAddr = val;
        } else if (key == "HR") {
            hrTargetAddr = val;
        } else if (key == "CAD") {
            cadTargetAddr = val;
        } else if (key == "SPD") {
            spdTargetAddr = val;
        } else if (key == "PWR") {
            pwrTargetAddr = val;
        } else if (key == "DI2") {
            di2TargetAddr = val;
        }
    }

    file.close();

    Serial.println("[SYS] Loaded sensor addresses:");
    for (int i = 0; i < 4; i++) {
        Serial.print("  WIT");
        Serial.print(i + 1);
        Serial.print(" = ");
        Serial.println(witTargetAddr[i]);
    }
    Serial.print("  HR  = ");  Serial.println(hrTargetAddr);
    Serial.print("  CAD = ");  Serial.println(cadTargetAddr);
    Serial.print("  SPD = ");  Serial.println(spdTargetAddr);
    Serial.print("  PWR = ");  Serial.println(pwrTargetAddr);
    Serial.print("  DI2 = ");  Serial.println(di2TargetAddr);

    return true;
}

// =======================================================
// Scan callback for cycling sensors
// Match by address from config file
// =======================================================
class ScanCB : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* dev) override {
        String addr = normalizeAddr(String(dev->getAddress().toString().c_str()));
        String name = dev->getName().c_str();

        Serial.print("[SCAN] ");
        Serial.print(addr);
        Serial.print(" | ");
        Serial.println(name.length() ? name : "(no name)");

        if (!di2Device && di2TargetAddr.length() > 0 && addr == di2TargetAddr) {
            di2Device = new NimBLEAdvertisedDevice(*dev);
            Serial.println("[SCAN] Matched DI2 by address");
        }

        if (!hrDevice && hrTargetAddr.length() > 0 && addr == hrTargetAddr) {
            hrDevice = new NimBLEAdvertisedDevice(*dev);
            Serial.println("[SCAN] Matched HR by address");
        }

        if (!cadDevice && cadTargetAddr.length() > 0 && addr == cadTargetAddr) {
            cadDevice = new NimBLEAdvertisedDevice(*dev);
            Serial.println("[SCAN] Matched CAD by address");
        }

        if (!spdDevice && spdTargetAddr.length() > 0 && addr == spdTargetAddr) {
            spdDevice = new NimBLEAdvertisedDevice(*dev);
            Serial.println("[SCAN] Matched SPD by address");
        }

        if (!pmDevice && pwrTargetAddr.length() > 0 && addr == pwrTargetAddr) {
            pmDevice = new NimBLEAdvertisedDevice(*dev);
            Serial.println("[SCAN] Matched PWR by address");
        }
    }
};

static ScanCB scanCB;

// =======================================================
// Generic cycling connector
// =======================================================
static void cleanupGenericClient(NimBLEClient** c) {
    if (*c) {
        if ((*c)->isConnected()) (*c)->disconnect();
        NimBLEDevice::deleteClient(*c);
        *c = nullptr;
    }
}

static void connectGeneric(
    NimBLEAdvertisedDevice* d,
    NimBLEClient** c,
    const char* svc,
    const char* chr,
    NimBLERemoteCharacteristic::notify_callback cb,
    const char* label
) {
    if (!d || *c) return;

    Serial.print("[HUD] Trying ");
    Serial.print(label);
    Serial.println(" connect...");

    NimBLEDevice::getScan()->stop();

    *c = NimBLEDevice::createClient();
    if (!*c) {
        Serial.print("[HUD] ");
        Serial.print(label);
        Serial.println(" createClient failed");
        return;
    }

    if (!(*c)->connect(d)) {
        Serial.print("[HUD] ");
        Serial.print(label);
        Serial.println(" connect failed");
        cleanupGenericClient(c);
        return;
    }

    auto s = (*c)->getService(svc);
    if (!s) {
        Serial.print("[HUD] ");
        Serial.print(label);
        Serial.println(" service not found");
        cleanupGenericClient(c);
        return;
    }

    auto chara = s->getCharacteristic(chr);
    if (!chara) {
        Serial.print("[HUD] ");
        Serial.print(label);
        Serial.println(" characteristic not found");
        cleanupGenericClient(c);
        return;
    }

    if (!chara->canNotify()) {
        Serial.print("[HUD] ");
        Serial.print(label);
        Serial.println(" characteristic cannot notify");
        cleanupGenericClient(c);
        return;
    }

    if (!chara->subscribe(true, cb)) {
        Serial.print("[HUD] ");
        Serial.print(label);
        Serial.println(" subscribe failed");
        cleanupGenericClient(c);
        return;
    }

    Serial.print("[HUD] ");
    Serial.print(label);
    Serial.println(" connected + subscribed");
}

static void connectDi2() {
    connectGeneric(di2Device, &di2Client, DI2_SVC_UUID, DI2_CHAR_UUID, di2Notify, "DI2");
}

static void connectHR() {
    connectGeneric(hrDevice, &hrClient, "180D", "2A37", hrNotify, "HR");
}

static void connectCadence() {
    connectGeneric(cadDevice, &cadClient, "1816", "2A5B", cadNotify, "CAD");
}

static void connectSpeed() {
    connectGeneric(spdDevice, &spdClient, "1816", "2A5B", spdNotify, "SPD");
}

static void connectPower() {
    connectGeneric(pmDevice, &pmClient, "1818", "2A63", powerNotify, "PWR");
}

// =======================================================
// WIT connect functions
// Only use address from config file
// Address type set to random (1).
// If needed, change to 0 for public.
// =======================================================
static bool connectOneWit(int idx) {
    if (idx < 0 || idx > 3) return false;
    if (sensors[idx].targetAddr.length() == 0) return false;

    cleanupWitClient(idx);

    sensors[idx].client = NimBLEDevice::createClient();
    if (!sensors[idx].client) {
        sensors[idx].status = "NC";
        return false;
    }

    sensors[idx].client->setConnectTimeout(10000);
    sensors[idx].client->setConnectionParams(24, 40, 0, 200);

    NimBLEAddress addr(sensors[idx].targetAddr.c_str(), 1);

    if (!sensors[idx].client->connect(addr, true, false, false)) {
        cleanupWitClient(idx);
        return false;
    }

    NimBLERemoteService* svc = sensors[idx].client->getService(WIT_SERVICE_UUID);
    if (!svc) {
        sensors[idx].status = "NS";
        cleanupWitClient(idx);
        return false;
    }

    NimBLERemoteCharacteristic* ch = svc->getCharacteristic(WIT_NOTIFY_UUID);
    if (!ch) {
        sensors[idx].status = "NC";
        cleanupWitClient(idx);
        return false;
    }

    if (!ch->canNotify()) {
        sensors[idx].status = "NT";
        cleanupWitClient(idx);
        return false;
    }

    switch (idx) {
        case 0:
            if (!ch->subscribe(true, notify0)) {
                sensors[idx].status = "SBF";
                cleanupWitClient(idx);
                return false;
            }
            break;
        case 1:
            if (!ch->subscribe(true, notify1)) {
                sensors[idx].status = "SBF";
                cleanupWitClient(idx);
                return false;
            }
            break;
        case 2:
            if (!ch->subscribe(true, notify2)) {
                sensors[idx].status = "SBF";
                cleanupWitClient(idx);
                return false;
            }
            break;
        case 3:
            if (!ch->subscribe(true, notify3)) {
                sensors[idx].status = "SBF";
                cleanupWitClient(idx);
                return false;
            }
            break;
    }

    sensors[idx].status = "DT";
    sensors[idx].lastUpdate = millis();
    return true;
}

static void connectMissingWitSensors() {
    static uint32_t lastRetryMs = 0;
    if (millis() - lastRetryMs < WIT_RETRY_PERIOD_MS) return;
    lastRetryMs = millis();

    for (int i = 0; i < 4; i++) {
        if (witConnected(i)) continue;
        if (sensors[i].targetAddr.length() == 0) continue;

        sensors[i].status = "TRY";

        Serial.print("[WIT] Attempting S");
        Serial.print(i + 1);
        Serial.print(" connect using file address: ");
        Serial.println(sensors[i].targetAddr);

        if (connectOneWit(i)) {
            Serial.print("[WIT] S");
            Serial.print(i + 1);
            Serial.println(" CONNECTED");
        } else {
            sensors[i].status = "CF";
            Serial.print("[WIT] S");
            Serial.print(i + 1);
            Serial.println(" CONNECT FAILED");
        }

        delay(WIT_CONNECT_GAP_MS);
    }
}

// =======================================================
// Derived angle formulas
// knee = 180 + S2 - S1
// body = 180 - S1 + S4
// arm  = S3 - S4
// Mapping:
// S1 = thigh
// S2 = calf
// S3 = arm
// S4 = back
// =======================================================
static void updateDerivedAngles() {
    float s1, s2, s3, s4;

    if (witFresh(0) && witFresh(1)) {
        s1 = sensors[0].xAngle; // thigh
        s2 = sensors[1].xAngle; // calf
        kneeAngle = wrap360(180.0f + s2 - s1);
    }

    if (witFresh(0) && witFresh(3)) {
        s1 = sensors[0].xAngle; // thigh
        s4 = sensors[3].xAngle; // back
        bodyThighAngle = wrap360(180.0f - s1 + s4);
    }

    if (witFresh(2) && witFresh(3)) {
        s3 = sensors[2].xAngle; // arm
        s4 = sensors[3].xAngle; // back
        armBodyAngle = wrap180Signed(s3 - s4);
    }
}

// =======================================================
// Posture warning logic
// =======================================================
static void updatePostureAlert() {
    postureAlertActive = false;
    postureMessage = "";

    updateDerivedAngles();

    if (witFresh(0) && witFresh(1)) {
        if (kneeAngle > KNEE_MAX_ALLOWED) {
            setPostureAlert(String(MSG_KNEE_TOO_STRAIGHT));
            return;
        }
    }

    if (witFresh(0) && witFresh(3)) {
        if (bodyThighAngle < BODY_THIGH_MIN_ALLOWED) {
            setPostureAlert(String(MSG_LEAN_FORWARD));
            return;
        }
        if (bodyThighAngle > BODY_THIGH_MAX_ALLOWED) {
            setPostureAlert(String(MSG_LEAN_BACK));
            return;
        }
    }

    if (witFresh(2) && witFresh(3)) {
        if (armBodyAngle < ARM_BODY_MIN_ALLOWED) {
            setPostureAlert(String(MSG_TUCK_ARMS_IN));
            return;
        }
        if (armBodyAngle > ARM_BODY_MAX_ALLOWED) {
            setPostureAlert(String(MSG_REACH_OUT));
            return;
        }
    }

    if (millis() - postureAlertLastSeenMs < POSTURE_ALERT_HOLD_MS && lastPostureMessage.length() > 0) {
        postureAlertActive = true;
        postureMessage = lastPostureMessage;
    }
}

// =======================================================
// Display drawing
// =======================================================
static void drawScreen1Normal() {
    u8g2_screen1.clearBuffer();
    u8g2_screen1.setFont(u8g2_font_5x8_tf);

    u8g2_screen1.drawStr(X1, Y_HDR, "SPD");
    u8g2_screen1.drawStr(X2, Y_HDR, "DIST");
    u8g2_screen1.drawStr(X3, Y_HDR, "GEAR");

    u8g2_screen1.drawStr(X1, Y_VAL, fmtSPD().c_str());
    u8g2_screen1.drawStr(X2, Y_VAL, fmtDIST().c_str());
    u8g2_screen1.drawStr(X3, Y_VAL, fmtGEAR().c_str());

    sendBufferMirrored(u8g2_screen1);
}

static void drawScreen2Normal() {
    u8g2_screen2.clearBuffer();
    u8g2_screen2.setFont(u8g2_font_5x8_tf);

    const int C1R = X2 - 1;
    const int C2R = X3 - 1;
    const int C3R = 127;

    auto drawRightInColumn = [&](int rightEdge, int y, const char* s) {
        int w = u8g2_screen2.getStrWidth(s);
        int x = rightEdge - w + 1;
        if (x < 0) x = 0;
        u8g2_screen2.drawStr(x, y, s);
    };

    drawRightInColumn(C1R, Y2_HDR, "HR");
    drawRightInColumn(C2R, Y2_HDR, "CAD");
    drawRightInColumn(C3R, Y2_HDR, "PWR");

    String hr  = fmtHR();
    String cad = fmtCAD();
    String pwr = fmtPWR();

    drawRightInColumn(C1R, Y2_VAL, hr.c_str());
    drawRightInColumn(C2R, Y2_VAL, cad.c_str());
    drawRightInColumn(C3R, Y2_VAL, pwr.c_str());

    sendBufferMirrored(u8g2_screen2);
}

static void drawNotificationOnScreen(U8G2& u8g2, const char* line1, const char* line2) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_5x8_tf);

    int w1 = u8g2.getStrWidth(line1);
    int x1 = (128 - w1) / 2;
    if (x1 < 0) x1 = 0;

    int w2 = u8g2.getStrWidth(line2);
    int x2 = (128 - w2) / 2;
    if (x2 < 0) x2 = 0;

    u8g2.drawFrame(0, 0, 128, 32);
    u8g2.drawStr(x1, 12, line1);
    u8g2.drawStr(x2, 25, line2);

    sendBufferMirrored(u8g2);
}

static void drawAlertScreens() {
    drawNotificationOnScreen(u8g2_screen1, "POSTURE ALERT", postureMessage.c_str());
    drawNotificationOnScreen(u8g2_screen2, "ADJUST POSITION", postureMessage.c_str());
}

static void drawScreens() {
    updatePostureAlert();

    if (postureAlertActive) {
        drawAlertScreens();
    } else {
        drawScreen1Normal();
        drawScreen2Normal();
    }
}

// =======================================================
// setup / loop
// =======================================================
void setup() {
    Serial.begin(115200);
    delay(300);

    Wire.begin(8, 9);
    Wire.setClock(100000);

    u8g2_screen1.begin();
    u8g2_screen1.setI2CAddress(OLED_ADDR_7BIT << 1);

    u8g2_screen2.begin();
    u8g2_screen2.setI2CAddress(OLED_ADDR_7BIT << 1);

    for (int i = 0; i < 4; i++) {
        sensors[i].targetAddr = "";
        sensors[i].client = nullptr;
        sensors[i].xAngle = 0.0f;
        sensors[i].yAngle = 0.0f;
        sensors[i].lastUpdate = 0;
        sensors[i].status = "BT";
    }

    postureMessage = "";
    lastPostureMessage = "";
    drawScreens();

    NimBLEDevice::init("ESP32-BIKE-HUD");

    if (!loadAddressesFromSPIFFS()) {
        Serial.println("[SYS] sensor address load failed");
        for (int i = 0; i < 4; i++) {
            sensors[i].status = "FILE";
        }
    } else {
        for (int i = 0; i < 4; i++) {
            sensors[i].status = "RDY";
        }
    }

    printBanner("SYSTEM START");

    for (int i = 0; i < 4; i++) {
        lastPrintedWitStatus[i] = "";
    }

    lastPrintedDi2Connected = di2Connected();
    lastPrintedHrConnected  = hrConnected();
    lastPrintedCadConnected = cadConnected();
    lastPrintedSpdConnected = spdConnected();
    lastPrintedPwrConnected = pwrConnected();
    lastPrintedAlert = "";

    Serial.println("[SYS] Serial monitor ready");
    Serial.println("[SYS] OLED initialized");
    Serial.println("[SYS] BLE initialized");
    Serial.println("[SYS] Waiting for cycling + WIT sensors...");

    NimBLEScan* scan = NimBLEDevice::getScan();
    scan->setScanCallbacks(&scanCB, false);
    scan->setInterval(45);
    scan->setWindow(15);
    scan->setActiveScan(true);
}

void loop() {
    unsigned long now = millis();

    if (now - lastSpeedUpdate > TIMEOUT_MS) spdVal = "0.0";
    if (now - lastGearUpdate  > TIMEOUT_MS) { hasLastGear = false; gearVal = "N/A"; }
    if (now - lastHrUpdate    > TIMEOUT_MS) hrVal = "N/A";
    if (now - lastCadUpdate   > TIMEOUT_MS) cadVal = "N/A";
    if (now - lastPwrUpdate   > TIMEOUT_MS) pwrVal = "N/A";

    if (di2Client && !di2Client->isConnected()) {
        cleanupDi2Client();
        hasLastGear = false;
        gearVal = "N/A";
    }
    if (hrClient && !hrClient->isConnected()) {
        cleanupHRClient();
        hrVal = "N/A";
    }
    if (cadClient && !cadClient->isConnected()) {
        cleanupCadClient();
        hasLastCrank = false;
        cadVal = "N/A";
    }
    if (spdClient && !spdClient->isConnected()) {
        cleanupSpdClient();
        hasLastWheel = false;
        spdVal = "0.0";
    }
    if (pmClient && !pmClient->isConnected()) {
        cleanupPwrClient();
        pwrVal = "N/A";
    }

    for (int i = 0; i < 4; i++) {
        if (sensors[i].client && !sensors[i].client->isConnected()) {
            cleanupWitClient(i);
            sensors[i].status = "DS";
        }

        if (witConnected(i) && millis() - sensors[i].lastUpdate > WIT_DATA_TIMEOUT_MS) {
            sensors[i].status = "WT";
        }
    }

    NimBLEScan* scan = NimBLEDevice::getScan();
    bool needMoreDevices = (!di2Device || !hrDevice || !cadDevice || !spdDevice || !pmDevice);

    if (needMoreDevices && !scan->isScanning()) {
        scan->start(0, false, false);
    }

    if (di2Device && !di2Client) {
        Serial.println("[HUD] Attempting DI2 reconnect...");
        connectDi2();
    }

    if (hrDevice && !hrClient) {
        Serial.println("[HUD] Attempting HR reconnect...");
        connectHR();
    }

    if (cadDevice && !cadClient) {
        Serial.println("[HUD] Attempting CAD reconnect...");
        connectCadence();
    }

    if (spdDevice && !spdClient) {
        Serial.println("[HUD] Attempting SPD reconnect...");
        connectSpeed();
    }

    if (pmDevice && !pmClient) {
        Serial.println("[HUD] Attempting PWR reconnect...");
        connectPower();
    }

    connectMissingWitSensors();

    drawScreens();

    printCyclingConnStatus();
    printWitStatusChanges();
    printAlertChanges();

    if (millis() - lastPeriodicStatusMs >= 2000) {
        lastPeriodicStatusMs = millis();
        printPeriodicStatusSnapshot();
    }

    delay(50);
}
//  //safe version of main.cpp without posture alert logic, WIT angle parsing, or SPIFFS loading. Just connects to sensors and displays data on OLEDs.
// #include <Arduino.h>
// #include <Wire.h>
// #include <U8g2lib.h>
// #include <NimBLEDevice.h>

// // =======================================================
// // OLEDs
// // =======================================================
// static const uint8_t OLED_ADDR_7BIT = 0x3C;   // try 0x3D if needed

// // Screen 1 (SPD/DIST/GEAR)
// U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2_screen1(U8G2_R0, U8X8_PIN_NONE);

// // Screen 2 (HR/CAD/PWR)  <-- changed from U8G2_R0 to U8G2_R2
// U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2_screen2(
//     U8G2_R2,
//     /* clock=*/7,
//     /* data=*/6,
//     /* reset=*/U8X8_PIN_NONE);

// // =======================================================
// // framebuffer horizontal mirror helpers
// // =======================================================
// static inline void mirrorHorizU8g2(U8G2 &u) {
//     uint8_t *buf = u.getBufferPtr();
//     const int w = u.getDisplayWidth();   // 128
//     const int h = u.getDisplayHeight();  // 32
//     const int pages = h / 8;             // 4

//     for (int p = 0; p < pages; ++p) {
//         int row = p * w;
//         for (int x = 0; x < w / 2; ++x) {
//             uint8_t tmp = buf[row + x];
//             buf[row + x] = buf[row + (w - 1 - x)];
//             buf[row + (w - 1 - x)] = tmp;
//         }
//     }
// }

// static inline void sendBufferMirrored(U8G2 &u) {
//     mirrorHorizU8g2(u);
//     u.sendBuffer();
//     mirrorHorizU8g2(u);
// }

// // =======================================================
// // Values
// // =======================================================
// String spdVal  = "0.0";
// String distVal = "0.00";
// String gearVal = "N/A";

// String hrVal   = "N/A";
// String cadVal  = "N/A";
// String pwrVal  = "N/A";

// volatile float totalDist_km = 0.0f;

// unsigned long lastSpeedUpdate = 0;
// unsigned long lastGearUpdate  = 0;
// unsigned long lastHrUpdate    = 0;
// unsigned long lastCadUpdate   = 0;
// unsigned long lastPwrUpdate   = 0;

// const unsigned long TIMEOUT_MS = 2000;

// // =======================================================
// // BLE constants/devices
// // =======================================================

// // ---- DI2 ----
// static const char* DI2_PREFIX     = "RDR7150 A6F";
// static const char* DI2_SVC_UUID   = "000018ef-5348-494d-414e-4f5f424c4500";
// static const char* DI2_CHAR_UUID  = "00002ac1-5348-494d-414e-4f5f424c4500";

// NimBLEAdvertisedDevice* di2Device = nullptr;
// NimBLEClient*           di2Client = nullptr;

// uint8_t lastGearPayload[32];
// size_t  lastGearLen = 0;
// bool    hasLastGear = false;

// // ---- Heart rate ----
// const char* HR_NAME = "COROS HEART RATE 8E2DA0";
// NimBLEAdvertisedDevice* hrDevice = nullptr;
// NimBLEClient*           hrClient = nullptr;

// // ---- Cadence (CSC) ----
// const char* CAD_NAME = "COROS CAD 766AD9";
// NimBLEAdvertisedDevice* cadDevice = nullptr;
// NimBLEClient*           cadClient = nullptr;

// // crank state
// bool     hasLastCrank   = false;
// uint16_t lastCrankRevs  = 0;
// uint16_t lastCrankTime  = 0; // 1/1024 s

// // ---- Speed sensor (CSC) ----
// const char* SPD_NAME = "COROS SPD 3C37F4";
// NimBLEAdvertisedDevice* spdDevice = nullptr;
// NimBLEClient*           spdClient = nullptr;

// // wheel state
// bool     hasLastWheel   = false;
// uint32_t lastWheelRevs  = 0;
// uint16_t lastWheelTime  = 0; // 1/1024 s

// // ---- Power meter ----
// const char* PM_NAME_PREFIX = "XPOWER-S";
// NimBLEAdvertisedDevice* pmDevice = nullptr;
// NimBLEClient*           pmClient = nullptr;

// static const float WHEEL_CIRCUM_MM = 2105.0f;

// // =======================================================
// // Helpers
// // =======================================================
// static inline bool di2Connected() { return di2Client && di2Client->isConnected(); }
// static inline bool hrConnected()  { return hrClient  && hrClient->isConnected(); }
// static inline bool cadConnected() { return cadClient && cadClient->isConnected(); }
// static inline bool spdConnected() { return spdClient && spdClient->isConnected(); }
// static inline bool pwrConnected() { return pmClient  && pmClient->isConnected(); }

// static String decodeGear(const uint8_t* data, size_t len) {
//     if (len < 6) return "N/A";
//     uint8_t front = data[3];
//     uint8_t rear  = data[5];
//     if (front < 1 || front > 2) return "N/A";
//     if (rear  < 1 || rear  > 12) return "N/A";
//     return String(front) + "/" + String(rear);
// }

// // =======================================================
// // Notify callbacks
// // =======================================================
// void di2Notify(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
//     if (len > sizeof(lastGearPayload)) len = sizeof(lastGearPayload);
//     memcpy(lastGearPayload, data, len);
//     lastGearLen = len;
//     hasLastGear = true;

//     gearVal = decodeGear(data, len);
//     lastGearUpdate = millis();
// }

// void hrNotify(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
//     if (len >= 2) {
//         hrVal = String((int)data[1]);
//         lastHrUpdate = millis();
//     }
// }

// void cadNotify(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
//     if (len < 1) return;

//     uint8_t flags = data[0];
//     size_t index = 1;

//     bool wheelPresent = (flags & 0x01) != 0;
//     bool crankPresent = (flags & 0x02) != 0;

//     if (wheelPresent) {
//         if (len < index + 6) return;
//         index += 6;
//     }
//     if (!crankPresent) return;
//     if (len < index + 4) return;

//     uint16_t revs  = (uint16_t)data[index] | ((uint16_t)data[index + 1] << 8);
//     uint16_t t1024 = (uint16_t)data[index + 2] | ((uint16_t)data[index + 3] << 8);

//     if (!hasLastCrank) {
//         hasLastCrank = true;
//         lastCrankRevs = revs;
//         lastCrankTime = t1024;
//         return;
//     }

//     uint16_t deltaRevs = (revs >= lastCrankRevs)
//         ? (revs - lastCrankRevs)
//         : (uint16_t)((uint32_t)revs + 65536u - lastCrankRevs);

//     uint16_t deltaTime = (t1024 >= lastCrankTime)
//         ? (t1024 - lastCrankTime)
//         : (uint16_t)((uint32_t)t1024 + 65536u - lastCrankTime);

//     lastCrankRevs = revs;
//     lastCrankTime = t1024;

//     if (deltaTime == 0 || deltaRevs == 0) return;

//     float dt_sec = (float)deltaTime / 1024.0f;
//     float rpm = ((float)deltaRevs / dt_sec) * 60.0f;
//     if (rpm < 0) rpm = 0;
//     if (rpm > 250) rpm = 250;

//     cadVal = String((int)(rpm + 0.5f));
//     lastCadUpdate = millis();
// }

// void spdNotify(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
//     if (len < 7) return;

//     uint8_t flags = data[0];
//     bool wheelPresent = (flags & 0x01) != 0;
//     if (!wheelPresent) return;

//     uint32_t revs =
//         (uint32_t)data[1] |
//         ((uint32_t)data[2] << 8) |
//         ((uint32_t)data[3] << 16) |
//         ((uint32_t)data[4] << 24);

//     uint16_t t1024 =
//         (uint16_t)data[5] |
//         ((uint16_t)data[6] << 8);

//     if (!hasLastWheel) {
//         hasLastWheel = true;
//         lastWheelRevs = revs;
//         lastWheelTime = t1024;
//         return;
//     }

//     uint32_t deltaRevs = (revs >= lastWheelRevs)
//         ? (revs - lastWheelRevs)
//         : ((0xFFFFFFFFu - lastWheelRevs) + 1u + revs);

//     uint16_t deltaTime = (t1024 >= lastWheelTime)
//         ? (t1024 - lastWheelTime)
//         : (uint16_t)((uint32_t)t1024 + 65536u - lastWheelTime);

//     lastWheelRevs = revs;
//     lastWheelTime = t1024;

//     if (deltaTime == 0 || deltaRevs == 0) return;

//     float dt_sec = (float)deltaTime / 1024.0f;

//     float distance_m = (float)deltaRevs * (WHEEL_CIRCUM_MM / 1000.0f);
//     float speed_mps  = distance_m / dt_sec;
//     float speed_kmh  = speed_mps * 3.6f;

//     if (speed_kmh < 0) speed_kmh = 0;
//     if (speed_kmh > 120) speed_kmh = 120;

//     spdVal = String(speed_kmh, 1);

//     totalDist_km += (float)deltaRevs * (WHEEL_CIRCUM_MM / 1000000.0f);
//     distVal = String(totalDist_km, 2);

//     lastSpeedUpdate = millis();
// }

// void powerNotify(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) {
//     if (len < 4) return;
//     int16_t instPower = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
//     if (instPower < 0) instPower = 0;
//     if (instPower > 3000) instPower = 3000;
//     pwrVal = String((int)instPower);
//     lastPwrUpdate = millis();
// }

// // =======================================================
// // Scan callback
// // =======================================================
// class ScanCB : public NimBLEAdvertisedDeviceCallbacks {
//     void onResult(NimBLEAdvertisedDevice* dev) override {
//         String name = dev->getName().c_str();

//         if (!di2Device && name.startsWith(DI2_PREFIX)) di2Device = dev;
//         if (!hrDevice  && name == HR_NAME)            hrDevice  = dev;
//         if (!cadDevice && name == CAD_NAME)           cadDevice = dev;
//         if (!spdDevice && name == SPD_NAME)           spdDevice = dev;

//         if (!pmDevice && name.startsWith(PM_NAME_PREFIX)) {
//             pmDevice = new NimBLEAdvertisedDevice(*dev);
//         }

//         if (di2Device && hrDevice && cadDevice && spdDevice) {
//             NimBLEDevice::getScan()->stop();
//         }
//     }
// };

// // =======================================================
// // Connect functions
// // =======================================================
// static void connectDi2() {
//     if (!di2Device || di2Client) return;

//     di2Client = NimBLEDevice::createClient();
//     if (!di2Client->connect(di2Device)) {
//         NimBLEDevice::deleteClient(di2Client);
//         di2Client = nullptr;
//         return;
//     }

//     auto svc = di2Client->getService(DI2_SVC_UUID);
//     if (!svc) return;
//     auto ch  = svc->getCharacteristic(DI2_CHAR_UUID);
//     if (!ch) return;
//     if (ch->canNotify()) ch->subscribe(true, di2Notify);
// }

// static void connectHR() {
//     if (!hrDevice || hrClient) return;

//     hrClient = NimBLEDevice::createClient();
//     if (!hrClient->connect(hrDevice)) {
//         NimBLEDevice::deleteClient(hrClient);
//         hrClient = nullptr;
//         return;
//     }

//     auto svc = hrClient->getService("180D");
//     if (!svc) return;
//     auto ch  = svc->getCharacteristic("2A37");
//     if (!ch) return;
//     if (ch->canNotify()) ch->subscribe(true, hrNotify);
// }

// static void connectCadence() {
//     if (!cadDevice || cadClient) return;

//     cadClient = NimBLEDevice::createClient();
//     if (!cadClient->connect(cadDevice)) {
//         NimBLEDevice::deleteClient(cadClient);
//         cadClient = nullptr;
//         return;
//     }

//     auto svc = cadClient->getService("1816");
//     if (!svc) return;
//     auto ch  = svc->getCharacteristic("2A5B");
//     if (!ch) return;
//     if (ch->canNotify()) ch->subscribe(true, cadNotify);
// }

// static void connectSpeed() {
//     if (!spdDevice || spdClient) return;

//     spdClient = NimBLEDevice::createClient();
//     if (!spdClient->connect(spdDevice)) {
//         NimBLEDevice::deleteClient(spdClient);
//         spdClient = nullptr;
//         return;
//     }

//     auto svc = spdClient->getService("1816");
//     if (!svc) return;
//     auto ch  = svc->getCharacteristic("2A5B");
//     if (!ch) return;
//     if (ch->canNotify()) ch->subscribe(true, spdNotify);
// }

// static void connectPower() {
//     if (!pmDevice || pmClient) return;

//     pmClient = NimBLEDevice::createClient();
//     if (!pmClient->connect(pmDevice) || !pmClient->isConnected()) {
//         NimBLEDevice::deleteClient(pmClient);
//         pmClient = nullptr;
//         return;
//     }

//     auto svc = pmClient->getService("1818");
//     if (!svc) return;
//     auto ch  = svc->getCharacteristic("2A63");
//     if (!ch) return;
//     if (ch->canNotify()) ch->subscribe(true, powerNotify);
// }

// // =======================================================
// // HUD drawing
// // =======================================================
// static const uint8_t X1 = 0;
// static const uint8_t X2 = 44;
// static const uint8_t X3 = 88;

// static const uint8_t Y_HDR = 18;
// static const uint8_t Y_VAL = 31;
// static const uint8_t Y2_HDR = 13;
// static const uint8_t Y2_VAL = 26;

// static String fmtSPD()  { return spdConnected() ? (spdVal + "kph") : String("N/A"); }
// static String fmtDIST() { return distVal; }
// static String fmtGEAR() { return (di2Connected() && hasLastGear) ? gearVal : String("N/A"); }

// static String fmtHR()   { return hrConnected()  ? hrVal  : String("N/A"); }
// static String fmtCAD()  { return cadConnected() ? cadVal : String("N/A"); }
// static String fmtPWR()  { return pwrConnected() ? pwrVal : String("N/A"); }

// static void drawScreen1() {
//     u8g2_screen1.clearBuffer();
//     u8g2_screen1.setFont(u8g2_font_5x8_tf);

//     u8g2_screen1.drawStr(X1, Y_HDR, "SPD");
//     u8g2_screen1.drawStr(X2, Y_HDR, "DIST");
//     u8g2_screen1.drawStr(X3, Y_HDR, "GEAR");

//     u8g2_screen1.drawStr(X1, Y_VAL, fmtSPD().c_str());
//     u8g2_screen1.drawStr(X2, Y_VAL, fmtDIST().c_str());
//     u8g2_screen1.drawStr(X3, Y_VAL, fmtGEAR().c_str());

//     sendBufferMirrored(u8g2_screen1);
// }

// static void drawScreen2() {
//     u8g2_screen2.clearBuffer();
//     u8g2_screen2.setFont(u8g2_font_5x8_tf);

//     const int C1R = X2 - 1;
//     const int C2R = X3 - 1;
//     const int C3R = 127;

//     auto drawRightInColumn = [&](int rightEdge, int y, const char* s) {
//         int w = u8g2_screen2.getStrWidth(s);
//         int x = rightEdge - w + 1;
//         if (x < 0) x = 0;
//         u8g2_screen2.drawStr(x, y, s);
//     };

//     drawRightInColumn(C1R, Y2_HDR, "HR");
//     drawRightInColumn(C2R, Y2_HDR, "CAD");
//     drawRightInColumn(C3R, Y2_HDR, "PWR");

//     String hr  = fmtHR();
//     String cad = fmtCAD();
//     String pwr = fmtPWR();

//     drawRightInColumn(C1R, Y2_VAL, hr.c_str());
//     drawRightInColumn(C2R, Y2_VAL, cad.c_str());
//     drawRightInColumn(C3R, Y2_VAL, pwr.c_str());

//     sendBufferMirrored(u8g2_screen2);
// }

// // =======================================================
// // setup / loop
// // =======================================================
// void setup() {
//     Serial.begin(115200);
//     delay(200);

//     // Screen 1 HW I2C: SDA=8, SCL=9
//     Wire.begin(8, 9);
//     Wire.setClock(100000);

//     u8g2_screen1.begin();
//     u8g2_screen1.setI2CAddress(OLED_ADDR_7BIT << 1);

//     // Screen 2 SW I2C: SDA=6, SCL=7
//     u8g2_screen2.begin();
//     u8g2_screen2.setI2CAddress(OLED_ADDR_7BIT << 1);

//     drawScreen1();
//     drawScreen2();

//     NimBLEDevice::init("ESP32-BIKE");
//     NimBLEScan* scan = NimBLEDevice::getScan();
//     scan->setAdvertisedDeviceCallbacks(new ScanCB(), true);
//     scan->setInterval(45);
//     scan->setWindow(15);
//     scan->setActiveScan(true);
// }

// void loop() {
//     unsigned long now = millis();

//     if (now - lastSpeedUpdate > TIMEOUT_MS) spdVal = "0.0";
//     if (now - lastGearUpdate  > TIMEOUT_MS) { hasLastGear = false; gearVal = "N/A"; }
//     if (now - lastHrUpdate    > TIMEOUT_MS) hrVal = "N/A";
//     if (now - lastCadUpdate   > TIMEOUT_MS) cadVal = "N/A";
//     if (now - lastPwrUpdate   > TIMEOUT_MS) pwrVal = "N/A";

//     if (di2Client && !di2Client->isConnected()) {
//         NimBLEDevice::deleteClient(di2Client);
//         di2Client = nullptr;
//         hasLastGear = false;
//         gearVal = "N/A";
//     }
//     if (hrClient && !hrClient->isConnected()) {
//         NimBLEDevice::deleteClient(hrClient);
//         hrClient = nullptr;
//         hrVal = "N/A";
//     }
//     if (cadClient && !cadClient->isConnected()) {
//         NimBLEDevice::deleteClient(cadClient);
//         cadClient = nullptr;
//         hasLastCrank = false;
//         cadVal = "N/A";
//     }
//     if (spdClient && !spdClient->isConnected()) {
//         NimBLEDevice::deleteClient(spdClient);
//         spdClient = nullptr;
//         hasLastWheel = false;
//         spdVal = "0.0";
//     }
//     if (pmClient && !pmClient->isConnected()) {
//         NimBLEDevice::deleteClient(pmClient);
//         pmClient = nullptr;
//         pwrVal = "N/A";
//     }

//     NimBLEScan* scan = NimBLEDevice::getScan();
//     bool needMoreDevices = (!di2Device || !hrDevice || !cadDevice || !spdDevice);
//     bool allConnected = di2Connected() && hrConnected() && cadConnected() && spdConnected();

//     if (!allConnected) {
//         if (needMoreDevices && !scan->isScanning()) {
//             scan->start(0, nullptr, false);
//         }
//     } else {
//         if (scan->isScanning()) scan->stop();
//     }

//     if (di2Device && !di2Client) connectDi2();
//     if (hrDevice  && !hrClient)  connectHR();
//     if (cadDevice && !cadClient) connectCadence();
//     if (spdDevice && !spdClient) connectSpeed();
//     if (pmDevice  && !pmClient)  connectPower();

//     drawScreen1();
//     drawScreen2();

//     delay(50);
// }