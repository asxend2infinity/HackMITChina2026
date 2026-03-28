#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <NimBLEDevice.h>

// =======================================================
// OLEDs
// =======================================================
static const uint8_t OLED_ADDR_7BIT = 0x3C;   // try 0x3D if needed

// Screen 1 (SPD/DIST/GEAR)
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2_screen1(U8G2_R0, U8X8_PIN_NONE);

// Screen 2 (HR/CAD/PWR)  <-- changed from U8G2_R0 to U8G2_R2
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2_screen2(
    U8G2_R2,
    /* clock=*/7,
    /* data=*/6,
    /* reset=*/U8X8_PIN_NONE);

// =======================================================
// framebuffer horizontal mirror helpers
// =======================================================
static inline void mirrorHorizU8g2(U8G2 &u) {
    uint8_t *buf = u.getBufferPtr();
    const int w = u.getDisplayWidth();   // 128
    const int h = u.getDisplayHeight();  // 32
    const int pages = h / 8;             // 4

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
// Values
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
// BLE constants/devices
// =======================================================

// ---- DI2 ----
static const char* DI2_PREFIX     = "RDR7150 A6F";
static const char* DI2_SVC_UUID   = "000018ef-5348-494d-414e-4f5f424c4500";
static const char* DI2_CHAR_UUID  = "00002ac1-5348-494d-414e-4f5f424c4500";

NimBLEAdvertisedDevice* di2Device = nullptr;
NimBLEClient*           di2Client = nullptr;

uint8_t lastGearPayload[32];
size_t  lastGearLen = 0;
bool    hasLastGear = false;

// ---- Heart rate ----
const char* HR_NAME = "COROS HEART RATE 8E2DA0";
NimBLEAdvertisedDevice* hrDevice = nullptr;
NimBLEClient*           hrClient = nullptr;

// ---- Cadence (CSC) ----
const char* CAD_NAME = "COROS CAD 766AD9";
NimBLEAdvertisedDevice* cadDevice = nullptr;
NimBLEClient*           cadClient = nullptr;

// crank state
bool     hasLastCrank   = false;
uint16_t lastCrankRevs  = 0;
uint16_t lastCrankTime  = 0; // 1/1024 s

// ---- Speed sensor (CSC) ----
const char* SPD_NAME = "COROS SPD 3C37F4";
NimBLEAdvertisedDevice* spdDevice = nullptr;
NimBLEClient*           spdClient = nullptr;

// wheel state
bool     hasLastWheel   = false;
uint32_t lastWheelRevs  = 0;
uint16_t lastWheelTime  = 0; // 1/1024 s

// ---- Power meter ----
const char* PM_NAME_PREFIX = "XPOWER-S";
NimBLEAdvertisedDevice* pmDevice = nullptr;
NimBLEClient*           pmClient = nullptr;

static const float WHEEL_CIRCUM_MM = 2105.0f;

// =======================================================
// Helpers
// =======================================================
static inline bool di2Connected() { return di2Client && di2Client->isConnected(); }
static inline bool hrConnected()  { return hrClient  && hrClient->isConnected(); }
static inline bool cadConnected() { return cadClient && cadClient->isConnected(); }
static inline bool spdConnected() { return spdClient && spdClient->isConnected(); }
static inline bool pwrConnected() { return pmClient  && pmClient->isConnected(); }

static String decodeGear(const uint8_t* data, size_t len) {
    if (len < 6) return "N/A";
    uint8_t front = data[3];
    uint8_t rear  = data[5];
    if (front < 1 || front > 2) return "N/A";
    if (rear  < 1 || rear  > 12) return "N/A";
    return String(front) + "/" + String(rear);
}

// =======================================================
// Notify callbacks
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
// Scan callback
// =======================================================
class ScanCB : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice* dev) override {
        String name = dev->getName().c_str();

        if (!di2Device && name.startsWith(DI2_PREFIX)) di2Device = dev;
        if (!hrDevice  && name == HR_NAME)            hrDevice  = dev;
        if (!cadDevice && name == CAD_NAME)           cadDevice = dev;
        if (!spdDevice && name == SPD_NAME)           spdDevice = dev;

        if (!pmDevice && name.startsWith(PM_NAME_PREFIX)) {
            pmDevice = new NimBLEAdvertisedDevice(*dev);
        }

        if (di2Device && hrDevice && cadDevice && spdDevice) {
            NimBLEDevice::getScan()->stop();
        }
    }
};

// =======================================================
// Connect functions
// =======================================================
static void connectDi2() {
    if (!di2Device || di2Client) return;

    di2Client = NimBLEDevice::createClient();
    if (!di2Client->connect(di2Device)) {
        NimBLEDevice::deleteClient(di2Client);
        di2Client = nullptr;
        return;
    }

    auto svc = di2Client->getService(DI2_SVC_UUID);
    if (!svc) return;
    auto ch  = svc->getCharacteristic(DI2_CHAR_UUID);
    if (!ch) return;
    if (ch->canNotify()) ch->subscribe(true, di2Notify);
}

static void connectHR() {
    if (!hrDevice || hrClient) return;

    hrClient = NimBLEDevice::createClient();
    if (!hrClient->connect(hrDevice)) {
        NimBLEDevice::deleteClient(hrClient);
        hrClient = nullptr;
        return;
    }

    auto svc = hrClient->getService("180D");
    if (!svc) return;
    auto ch  = svc->getCharacteristic("2A37");
    if (!ch) return;
    if (ch->canNotify()) ch->subscribe(true, hrNotify);
}

static void connectCadence() {
    if (!cadDevice || cadClient) return;

    cadClient = NimBLEDevice::createClient();
    if (!cadClient->connect(cadDevice)) {
        NimBLEDevice::deleteClient(cadClient);
        cadClient = nullptr;
        return;
    }

    auto svc = cadClient->getService("1816");
    if (!svc) return;
    auto ch  = svc->getCharacteristic("2A5B");
    if (!ch) return;
    if (ch->canNotify()) ch->subscribe(true, cadNotify);
}

static void connectSpeed() {
    if (!spdDevice || spdClient) return;

    spdClient = NimBLEDevice::createClient();
    if (!spdClient->connect(spdDevice)) {
        NimBLEDevice::deleteClient(spdClient);
        spdClient = nullptr;
        return;
    }

    auto svc = spdClient->getService("1816");
    if (!svc) return;
    auto ch  = svc->getCharacteristic("2A5B");
    if (!ch) return;
    if (ch->canNotify()) ch->subscribe(true, spdNotify);
}

static void connectPower() {
    if (!pmDevice || pmClient) return;

    pmClient = NimBLEDevice::createClient();
    if (!pmClient->connect(pmDevice) || !pmClient->isConnected()) {
        NimBLEDevice::deleteClient(pmClient);
        pmClient = nullptr;
        return;
    }

    auto svc = pmClient->getService("1818");
    if (!svc) return;
    auto ch  = svc->getCharacteristic("2A63");
    if (!ch) return;
    if (ch->canNotify()) ch->subscribe(true, powerNotify);
}

// =======================================================
// HUD drawing
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

static void drawScreen1() {
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

static void drawScreen2() {
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

// =======================================================
// setup / loop
// =======================================================
void setup() {
    Serial.begin(115200);
    delay(200);

    // Screen 1 HW I2C: SDA=8, SCL=9
    Wire.begin(8, 9);
    Wire.setClock(100000);

    u8g2_screen1.begin();
    u8g2_screen1.setI2CAddress(OLED_ADDR_7BIT << 1);

    // Screen 2 SW I2C: SDA=6, SCL=7
    u8g2_screen2.begin();
    u8g2_screen2.setI2CAddress(OLED_ADDR_7BIT << 1);

    drawScreen1();
    drawScreen2();

    NimBLEDevice::init("ESP32-BIKE");
    NimBLEScan* scan = NimBLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(new ScanCB(), true);
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
        NimBLEDevice::deleteClient(di2Client);
        di2Client = nullptr;
        hasLastGear = false;
        gearVal = "N/A";
    }
    if (hrClient && !hrClient->isConnected()) {
        NimBLEDevice::deleteClient(hrClient);
        hrClient = nullptr;
        hrVal = "N/A";
    }
    if (cadClient && !cadClient->isConnected()) {
        NimBLEDevice::deleteClient(cadClient);
        cadClient = nullptr;
        hasLastCrank = false;
        cadVal = "N/A";
    }
    if (spdClient && !spdClient->isConnected()) {
        NimBLEDevice::deleteClient(spdClient);
        spdClient = nullptr;
        hasLastWheel = false;
        spdVal = "0.0";
    }
    if (pmClient && !pmClient->isConnected()) {
        NimBLEDevice::deleteClient(pmClient);
        pmClient = nullptr;
        pwrVal = "N/A";
    }

    NimBLEScan* scan = NimBLEDevice::getScan();
    bool needMoreDevices = (!di2Device || !hrDevice || !cadDevice || !spdDevice);
    bool allConnected = di2Connected() && hrConnected() && cadConnected() && spdConnected();

    if (!allConnected) {
        if (needMoreDevices && !scan->isScanning()) {
            scan->start(0, nullptr, false);
        }
    } else {
        if (scan->isScanning()) scan->stop();
    }

    if (di2Device && !di2Client) connectDi2();
    if (hrDevice  && !hrClient)  connectHR();
    if (cadDevice && !cadClient) connectCadence();
    if (spdDevice && !spdClient) connectSpeed();
    if (pmDevice  && !pmClient)  connectPower();

    drawScreen1();
    drawScreen2();

    delay(50);
}

