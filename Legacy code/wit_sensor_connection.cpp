#include <Arduino.h>
#include <SPIFFS.h>
#include <U8g2lib.h>
#include <NimBLEDevice.h>

// =======================================================
// OLEDs
// Screen 1: SDA=6, SCL=7
// Screen 2: SDA=8, SCL=9
// =======================================================
U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C screen1(
    U8G2_R0,
    /* clock=*/7,
    /* data=*/6,
    /* reset=*/U8X8_PIN_NONE
);

U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C screen2(
    U8G2_R0,
    /* clock=*/9,
    /* data=*/8,
    /* reset=*/U8X8_PIN_NONE
);

// =======================================================
// WIT BLE
// =======================================================
static const char* WIT_SERVICE_UUID = "0000ffe5-0000-1000-8000-00805f9a34fb";
static const char* WIT_NOTIFY_UUID  = "0000ffe4-0000-1000-8000-00805f9a34fb";
static const char* ADDR_FILE        = "/wit_addresses.txt";

// =======================================================
// Timing
// =======================================================
static const uint32_t DATA_TIMEOUT_MS   = 2000;
static const uint32_t RESCAN_PERIOD_MS  = 3000;
static const uint32_t SCAN_TIME_MS      = 4000;
static const uint32_t CONNECT_GAP_MS    = 500;

// =======================================================
// Sensor state
// =======================================================
struct SensorSlot {
    String targetAddr;
    NimBLEClient* client;
    float pitch;
    uint32_t lastUpdate;
    String status;
};

SensorSlot sensors[4];

// =======================================================
// Notify callback type
// =======================================================
using NotifyCB = void (*)(NimBLERemoteCharacteristic*, uint8_t*, size_t, bool);

// =======================================================
// Helpers
// =======================================================
static String normalizeAddr(const String& s) {
    String t = s;
    t.trim();
    t.toLowerCase();
    return t;
}

static bool isConnected(int idx) {
    return sensors[idx].client && sensors[idx].client->isConnected();
}

static void resetClient(int idx) {
    if (sensors[idx].client) {
        NimBLEDevice::deleteClient(sensors[idx].client);
        sensors[idx].client = nullptr;
    }
}

static String pitchText(int idx) {
    if (!isConnected(idx)) return "---";
    if (millis() - sensors[idx].lastUpdate > DATA_TIMEOUT_MS) return "---";
    return String(sensors[idx].pitch, 1);
}

// =======================================================
// Display
// =======================================================
static void drawRunScreen(U8G2& u8g2, int a, int b) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_5x8_tf);

    String h1 = "S" + String(a + 1) + ":" + sensors[a].status;
    String h2 = "S" + String(b + 1) + ":" + sensors[b].status;

    u8g2.drawStr(0, 8, h1.c_str());
    u8g2.drawStr(68, 8, h2.c_str());

    u8g2.drawStr(0, 18, "PIT");
    u8g2.drawStr(68, 18, "PIT");

    String v1 = pitchText(a);
    String v2 = pitchText(b);

    u8g2.drawStr(0, 31, v1.c_str());
    u8g2.drawStr(68, 31, v2.c_str());

    u8g2.sendBuffer();
}

static void drawScreens() {
    drawRunScreen(screen1, 0, 1);
    drawRunScreen(screen2, 2, 3);
}

// =======================================================
// WIT packet parsing
// =======================================================
static void parsePitchPacket(int idx, uint8_t* data, size_t len) {
    // Combined BLE frame: 55 61 ...
    if (len >= 20 && data[0] == 0x55 && data[1] == 0x61) {
        int16_t pitchRaw = (int16_t)((data[17] << 8) | data[16]);
        sensors[idx].pitch = pitchRaw / 32768.0f * 180.0f;
        sensors[idx].lastUpdate = millis();
        sensors[idx].status = "DT";
        return;
    }

    // Classic 11-byte angle frame: 55 53 ...
    for (size_t i = 0; i + 10 < len; i++) {
        if (data[i] == 0x55 && data[i + 1] == 0x53) {
            int16_t pitchRaw = (int16_t)((data[i + 5] << 8) | data[i + 4]);
            sensors[idx].pitch = pitchRaw / 32768.0f * 180.0f;
            sensors[idx].lastUpdate = millis();
            sensors[idx].status = "DT";
            return;
        }
    }
}

// =======================================================
// Notify callbacks
// =======================================================
void notify0(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) { parsePitchPacket(0, data, len); }
void notify1(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) { parsePitchPacket(1, data, len); }
void notify2(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) { parsePitchPacket(2, data, len); }
void notify3(NimBLERemoteCharacteristic*, uint8_t* data, size_t len, bool) { parsePitchPacket(3, data, len); }

NotifyCB notifyCallbacks[4] = {
    notify0, notify1, notify2, notify3
};

// =======================================================
// File loading
// data/wit_addresses.txt format:
// line 1 -> sensor 1 address
// line 2 -> sensor 2 address
// line 3 -> sensor 3 address
// line 4 -> sensor 4 address
// =======================================================
static bool loadAddressesFromSPIFFS() {
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS mount failed");
        return false;
    }

    File file = SPIFFS.open(ADDR_FILE, "r");
    if (!file) {
        Serial.println("Failed to open /wit_addresses.txt");
        return false;
    }

    int loaded = 0;
    while (file.available() && loaded < 4) {
        String line = file.readStringUntil('\n');
        line.trim();

        if (line.length() == 0) continue;

        sensors[loaded].targetAddr = normalizeAddr(line);
        Serial.print("Loaded S");
        Serial.print(loaded + 1);
        Serial.print(": ");
        Serial.println(sensors[loaded].targetAddr);
        loaded++;
    }

    file.close();

    if (loaded < 4) {
        Serial.println("Address file does not contain 4 valid lines");
        return false;
    }

    return true;
}

// =======================================================
// Connection / discovery
// =======================================================
static bool connectOne(int idx, const NimBLEAddress& addr, const char* code) {
    sensors[idx].status = code;
    drawScreens();

    resetClient(idx);

    sensors[idx].client = NimBLEDevice::createClient();
    if (!sensors[idx].client) {
        sensors[idx].status = "NC";
        return false;
    }

    sensors[idx].client->setConnectTimeout(10000);
    sensors[idx].client->setConnectionParams(24, 40, 0, 200);

    if (!sensors[idx].client->connect(addr, true, false, false)) {
        resetClient(idx);
        return false;
    }

    sensors[idx].status = "LK";
    drawScreens();
    return true;
}

static bool discoverAndSubscribeOne(int idx) {
    NimBLERemoteService* svc = sensors[idx].client->getService(WIT_SERVICE_UUID);
    if (!svc) {
        sensors[idx].status = "NS";
        resetClient(idx);
        return false;
    }

    NimBLERemoteCharacteristic* ch = svc->getCharacteristic(WIT_NOTIFY_UUID);
    if (!ch) {
        sensors[idx].status = "NC";
        resetClient(idx);
        return false;
    }

    if (!ch->canNotify()) {
        sensors[idx].status = "NT";
        resetClient(idx);
        return false;
    }

    sensors[idx].status = "SB";
    drawScreens();

    if (!ch->subscribe(true, notifyCallbacks[idx])) {
        sensors[idx].status = "CF";
        resetClient(idx);
        return false;
    }

    sensors[idx].status = "WT";
    sensors[idx].lastUpdate = millis();
    return true;
}

static void connectMissingSensors() {
    for (int i = 0; i < 4; i++) {
        if (isConnected(i)) continue;
        if (sensors[i].targetAddr.length() == 0) continue;

        NimBLEAddress randomAddr(sensors[i].targetAddr.c_str(), 1);
        NimBLEAddress publicAddr(sensors[i].targetAddr.c_str(), 0);

        Serial.print("Trying S");
        Serial.print(i + 1);
        Serial.print(" random: ");
        Serial.println(sensors[i].targetAddr);

        if (connectOne(i, randomAddr, "CR")) {
            if (discoverAndSubscribeOne(i)) {
                Serial.print("S");
                Serial.print(i + 1);
                Serial.println(" connected using random type");
                delay(CONNECT_GAP_MS);
                continue;
            }
        }

        Serial.print("Trying S");
        Serial.print(i + 1);
        Serial.print(" public: ");
        Serial.println(sensors[i].targetAddr);

        if (connectOne(i, publicAddr, "CP")) {
            if (discoverAndSubscribeOne(i)) {
                Serial.print("S");
                Serial.print(i + 1);
                Serial.println(" connected using public type");
                delay(CONNECT_GAP_MS);
                continue;
            }
        }

        sensors[i].status = "CF";
        delay(CONNECT_GAP_MS);
    }
}

// =======================================================
// Scan to see which target addresses are currently visible
// =======================================================
static void runScanStep() {
    NimBLEScan* scan = NimBLEDevice::getScan();
    if (!scan) return;

    for (int i = 0; i < 4; i++) {
        if (!isConnected(i) && sensors[i].targetAddr.length()) {
            sensors[i].status = "SC";
        }
    }

    drawScreens();

    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(99);

    Serial.println();
    Serial.println("===== RUN SCAN START =====");

    NimBLEScanResults results = scan->getResults(SCAN_TIME_MS, false);
    int count = results.getCount();

    Serial.print("Devices found: ");
    Serial.println(count);

    for (int i = 0; i < count; i++) {
        const NimBLEAdvertisedDevice* dev = results.getDevice(i);
        if (!dev) continue;

        String name = dev->getName().c_str();
        String addr = normalizeAddr(String(dev->getAddress().toString().c_str()));

        if (name.length() == 0) name = "(no name)";

        Serial.print(i);
        Serial.print(" | ");
        Serial.print(name);
        Serial.print(" | ");
        Serial.print(addr);
        Serial.print(" | RSSI ");
        Serial.println(dev->getRSSI());

        for (int s = 0; s < 4; s++) {
            if (isConnected(s)) continue;
            if (sensors[s].targetAddr.length() == 0) continue;

            if (addr == sensors[s].targetAddr) {
                sensors[s].status = "FD";
            }
        }
    }

    scan->clearResults();
    Serial.println("===== RUN SCAN END =====");
}

// =======================================================
// Setup / loop
// =======================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    screen1.begin();
    screen1.setI2CAddress(0x3C << 1);

    screen2.begin();
    screen2.setI2CAddress(0x3C << 1);

    for (int i = 0; i < 4; i++) {
        sensors[i].targetAddr = "";
        sensors[i].client = nullptr;
        sensors[i].pitch = 0.0f;
        sensors[i].lastUpdate = 0;
        sensors[i].status = "BT";
    }

    drawScreens();

    NimBLEDevice::init("");

    if (!loadAddressesFromSPIFFS()) {
        for (int i = 0; i < 4; i++) {
            sensors[i].status = "FILE";
        }
        drawScreens();
        Serial.println("Address load failed");
        return;
    }

    for (int i = 0; i < 4; i++) {
        sensors[i].status = "RDY";
    }

    drawScreens();
    Serial.println("Ready");
}

void loop() {
    static uint32_t lastScanMs = 0;

    for (int i = 0; i < 4; i++) {
        if (sensors[i].client && !sensors[i].client->isConnected()) {
            resetClient(i);
            sensors[i].status = "DS";
        }

        if (isConnected(i) && millis() - sensors[i].lastUpdate > DATA_TIMEOUT_MS) {
            sensors[i].status = "WT";
        }
    }

    bool needAny = false;
    for (int i = 0; i < 4; i++) {
        if (!isConnected(i) && sensors[i].targetAddr.length()) {
            needAny = true;
            break;
        }
    }

    if (needAny && millis() - lastScanMs >= RESCAN_PERIOD_MS) {
        lastScanMs = millis();
        runScanStep();
        connectMissingSensors();
    }

    drawScreens();
    delay(50);
}