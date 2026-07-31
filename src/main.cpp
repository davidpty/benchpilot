
//======================================================================
//1. INCLUDES & PIN CONFIG
//======================================================================

#include <Arduino.h>
#include <HardwareSerial.h>
#include <driver/uart.h>
#include <soc/uart_reg.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <SPI.h>
#include <SD.h>
#include <ESPmDNS.h>

#include "wifi_config.h"

#define SEATALK_BAUD        4800
#define SEATALK_RX_PIN      35
#define TFT_BL_PIN          27
#define MSG_LOG_SIZE        300
#define MSG_TEXT_LEN        128
#define BUS_IDLE_MS         12
#define AP_RECENT_MS        5000   // alarm recency window (ms since apUpdated)
#define HDG_OVERRIDE_MS     5000   // ms without 0x9C before 0x84 heading is accepted
// Touch layout constants (320×240 CYD)
#define LOG_SCROLL_LEFT     107
#define LOG_SCROLL_RIGHT    214
#define LOG_SCROLL_MID      128
#define LOG_TAP_BOUNDARY    136
#define TOUCH_DEBOUNCE_MS   300

static auto &Seatalk = Serial2;

// TX state
static bool _txActive = false;
static unsigned long _lastBusy = 0;
static uint8_t _suppressEchoCmd = 0;

// Heading TX state (periodic)
static bool _hdgActive = false;
static bool _hdgAutoDrift = true;
static uint16_t _hdgDeg = 0;
static unsigned long _hdgInterval = 250;
static unsigned long _lastHdgTx = 0;
static float _hdgFrac = 0;

#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33
// --- SD Card Logging ---
#define SD_CS       5
#define SD_SCK      18
#define SD_MOSI     23
#define SD_MISO     19
#define SD_LOG_PCT  90

static File logFile;
static int logNumber = 0;

//======================================================================
//2. DRIVERS — SoftSPI, XPT2046 touch
//======================================================================

// --- SoftSPI: bit-banged SPI driver (inherits SPIClass for SPI transaction API) ---
class SoftSPI : public SPIClass {
    void wait(uint_fast8_t del) { for (uint_fast8_t i = 0; i < del; i++) asm volatile("nop"); }
    uint8_t _cke, _ckp, _delay = 2;
    uint8_t _miso, _mosi, _sck, _order = MSBFIRST;
public:
    SoftSPI(uint8_t mosi, uint8_t miso, uint8_t sck) : _miso(miso), _mosi(mosi), _sck(sck) {}
    void begin() { pinMode(_mosi, OUTPUT); pinMode(_miso, INPUT); pinMode(_sck, OUTPUT); }
    void end()   { pinMode(_mosi, INPUT); pinMode(_miso, INPUT); pinMode(_sck, INPUT); }
    void setBitOrder(uint8_t o) { _order = o & 1; }
    void setDataMode(uint8_t mode) {
        switch (mode) {
            case SPI_MODE0: _ckp = 0; _cke = 0; break;
            case SPI_MODE1: _ckp = 0; _cke = 1; break;
            case SPI_MODE2: _ckp = 1; _cke = 0; break;
            case SPI_MODE3: _ckp = 1; _cke = 1; break;
        }
        digitalWrite(_sck, _ckp ? HIGH : LOW);
    }
    void setClockDivider(uint32_t div) {
        switch (div) {
            case SPI_CLOCK_DIV2:   _delay = 2;   break;
            case SPI_CLOCK_DIV4:   _delay = 4;   break;
            case SPI_CLOCK_DIV8:   _delay = 8;   break;
            case SPI_CLOCK_DIV16:  _delay = 16;  break;
            case SPI_CLOCK_DIV32:  _delay = 32;  break;
            case SPI_CLOCK_DIV64:  _delay = 64;  break;
            case SPI_CLOCK_DIV128: _delay = 128; break;
            default:               _delay = 128; break;
        }
    }
    uint8_t transfer(uint8_t val) {
        uint8_t out = 0;
        if (_order == MSBFIRST) {
            uint8_t v2 =
                ((val & 0x01) << 7) | ((val & 0x02) << 5) |
                ((val & 0x04) << 3) | ((val & 0x08) << 1) |
                ((val & 0x10) >> 1) | ((val & 0x20) >> 3) |
                ((val & 0x40) >> 5) | ((val & 0x80) >> 7);
            val = v2;
        }
        uint8_t del = _delay >> 1, bval = 0;
        int sck = (_ckp) ? HIGH : LOW;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (_cke) { sck ^= 1; digitalWrite(_sck, sck); wait(del); }
            digitalWrite(_mosi, (val & (1 << bit)) ? HIGH : LOW);
            wait(del);
            sck ^= 1; digitalWrite(_sck, sck);
            bval = digitalRead(_miso);
            if (_order == MSBFIRST) { out <<= 1; out |= bval; }
            else { out >>= 1; out |= bval << 7; }
            wait(del);
            if (!_cke) { sck ^= 1; digitalWrite(_sck, sck); }
        }
        return out;
    }
    uint16_t transfer16(uint16_t data) {
        union { uint16_t val; struct { uint8_t lsb; uint8_t msb; }; } in, out;
        in.val = data;
        if (_order == MSBFIRST) { out.msb = transfer(in.msb); out.lsb = transfer(in.lsb); }
        else { out.lsb = transfer(in.lsb); out.msb = transfer(in.msb); }
        return out.val;
    }
};

// --- Touchscreen reimplementation (no external library dependency) ---
class TS_Point {
public:
    int16_t x, y, z;
    TS_Point() : x(0), y(0), z(0) {}
    TS_Point(int16_t x, int16_t y, int16_t z) : x(x), y(y), z(z) {}
    bool operator==(TS_Point p) { return p.x == x && p.y == y && p.z == z; }
    bool operator!=(TS_Point p) { return p.x != x || p.y != y || p.z != z; }
};

#define Z_THRESHOLD     400
#define Z_THRESHOLD_INT 75
#define MSEC_THRESHOLD  3
#define SPI_SETTING     SPISettings(2000000, MSBFIRST, SPI_MODE0)

class XPT2046_TouchscreenSOFTSPI {
    uint8_t csPin, tirqPin, rotation = 1;
    int16_t xraw = 0, yraw = 0, zraw = 0;
    uint32_t msraw = 0x80000000;

    static int16_t besttwoavg(int16_t x, int16_t y, int16_t z) {
        int16_t da = (x > y) ? x - y : y - x;
        int16_t db = (x > z) ? x - z : z - x;
        int16_t dc = (z > y) ? z - y : y - z;
        if (da <= db && da <= dc) return (x + y) >> 1;
        if (db <= da && db <= dc) return (x + z) >> 1;
        return (y + z) >> 1;
    }

    void update(SoftSPI *spi) {
        int16_t data[6];
        if (!isrWake) return;
        uint32_t now = millis();
        if (now - msraw < MSEC_THRESHOLD) return;

        spi->beginTransaction(SPI_SETTING);
        digitalWrite(csPin, LOW);
        spi->transfer(0xB1);
        int16_t z1 = spi->transfer16(0xC1) >> 3;
        int z = z1 + 4095;
        int16_t z2 = spi->transfer16(0x91) >> 3;
        z -= z2;

        if (z >= Z_THRESHOLD) {
            spi->transfer16(0x91);
            data[0] = spi->transfer16(0xD1) >> 3;
            data[1] = spi->transfer16(0x91) >> 3;
            data[2] = spi->transfer16(0xD1) >> 3;
            data[3] = spi->transfer16(0x91) >> 3;
        } else {
            data[0] = data[1] = data[2] = data[3] = 0;
        }
        data[4] = spi->transfer16(0xD0) >> 3;
        data[5] = spi->transfer16(0) >> 3;
        digitalWrite(csPin, HIGH);
        spi->endTransaction();

        if (z < 0) z = 0;
        if (z < Z_THRESHOLD) { zraw = 0; if (z < Z_THRESHOLD_INT && 255 != tirqPin) isrWake = false; return; }
        zraw = z;

        int16_t x = besttwoavg(data[0], data[2], data[4]);
        int16_t y = besttwoavg(data[1], data[3], data[5]);

        if (z >= Z_THRESHOLD) {
            msraw = now;
            switch (rotation) {
                case 0: xraw = 4095 - y; yraw = x; break;
                case 1: xraw = x; yraw = y; break;
                case 2: xraw = y; yraw = 4095 - x; break;
                default: xraw = 4095 - x; yraw = 4095 - y; break;
            }
        }
    }

public:
    volatile bool isrWake = true;
    constexpr XPT2046_TouchscreenSOFTSPI(uint8_t cspin, uint8_t tirq = 255) : csPin(cspin), tirqPin(tirq) {}

    bool begin(SoftSPI *spi) {
        spi->begin();
        pinMode(csPin, OUTPUT);
        digitalWrite(csPin, HIGH);
        if (255 != tirqPin) {
            pinMode(tirqPin, INPUT_PULLUP);
            attachInterrupt(digitalPinToInterrupt(tirqPin), [] { if (isrPinptr) isrPinptr->isrWake = true; }, FALLING);
            isrPinptr = this;
        }
        return true;
    }

    TS_Point getPoint(SoftSPI *spi) { update(spi); return TS_Point(xraw, yraw, zraw); }
    bool tirqTouched() { return isrWake; }
    bool touched(SoftSPI *spi) { update(spi); return zraw >= Z_THRESHOLD; }
    void readData(SoftSPI *spi, uint16_t *x, uint16_t *y, uint8_t *z) { update(spi); *x = xraw; *y = yraw; *z = zraw; }
    bool bufferEmpty() { return (millis() - msraw) < MSEC_THRESHOLD; }
    uint8_t bufferSize() { return 1; }
    void setRotation(uint8_t n) { rotation = n % 4; }

private:
    static XPT2046_TouchscreenSOFTSPI *isrPinptr;
};
XPT2046_TouchscreenSOFTSPI *XPT2046_TouchscreenSOFTSPI::isrPinptr = nullptr;

static SoftSPI tsi(XPT2046_MOSI, XPT2046_MISO, XPT2046_CLK);
static XPT2046_TouchscreenSOFTSPI touchscreen(XPT2046_CS, XPT2046_IRQ);
static int touchX, touchY;
static unsigned long lastBtnPress = 0;

enum SensorIdx : uint8_t { SI_DEPTH, SI_WIND_ANGLE, SI_WIND_SPEED, SI_STW, SI_SOG, SI_COG, SI_WATER_TEMP, SI_END };

//======================================================================
//3. DATA MODEL — BoatData, widgets, bands, colors, buttons
//======================================================================

TFT_eSPI tft;
WebServer server(80);
DNSServer dnsServer;

static unsigned long totalBytes = 0;
static bool wifiConnected = false;
static unsigned long lastPacketTime = 0;

struct BoatData {
    // Autopilot (from 0x84)
    int heading = -1;
    int rudder = 0;
    bool rudderValid = false;
    int mode = -1;
    int targetHeading = -1;
    int turnDirection = 0;
    int failureCode = -1;
    bool offCourse = false, windShift = false, largeXTE = false;
    bool noData = false, autoRelease = false;
    unsigned long apUpdated = 0;
    unsigned long last9cMs = 0;

    // Sensors
    int depth = -1, windAngle = -1, windSpeed = -1;
    int stw = -1, sog = -1, cog = -1, waterTemp = -1;
    unsigned long sensorSeen[SI_END] = {0};
    static constexpr int SENSOR_TIMEOUT = 10000;

    // GPS
    int gpsLatDeg = -1, gpsLatMin = -1, gpsLonDeg = -1, gpsLonMin = -1;
    bool gpsLatNorth = true, gpsLonEast = true;
    int gpsSats = -1, gpsHDOP = -1;
    unsigned long gpsSeen = 0;

    int tftRefresh = 0;

    // Message log
    char msgLog[MSG_LOG_SIZE][MSG_TEXT_LEN];
    int msgLogWriteIdx = 0;
    int msgLogCount = 0;

    void pushMsg(const char *text);
};
static BoatData g;

void BoatData::pushMsg(const char *text) {
    strncpy(g.msgLog[g.msgLogWriteIdx], text, MSG_TEXT_LEN - 1);
    g.msgLog[g.msgLogWriteIdx][MSG_TEXT_LEN - 1] = 0;
    g.msgLogWriteIdx = (g.msgLogWriteIdx + 1) % MSG_LOG_SIZE;
    if (g.msgLogCount < MSG_LOG_SIZE) g.msgLogCount++;
}

static void buildLogLine(char *buf, size_t sz, unsigned long ts,
                         const uint8_t *data, int len, const char *desc) {
    unsigned long secs = ts / 1000;
    int pos = snprintf(buf, sz, "%02lu:%02lu:%02lu,",
                       secs / 3600, (secs / 60) % 60, secs % 60);
    for (int i = 0; i < len && pos < (int)sz - 10; i++)
        pos += snprintf(buf + pos, sz - pos, " %02X", data[i]);
    pos += snprintf(buf + pos, sz - pos, ", %s", desc);
}

// 192x192 PNG, 1324 bytes — dark bg + cyan circle
// --- Display model — single source of truth for widget order ---
enum Widget : uint8_t { W_MODE, W_HEADING, W_RUDDER, W_ALARMS, W_TARGET, W_BTNS, W_END };
static const Widget widgetOrder[] = { W_MODE, W_HEADING, W_TARGET, W_ALARMS, W_RUDDER, W_BTNS, W_END };

// --- CYD layout bands — explicit y/height so widgets can never overlap ---
// Screen 320x240: top zone y=0..136 (buttons start y=136), status bar y=228..240.
struct Band { int16_t y, h; };
static const Band BAND[W_END] = {
    {  2, 18 },  // W_MODE    font2 16px → glyphs 3..19
    { 22, 54 },  // W_HEADING font7 48px → glyphs 25..73
    {110, 22 },  // W_RUDDER  track x4..315 y110..131, white-bordered box w/ value pill
    { 98, 10 },  // W_ALARMS  font1 8px  → glyphs 99..106
    { 78, 18 },  // W_TARGET  font2 16px → glyphs 79..95
    {  0,  0 },  // W_BTNS    fixed button coords, no band
};
static const Band STATUS_BAND = { 228, 12 };  // bottom status bar, font1

// --- Render-state cache — dirty-checking for flicker-free redraws ---
struct BandState { char text[56]; int16_t w; bool valid; };
static BandState gBand[W_END];
static BandState gStatusBand;
static bool gBtnsDrawn = false;
static bool gOtaActive = false;
struct RdrState { int8_t val; bool valid; bool drawn; int8_t numW; };
static RdrState gRdr = { 0, false, false, 0 };
static bool gLogView = false;
static int gLogScroll = 0;
static int gLogLastWriteIdx = -1;
static int gLogFrozenNewest = -1;
static int gLogViewDir = 0;
static int _flashIdx = -1;
static unsigned long _flashEnd = 0;
static int _driftFlashIdx = -1;

// Persistent log display slot cache
#define MAX_SLOTS 50
static struct Slot { char key[64]; int count; char text[MSG_TEXT_LEN]; bool seen; } gSlots[MAX_SLOTS];
static int gSlotCount = 0;

// Force full redraw of all bands after any fillScreen (WiFi change, OTA).
static void invalidateBands() {
    for (int i = 0; i < W_END; i++) gBand[i].valid = false;
    gStatusBand.valid = false;
    gRdr.drawn = false;
    gBtnsDrawn = false;
}

// ============= TFT color constants =============
// Conversion to 24-bit web colors (for HTML/JS) — run:
//   python3 -c "c=0xVVVV;r=(c>>11&31)*255//31;g=(c>>5&63)*255//63;b=(c&31)*255//31;print(f'#{r:02x}{g:02x}{b:02x}')"
#define TFT_MODE_STBY   0xF81F  // magenta, matches fallback
#define TFT_MODE_ACT    0x07E0  // green
#define TFT_MODE_FALL   TFT_MODE_STBY
#define TFT_HDG         0x07FF  // cyan
#define TFT_HDG_SIM     0xFFA0  // yellow — simulated heading
#define TFT_RDR_NORM    0x07FF  // cyan
#define TFT_RDR_EXT     0xF800  // red
#define TFT_ALARM       0xF800  // red
#define TFT_ALARM_WARN  0xFB20  // orange
#define TFT_TRG         0x07E0  // green — starboard turn
#define TFT_TRG_PORT    0xC800  // red — port turn
#define TFT_TRG_ON      0x07FF  // cyan — at course (matches heading)
#define TFT_BTN_AP      0x4002  // dark red
#define TFT_BTN_AP_HI   0xD000  // bright red (active AP mode)
#define TFT_BTN_ADJ     TFT_BTN_NAV
#define TFT_BTN_NAV     0x008C  // dark blue
#define TFT_BTN_FLUXSIM    0xD640  // yellow (active)
#define TFT_BTN_FLUXSIM_DIM 0x4A69  // dark grey (inactive)

// --- Log view muted colors (grey tones → desaturated hues) ---
#define TFT_LOG_TS   0x6C54  // muted steel blue
#define TFT_LOG_HX   0x6C4D  // muted sage green
#define TFT_LOG_SEP  0x7B4F  // muted mauve
#define TFT_LOG_DESC 0xD508  // muted amber
#define TFT_LOG_HI   0xFDC0  // bright amber (newest entry)
#define TFT_HL_BG   0x4980  // warm amber bg for newest row
#define TFT_LOG_SEP_HI 0xE5B8  // bright mauve (counter on hl)
#define TFT_LOG_TS_HI  0x7EFC  // bright steel blue (ts on hl)
#define TFT_LOG_HX_HI  0x87C0  // bright sage green (hex on hl)
#define TFT_LOG_DESC_TX 0xD400  // muted orange (TX)
#define TFT_LOG_HI_TX   0xFD40  // bright orange (TX newest)

// --- Touchscreen buttons ---
// Convention: when adding a button here, also add a <button> in handleRoot() HTML
// with the same api= value and onclick handler. Layout must match between both.
#define NUM_BUTTONS 14
struct ButtonDef {
    const char *api;
    const char *label;
    uint8_t cmd;
    uint32_t color;
    int x, y, w, h;
};
static ButtonDef btns[NUM_BUTTONS] = {
    {"auto",     "AUTO",  0x01, TFT_BTN_AP,  220, 136, 96,  56},
    {"standby",  "STBY",  0x02, TFT_BTN_AP,  4,   136, 96,  56},
    {"plus1",    "+1",    0x07, TFT_BTN_ADJ, 164, 136, 52,  27},
    {"minus1",   "-1",    0x05, TFT_BTN_ADJ, 108, 136, 52,  27},
    {"plus10",   "+10",   0x08, TFT_BTN_ADJ, 164, 167, 52,  27},
    {"minus10",  "-10",   0x06, TFT_BTN_ADJ, 108, 167, 52,  27},
    {"wind",     "WIND",  0x23, TFT_BTN_NAV, 4,   198, 96,  24},
    {"fluxsim",  "SIM",   0xff, TFT_BTN_FLUXSIM,108, 198, 52,  24},
    {"fixedhdg", "SET",   0xF9, TFT_BTN_FLUXSIM,164, 198, 52,  24},
    {"track",    "TRACK", 0x28, TFT_BTN_NAV, 220, 198, 96,  24},
    {"hdg_m10",  "-10",   0xFE, TFT_BTN_FLUXSIM, 4,  198, 48, 24},
    {"hdg_m1",   "-1",    0xFD, TFT_BTN_FLUXSIM, 52, 198, 48, 24},
    {"hdg_p1",   "+1",    0xFC, TFT_BTN_FLUXSIM, 220, 198, 48, 24},
    {"hdg_p10",  "+10",   0xFB, TFT_BTN_FLUXSIM, 268, 198, 48, 24},
};

static int seatalkHeading(uint8_t byte1, uint8_t byte2) {
    uint8_t u = (byte1 >> 4) & 0x0F;
    int deg = ((u & 3) * 90)
            + ((byte2 & 0x3F) * 2)
            + ((u & 0x0C) ? (((u & 0x0C) == 0x0C) ? 2 : 1) : 0);
    if (deg > 360) deg -= 360;
    return deg;
}

// Mode display helpers — single source of truth for TFT + JSON
static const char *modeLabel(int mode) {
    return mode == 2 ? "AUTO" : mode == 4 ? "VANE"
         : mode == 8 ? "TRACK" : mode == 0 ? "STANDBY" : "MODE";
}
static uint16_t modeColor(int mode) {
    return mode == 2 || mode == 4 || mode == 8 ? TFT_MODE_ACT
         : mode == 0 ? TFT_MODE_STBY : TFT_MODE_FALL;
}
static const char *modeColorName(int mode) {
    return mode == 2 || mode == 4 || mode == 8 ? "green"
         : mode == 0 ? "yellow" : "dim";
}

//======================================================================
//4. SD LOGGING
//======================================================================

// --- SD card helpers ---
static int findNextLogNumber() {
    int maxN = 0;
    File root = SD.open("/");
    if (root) {
        File entry = root.openNextFile();
        while (entry) {
            if (!entry.isDirectory()) {
                String n(entry.name());
                if (n.startsWith("LOG_") && n.endsWith(".TXT")) {
                    int v = n.substring(4, 8).toInt();
                    if (v > maxN) maxN = v;
                }
            }
            entry.close();
            entry = root.openNextFile();
        }
        root.close();
    }
    return maxN + 1;
}

static void cleanupOldLogs() {
    uint64_t used = SD.usedBytes();
    uint64_t total = SD.totalBytes();
    if (used < total * SD_LOG_PCT / 100) return;
    File root = SD.open("/");
    if (!root) return;
    struct LogFile { int num; char name[32]; };
    LogFile files[64];
    int count = 0;
    File entry = root.openNextFile();
    while (entry && count < 64) {
        const char *name = entry.name();
        if (strncmp(name, "LOG_", 4) == 0) {
            files[count].num = atoi(name + 4);
            strncpy(files[count].name, name, 31);
            files[count].name[31] = 0;
            count++;
        }
        entry.close();
        entry = root.openNextFile();
    }
    root.close();
    if (count == 0) return;
    for (int i = 0; i < count - 1; i++)
        for (int j = i + 1; j < count; j++)
            if (files[j].num < files[i].num) {
                LogFile t = files[i]; files[i] = files[j]; files[j] = t;
            }
    for (int i = 0; i < count && SD.usedBytes() >= total * SD_LOG_PCT / 100; i++)
        SD.remove(files[i].name);
}

static int _logPending = 0;
static unsigned long _lastFlush = 0;
static const int LOG_FLUSH_ENTRIES = 50;
static const unsigned long LOG_FLUSH_MS = 30000;

static void logDatagram(const char *line) {
    if (!logFile) return;
    logFile.println(line);
    _logPending++;
    unsigned long now = millis();
    if (_logPending >= LOG_FLUSH_ENTRIES || now - _lastFlush >= LOG_FLUSH_MS) {
        logFile.flush();
        _logPending = 0;
        _lastFlush = now;
    }
    cleanupOldLogs();
}

static bool initSD() {
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if (!SD.begin(SD_CS, SPI, 4000000)) return false;
    logNumber = findNextLogNumber();
    char filename[32];
    snprintf(filename, sizeof(filename), "/LOG_%04d.TXT", logNumber);
    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile) return false;
    logFile.println(filename + 1);
    logFile.flush();
    return true;
}


//======================================================================
//5. SEATALK — TX, protocol helpers, RX parser
//======================================================================

static bool sendDatagram(const uint8_t *data, int len) {
    unsigned long deadline = millis() + 100;
    while (_txActive) {
        if (millis() > deadline) return false;
        delay(1);
    }
    _txActive = true;

    deadline = millis() + 50;
    while (millis() - _lastBusy < BUS_IDLE_MS) {
        if (millis() > deadline) { _txActive = false; return false; }
        delay(1);
    }

    while (Seatalk.available()) Seatalk.read();

    bool ok = true;
    for (int i = 0; i < len; i++) {
        uint8_t b = data[i];
        uint8_t p = 0;
        for (int j = 0; j < 8; j++) p ^= (b >> j) & 1;
        int want = (i == 0) ? 1 : 0;
        uart_set_parity(UART_NUM_2, (p == want) ? UART_PARITY_EVEN : UART_PARITY_ODD);

        Seatalk.write(b);
        unsigned long echoDeadline = millis() + 10;
        bool gotEcho = false;
        while (millis() < echoDeadline) {
            if (Seatalk.available()) {
                if (Seatalk.read() != b) ok = false;
                gotEcho = true;
                break;
            }
            delay(1);
        }
        if (!gotEcho) ok = false;
    }

    while (Seatalk.available()) Seatalk.read();
    uart_set_parity(UART_NUM_2, UART_PARITY_EVEN);
    _lastBusy = millis();
    _txActive = false;
    return ok;
}

static void sendHeading() {
    if (!_hdgActive) return;
    unsigned long now = millis();
    if (now - _lastHdgTx < _hdgInterval) return;

    int8_t rudder = g.rudder;
    uint16_t oldDeg = _hdgDeg;
    if (_hdgActive && _hdgAutoDrift && g.rudderValid && rudder >= -30 && rudder <= 30 && rudder != 0) {
        float r = (float)rudder;
        float rate = (fabs(r) / 30.0f) * 3.0f;
        float step = rate * (_hdgInterval / 1000.0f);
        if (r > 0)      _hdgFrac += step;
        else if (r < 0) _hdgFrac -= step;
        while (_hdgFrac >= 1.0f)  { _hdgFrac -= 1.0f; _hdgDeg = (_hdgDeg + 1) % 360; }
        while (_hdgFrac <= -1.0f) { _hdgFrac += 1.0f; _hdgDeg = (_hdgDeg + 359) % 360; }
    }
    if (_hdgDeg != oldDeg)
        _driftFlashIdx = (_hdgDeg > oldDeg || (oldDeg - _hdgDeg) > 180) ? 12 : 11;

    int deg = _hdgDeg % 360;
    uint8_t u = deg / 90;
    uint8_t temp = deg % 90;
    uint8_t vw = temp / 2;
    uint8_t odd = temp - vw * 2;
    u |= (odd * 2) << 2;
    uint8_t msg[] = {0x89, (uint8_t)((u << 4) | 0x02), vw, 0x00, 0x20};
    _suppressEchoCmd = 0x89;
    sendDatagram(msg, 5);
    _suppressEchoCmd = 0;
    _lastHdgTx = now;
    char desc[MSG_TEXT_LEN], line[MSG_TEXT_LEN];
    snprintf(desc, sizeof(desc), "> HDG %d", deg);
    uint8_t dg[] = {0x9C};
    buildLogLine(line, MSG_TEXT_LEN, millis(), dg, 1, desc);
    g.pushMsg(line);
    logDatagram(line);
}

static bool _userCommanded = false;
static int  _lastUserMode = -1;

static void simHdg_auto(uint16_t d) {
    _hdgDeg = d; _hdgActive = true; _hdgAutoDrift = true;
    _hdgInterval = 250; _lastHdgTx = 0; _hdgFrac = 0; gBtnsDrawn = false;
}
static void simHdg_fixed(uint16_t d) {
    _hdgDeg = d; _hdgActive = true; _hdgAutoDrift = false;
    _hdgInterval = 250; _lastHdgTx = 0; _hdgFrac = 0; gBtnsDrawn = false;
}
static void stopHdg_int() {
    _hdgActive = false; _hdgDeg = 0; _hdgFrac = 0; gBtnsDrawn = false;
}

static void pressAutopilotButton(const struct ButtonDef &b) {
    uint8_t dg[] = {0x86, 0x21, b.cmd, (uint8_t)(b.cmd ^ 0xFF)};
    char desc[MSG_TEXT_LEN];
    char line[MSG_TEXT_LEN];
    _suppressEchoCmd = 0x86;
    bool ok = sendDatagram(dg, 4);
    _suppressEchoCmd = 0;
    if (ok) {
        snprintf(desc, sizeof(desc), "> %s", b.label);
        if (b.cmd == 0x01) _lastUserMode = 2;
        if (b.cmd == 0x02) _lastUserMode = 0;
        _userCommanded = true;
    } else {
        snprintf(desc, sizeof(desc), "> %s FAIL", b.label);
    }
    buildLogLine(line, MSG_TEXT_LEN, millis(), dg, 4, desc);
    g.pushMsg(line);
    logDatagram(line);
}

// Table-driven SeaTalk parser — each cmd/msg-handler pair is a table entry.
// Handlers extract data from buf, update g fields, and write desc.
typedef void (*SeaTalkParseFn)(const uint8_t *buf, uint8_t msgLen, unsigned long now, char *desc);
struct SeaTalkCmd { uint8_t cmd; uint8_t minLen; SeaTalkParseFn parse; };

static void parseHeadingCommon(const uint8_t *buf, uint8_t msgLen, unsigned long now, char *desc) {
    int deg = seatalkHeading(buf[1], buf[2]);
    if (buf[0] == 0x53) { g.cog = deg; g.sensorSeen[SI_COG] = now; }
    else g.heading = deg;
    snprintf(desc, MSG_TEXT_LEN, "HDG %d", deg);
}
static void parse0x84(const uint8_t *buf, uint8_t msgLen, unsigned long now, char *desc) {
    int deg = seatalkHeading(buf[1], buf[2]);
    if (millis() - g.last9cMs > HDG_OVERRIDE_MS) g.heading = deg;
    g.mode = buf[4] & 0x0F;
    g.rudder = (int8_t)buf[6];
    g.rudderValid = true;
    g.targetHeading = (((buf[2] & 0xC0) >> 6) * 90) + (buf[3] / 2);
    g.turnDirection = (buf[1] & 0x80) ? 2 : 1;
    g.offCourse   = buf[5] & 0x04;
    g.windShift   = buf[5] & 0x08;
    g.largeXTE    = buf[7] & 0x10;
    g.noData      = buf[7] & 0x08;
    g.autoRelease = buf[7] & 0x80;
    g.apUpdated = now;
    snprintf(desc, MSG_TEXT_LEN, "HDG %d TRG %d RDR %d", deg, g.targetHeading, g.rudder);
}
static void parse0x9C(const uint8_t *buf, uint8_t msgLen, unsigned long now, char *desc) {
    int deg = seatalkHeading(buf[1], buf[2]);
    g.heading = deg;
    g.last9cMs = now;
    g.rudder = (int8_t)buf[3];
    g.rudderValid = true;
    snprintf(desc, MSG_TEXT_LEN, "HDG %d RDR %d", deg, g.rudder);
}
static void parse0x10(const uint8_t *buf, uint8_t msgLen, unsigned long now, char *desc) {
    if (buf[1] == 0x01 && msgLen >= 4) {
        g.windAngle = ((buf[2] << 8) | buf[3]);
        g.sensorSeen[SI_WIND_ANGLE] = now;
        snprintf(desc, MSG_TEXT_LEN, "AWA %d", g.windAngle);
    } else {
        parseHeadingCommon(buf, msgLen, now, desc);
    }
}
static void parse0x90(const uint8_t *buf, uint8_t msgLen, unsigned long now, char *desc) {
    snprintf(desc, MSG_TEXT_LEN, "SYS %d", buf[1] & 0x7F);
}
static void parse0x83(const uint8_t *buf, uint8_t msgLen, unsigned long now, char *desc) {
    g.failureCode = buf[2];
    snprintf(desc, MSG_TEXT_LEN, "FAIL %d", g.failureCode);
}
static void parse0x00(const uint8_t *buf, uint8_t msgLen, unsigned long now, char *desc) {
    g.depth = ((buf[3] << 8) | buf[4]);
    g.sensorSeen[SI_DEPTH] = now;
    snprintf(desc, MSG_TEXT_LEN, "DPT %d", g.depth);
}
static void parse0x11(const uint8_t *buf, uint8_t msgLen, unsigned long now, char *desc) {
    g.windSpeed = ((buf[2] << 8) | buf[3]);
    g.sensorSeen[SI_WIND_SPEED] = now;
    snprintf(desc, MSG_TEXT_LEN, "AWS %d", g.windSpeed);
}
static void parse0x20(const uint8_t *buf, uint8_t msgLen, unsigned long now, char *desc) {
    g.stw = ((buf[2] << 8) | buf[3]);
    g.sensorSeen[SI_STW] = now;
    snprintf(desc, MSG_TEXT_LEN, "STW %d", g.stw);
}
static void parse0x52(const uint8_t *buf, uint8_t msgLen, unsigned long now, char *desc) {
    g.sog = ((buf[2] << 8) | buf[3]);
    g.sensorSeen[SI_SOG] = now;
    snprintf(desc, MSG_TEXT_LEN, "SOG %d", g.sog);
}
static void parse0x23(const uint8_t *buf, uint8_t msgLen, unsigned long now, char *desc) {
    g.waterTemp = buf[2];
    g.sensorSeen[SI_WATER_TEMP] = now;
    snprintf(desc, MSG_TEXT_LEN, "TEMP %d", g.waterTemp);
}
static void parse0x50(const uint8_t *buf, uint8_t msgLen, unsigned long now, char *desc) {
    g.gpsLatDeg = buf[2];
    int min100 = ((buf[3] << 8) | buf[4]) & 0x7FFF;
    g.gpsLatMin = min100;
    g.gpsLatNorth = !(buf[3] & 0x80);
    g.gpsSeen = now;
    snprintf(desc, MSG_TEXT_LEN, "LAT %d %d %c", g.gpsLatDeg, g.gpsLatMin, g.gpsLatNorth ? 'N' : 'S');
}
static void parse0x51(const uint8_t *buf, uint8_t msgLen, unsigned long now, char *desc) {
    g.gpsLonDeg = buf[2];
    int min100 = ((buf[3] << 8) | buf[4]) & 0x7FFF;
    g.gpsLonMin = min100;
    g.gpsLonEast = !(buf[3] & 0x80);
    g.gpsSeen = now;
    snprintf(desc, MSG_TEXT_LEN, "LON %d %d %c", g.gpsLonDeg, g.gpsLonMin, g.gpsLonEast ? 'E' : 'W');
}
static void parse0x58(const uint8_t *buf, uint8_t msgLen, unsigned long now, char *desc) {
    uint8_t z = buf[1] >> 4;
    g.gpsLatNorth = !(z & 0x01);
    g.gpsLonEast = !(z & 0x02);
    g.gpsLatDeg = buf[2];
    int latMin1000 = ((buf[3] << 8) | buf[4]);
    g.gpsLatMin = latMin1000 / 10;
    g.gpsLonDeg = buf[5];
    int lonMin1000 = ((buf[6] << 8) | buf[7]);
    g.gpsLonMin = lonMin1000 / 10;
    g.gpsSeen = now;
    snprintf(desc, MSG_TEXT_LEN, "GPS %d %d %c %d %d %c",
             g.gpsLatDeg, g.gpsLatMin, g.gpsLatNorth ? 'N' : 'S',
             g.gpsLonDeg, g.gpsLonMin, g.gpsLonEast ? 'E' : 'W');
}
static void parse0x57(const uint8_t *buf, uint8_t msgLen, unsigned long now, char *desc) {
    g.gpsSats = buf[2];
    g.gpsHDOP = buf[3];
    snprintf(desc, MSG_TEXT_LEN, "SATS %d HDOP %d", g.gpsSats, g.gpsHDOP);
}

static const SeaTalkCmd SEATALK_CMDS[] = {
    {0x00, 5, parse0x00}, {0x10, 3, parse0x10}, {0x11, 4, parse0x11},
    {0x20, 4, parse0x20}, {0x23, 3, parse0x23}, {0x50, 5, parse0x50},
    {0x51, 5, parse0x51}, {0x52, 4, parse0x52}, {0x53, 3, parseHeadingCommon},
    {0x57, 4, parse0x57}, {0x58, 8, parse0x58}, {0x83, 3, parse0x83},
    {0x84, 9, parse0x84}, {0x89, 3, parseHeadingCommon}, {0x90, 3, parse0x90},
    {0x9C, 4, parse0x9C},
};

static void processByte(uint8_t b) {
    static uint8_t buf[32];
    static int len = 0;
    static unsigned long lastMs = 0;

    unsigned long now = millis();
    if (len > 0 && (now - lastMs) >= 50)
        len = 0;
    if (len < 32) buf[len++] = b;
    lastMs = now;

    if (len >= 3) {
        uint8_t cmd = buf[0];
        int msgLen = 3 + (buf[1] & 0x0F);
        if (msgLen >= 3 && len >= msgLen) {
            char desc[MSG_TEXT_LEN];
            char line[MSG_TEXT_LEN];

            const SeaTalkCmd *match = nullptr;
            for (const auto &c : SEATALK_CMDS) {
                if (c.cmd == cmd && msgLen >= c.minLen) { match = &c; break; }
            }
            if (match) match->parse(buf, msgLen, now, desc);
            else snprintf(desc, sizeof(desc), "DATAGRAM");

            if (cmd == 0x84) {
                if (_userCommanded && g.mode == 0 && _lastUserMode == 2) {
                    buildLogLine(line, MSG_TEXT_LEN, millis(), buf, msgLen, "AP FAULTED TO STBY");
                    g.pushMsg(line);
                    logDatagram(line);
                    _userCommanded = false;
                }
                if (g.mode == 2) _userCommanded = false;
            }
            len = 0;

            if (cmd == _suppressEchoCmd) { _suppressEchoCmd = 0; }
            else {
                buildLogLine(line, MSG_TEXT_LEN, millis(), buf, msgLen, desc);
                g.pushMsg(line);
                logDatagram(line);
            }
        }
    }
}


//======================================================================
//6. AUDIO — speaker
//======================================================================

// --- Speaker ---
#define SPEAKER_PIN     26
#define SPEAKER_CHANNEL 0
#define SPEAKER_RES     8

struct Note { uint16_t freq; uint16_t dur; };

static const Note STARTUP_MELODY[] = {
    {523, 150}, {659, 150}, {784, 250}
};

#define STARTUP_MELODY_LEN (sizeof(STARTUP_MELODY) / sizeof(STARTUP_MELODY[0]))

enum SpkState { SPK_IDLE, SPK_ACTIVE };
static SpkState _spkState = SPK_IDLE;
static const Note *_spkMelody = nullptr;
static int _spkLen = 0, _spkIdx = -1;
static unsigned long _spkNoteEnd = 0;

// --- Non-blocking speaker ---
static void playMelody(const Note *melody, int len) {
    if (_spkState == SPK_ACTIVE) {
        ledcWrite(SPEAKER_CHANNEL, 0);
    }
    _spkMelody = melody;
    _spkLen = len;
    _spkIdx = -1;
    _spkState = SPK_ACTIVE;
    _spkNoteEnd = 0;
}

static void playBeep(uint16_t freq, uint16_t dur) {
    static const Note beep[] = {{freq, dur}};
    playMelody(beep, 1);
}
static void playTick() { playBeep(2600, 8); }

static void updateSpeaker() {
    if (_spkState != SPK_ACTIVE) return;
    unsigned long now = millis();
    if (now < _spkNoteEnd) return;

    if (_spkIdx >= 0) {
        ledcWrite(SPEAKER_CHANNEL, 0);
    }

    _spkIdx++;
    if (_spkIdx >= _spkLen) {
        ledcDetachPin(SPEAKER_PIN);
        _spkState = SPK_IDLE;
        return;
    }

    auto &n = _spkMelody[_spkIdx];
    if (n.freq > 0) {
        ledcDetachPin(SPEAKER_PIN);
        ledcAttachPin(SPEAKER_PIN, SPEAKER_CHANNEL);
        ledcSetup(SPEAKER_CHANNEL, n.freq, SPEAKER_RES);
        ledcWrite(SPEAKER_CHANNEL, 1 << (SPEAKER_RES - 1));
    }

    _spkNoteEnd = now + n.dur;
}


//======================================================================
//7. DISPLAY — bands, widgets, log view
//======================================================================

static void drawAllButtons() {
    for (int i = 0; i < NUM_BUTTONS; i++) {
        auto &b = btns[i];
        if (!_hdgActive && (b.cmd == 0xFE || b.cmd == 0xFD || b.cmd == 0xFC || b.cmd == 0xFB))
            continue;
        if (_hdgActive && (b.cmd == 0x23 || b.cmd == 0x28))
            continue;
        uint16_t c;
        if (b.cmd == 0xff)
            c = (_hdgActive && _hdgAutoDrift) ? TFT_BTN_FLUXSIM : TFT_BTN_FLUXSIM_DIM;
        else if (b.cmd == 0xF9)
            c = (_hdgActive && !_hdgAutoDrift) ? TFT_BTN_FLUXSIM : TFT_BTN_FLUXSIM_DIM;
        else if (strcmp(b.api, "auto") == 0 && g.mode == 2)
            c = TFT_BTN_AP_HI;
        else if (strcmp(b.api, "standby") == 0 && g.mode == 0)
            c = TFT_BTN_AP_HI;
        else if (_hdgActive && (b.cmd == 0xFE || b.cmd == 0xFD || b.cmd == 0xFC || b.cmd == 0xFB))
            c = TFT_BTN_FLUXSIM_DIM;
        else
            c = b.color;
        tft.fillRect(b.x, b.y, b.w, b.h, c);
        tft.drawRect(b.x, b.y, b.w, b.h, TFT_DARKGREY);
        tft.setTextColor(TFT_WHITE, c);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(b.label, b.x + b.w / 2, b.y + b.h / 2, 2);
    }
    gBtnsDrawn = true;
}

static void drawStatusBar() {
    tft.fillScreen(TFT_BLACK);
    invalidateBands();
    g.tftRefresh = 0;
    drawAllButtons();
}

static void flashButton(int idx) {
    auto &b = btns[idx];
    tft.fillRect(b.x, b.y, b.w, b.h, TFT_WHITE);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(b.label, b.x + b.w / 2, b.y + b.h / 2, 2);
    playBeep(1760, 30);
    _flashIdx = idx;
    _flashEnd = millis() + 80;
}

// Erase only the side strips between old and new centered text widths in a band.
static void eraseBandSides(const Band &b, int oldW, int newW) {
    int oldL = 160 - oldW / 2, newL = 160 - newW / 2;
    if (newL > oldL) tft.fillRect(oldL, b.y, newL - oldL, b.h, TFT_BLACK);
    int newR = newL + newW, oldR = oldL + oldW;
    if (oldR > newR) tft.fillRect(newR, b.y, oldR - newR, b.h, TFT_BLACK);
}

// Draw text centered in a band; flicker-free: unchanged content is skipped,
// changed content overwrites glyphs via opaque background (no black flash).
// Only when the new text is narrower are the old side strips erased.
static void drawBandText(const char *s, const Band &b, uint8_t font, uint16_t fg,
                         BandState &st) {
    if (st.valid && strcmp(s, st.text) == 0) return;
    tft.setTextFont(font);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(fg, TFT_BLACK);
    int w = tft.textWidth(s);
    if (!st.valid) {
        tft.fillRect(0, b.y, 320, b.h, TFT_BLACK);  // first draw after screen clear
    } else if (w < st.w) {
        eraseBandSides(b, st.w, w);
    }
    tft.drawString(s, 160, b.y + b.h / 2);
    strncpy(st.text, s, sizeof(st.text) - 1);
    st.text[sizeof(st.text) - 1] = 0;
    st.w = w;
    st.valid = true;
}

// --- Alarm badges: one segment per alarm, per-type colors, 12px gaps, no boxes ---
struct AlarmSeg { const char *t; uint16_t c; };
#define ALARM_GAP 12  // px between badges (2x font1 space width)

// Draw alarm badges centered as a group; flicker-free: unchanged set is skipped,
// on change only the group span (+ old side strips) is erased, never the band.
static void drawAlarmBand(const AlarmSeg *segs, int n) {
    const Band &b = BAND[W_ALARMS];
    BandState &st = gBand[W_ALARMS];
    char key[56] = "";
    for (int i = 0; i < n; i++)
        strncat(key, segs[i].t, sizeof(key) - strlen(key) - 1);
    if (st.valid && strcmp(key, st.text) == 0) return;
    tft.setTextFont(1);
    int widths[6];
    int total = n > 0 ? ALARM_GAP * (n - 1) : 0;
    for (int i = 0; i < n; i++) { widths[i] = tft.textWidth(segs[i].t); total += widths[i]; }
    if (!st.valid) tft.fillRect(0, b.y, 320, b.h, TFT_BLACK);
    else if (total < st.w) eraseBandSides(b, st.w, total);
    int x = 160 - total / 2;
    if (total > 0) tft.fillRect(x, b.y, total, b.h, TFT_BLACK);  // span: segment boundaries move
    tft.setTextDatum(ML_DATUM);
    int yc = b.y + b.h / 2;
    for (int i = 0; i < n; i++) {
        tft.setTextColor(segs[i].c);  // single-arg = transparent background
        tft.drawString(segs[i].t, x, yc);
        x += widths[i] + ALARM_GAP;
    }
    strncpy(st.text, key, sizeof(st.text) - 1);
    st.text[sizeof(st.text) - 1] = 0;
    st.w = total;
    st.valid = true;
}

static void drawWidgets() {
    for (auto w : widgetOrder) {
        switch (w) {
        case W_MODE: {
            drawBandText(modeLabel(g.mode), BAND[W_MODE], 2, modeColor(g.mode), gBand[W_MODE]);
            break;
        }
        case W_HEADING: {
            uint16_t hdgColor = _hdgActive ? TFT_HDG_SIM : TFT_HDG;
            if (g.heading >= 0) {
                char buf[8];
                snprintf(buf, sizeof(buf), "%d", g.heading);
                drawBandText(buf, BAND[W_HEADING], 7, hdgColor, gBand[W_HEADING]);
            } else {
                gBand[W_HEADING].valid = false;
                drawBandText("HDG", BAND[W_HEADING], 7, hdgColor, gBand[W_HEADING]);
            }
            break;
        }
        case W_RUDDER: {
            const Band &b = BAND[W_RUDDER];
            const int cx = 160;                   // screen center
            const int tx = 4, tw = 312;           // track outline — same span as buttons (4..315)
            const int ty = b.y, th = b.h;         // track = full band height (110..131)
            const int span = 150;                 // px from center to ±30° → exactly 5px per degree
            const int fy = b.y + 2, fh = b.h - 4; // fill area (112..129, 2px inset from track)
            // box uses ty/th (full track outline) so its border merges with the track edge
            auto drawTicks = [&]() {
                tft.drawFastVLine(cx, b.y + 1, b.h - 2, TFT_DARKGREY);   // center tick
                for (int i = 5; i < 30; i += 5) {                        // ±5,10,15,20,25
                    tft.drawFastVLine(cx - i * (span / 30), fy, fh, TFT_DARKGREY);
                    tft.drawFastVLine(cx + i * (span / 30), fy, fh, TFT_DARKGREY);
                }
            };
            if (!gRdr.drawn) {
                tft.fillRect(0, b.y, 320, b.h, TFT_BLACK);
                tft.drawRect(tx, ty, tw, th, TFT_DARKGREY);
                drawTicks();
                gRdr.drawn = true;
                gRdr.valid = false;                  // force fill evaluation below
                gRdr.val = 0;
                gRdr.numW = 0;
            }
            int val = g.rudderValid ? g.rudder : 0;
            if (val == gRdr.val && g.rudderValid == gRdr.valid && gRdr.numW > 0) break;
            // Erase previous fill
            if (gRdr.valid && gRdr.val != 0) {
                int ow = constrain(abs(gRdr.val), 0, 30) * span / 30;
                int ox = gRdr.val >= 0 ? cx + 2 : cx - 2 - ow + 1;
                tft.fillRect(ox, fy, ow, fh, TFT_BLACK);
            }
            // Erase previous box and restore track outline
            if (gRdr.numW > 0) {
                tft.fillRect(cx - 25, ty, 51, th, TFT_BLACK);
                tft.drawRect(tx, ty, tw, th, TFT_DARKGREY);
            }
            // Fill
            if (g.rudderValid) {
                int av = constrain(abs(val), 0, 30);
                int w = av * span / 30;
                uint16_t c = av >= 30 ? TFT_RDR_EXT : TFT_RDR_NORM;
                if (w > 0) {
                    if (val >= 0) tft.fillRect(cx + 2, fy, w, fh, c);
                    else          tft.fillRect(cx - 2 - w + 1, fy, w, fh, c);
                }
            }
            drawTicks();
            // Grey-bordered box aligned with the first ticks (±25px), outline height
            char buf[5];
            if (g.rudderValid) snprintf(buf, sizeof(buf), "%d", val);
            else               strcpy(buf, "RDR");
            tft.setTextFont(2);
            tft.setTextDatum(MC_DATUM);
            int numW = tft.textWidth(buf);
            tft.drawRect(cx - 25, ty, 51, th, TFT_DARKGREY);
            tft.fillRect(cx - 24, ty + 1, 49, th - 2, TFT_BLACK);
            tft.setTextColor(TFT_WHITE);
            tft.drawString(buf, cx, b.y + b.h / 2);
            gRdr.numW = numW;
            gRdr.val = val;
            gRdr.valid = g.rudderValid;
            break;
        }
        case W_ALARMS: {
            AlarmSeg segs[6];
            int n = 0;
            bool recent = millis() - g.apUpdated < AP_RECENT_MS;
            if (g.autoRelease && recent) segs[n++] = {"AUTO REL", TFT_ALARM};
            if (g.offCourse && recent)   segs[n++] = {"OFF CRS",  TFT_ALARM};
            if (g.windShift && recent)   segs[n++] = {"WND SHFT", TFT_ALARM_WARN};
            if (g.largeXTE && recent)    segs[n++] = {"LRG XTE",  TFT_ALARM};
            if (g.noData && recent)      segs[n++] = {"NO DATA",  TFT_ALARM_WARN};
            drawAlarmBand(segs, n);
            break;
        }
        case W_TARGET: {
            char buf[12] = "TRG";
            uint16_t color = TFT_TRG;
            if (g.mode >= 2 && g.targetHeading >= 0 && g.heading >= 0) {
                int diff = (g.targetHeading - g.heading + 540) % 360 - 180;
                if (abs(diff) <= 3) {
                    snprintf(buf, sizeof(buf), "%d", g.targetHeading);
                    color = TFT_TRG_ON;
                } else if (g.turnDirection == 2) {
                    snprintf(buf, sizeof(buf), "> %d", g.targetHeading);
                    color = TFT_TRG;
                } else {
                    snprintf(buf, sizeof(buf), "< %d", g.targetHeading);
                    color = TFT_TRG_PORT;
                }
            } else if (g.mode >= 2 && g.targetHeading >= 0) {
                snprintf(buf, sizeof(buf), "%c %d",
                         g.turnDirection == 2 ? '>' : '<',
                         g.targetHeading);
            }
            drawBandText(buf, BAND[W_TARGET], 2, color, gBand[W_TARGET]);
            break;
        }
        case W_BTNS: {
            static int lastMode = -1;
            if (!gBtnsDrawn || g.mode != lastMode) {
                drawAllButtons();
                lastMode = g.mode;
            }
            break;
        }
        }
    }
}

static void msgKey(const char *msg, char *key, int keyLen) {
    const char *c1 = strrchr(msg, ',');
    const char *body = c1 ? c1 + 1 : msg;
    int ki = 0;
    bool lastSpace = true;
    for (; *body && ki < keyLen - 1; body++) {
        char ch = *body;
        if ((ch >= '0' && ch <= '9') || ch == '-') continue;
        if (ch == ' ') { if (lastSpace) continue; lastSpace = true; }
        else lastSpace = false;
        key[ki++] = ch;
    }
    while (ki > 0 && key[ki-1] == ' ') ki--;
    key[ki] = 0;
}

static void logViewScroll(int dir) {
    const int MAX_VISIBLE = (STATUS_BAND.y - 6) / 13;
    int active = 0;
    for (int i = 0; i < gSlotCount; i++)
        if (gSlots[i].count > 0) active++;
    if (active == 0) return;
    int maxScroll = max(0, active - MAX_VISIBLE);
    int step = max(1, MAX_VISIBLE / 3);
    if (dir > 0) gLogScroll = min(gLogScroll + step, maxScroll);
    else gLogScroll = max(0, gLogScroll - step);
}

static void drawLogView() {
    int total = min(g.msgLogCount, (int)MSG_LOG_SIZE);

    const int LINE_H = 13;
    const int X_CNT = 5, X_TS = 40, X_HX = 85, X_DESC = 210;
    const int HX_MAX_W = X_DESC - X_HX - 4;
    const int TOP = 6;
    const int MAX_VISIBLE = (STATUS_BAND.y - TOP) / LINE_H;

    // Update existing slots from ring buffer, add new ones
    for (int i = 0; i < gSlotCount; i++) gSlots[i].seen = false;

    int newestIdx = (g.msgLogWriteIdx - 1 + MSG_LOG_SIZE) % MSG_LOG_SIZE;
    char curKey[64];
    static int gNewestSlot = -1;
    bool firstEncounter = true;

    for (int i = 0; i < total; i++) {
        int idx = (newestIdx - i + MSG_LOG_SIZE) % MSG_LOG_SIZE;
        const char *msg = g.msgLog[idx];
        msgKey(msg, curKey, 64);

        int found = -1;
        for (int j = 0; j < gSlotCount; j++) {
            if (strcmp(gSlots[j].key, curKey) == 0) { found = j; break; }
        }
        if (found >= 0) {
            if (!gSlots[found].seen) {
                gSlots[found].count = 1;
                gSlots[found].seen = true;
                strncpy(gSlots[found].text, msg, MSG_TEXT_LEN - 1);
                if (firstEncounter) { gNewestSlot = found; firstEncounter = false; }
            } else {
                gSlots[found].count++;
                if (gSlots[found].count > 999) gSlots[found].count = 1;
            }
        } else if (gSlotCount < MAX_SLOTS) {
            strcpy(gSlots[gSlotCount].key, curKey);
            gSlots[gSlotCount].count = 1;
            gSlots[gSlotCount].seen = true;
            strncpy(gSlots[gSlotCount].text, msg, MSG_TEXT_LEN - 1);
            if (firstEncounter) { gNewestSlot = gSlotCount; firstEncounter = false; }
            gSlotCount++;
        }
    }

    bool anyActive = false;
    for (int i = 0; i < gSlotCount; i++)
        if (gSlots[i].count > 0) { anyActive = true; break; }

    if (!anyActive) {
        tft.fillRect(0, 0, 320, STATUS_BAND.y, TFT_BLACK);
        return;
    }

    int active = 0;
    for (int i = 0; i < gSlotCount; i++)
        if (gSlots[i].count > 0) active++;
    int maxScroll = max(0, active - MAX_VISIBLE);
    gLogScroll = constrain(gLogScroll, 0, maxScroll);

    static int lastScroll = -1;
    bool scrollChanged = (gLogScroll != lastScroll);
    lastScroll = gLogScroll;
    if (gLogScroll == 0) {
        if (!scrollChanged && g.msgLogWriteIdx == gLogLastWriteIdx) return;
        if (g.msgLogWriteIdx != gLogLastWriteIdx) playTick();
        gLogLastWriteIdx = g.msgLogWriteIdx;
    } else {
        if (!scrollChanged) return;
    }

    tft.setTextFont(2); tft.setTextDatum(ML_DATUM);
    char buf[MSG_TEXT_LEN];
    int drawY = TOP;
    int visIdx = -1;

    for (int i = 0; i < gSlotCount && drawY + LINE_H <= STATUS_BAND.y; i++) {
        if (gSlots[i].count == 0) continue;
        visIdx++;
        if (visIdx < gLogScroll) continue;

        int y = drawY;
        uint16_t rowBg = (i == gNewestSlot) ? TFT_HL_BG : TFT_BLACK;
        int barY = y - LINE_H/2;
        tft.fillRect(0, barY, 320, LINE_H, rowBg);

        snprintf(buf, sizeof(buf), "x%d", gSlots[i].count);
        tft.setTextColor(i == gNewestSlot ? TFT_LOG_SEP_HI : TFT_LOG_SEP);
        tft.drawString(buf, X_CNT, y);

        const char *t = gSlots[i].text;
        const char *c1 = strchr(t, ',');
        const char *c2 = strrchr(t, ',');
        if (c1 && c2 && c2 > c1) {
            int n = c1 - t;
            if (n >= 3 && t[2] == ':')
                { memcpy(buf, t + 3, n - 3); buf[n - 3] = 0; }
            else { memcpy(buf, t, n); buf[n] = 0; }
            tft.setTextColor(i == gNewestSlot ? TFT_LOG_TS_HI : TFT_LOG_TS);
            tft.drawString(buf, X_TS, y);

            n = c2 - c1 - 1;
            memcpy(buf, c1 + 1, n); buf[n] = 0;
            int hxW = tft.textWidth(buf);
            tft.setTextColor(i == gNewestSlot ? TFT_LOG_HX_HI : TFT_LOG_HX);
            if (hxW > HX_MAX_W && drawY + LINE_H * 2 <= STATUS_BAND.y) {
                int fit = 0;
                while (fit < n) {
                    char tmp = buf[fit + 1]; buf[fit + 1] = 0;
                    if (tft.textWidth(buf) > HX_MAX_W) { buf[fit + 1] = tmp; break; }
                    buf[fit + 1] = tmp; fit++;
                }
                char saved = buf[fit]; buf[fit] = 0;
                tft.drawString(buf, X_HX, y);
                buf[fit] = saved;
                int y2 = y + LINE_H;
                int barY2 = y2 - LINE_H/2;
                tft.fillRect(0, barY2, 320, LINE_H, rowBg);
                tft.drawString(buf + fit, X_HX + 8, y2);
                drawY += LINE_H;
            } else {
                tft.drawString(buf, X_HX, y);
            }

            const char *desc = c2 + 1;
            bool isTx = (desc[0] == '>' && desc[1] == ' ');
            tft.setTextColor(i == gNewestSlot ? (isTx ? TFT_LOG_HI_TX : TFT_LOG_HI)
                                             : (isTx ? TFT_LOG_DESC_TX : TFT_LOG_DESC));
            tft.drawString(isTx ? desc + 2 : desc, X_DESC, y);
        }
        drawY += LINE_H;
    }

    if (drawY < STATUS_BAND.y)
        tft.fillRect(0, drawY, 320, STATUS_BAND.y - drawY, TFT_BLACK);
}

static String formatUptime(unsigned long ms) {
    unsigned long secs = ms / 1000;
    unsigned long mins = secs / 60;
    unsigned long hrs = mins / 60;
    char buf[16];
    snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", hrs, mins % 60, secs % 60);
    return String(buf);
}

static void updateDisplay() {
    if (gOtaActive) return;
    if (millis() - g.tftRefresh < 500) return;
    g.tftRefresh = millis();
    if (gLogView) {
        drawLogView();
    } else {
        drawWidgets();
    }
    char buf[48];
    snprintf(buf, sizeof(buf), "%s  %s  %s",
             gLogView ? "LOG" : "HELM",
             (wifiConnected && WiFi.status() == WL_CONNECTED)
                 ? WiFi.localIP().toString().c_str() : AP_SSID,
             formatUptime(millis()).c_str());
    drawBandText(buf, STATUS_BAND, 1, TFT_DARKGREY, gStatusBand);
}


//======================================================================
//8. ASSETS — embedded icons (do not hand-edit)
//======================================================================

static const uint8_t ICON_192[] = {
  0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A, 0x00, 0x00, 0x00, 0x0D,
  0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0xC0,
  0x08, 0x02, 0x00, 0x00, 0x00, 0xDD, 0xBE, 0xFB, 0x50, 0x00, 0x00, 0x04,
  0xF3, 0x49, 0x44, 0x41, 0x54, 0x78, 0x9C, 0xED, 0x92, 0x4B, 0x6E, 0xDD,
  0x40, 0x10, 0x03, 0xBD, 0xE6, 0xFD, 0xEF, 0xEB, 0x20, 0x90, 0xE0, 0xF8,
  0xEF, 0x18, 0xFD, 0x38, 0xEC, 0x99, 0xAE, 0x02, 0xD7, 0x4D, 0x69, 0x58,
  0x4F, 0x02, 0x28, 0xF0, 0x94, 0xFE, 0x00, 0xD8, 0x1B, 0x04, 0x82, 0x12,
  0x08, 0x04, 0x25, 0x10, 0x08, 0x4A, 0x20, 0x10, 0x94, 0x40, 0x20, 0x28,
  0x81, 0x40, 0x50, 0x02, 0x81, 0xA0, 0x04, 0x02, 0x41, 0x09, 0x04, 0x82,
  0x12, 0x08, 0x04, 0x25, 0x10, 0x08, 0x4A, 0x20, 0x10, 0x94, 0x40, 0x20,
  0x28, 0x81, 0x40, 0x50, 0x02, 0x81, 0xA0, 0x04, 0x02, 0x41, 0x09, 0x04,
  0x82, 0x12, 0x08, 0x04, 0x25, 0x10, 0x08, 0x4A, 0x20, 0x10, 0x94, 0x40,
  0x20, 0x28, 0x81, 0x40, 0x50, 0x02, 0x81, 0xA0, 0x04, 0x02, 0x41, 0x09,
  0x04, 0x82, 0x12, 0x08, 0x04, 0x25, 0x10, 0x08, 0x4A, 0x20, 0xD0, 0x7B,
  0x9E, 0x9E, 0x9F, 0xBF, 0x4F, 0xFA, 0x03, 0x7B, 0x31, 0x5A, 0xA0, 0x1F,
  0x5D, 0xF9, 0xFF, 0xA4, 0x7F, 0x25, 0xC6, 0x2C, 0x81, 0x1E, 0x68, 0x0C,
  0x3E, 0x5D, 0x8C, 0x10, 0x68, 0x99, 0x37, 0x03, 0x4D, 0x3A, 0x59, 0xA0,
  0xA0, 0x37, 0x73, 0x4C, 0x3A, 0x53, 0xA0, 0xB8, 0x2E, 0x73, 0x34, 0x3A,
  0x4A, 0xA0, 0xB8, 0x1F, 0x03, 0x4D, 0x3A, 0x44, 0xA0, 0xB8, 0x10, 0x63,
  0x35, 0xDA, 0x5E, 0xA0, 0xB8, 0x04, 0xC3, 0x35, 0xDA, 0x5B, 0xA0, 0xF8,
  0xF6, 0x38, 0xB4, 0xAB, 0x40, 0xF1, 0xC9, 0xD1, 0xE8, 0x62, 0x3F, 0x81,
  0xE2, 0x33, 0xA3, 0xD1, 0x6B, 0x36, 0x13, 0x28, 0xBE, 0x2E, 0x0E, 0xBD,
  0x63, 0x27, 0x81, 0xE2, 0xBB, 0xE2, 0xD0, 0x47, 0xF6, 0x10, 0x28, 0x3E,
  0x27, 0x1A, 0x7D, 0xC5, 0x06, 0x02, 0xC5, 0x57, 0xC4, 0xA1, 0x6F, 0xE8,
  0x2E, 0x50, 0x7C, 0xBF, 0x0E, 0x49, 0x8F, 0xF0, 0x1D, 0xAD, 0x05, 0x8A,
  0x2F, 0xD7, 0x27, 0xE9, 0x29, 0xBE, 0xA4, 0xAF, 0x40, 0xF1, 0xCD, 0xBA,
  0x25, 0x3D, 0xC8, 0xE7, 0x74, 0x14, 0x28, 0x3E, 0x55, 0xE7, 0xA4, 0xC7,
  0x79, 0x4F, 0x3B, 0x81, 0xE2, 0x0B, 0xF5, 0x4F, 0x7A, 0xA2, 0x37, 0x20,
  0xD0, 0x7E, 0x49, 0x4F, 0xF4, 0x86, 0x5E, 0x02, 0xC5, 0xB7, 0xD9, 0x25,
  0xE9, 0xA1, 0xFE, 0xD1, 0x48, 0xA0, 0xF8, 0x2A, 0x7B, 0x25, 0x3D, 0xD7,
  0x4D, 0x17, 0x81, 0xE2, 0x7B, 0xEC, 0x98, 0xF4, 0x68, 0x7F, 0x69, 0x21,
  0x50, 0x7C, 0x89, 0x7D, 0x93, 0x9E, 0xAE, 0x81, 0x40, 0xF1, 0x0D, 0x76,
  0x4F, 0x78, 0xBE, 0x6C, 0xBD, 0x10, 0x08, 0x81, 0x4A, 0xF5, 0xE9, 0xD7,
  0x3F, 0x23, 0xC9, 0x05, 0x93, 0xDD, 0xE9, 0x77, 0x3F, 0x29, 0xB1, 0x11,
  0x63, 0xC5, 0xE9, 0x17, 0x3F, 0x2F, 0x99, 0x1D, 0x23, 0xAD, 0x42, 0x20,
  0x04, 0x2A, 0xB5, 0xA6, 0xDF, 0xFA, 0xD4, 0x04, 0xA6, 0x5C, 0x5F, 0x29,
  0x04, 0x42, 0xA0, 0x52, 0x65, 0xFA, 0x95, 0xCF, 0xCE, 0xEA, 0x35, 0x57,
  0xF7, 0xA5, 0xDF, 0x77, 0x42, 0x96, 0x0E, 0xBA, 0xB2, 0x4C, 0x08, 0x84,
  0x40, 0xA5, 0xB2, 0xF4, 0xCB, 0xCE, 0xC9, 0xBA, 0x4D, 0x97, 0x35, 0x09,
  0x81, 0x10, 0xA8, 0xD4, 0x94, 0x7E, 0xD3, 0x69, 0x59, 0x34, 0xEB, 0x9A,
  0x1A, 0x21, 0x10, 0x02, 0x55, 0x9B, 0xD2, 0x0F, 0x3A, 0x2D, 0x8B, 0x66,
  0x5D, 0x54, 0x93, 0x7E, 0xCD, 0x99, 0x59, 0xB1, 0xEC, 0x82, 0x0E, 0x21,
  0x10, 0x02, 0x95, 0x3A, 0xD2, 0xEF, 0x38, 0x39, 0xF6, 0x71, 0xDD, 0x05,
  0x42, 0x20, 0x04, 0xAA, 0x76, 0xA4, 0x1F, 0x71, 0x72, 0xEC, 0xE3, 0xDA,
  0x0B, 0xD2, 0x2F, 0x48, 0xBC, 0xFB, 0x5A, 0xAF, 0x0B, 0x81, 0x1A, 0xC4,
  0xBB, 0xAF, 0xF5, 0xBA, 0x10, 0xA8, 0x41, 0xBC, 0xFB, 0x7A, 0xAF, 0xA7,
  0xDF, 0x8E, 0x5C, 0x31, 0x4E, 0xEC, 0x3B, 0x2D, 0x04, 0x6A, 0x13, 0xE3,
  0xC4, 0xBE, 0xD3, 0x42, 0xA0, 0x36, 0x31, 0x4E, 0xEC, 0x3B, 0x2D, 0x04,
  0x6A, 0x13, 0xE3, 0xC4, 0xBE, 0xD3, 0x42, 0xA0, 0x36, 0x31, 0x4E, 0x6C,
  0x3C, 0x9D, 0x7E, 0x35, 0xF2, 0x3A, 0xAE, 0x95, 0x4D, 0x77, 0x85, 0x40,
  0xCD, 0xE2, 0x5A, 0xD9, 0x74, 0x57, 0x08, 0xD4, 0x2C, 0xAE, 0x95, 0x4D,
  0x77, 0x85, 0x40, 0xCD, 0xE2, 0x5A, 0xD9, 0x74, 0x57, 0x08, 0xD4, 0x2C,
  0xAE, 0x95, 0x4D, 0x77, 0x85, 0x40, 0xCD, 0xE2, 0x5A, 0xD9, 0x74, 0x57,
  0x08, 0xD4, 0x2C, 0xAE, 0x95, 0x5D, 0x77, 0xD3, 0xEF, 0x45, 0x3E, 0xC6,
  0x32, 0xB4, 0xE3, 0xA8, 0x10, 0xA8, 0x65, 0x2C, 0x43, 0x3B, 0x8E, 0x0A,
  0x81, 0x5A, 0xC6, 0x32, 0xB4, 0xE3, 0xA8, 0x10, 0xA8, 0x65, 0x2C, 0x43,
  0x3B, 0x8E, 0x0A, 0x81, 0x5A, 0xC6, 0x32, 0xB4, 0xE3, 0xA8, 0x10, 0xA8,
  0x65, 0x2C, 0x43, 0x3B, 0x8E, 0x0A, 0x81, 0x5A, 0xC6, 0x32, 0xB4, 0xE3,
  0xA8, 0x10, 0xA8, 0x65, 0x2C, 0x43, 0x3B, 0x8E, 0x0A, 0x81, 0x5A, 0xC6,
  0x32, 0xB4, 0xE3, 0xA8, 0x10, 0xA8, 0x65, 0x2C, 0x43, 0x3B, 0x8E, 0x0A,
  0x81, 0x5A, 0xC6, 0x32, 0xB4, 0xE3, 0xE8, 0x7D, 0x3A, 0xFD, 0x5E, 0xE4,
  0x75, 0x5C, 0x2B, 0x9B, 0xEE, 0x0A, 0x81, 0x9A, 0xC5, 0xB5, 0xB2, 0xE9,
  0xAE, 0x10, 0xA8, 0x59, 0x5C, 0x2B, 0x9B, 0xEE, 0x0A, 0x81, 0x9A, 0xC5,
  0xB5, 0xB2, 0xE9, 0xAE, 0x10, 0xA8, 0x59, 0x5C, 0x2B, 0x9B, 0xEE, 0x0A,
  0x81, 0x9A, 0xC5, 0xB5, 0xB2, 0xE9, 0xAE, 0x10, 0xA8, 0x59, 0x5C, 0x2B,
  0x9B, 0xEE, 0xDE, 0xD7, 0xD3, 0xAF, 0x46, 0xAE, 0x18, 0x27, 0xF6, 0x9D,
  0x16, 0x02, 0xB5, 0x89, 0x71, 0x62, 0xDF, 0x69, 0x21, 0x50, 0x9B, 0x18,
  0x27, 0xF6, 0x9D, 0x16, 0x02, 0xB5, 0x89, 0x71, 0x62, 0xDF, 0x69, 0x21,
  0x50, 0x9B, 0x18, 0x27, 0xF6, 0x9D, 0xBE, 0x0B, 0xD2, 0x6F, 0x47, 0xBC,
  0xFB, 0x5A, 0xAF, 0x0B, 0x81, 0x1A, 0xC4, 0xBB, 0xAF, 0xF5, 0xBA, 0x10,
  0xA8, 0x41, 0xBC, 0xFB, 0x5A, 0xAF, 0xDF, 0x1D, 0xE9, 0x17, 0x9C, 0x1C,
  0xFB, 0xB8, 0xEE, 0x02, 0x21, 0x10, 0x02, 0x55, 0x3B, 0xD2, 0x8F, 0x38,
  0x39, 0xF6, 0x71, 0xDD, 0x05, 0x77, 0x4D, 0xFA, 0x1D, 0x67, 0x66, 0xC5,
  0xB2, 0x0B, 0x3A, 0x84, 0x40, 0x08, 0xF4, 0x80, 0xA6, 0xF4, 0x6B, 0x4E,
  0xCB, 0xA2, 0x59, 0xD7, 0xD4, 0x08, 0x81, 0x10, 0xA8, 0xDA, 0x94, 0x7E,
  0xD0, 0x69, 0x59, 0x34, 0xEB, 0x9A, 0x9A, 0xBB, 0x2C, 0xFD, 0xA6, 0x73,
  0xB2, 0x6E, 0xD3, 0x65, 0x4D, 0x42, 0x20, 0x04, 0x7A, 0x40, 0x5F, 0xFA,
  0x65, 0x27, 0x64, 0xE9, 0xA0, 0x2B, 0xCB, 0x84, 0x40, 0x08, 0xF4, 0x80,
  0xCA, 0xF4, 0xFB, 0x9E, 0x9D, 0xD5, 0x6B, 0x2E, 0xEE, 0xBB, 0x5B, 0xD3,
  0xAF, 0x7C, 0x6A, 0x02, 0x53, 0xAE, 0xAF, 0x14, 0x02, 0x21, 0xD0, 0x03,
  0x8A, 0xD3, 0x6F, 0x7D, 0x5E, 0x32, 0x3B, 0x46, 0x5A, 0x85, 0x40, 0x08,
  0xF4, 0x80, 0xEE, 0xF4, 0x8B, 0x9F, 0x94, 0xD8, 0x88, 0xA9, 0xE2, 0xBB,
  0x3E, 0xFD, 0xEE, 0x67, 0x24, 0xB9, 0x60, 0xB0, 0xFB, 0xFE, 0x82, 0xF4,
  0xEB, 0xEF, 0x9E, 0xF0, 0x7C, 0xD9, 0x7A, 0x21, 0x10, 0x02, 0xD5, 0x89,
  0x6F, 0xB0, 0x6F, 0xD2, 0xD3, 0xF5, 0x10, 0x48, 0x38, 0xB4, 0xA7, 0x3D,
  0xEA, 0x23, 0x90, 0x70, 0x68, 0x43, 0x7B, 0xD4, 0x4A, 0x20, 0xE1, 0xD0,
  0x6E, 0xF6, 0xA8, 0x9B, 0x40, 0xC2, 0xA1, 0xAD, 0xEC, 0x11, 0x02, 0xED,
  0x98, 0xF4, 0x44, 0x6F, 0x68, 0x27, 0x90, 0x70, 0x68, 0x1F, 0x7B, 0xD4,
  0x53, 0xA0, 0x8B, 0xF8, 0x54, 0xDD, 0x92, 0x1E, 0xE4, 0x73, 0xFA, 0x0A,
  0x24, 0x1C, 0x6A, 0x6F, 0x8F, 0x9A, 0x0B, 0x24, 0x1C, 0xEA, 0x6D, 0x8F,
  0xFA, 0x0B, 0xA4, 0xF1, 0x0E, 0xA5, 0x9F, 0xFF, 0x07, 0x36, 0x10, 0x48,
  0x83, 0x1D, 0x4A, 0x3F, 0xFC, 0xCF, 0xEC, 0x21, 0xD0, 0x45, 0x7C, 0x4E,
  0xD4, 0xF9, 0xC8, 0x4E, 0x02, 0x69, 0x8C, 0x43, 0xE9, 0x67, 0xFE, 0x05,
  0x9B, 0x09, 0xA4, 0x01, 0x0E, 0xA5, 0x1F, 0xF8, 0x77, 0xEC, 0x27, 0xD0,
  0x45, 0x7C, 0x66, 0xD4, 0xB9, 0xD8, 0x55, 0xA0, 0x8B, 0xF8, 0xE4, 0x93,
  0xD5, 0xB9, 0xD8, 0x5B, 0x20, 0x1D, 0xE1, 0x50, 0xFA, 0x09, 0x4B, 0x6C,
  0x2F, 0xD0, 0x45, 0x5C, 0x82, 0x81, 0xEA, 0x5C, 0x1C, 0x22, 0xD0, 0x45,
  0x5C, 0x88, 0x51, 0xEA, 0x5C, 0x1C, 0x25, 0xD0, 0x0B, 0x71, 0x3F, 0x8E,
  0xF7, 0xE6, 0x85, 0x33, 0x05, 0xBA, 0x88, 0xEB, 0x72, 0xB6, 0x3A, 0x17,
  0x27, 0x0B, 0xF4, 0x02, 0xDE, 0xF8, 0x18, 0x21, 0xD0, 0x0B, 0x78, 0xF3,
  0x70, 0x66, 0x09, 0xF4, 0x0E, 0x8C, 0xA9, 0x33, 0x5A, 0xA0, 0x4F, 0xC1,
  0x95, 0x5F, 0x81, 0x40, 0x50, 0x02, 0x81, 0xA0, 0x04, 0x02, 0x41, 0x09,
  0x04, 0x82, 0x12, 0x08, 0x04, 0x25, 0x10, 0x08, 0x4A, 0x20, 0x10, 0x94,
  0x40, 0x20, 0x28, 0x81, 0x40, 0x50, 0x02, 0x81, 0xA0, 0x04, 0x02, 0x41,
  0x09, 0x04, 0x82, 0x12, 0x08, 0x04, 0x25, 0x10, 0x08, 0x4A, 0x20, 0x10,
  0x94, 0x40, 0x20, 0x28, 0x81, 0x40, 0x50, 0x02, 0x81, 0xA0, 0x04, 0x02,
  0x41, 0x09, 0x04, 0x82, 0x12, 0x08, 0x04, 0x25, 0x10, 0x08, 0x4A, 0x20,
  0x10, 0x94, 0x40, 0x20, 0x28, 0x81, 0x40, 0x50, 0x02, 0x81, 0xA0, 0x04,
  0x02, 0x41, 0x09, 0x04, 0x82, 0x12, 0x7F, 0x00, 0xF8, 0x70, 0xDA, 0xE9,
  0x7C, 0xDC, 0xDE, 0x6B, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4E, 0x44,
  0xAE, 0x42, 0x60, 0x82,
};

// 512x512 PNG, 4452 bytes — dark bg + cyan circle
static const uint8_t ICON_512[] = {
  0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A, 0x00, 0x00, 0x00, 0x0D,
  0x49, 0x48, 0x44, 0x52, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00,
  0x08, 0x02, 0x00, 0x00, 0x00, 0x7B, 0x1A, 0x43, 0xAD, 0x00, 0x00, 0x11,
  0x2B, 0x49, 0x44, 0x41, 0x54, 0x78, 0x9C, 0xED, 0xD5, 0x51, 0x4E, 0x23,
  0xC9, 0x16, 0x45, 0xD1, 0xFE, 0xAE, 0xF9, 0xCF, 0xF7, 0x3D, 0x95, 0x1A,
  0x89, 0xC6, 0x65, 0x0A, 0x83, 0x9D, 0xB1, 0x23, 0xF3, 0xAE, 0xA5, 0x3D,
  0x00, 0x90, 0x33, 0xEE, 0xF9, 0xE7, 0x17, 0x00, 0x23, 0xFD, 0x53, 0xFF,
  0x01, 0x00, 0x34, 0x0C, 0x00, 0xC0, 0x50, 0x06, 0x00, 0x60, 0x28, 0x03,
  0x00, 0x30, 0x94, 0x01, 0x00, 0x18, 0xCA, 0x00, 0x00, 0x0C, 0x65, 0x00,
  0x00, 0x86, 0x32, 0x00, 0x00, 0x43, 0x19, 0x00, 0x80, 0xA1, 0x0C, 0x00,
  0xC0, 0x50, 0x06, 0x00, 0x60, 0x28, 0x03, 0x00, 0x30, 0x94, 0x01, 0x00,
  0x18, 0xCA, 0x00, 0x00, 0x0C, 0x65, 0x00, 0x00, 0x86, 0x32, 0x00, 0x00,
  0x43, 0x19, 0x00, 0x80, 0xA1, 0x0C, 0x00, 0xC0, 0x50, 0x06, 0x00, 0x60,
  0x28, 0x03, 0x00, 0x30, 0x94, 0x01, 0x00, 0x18, 0xCA, 0x00, 0x00, 0x0C,
  0x65, 0x00, 0x00, 0x86, 0x32, 0x00, 0x00, 0x43, 0x19, 0x00, 0x80, 0xA1,
  0x0C, 0x00, 0xC0, 0x50, 0x06, 0x00, 0x60, 0x28, 0x03, 0x00, 0x30, 0x94,
  0x01, 0x00, 0x18, 0xCA, 0x00, 0x00, 0x0C, 0x65, 0x00, 0x00, 0x86, 0x32,
  0x00, 0x00, 0x43, 0x19, 0x00, 0x80, 0xA1, 0x0C, 0x00, 0xC0, 0x50, 0x06,
  0x00, 0x60, 0x28, 0x03, 0x00, 0x30, 0x94, 0x01, 0x00, 0x18, 0xCA, 0x00,
  0x00, 0x0C, 0x65, 0x00, 0x00, 0x86, 0x32, 0x00, 0x00, 0x43, 0x19, 0x00,
  0x80, 0xA1, 0x0C, 0x00, 0xC0, 0x50, 0x06, 0x00, 0x60, 0x28, 0x03, 0x00,
  0x30, 0x94, 0x01, 0x00, 0x18, 0xCA, 0x00, 0x00, 0x0C, 0x65, 0x00, 0x00,
  0x86, 0x32, 0x00, 0x00, 0x43, 0x19, 0x00, 0x80, 0xA1, 0x0C, 0x00, 0xC0,
  0x50, 0x06, 0x00, 0x60, 0x28, 0x03, 0x00, 0x30, 0x94, 0x01, 0x00, 0x18,
  0xCA, 0x00, 0x00, 0x0C, 0x65, 0x00, 0x00, 0x86, 0x32, 0x00, 0x00, 0x43,
  0x19, 0x00, 0x80, 0xA1, 0x0C, 0x00, 0xC0, 0x50, 0x06, 0x00, 0x60, 0x28,
  0x03, 0x00, 0x30, 0x94, 0x01, 0x00, 0x18, 0xCA, 0x00, 0x00, 0x0C, 0x65,
  0x00, 0x00, 0x86, 0x32, 0x00, 0x00, 0x43, 0x19, 0x00, 0x80, 0xA1, 0x0C,
  0x00, 0xC0, 0x50, 0x06, 0x00, 0x60, 0x28, 0x03, 0x00, 0x30, 0x94, 0x01,
  0x00, 0x18, 0xCA, 0x00, 0x00, 0x0C, 0x65, 0x00, 0x00, 0x86, 0x32, 0x00,
  0x00, 0x43, 0x19, 0x00, 0x80, 0xA1, 0x0C, 0x00, 0xC0, 0x50, 0x06, 0x00,
  0x60, 0x28, 0x03, 0x00, 0x30, 0x94, 0x01, 0x60, 0x84, 0x7F, 0xFE, 0xF7,
  0xBF, 0xEF, 0x56, 0xFF, 0xC9, 0x70, 0x38, 0x03, 0xC0, 0xE9, 0xFD, 0xE0,
  0xB8, 0xBF, 0xAA, 0xFA, 0x5F, 0x87, 0xA7, 0x18, 0x00, 0xCE, 0x24, 0xBC,
  0xF5, 0x56, 0x81, 0xEB, 0x31, 0x00, 0xEC, 0x2B, 0x3F, 0xE5, 0x26, 0x81,
  0x6B, 0x33, 0x00, 0x6C, 0x24, 0xBF, 0xD4, 0xF6, 0x80, 0x51, 0x0C, 0x00,
  0xB1, 0xFC, 0x1C, 0x1B, 0x03, 0xC6, 0x32, 0x00, 0x04, 0xF2, 0xB3, 0xBB,
  0x67, 0xF5, 0xCF, 0xC2, 0x38, 0x06, 0x80, 0x45, 0xF2, 0xF3, 0x7A, 0xAE,
  0xEA, 0x9F, 0x8B, 0x11, 0x0C, 0x00, 0xC7, 0xCA, 0x2F, 0xE9, 0xD9, 0xAB,
  0x7F, 0x40, 0xAE, 0xCC, 0x00, 0x70, 0x88, 0xFC, 0x6E, 0x5E, 0xAF, 0xFA,
  0x27, 0xE5, 0x82, 0x0C, 0x00, 0xAF, 0x94, 0x5F, 0xC9, 0x09, 0xD5, 0x3F,
  0x32, 0xD7, 0x61, 0x00, 0x78, 0x81, 0xFC, 0x26, 0xCE, 0xAC, 0xFE, 0xD9,
  0x39, 0x3D, 0x03, 0xC0, 0x53, 0xF2, 0x23, 0xA8, 0xFA, 0x13, 0xE0, 0xC4,
  0x0C, 0x00, 0x3F, 0x91, 0x5F, 0x3D, 0xFD, 0x59, 0xFD, 0x51, 0x70, 0x3E,
  0x06, 0x80, 0xEF, 0xC9, 0xCF, 0x9C, 0xFE, 0x5E, 0xFD, 0x81, 0x70, 0x26,
  0x06, 0x80, 0x87, 0xE4, 0x77, 0x4D, 0xDF, 0xAD, 0xFE, 0x64, 0x38, 0x01,
  0x03, 0xC0, 0x17, 0xF2, 0x43, 0xA6, 0x67, 0xAA, 0x3F, 0x1F, 0xB6, 0x66,
  0x00, 0xF8, 0x54, 0x7E, 0xBC, 0xF4, 0xAA, 0xEA, 0x4F, 0x89, 0x4D, 0x19,
  0x00, 0xEE, 0xC8, 0x0F, 0x96, 0x8E, 0xA8, 0xFE, 0xAC, 0xD8, 0x8E, 0x01,
  0xE0, 0x83, 0xFC, 0x48, 0xE9, 0xE8, 0xEA, 0x4F, 0x8C, 0x8D, 0x18, 0x00,
  0xDE, 0xE4, 0x87, 0x49, 0x2B, 0xAB, 0x3F, 0x37, 0xB6, 0x60, 0x00, 0x70,
  0xFA, 0xE7, 0x56, 0x7F, 0x7A, 0xC4, 0x0C, 0xC0, 0x68, 0xF9, 0x01, 0xD2,
  0x0E, 0xD5, 0x9F, 0x21, 0x19, 0x03, 0x30, 0x54, 0x7E, 0x74, 0xB4, 0x5B,
  0xF5, 0x27, 0x49, 0xC0, 0x00, 0x4C, 0x94, 0xDF, 0x1A, 0xED, 0x59, 0xFD,
  0x61, 0xB2, 0x9A, 0x01, 0x98, 0x25, 0x3F, 0x31, 0xDA, 0xBF, 0xFA, 0x23,
  0x65, 0x1D, 0x03, 0x30, 0x45, 0x7E, 0x56, 0x74, 0xAE, 0xEA, 0x0F, 0x96,
  0x15, 0x0C, 0xC0, 0x08, 0xF9, 0x35, 0xD1, 0x19, 0xAB, 0x3F, 0x5B, 0x0E,
  0x67, 0x00, 0x2E, 0x2E, 0x3F, 0x22, 0x3A, 0x7B, 0xF5, 0x27, 0xCC, 0x81,
  0x0C, 0xC0, 0x65, 0xE5, 0x87, 0x43, 0x57, 0xAA, 0xFE, 0x9C, 0x39, 0x84,
  0x01, 0xB8, 0xA6, 0xFC, 0x5E, 0xE8, 0x7A, 0xD5, 0x1F, 0x35, 0xAF, 0x67,
  0x00, 0xAE, 0x26, 0x3F, 0x13, 0xBA, 0x76, 0xF5, 0x07, 0xCE, 0x2B, 0x19,
  0x80, 0x4B, 0xC9, 0xAF, 0x83, 0x26, 0x54, 0x7F, 0xE6, 0xBC, 0x8C, 0x01,
  0xB8, 0x8E, 0xFC, 0x2E, 0x68, 0x4E, 0xF5, 0xC7, 0xCE, 0x6B, 0x18, 0x80,
  0x2B, 0xC8, 0xCF, 0x81, 0x66, 0x56, 0x7F, 0xF8, 0x3C, 0xCB, 0x00, 0x9C,
  0x5E, 0x7E, 0x05, 0x34, 0xB9, 0xFA, 0xF3, 0xE7, 0x29, 0x06, 0xE0, 0xDC,
  0xF2, 0xF7, 0x2F, 0xD5, 0x8F, 0x80, 0x9F, 0x33, 0x00, 0x67, 0x95, 0x3F,
  0x7B, 0xE9, 0xBF, 0xD5, 0x0F, 0x82, 0x9F, 0x30, 0x00, 0xA7, 0x94, 0xBF,
  0x76, 0xE9, 0xCF, 0xEA, 0x67, 0xC1, 0xB7, 0x19, 0x80, 0xF3, 0xC9, 0xDF,
  0xB9, 0xF4, 0x59, 0xF5, 0xE3, 0xE0, 0x7B, 0x0C, 0xC0, 0x99, 0xE4, 0xCF,
  0x5B, 0x7A, 0xA4, 0xFA, 0xA1, 0xF0, 0x28, 0x03, 0x70, 0x1A, 0xF9, 0xAB,
  0x96, 0x1E, 0xAF, 0x7E, 0x2E, 0x3C, 0xC4, 0x00, 0x9C, 0x43, 0xFE, 0x9E,
  0xA5, 0xEF, 0x56, 0x3F, 0x1A, 0xBE, 0x66, 0x00, 0x4E, 0x20, 0x7F, 0xC9,
  0xD2, 0xCF, 0xAA, 0x9F, 0x0E, 0x5F, 0x30, 0x00, 0xBB, 0xCB, 0xDF, 0xB0,
  0xF4, 0x4C, 0xF5, 0x03, 0xE2, 0x6F, 0x0C, 0xC0, 0xD6, 0xF2, 0xD7, 0x2B,
  0x3D, 0x5F, 0xFD, 0x8C, 0xF8, 0x94, 0x01, 0xD8, 0x54, 0xFE, 0x68, 0xA5,
  0xD7, 0x56, 0x3F, 0x29, 0xEE, 0x30, 0x00, 0x3B, 0xCA, 0xDF, 0xAA, 0x74,
  0x44, 0xF5, 0xC3, 0xE2, 0x96, 0x01, 0xD8, 0x4E, 0xFE, 0x4A, 0xA5, 0xE3,
  0xAA, 0x9F, 0x17, 0x1F, 0x18, 0x80, 0xBD, 0xE4, 0xEF, 0x53, 0x3A, 0xBA,
  0xFA, 0x91, 0xF1, 0xCE, 0x00, 0x6C, 0x24, 0x7F, 0x99, 0xD2, 0x9A, 0xEA,
  0xA7, 0xC6, 0x1B, 0x03, 0xB0, 0x8B, 0xFC, 0x4D, 0x4A, 0x2B, 0xAB, 0x1F,
  0x1C, 0xBF, 0x19, 0x80, 0x2D, 0xE4, 0xAF, 0x51, 0x5A, 0x5F, 0xFD, 0xEC,
  0x30, 0x00, 0x1B, 0xC8, 0xDF, 0xA1, 0x54, 0x55, 0x3F, 0xBE, 0xE9, 0x0C,
  0x40, 0x2C, 0x7F, 0x81, 0x52, 0x5B, 0xFD, 0x04, 0x47, 0x33, 0x00, 0xA5,
  0xFC, 0xED, 0x49, 0x3B, 0x54, 0x3F, 0xC4, 0xB9, 0x0C, 0x40, 0x26, 0x7F,
  0x75, 0xD2, 0x3E, 0xD5, 0xCF, 0x71, 0x28, 0x03, 0xD0, 0xC8, 0xDF, 0x9B,
  0xB4, 0x5B, 0xF5, 0xA3, 0x9C, 0xC8, 0x00, 0x04, 0xF2, 0x97, 0x26, 0xED,
  0x59, 0xFD, 0x34, 0xC7, 0x31, 0x00, 0xAB, 0xE5, 0x6F, 0x4C, 0xDA, 0xB9,
  0xFA, 0x81, 0xCE, 0x62, 0x00, 0x56, 0xCB, 0x1F, 0x98, 0xB4, 0x73, 0xF5,
  0x03, 0x9D, 0xC5, 0x00, 0x2C, 0x95, 0xBF, 0x2E, 0x69, 0xFF, 0xEA, 0x67,
  0x3A, 0x88, 0x01, 0x58, 0x27, 0x7F, 0x57, 0xD2, 0x59, 0xAA, 0x1F, 0xEB,
  0x14, 0x06, 0x60, 0x91, 0xFC, 0x45, 0x49, 0xE7, 0xAA, 0x7E, 0xB2, 0x23,
  0x18, 0x80, 0x15, 0xF2, 0xB7, 0x24, 0x9D, 0xB1, 0xFA, 0xE1, 0x5E, 0x9F,
  0x01, 0x38, 0x5C, 0xFE, 0x8A, 0xA4, 0xF3, 0x56, 0x3F, 0xDF, 0x8B, 0x33,
  0x00, 0xC7, 0xCA, 0xDF, 0x8F, 0x74, 0xF6, 0xEA, 0x47, 0x7C, 0x65, 0x06,
  0xE0, 0x58, 0xF9, 0xE3, 0x91, 0xCE, 0x5E, 0xFD, 0x88, 0xAF, 0xCC, 0x00,
  0x1C, 0x28, 0x7F, 0x39, 0xD2, 0x35, 0xAA, 0x9F, 0xF2, 0x65, 0x19, 0x80,
  0xA3, 0xE4, 0x6F, 0x46, 0xBA, 0x52, 0xF5, 0x83, 0xBE, 0x26, 0x03, 0x70,
  0x88, 0xFC, 0xB5, 0x48, 0xD7, 0xAB, 0x7E, 0xD6, 0x17, 0x64, 0x00, 0x0E,
  0x91, 0x3F, 0x15, 0xE9, 0x7A, 0xD5, 0xCF, 0xFA, 0x82, 0x0C, 0xC0, 0xEB,
  0xE5, 0xEF, 0x44, 0xBA, 0x6A, 0xF5, 0xE3, 0xBE, 0x1A, 0x03, 0xF0, 0x62,
  0xF9, 0x0B, 0x91, 0xAE, 0x5D, 0xFD, 0xC4, 0x2F, 0xC5, 0x00, 0xBC, 0x52,
  0xFE, 0x36, 0xA4, 0x09, 0xD5, 0x0F, 0xFD, 0x3A, 0x0C, 0xC0, 0x2B, 0xE5,
  0x0F, 0x43, 0x9A, 0x50, 0xFD, 0xD0, 0xAF, 0xC3, 0x00, 0xBC, 0x4C, 0xFE,
  0x2A, 0xA4, 0x39, 0xD5, 0xCF, 0xFD, 0x22, 0x0C, 0xC0, 0x6B, 0xE4, 0xEF,
  0x41, 0x9A, 0x56, 0xFD, 0xE8, 0xAF, 0xC0, 0x00, 0xBC, 0x40, 0xFE, 0x12,
  0xA4, 0x99, 0xD5, 0x4F, 0xFF, 0xF4, 0x0C, 0xC0, 0x0B, 0xE4, 0xCF, 0x40,
  0x9A, 0x59, 0xFD, 0xF4, 0x4F, 0xCF, 0x00, 0x3C, 0x2B, 0x7F, 0x03, 0xD2,
  0xE4, 0xEA, 0x03, 0x70, 0x6E, 0x06, 0xE0, 0x29, 0xF9, 0xD7, 0x2F, 0xA9,
  0x3E, 0x03, 0x27, 0x66, 0x00, 0x9E, 0x92, 0x7F, 0xFA, 0x92, 0xEA, 0x33,
  0x70, 0x62, 0x06, 0xE0, 0xE7, 0xF2, 0xEF, 0x5E, 0xD2, 0xBF, 0xD5, 0xC7,
  0xE0, 0xAC, 0x0C, 0xC0, 0xCF, 0xE5, 0x1F, 0xBD, 0xA4, 0x7F, 0xAB, 0x8F,
  0xC1, 0x59, 0x19, 0x80, 0x1F, 0xCA, 0xBF, 0x78, 0x49, 0xFF, 0xAD, 0x3E,
  0x09, 0xA7, 0x64, 0x00, 0x7E, 0x22, 0xFF, 0xD6, 0x25, 0xFD, 0x59, 0x7D,
  0x18, 0xCE, 0xC7, 0x00, 0xFC, 0x44, 0xFE, 0xA1, 0x4B, 0xFA, 0xB3, 0xFA,
  0x30, 0x9C, 0x8F, 0x01, 0xF8, 0xB6, 0xFC, 0x2B, 0x97, 0xF4, 0x59, 0xF5,
  0x79, 0x38, 0x19, 0x03, 0xF0, 0x6D, 0xF9, 0x27, 0x2E, 0xE9, 0xB3, 0xEA,
  0xF3, 0x70, 0x32, 0x06, 0xE0, 0x7B, 0xF2, 0xEF, 0x5B, 0xD2, 0xDF, 0xAB,
  0x8F, 0xC4, 0x99, 0x18, 0x80, 0x6F, 0xC8, 0xBF, 0x6C, 0x49, 0x8F, 0x54,
  0x9F, 0x8A, 0xD3, 0x30, 0x00, 0xDF, 0x90, 0x7F, 0xD6, 0x92, 0x1E, 0xA9,
  0x3E, 0x15, 0xA7, 0x61, 0x00, 0x1E, 0x95, 0x7F, 0xD3, 0x92, 0x1E, 0xAF,
  0x3E, 0x18, 0xE7, 0x60, 0x00, 0x1E, 0x95, 0x7F, 0xD0, 0x92, 0x1E, 0xAF,
  0x3E, 0x18, 0xE7, 0x60, 0x00, 0x1E, 0x92, 0x7F, 0xCD, 0x92, 0xBE, 0x5B,
  0x7D, 0x36, 0x4E, 0xC0, 0x00, 0x3C, 0x24, 0xFF, 0x94, 0x25, 0x7D, 0xB7,
  0xFA, 0x6C, 0x9C, 0x80, 0x01, 0xF8, 0x5A, 0xFE, 0x1D, 0x4B, 0xFA, 0x59,
  0xF5, 0xF1, 0xD8, 0x9D, 0x01, 0xF8, 0x5A, 0xFE, 0x11, 0x4B, 0xFA, 0x59,
  0xF5, 0xF1, 0xD8, 0x9D, 0x01, 0xF8, 0x42, 0xFE, 0x05, 0x4B, 0x7A, 0xA6,
  0xFA, 0x84, 0x6C, 0xCD, 0x00, 0x7C, 0x21, 0xFF, 0x7C, 0x25, 0x3D, 0x53,
  0x7D, 0x42, 0xB6, 0x66, 0x00, 0xFE, 0x26, 0xFF, 0x76, 0x25, 0x3D, 0x5F,
  0x7D, 0x48, 0xF6, 0x65, 0x00, 0xFE, 0x26, 0xFF, 0x70, 0x25, 0x3D, 0x5F,
  0x7D, 0x48, 0xF6, 0x65, 0x00, 0x3E, 0x95, 0x7F, 0xB5, 0x92, 0x5E, 0x55,
  0x7D, 0x4E, 0x36, 0x65, 0x00, 0x3E, 0x95, 0x7F, 0xB2, 0x92, 0x5E, 0x55,
  0x7D, 0x4E, 0x36, 0x65, 0x00, 0xEE, 0xCB, 0xBF, 0x57, 0x49, 0xAF, 0xAD,
  0x3E, 0x2A, 0x3B, 0x32, 0x00, 0xF7, 0xE5, 0x1F, 0xAB, 0xA4, 0xD7, 0x56,
  0x1F, 0x95, 0x1D, 0x19, 0x80, 0x3B, 0xF2, 0x2F, 0x55, 0xD2, 0x11, 0xD5,
  0xA7, 0x65, 0x3B, 0x06, 0xE0, 0x8E, 0xFC, 0x33, 0x95, 0x74, 0x44, 0xF5,
  0x69, 0xD9, 0x8E, 0x01, 0xB8, 0x23, 0xFF, 0x4C, 0x25, 0x1D, 0x51, 0x7D,
  0x5A, 0xB6, 0x63, 0x00, 0x6E, 0xE5, 0xDF, 0xA8, 0xA4, 0xE3, 0xAA, 0x0F,
  0xCC, 0x5E, 0x0C, 0xC0, 0xAD, 0xFC, 0x03, 0x95, 0x74, 0x5C, 0xF5, 0x81,
  0xD9, 0x8B, 0x01, 0xF8, 0x20, 0xFF, 0x3A, 0x25, 0x1D, 0x5D, 0x7D, 0x66,
  0x36, 0x62, 0x00, 0x3E, 0xC8, 0x3F, 0x4D, 0x49, 0x47, 0x57, 0x9F, 0x99,
  0x8D, 0x18, 0x80, 0x0F, 0xF2, 0x4F, 0x53, 0xD2, 0xD1, 0xD5, 0x67, 0x66,
  0x23, 0x06, 0xE0, 0x5D, 0xFE, 0x5D, 0x4A, 0x5A, 0x53, 0x7D, 0x6C, 0x76,
  0x61, 0x00, 0xDE, 0xE5, 0x1F, 0xA5, 0xA4, 0x35, 0xD5, 0xC7, 0x66, 0x17,
  0x06, 0xE0, 0x4D, 0xFE, 0x45, 0x4A, 0x5A, 0x59, 0x7D, 0x72, 0xB6, 0x60,
  0x00, 0xDE, 0xE4, 0x9F, 0xA3, 0xA4, 0x95, 0xD5, 0x27, 0x67, 0x0B, 0x06,
  0xE0, 0x4D, 0xFE, 0x39, 0x4A, 0x5A, 0x59, 0x7D, 0x72, 0xB6, 0x60, 0x00,
  0x7E, 0xCB, 0xBF, 0x45, 0x49, 0xEB, 0xAB, 0x0F, 0x4F, 0xCF, 0x00, 0xFC,
  0x96, 0x7F, 0x88, 0x92, 0xD6, 0x57, 0x1F, 0x9E, 0x9E, 0x01, 0xF8, 0x2D,
  0xFF, 0x10, 0x25, 0xAD, 0xAF, 0x3E, 0x3C, 0x3D, 0x03, 0xE0, 0xFA, 0x4B,
  0x73, 0xAB, 0xCF, 0x4F, 0xCC, 0x00, 0x18, 0x00, 0x69, 0x6E, 0xF5, 0xF9,
  0x89, 0x19, 0x00, 0x03, 0x20, 0xCD, 0xAD, 0x3E, 0x3F, 0xB1, 0xE9, 0x03,
  0x90, 0x7F, 0x7F, 0x92, 0xDA, 0xEA, 0x23, 0x54, 0x32, 0x00, 0xFD, 0xF7,
  0x27, 0x29, 0xAC, 0x3E, 0x42, 0x25, 0x03, 0xD0, 0x7F, 0x7F, 0x92, 0xC2,
  0xEA, 0x23, 0x54, 0x1A, 0x3D, 0x00, 0xF9, 0x97, 0x27, 0x69, 0x87, 0xEA,
  0x53, 0x94, 0x31, 0x00, 0x92, 0xA6, 0x57, 0x9F, 0xA2, 0x8C, 0x01, 0x90,
  0x34, 0xBD, 0xFA, 0x14, 0x65, 0x0C, 0x80, 0xA4, 0xE9, 0xD5, 0xA7, 0x28,
  0x33, 0x77, 0x00, 0xF2, 0x6F, 0x4E, 0xD2, 0x3E, 0xD5, 0x07, 0xA9, 0x61,
  0x00, 0x24, 0xC9, 0x00, 0x0C, 0x93, 0x7F, 0x70, 0x92, 0xF6, 0xA9, 0x3E,
  0x48, 0x0D, 0x03, 0x20, 0x49, 0x06, 0x60, 0x92, 0xFC, 0x6B, 0x93, 0xB4,
  0x5B, 0xF5, 0x59, 0x0A, 0x18, 0x00, 0x49, 0xFA, 0x5D, 0x7D, 0x96, 0x02,
  0x06, 0x40, 0x92, 0x7E, 0x57, 0x9F, 0xA5, 0x80, 0x01, 0x90, 0xA4, 0xDF,
  0xD5, 0x67, 0x29, 0x30, 0x71, 0x00, 0xF2, 0xEF, 0x4C, 0xD2, 0x9E, 0xD5,
  0xC7, 0x69, 0x35, 0x03, 0x20, 0x49, 0x6F, 0xD5, 0xC7, 0x69, 0x35, 0x03,
  0x20, 0x49, 0x6F, 0xD5, 0xC7, 0x69, 0x35, 0x03, 0x20, 0x49, 0x6F, 0xD5,
  0xC7, 0x69, 0x35, 0x03, 0x20, 0x49, 0x6F, 0xD5, 0xC7, 0x69, 0xB5, 0x71,
  0x03, 0x90, 0x7F, 0x61, 0x92, 0x76, 0xAE, 0x3E, 0x51, 0x4B, 0x19, 0x00,
  0x49, 0x7A, 0xAF, 0x3E, 0x51, 0x4B, 0x19, 0x00, 0x49, 0x7A, 0xAF, 0x3E,
  0x51, 0x4B, 0x19, 0x00, 0x49, 0x7A, 0xAF, 0x3E, 0x51, 0x4B, 0x19, 0x00,
  0x49, 0x7A, 0xAF, 0x3E, 0x51, 0x4B, 0x19, 0x00, 0x49, 0x7A, 0xAF, 0x3E,
  0x51, 0x4B, 0xCD, 0x1A, 0x80, 0xFC, 0xDB, 0x92, 0xB4, 0x7F, 0xF5, 0xA1,
  0x5A, 0xC7, 0x00, 0x48, 0xD2, 0x87, 0xEA, 0x43, 0xB5, 0x8E, 0x01, 0x90,
  0xA4, 0x0F, 0xD5, 0x87, 0x6A, 0x1D, 0x03, 0x20, 0x49, 0x1F, 0xAA, 0x0F,
  0xD5, 0x3A, 0x06, 0x40, 0x92, 0x3E, 0x54, 0x1F, 0xAA, 0x75, 0x0C, 0x80,
  0x24, 0x7D, 0xA8, 0x3E, 0x54, 0xEB, 0x18, 0x00, 0x49, 0xFA, 0x50, 0x7D,
  0xA8, 0xD6, 0x19, 0x34, 0x00, 0xF9, 0x57, 0x25, 0xE9, 0x2C, 0xD5, 0xE7,
  0x6A, 0x11, 0x03, 0x20, 0x49, 0xB7, 0xD5, 0xE7, 0x6A, 0x11, 0x03, 0x20,
  0x49, 0xB7, 0xD5, 0xE7, 0x6A, 0x11, 0x03, 0x20, 0x49, 0xB7, 0xD5, 0xE7,
  0x6A, 0x11, 0x03, 0x20, 0x49, 0xB7, 0xD5, 0xE7, 0x6A, 0x11, 0x03, 0x20,
  0x49, 0xB7, 0xD5, 0xE7, 0x6A, 0x11, 0x03, 0x20, 0x49, 0xB7, 0xD5, 0xE7,
  0x6A, 0x11, 0x03, 0x20, 0x49, 0xB7, 0xD5, 0xE7, 0x6A, 0x11, 0x03, 0x20,
  0x49, 0xB7, 0xD5, 0xE7, 0x6A, 0x11, 0x03, 0x20, 0x49, 0xB7, 0xD5, 0xE7,
  0x6A, 0x11, 0x03, 0x20, 0x49, 0xB7, 0xD5, 0xE7, 0x6A, 0x11, 0x03, 0x20,
  0x49, 0xB7, 0xD5, 0xE7, 0x6A, 0x91, 0x29, 0x03, 0x90, 0x7F, 0x4F, 0x92,
  0xCE, 0x55, 0x7D, 0xB4, 0x56, 0x30, 0x00, 0x92, 0x74, 0xA7, 0xFA, 0x68,
  0xAD, 0x60, 0x00, 0x24, 0xE9, 0x4E, 0xF5, 0xD1, 0x5A, 0xC1, 0x00, 0x48,
  0xD2, 0x9D, 0xEA, 0xA3, 0xB5, 0x82, 0x01, 0x90, 0xA4, 0x3B, 0xD5, 0x47,
  0x6B, 0x05, 0x03, 0x20, 0x49, 0x77, 0xAA, 0x8F, 0xD6, 0x0A, 0x06, 0x40,
  0x92, 0xEE, 0x54, 0x1F, 0xAD, 0x15, 0x0C, 0x80, 0x24, 0xDD, 0xA9, 0x3E,
  0x5A, 0x2B, 0x18, 0x00, 0x49, 0xBA, 0x53, 0x7D, 0xB4, 0x56, 0x30, 0x00,
  0x92, 0x74, 0xA7, 0xFA, 0x68, 0xAD, 0x60, 0x00, 0x24, 0xE9, 0x4E, 0xF5,
  0xD1, 0x5A, 0xC1, 0x00, 0x48, 0xD2, 0x9D, 0xEA, 0xA3, 0xB5, 0x82, 0x01,
  0x90, 0xA4, 0x3B, 0xD5, 0x47, 0x6B, 0x05, 0x03, 0x20, 0x49, 0x77, 0xAA,
  0x8F, 0xD6, 0x0A, 0x06, 0x40, 0x92, 0xEE, 0x54, 0x1F, 0xAD, 0x15, 0x0C,
  0x80, 0x24, 0xDD, 0xA9, 0x3E, 0x5A, 0x2B, 0x18, 0x00, 0x49, 0xBA, 0x53,
  0x7D, 0xB4, 0x56, 0x98, 0x32, 0x00, 0xBF, 0x6C, 0x80, 0xA4, 0x87, 0xAB,
  0xCF, 0xD5, 0x22, 0x06, 0x40, 0x92, 0x6E, 0xAB, 0xCF, 0xD5, 0x22, 0x06,
  0x40, 0x92, 0x6E, 0xAB, 0xCF, 0xD5, 0x22, 0x06, 0x40, 0x92, 0x6E, 0xAB,
  0xCF, 0xD5, 0x22, 0x06, 0x40, 0x92, 0x6E, 0xAB, 0xCF, 0xD5, 0x22, 0x06,
  0x40, 0x92, 0x6E, 0xAB, 0xCF, 0xD5, 0x22, 0x06, 0x40, 0x92, 0x6E, 0xAB,
  0xCF, 0xD5, 0x22, 0x06, 0x40, 0x92, 0x6E, 0xAB, 0xCF, 0xD5, 0x22, 0x06,
  0x40, 0x92, 0x6E, 0xAB, 0xCF, 0xD5, 0x22, 0x06, 0x40, 0x92, 0x6E, 0xAB,
  0xCF, 0xD5, 0x22, 0x06, 0x40, 0x92, 0x6E, 0xAB, 0xCF, 0xD5, 0x22, 0x06,
  0x40, 0x92, 0x6E, 0xAB, 0xCF, 0xD5, 0x22, 0x83, 0x06, 0xE0, 0x97, 0x0D,
  0x90, 0xF4, 0x40, 0xF5, 0xA1, 0x5A, 0xC7, 0x00, 0x48, 0xD2, 0x87, 0xEA,
  0x43, 0xB5, 0x8E, 0x01, 0x90, 0xA4, 0x0F, 0xD5, 0x87, 0x6A, 0x1D, 0x03,
  0x20, 0x49, 0x1F, 0xAA, 0x0F, 0xD5, 0x3A, 0x06, 0x40, 0x92, 0x3E, 0x54,
  0x1F, 0xAA, 0x75, 0x0C, 0x80, 0x24, 0x7D, 0xA8, 0x3E, 0x54, 0xEB, 0x18,
  0x00, 0x49, 0xFA, 0x50, 0x7D, 0xA8, 0xD6, 0x99, 0x35, 0x00, 0xBF, 0x6C,
  0x80, 0xA4, 0xBF, 0x56, 0x9F, 0xA8, 0xA5, 0x0C, 0x80, 0x24, 0xBD, 0x57,
  0x9F, 0xA8, 0xA5, 0x0C, 0x80, 0x24, 0xBD, 0x57, 0x9F, 0xA8, 0xA5, 0x0C,
  0x80, 0x24, 0xBD, 0x57, 0x9F, 0xA8, 0xA5, 0x0C, 0x80, 0x24, 0xBD, 0x57,
  0x9F, 0xA8, 0xA5, 0x0C, 0x80, 0x24, 0xBD, 0x57, 0x9F, 0xA8, 0xA5, 0xC6,
  0x0D, 0xC0, 0x2F, 0x1B, 0x20, 0xE9, 0x93, 0xEA, 0xE3, 0xB4, 0x9A, 0x01,
  0x90, 0xA4, 0xB7, 0xEA, 0xE3, 0xB4, 0x9A, 0x01, 0x90, 0xA4, 0xB7, 0xEA,
  0xE3, 0xB4, 0x9A, 0x01, 0x90, 0xA4, 0xB7, 0xEA, 0xE3, 0xB4, 0x9A, 0x01,
  0x90, 0xA4, 0xB7, 0xEA, 0xE3, 0xB4, 0xDA, 0xC4, 0x01, 0xF8, 0x65, 0x03,
  0x24, 0xFD, 0x51, 0x7D, 0x96, 0x02, 0x06, 0x40, 0x92, 0x7E, 0x57, 0x9F,
  0xA5, 0x80, 0x01, 0x90, 0xA4, 0xDF, 0xD5, 0x67, 0x29, 0x60, 0x00, 0x24,
  0xE9, 0x77, 0xF5, 0x59, 0x0A, 0x0C, 0x1D, 0x80, 0x5F, 0x36, 0x40, 0xD2,
  0x7F, 0xAA, 0x0F, 0x52, 0xC3, 0x00, 0x48, 0x92, 0x01, 0x18, 0x26, 0xFF,
  0xE0, 0x24, 0xED, 0x53, 0x7D, 0x90, 0x1A, 0x06, 0x40, 0x92, 0x0C, 0xC0,
  0x3C, 0xF9, 0x37, 0x27, 0x69, 0x87, 0xEA, 0x53, 0x94, 0x31, 0x00, 0x92,
  0xA6, 0x57, 0x9F, 0xA2, 0x8C, 0x01, 0x90, 0x34, 0xBD, 0xFA, 0x14, 0x65,
  0x0C, 0x80, 0xA4, 0xE9, 0xD5, 0xA7, 0x28, 0x33, 0x7A, 0x00, 0x7E, 0xD9,
  0x00, 0x69, 0x7C, 0xF5, 0x11, 0x2A, 0x19, 0x80, 0xFE, 0xFB, 0x93, 0x14,
  0x56, 0x1F, 0xA1, 0x92, 0x01, 0xE8, 0xBF, 0x3F, 0x49, 0x61, 0xF5, 0x11,
  0x2A, 0x4D, 0x1F, 0x80, 0x5F, 0x36, 0x40, 0x1A, 0x5C, 0x7D, 0x7E, 0x62,
  0x06, 0xC0, 0x00, 0x48, 0x73, 0xAB, 0xCF, 0x4F, 0xCC, 0x00, 0x18, 0x00,
  0x69, 0x6E, 0xF5, 0xF9, 0x89, 0x19, 0x80, 0xDF, 0xF2, 0xAF, 0x50, 0xD2,
  0xFA, 0xEA, 0xC3, 0xD3, 0x33, 0x00, 0xBF, 0xE5, 0x1F, 0xA2, 0xA4, 0xF5,
  0xD5, 0x87, 0xA7, 0x67, 0x00, 0x7E, 0xCB, 0x3F, 0x44, 0x49, 0xEB, 0xAB,
  0x0F, 0x4F, 0xCF, 0x00, 0xBC, 0xC9, 0xBF, 0x45, 0x49, 0x2B, 0xAB, 0x4F,
  0xCE, 0x16, 0x0C, 0xC0, 0x9B, 0xFC, 0x73, 0x94, 0xB4, 0xB2, 0xFA, 0xE4,
  0x6C, 0xC1, 0x00, 0xBC, 0xC9, 0x3F, 0x47, 0x49, 0x2B, 0xAB, 0x4F, 0xCE,
  0x16, 0x0C, 0xC0, 0xBB, 0xFC, 0x8B, 0x94, 0xB4, 0xA6, 0xFA, 0xD8, 0xEC,
  0xC2, 0x00, 0xBC, 0xCB, 0x3F, 0x4A, 0x49, 0x6B, 0xAA, 0x8F, 0xCD, 0x2E,
  0x0C, 0xC0, 0x07, 0xF9, 0x77, 0x29, 0xE9, 0xE8, 0xEA, 0x33, 0xB3, 0x11,
  0x03, 0xF0, 0x41, 0xFE, 0x69, 0x4A, 0x3A, 0xBA, 0xFA, 0xCC, 0x6C, 0xC4,
  0x00, 0x7C, 0x90, 0x7F, 0x9A, 0x92, 0x8E, 0xAE, 0x3E, 0x33, 0x1B, 0x31,
  0x00, 0xB7, 0xF2, 0xAF, 0x53, 0xD2, 0x71, 0xD5, 0x07, 0x66, 0x2F, 0x06,
  0xE0, 0x56, 0xFE, 0x81, 0x4A, 0x3A, 0xAE, 0xFA, 0xC0, 0xEC, 0xC5, 0x00,
  0xDC, 0x91, 0x7F, 0xA3, 0x92, 0x8E, 0xA8, 0x3E, 0x2D, 0xDB, 0x31, 0x00,
  0x77, 0xE4, 0x9F, 0xA9, 0xA4, 0x23, 0xAA, 0x4F, 0xCB, 0x76, 0x0C, 0xC0,
  0x1D, 0xF9, 0x67, 0x2A, 0xE9, 0x88, 0xEA, 0xD3, 0xB2, 0x1D, 0x03, 0x70,
  0x5F, 0xFE, 0xA5, 0x4A, 0x7A, 0x6D, 0xF5, 0x51, 0xD9, 0x91, 0x01, 0xB8,
  0x2F, 0xFF, 0x58, 0x25, 0xBD, 0xB6, 0xFA, 0xA8, 0xEC, 0xC8, 0x00, 0x7C,
  0x2A, 0xFF, 0x5E, 0x25, 0xBD, 0xAA, 0xFA, 0x9C, 0x6C, 0xCA, 0x00, 0x7C,
  0x2A, 0xFF, 0x64, 0x25, 0xBD, 0xAA, 0xFA, 0x9C, 0x6C, 0xCA, 0x00, 0xFC,
  0x4D, 0xFE, 0xD5, 0x4A, 0x7A, 0xBE, 0xFA, 0x90, 0xEC, 0xCB, 0x00, 0xFC,
  0x4D, 0xFE, 0xE1, 0x4A, 0x7A, 0xBE, 0xFA, 0x90, 0xEC, 0xCB, 0x00, 0x7C,
  0x21, 0xFF, 0x76, 0x25, 0x3D, 0x53, 0x7D, 0x42, 0xB6, 0x66, 0x00, 0xBE,
  0x90, 0x7F, 0xBE, 0x92, 0x9E, 0xA9, 0x3E, 0x21, 0x5B, 0x33, 0x00, 0x5F,
  0xCB, 0xBF, 0x60, 0x49, 0x3F, 0xAB, 0x3E, 0x1E, 0xBB, 0x33, 0x00, 0x5F,
  0xCB, 0x3F, 0x62, 0x49, 0x3F, 0xAB, 0x3E, 0x1E, 0xBB, 0x33, 0x00, 0x0F,
  0xC9, 0xBF, 0x63, 0x49, 0xDF, 0xAD, 0x3E, 0x1B, 0x27, 0x60, 0x00, 0x1E,
  0x92, 0x7F, 0xCA, 0x92, 0xBE, 0x5B, 0x7D, 0x36, 0x4E, 0xC0, 0x00, 0x3C,
  0x2A, 0xFF, 0x9A, 0x25, 0x3D, 0x5E, 0x7D, 0x30, 0xCE, 0xC1, 0x00, 0x3C,
  0x2A, 0xFF, 0xA0, 0x25, 0x3D, 0x5E, 0x7D, 0x30, 0xCE, 0xC1, 0x00, 0x7C,
  0x43, 0xFE, 0x4D, 0x4B, 0x7A, 0xA4, 0xFA, 0x54, 0x9C, 0x86, 0x01, 0xF8,
  0x86, 0xFC, 0xB3, 0x96, 0xF4, 0x48, 0xF5, 0xA9, 0x38, 0x0D, 0x03, 0xF0,
  0x3D, 0xF9, 0x97, 0x2D, 0xE9, 0xEF, 0xD5, 0x47, 0xE2, 0x4C, 0x0C, 0xC0,
  0xB7, 0xE5, 0xDF, 0xB7, 0xA4, 0xCF, 0xAA, 0xCF, 0xC3, 0xC9, 0x18, 0x80,
  0x6F, 0xCB, 0x3F, 0x71, 0x49, 0x9F, 0x55, 0x9F, 0x87, 0x93, 0x31, 0x00,
  0x3F, 0x91, 0x7F, 0xE5, 0x92, 0xFE, 0xAC, 0x3E, 0x0C, 0xE7, 0x63, 0x00,
  0x7E, 0x22, 0xFF, 0xD0, 0x25, 0xFD, 0x59, 0x7D, 0x18, 0xCE, 0xC7, 0x00,
  0xFC, 0x50, 0xFE, 0xAD, 0x4B, 0xFA, 0x6F, 0xF5, 0x49, 0x38, 0x25, 0x03,
  0xF0, 0x73, 0xF9, 0x17, 0x2F, 0xE9, 0xDF, 0xEA, 0x63, 0x70, 0x56, 0x06,
  0xE0, 0xE7, 0xF2, 0x8F, 0x5E, 0xD2, 0xBF, 0xD5, 0xC7, 0xE0, 0xAC, 0x0C,
  0xC0, 0x53, 0xF2, 0xEF, 0x5E, 0x52, 0x7D, 0x06, 0x4E, 0xCC, 0x00, 0x3C,
  0x25, 0xFF, 0xF4, 0x25, 0xD5, 0x67, 0xE0, 0xC4, 0x0C, 0xC0, 0xB3, 0xF2,
  0xAF, 0x5F, 0x9A, 0x5C, 0x7D, 0x00, 0xCE, 0xCD, 0x00, 0xBC, 0x40, 0xFE,
  0x06, 0xA4, 0x99, 0xD5, 0x4F, 0xFF, 0xF4, 0x0C, 0xC0, 0x0B, 0xE4, 0xCF,
  0x40, 0x9A, 0x59, 0xFD, 0xF4, 0x4F, 0xCF, 0x00, 0xBC, 0x46, 0xFE, 0x12,
  0xA4, 0x69, 0xD5, 0x8F, 0xFE, 0x0A, 0x0C, 0xC0, 0xCB, 0xE4, 0xEF, 0x41,
  0x9A, 0x53, 0xFD, 0xDC, 0x2F, 0xC2, 0x00, 0xBC, 0x52, 0xFE, 0x2A, 0xA4,
  0x09, 0xD5, 0x0F, 0xFD, 0x3A, 0x0C, 0xC0, 0x2B, 0xE5, 0x0F, 0x43, 0x9A,
  0x50, 0xFD, 0xD0, 0xAF, 0xC3, 0x00, 0xBC, 0x58, 0xFE, 0x36, 0xA4, 0x6B,
  0x57, 0x3F, 0xF1, 0x4B, 0x31, 0x00, 0xAF, 0x97, 0xBF, 0x10, 0xE9, 0xAA,
  0xD5, 0x8F, 0xFB, 0x6A, 0x0C, 0xC0, 0x21, 0xF2, 0x77, 0x22, 0x5D, 0xAF,
  0xFA, 0x59, 0x5F, 0x90, 0x01, 0x38, 0x44, 0xFE, 0x54, 0xA4, 0xEB, 0x55,
  0x3F, 0xEB, 0x0B, 0x32, 0x00, 0x47, 0xC9, 0x5F, 0x8B, 0x74, 0xA5, 0xEA,
  0x07, 0x7D, 0x4D, 0x06, 0xE0, 0x40, 0xF9, 0x9B, 0x91, 0xAE, 0x51, 0xFD,
  0x94, 0x2F, 0xCB, 0x00, 0x1C, 0x2B, 0x7F, 0x39, 0xD2, 0xD9, 0xAB, 0x1F,
  0xF1, 0x95, 0x19, 0x80, 0x63, 0xE5, 0x8F, 0x47, 0x3A, 0x7B, 0xF5, 0x23,
  0xBE, 0x32, 0x03, 0x70, 0xB8, 0xFC, 0xFD, 0x48, 0xE7, 0xAD, 0x7E, 0xBE,
  0x17, 0x67, 0x00, 0x56, 0xC8, 0x5F, 0x91, 0x74, 0xC6, 0xEA, 0x87, 0x7B,
  0x7D, 0x06, 0x60, 0x91, 0xFC, 0x2D, 0x49, 0xE7, 0xAA, 0x7E, 0xB2, 0x23,
  0x18, 0x80, 0x75, 0xF2, 0x17, 0x25, 0x9D, 0xA5, 0xFA, 0xB1, 0x4E, 0x61,
  0x00, 0x96, 0xCA, 0xDF, 0x95, 0xB4, 0x7F, 0xF5, 0x33, 0x1D, 0xC4, 0x00,
  0xAC, 0x96, 0xBF, 0x2E, 0x69, 0xE7, 0xEA, 0x07, 0x3A, 0x8B, 0x01, 0x58,
  0x2D, 0x7F, 0x60, 0xD2, 0xCE, 0xD5, 0x0F, 0x74, 0x16, 0x03, 0x10, 0xC8,
  0xDF, 0x98, 0xB4, 0x67, 0xF5, 0xD3, 0x1C, 0xC7, 0x00, 0x34, 0xF2, 0x97,
  0x26, 0xED, 0x56, 0xFD, 0x28, 0x27, 0x32, 0x00, 0x99, 0xFC, 0xBD, 0x49,
  0xFB, 0x54, 0x3F, 0xC7, 0xA1, 0x0C, 0x40, 0x29, 0x7F, 0x75, 0xD2, 0x0E,
  0xD5, 0x0F, 0x71, 0x2E, 0x03, 0x10, 0xCB, 0xDF, 0x9E, 0xD4, 0x56, 0x3F,
  0xC1, 0xD1, 0x0C, 0x40, 0x2F, 0x7F, 0x81, 0x52, 0x55, 0xFD, 0xF8, 0xA6,
  0x33, 0x00, 0x5B, 0xC8, 0xDF, 0xA1, 0xB4, 0xBE, 0xFA, 0xD9, 0x61, 0x00,
  0xB6, 0x91, 0xBF, 0x46, 0x69, 0x65, 0xF5, 0x83, 0xE3, 0x37, 0x03, 0xB0,
  0x91, 0xFC, 0x4D, 0x4A, 0x6B, 0xAA, 0x9F, 0x1A, 0x6F, 0x0C, 0xC0, 0x5E,
  0xF2, 0x97, 0x29, 0x1D, 0x5D, 0xFD, 0xC8, 0x78, 0x67, 0x00, 0xB6, 0x93,
  0xBF, 0x4F, 0xE9, 0xB8, 0xEA, 0xE7, 0xC5, 0x07, 0x06, 0x60, 0x47, 0xF9,
  0x2B, 0x95, 0x8E, 0xA8, 0x7E, 0x58, 0xDC, 0x32, 0x00, 0x9B, 0xCA, 0xDF,
  0xAA, 0xF4, 0xDA, 0xEA, 0x27, 0xC5, 0x1D, 0x06, 0x60, 0x6B, 0xF9, 0xA3,
  0x95, 0x9E, 0xAF, 0x7E, 0x46, 0x7C, 0xCA, 0x00, 0xEC, 0x2E, 0x7F, 0xBD,
  0xD2, 0x33, 0xD5, 0x0F, 0x88, 0xBF, 0x31, 0x00, 0x27, 0x90, 0xBF, 0x61,
  0xE9, 0x67, 0xD5, 0x4F, 0x87, 0x2F, 0x18, 0x80, 0x73, 0xC8, 0x5F, 0xB2,
  0xF4, 0xDD, 0xEA, 0x47, 0xC3, 0xD7, 0x0C, 0xC0, 0x69, 0xE4, 0xEF, 0x59,
  0x7A, 0xBC, 0xFA, 0xB9, 0xF0, 0x10, 0x03, 0x70, 0x26, 0xF9, 0xAB, 0x96,
  0x1E, 0xA9, 0x7E, 0x28, 0x3C, 0xCA, 0x00, 0x9C, 0x4F, 0xFE, 0xBC, 0xA5,
  0xCF, 0xAA, 0x1F, 0x07, 0xDF, 0x63, 0x00, 0x4E, 0x29, 0x7F, 0xE7, 0xD2,
  0x9F, 0xD5, 0xCF, 0x82, 0x6F, 0x33, 0x00, 0x67, 0x95, 0xBF, 0x76, 0xE9,
  0xBF, 0xD5, 0x0F, 0x82, 0x9F, 0x30, 0x00, 0xE7, 0x96, 0x3F, 0x7B, 0xA9,
  0x7E, 0x04, 0xFC, 0x9C, 0x01, 0x38, 0xBD, 0xFC, 0xFD, 0x6B, 0x72, 0xF5,
  0xE7, 0xCF, 0x53, 0x0C, 0xC0, 0x15, 0xE4, 0x57, 0x40, 0x33, 0xAB, 0x3F,
  0x7C, 0x9E, 0x65, 0x00, 0xAE, 0x23, 0x3F, 0x07, 0x9A, 0x53, 0xFD, 0xB1,
  0xF3, 0x1A, 0x06, 0xE0, 0x52, 0xF2, 0xBB, 0xA0, 0x09, 0xD5, 0x9F, 0x39,
  0x2F, 0x63, 0x00, 0xAE, 0x26, 0xBF, 0x0E, 0xBA, 0x76, 0xF5, 0x07, 0xCE,
  0x2B, 0x19, 0x80, 0x6B, 0xCA, 0xCF, 0x84, 0xAE, 0x57, 0xFD, 0x51, 0xF3,
  0x7A, 0x06, 0xE0, 0xB2, 0xF2, 0x7B, 0xA1, 0x2B, 0x55, 0x7F, 0xCE, 0x1C,
  0xC2, 0x00, 0x5C, 0x5C, 0x7E, 0x38, 0x74, 0xF6, 0xEA, 0x4F, 0x98, 0x03,
  0x19, 0x80, 0x11, 0xF2, 0x23, 0xA2, 0x33, 0x56, 0x7F, 0xB6, 0x1C, 0xCE,
  0x00, 0x4C, 0x91, 0x5F, 0x13, 0x9D, 0xAB, 0xFA, 0x83, 0x65, 0x05, 0x03,
  0x30, 0x4B, 0x7E, 0x56, 0xB4, 0x7F, 0xF5, 0x47, 0xCA, 0x3A, 0x06, 0x60,
  0xA2, 0xFC, 0xC4, 0x68, 0xCF, 0xEA, 0x0F, 0x93, 0xD5, 0x0C, 0xC0, 0x50,
  0xF9, 0xAD, 0xD1, 0x6E, 0xD5, 0x9F, 0x24, 0x01, 0x03, 0x30, 0x5A, 0x7E,
  0x74, 0xB4, 0x43, 0xF5, 0x67, 0x48, 0xC6, 0x00, 0x60, 0x06, 0xE6, 0x56,
  0x7F, 0x7A, 0xC4, 0x0C, 0x00, 0x6F, 0xF2, 0x63, 0xA4, 0x95, 0xD5, 0x9F,
  0x1B, 0x5B, 0x30, 0x00, 0x7C, 0x90, 0x1F, 0x26, 0x1D, 0x5D, 0xFD, 0x89,
  0xB1, 0x11, 0x03, 0xC0, 0x1D, 0xF9, 0x91, 0xD2, 0x11, 0xD5, 0x9F, 0x15,
  0xDB, 0x31, 0x00, 0x7C, 0x2A, 0x3F, 0x58, 0x7A, 0x55, 0xF5, 0xA7, 0xC4,
  0xA6, 0x0C, 0x00, 0x5F, 0xC8, 0x8F, 0x97, 0x9E, 0xA9, 0xFE, 0x7C, 0xD8,
  0x9A, 0x01, 0xE0, 0x21, 0xF9, 0x21, 0xD3, 0x77, 0xAB, 0x3F, 0x19, 0x4E,
  0xC0, 0x00, 0xF0, 0x3D, 0xF9, 0x5D, 0xD3, 0xDF, 0xAB, 0x3F, 0x10, 0xCE,
  0xC4, 0x00, 0xF0, 0x13, 0xF9, 0x99, 0xD3, 0x9F, 0xD5, 0x1F, 0x05, 0xE7,
  0x63, 0x00, 0x78, 0x4A, 0x7E, 0xF5, 0x54, 0x7F, 0x02, 0x9C, 0x98, 0x01,
  0xE0, 0x05, 0xF2, 0x23, 0x38, 0xB3, 0xFA, 0x67, 0xE7, 0xF4, 0x0C, 0x00,
  0xAF, 0x94, 0xDF, 0xC4, 0x09, 0xD5, 0x3F, 0x32, 0xD7, 0x61, 0x00, 0x38,
  0x44, 0x7E, 0x25, 0xAF, 0x57, 0xFD, 0x93, 0x72, 0x41, 0x06, 0x80, 0x63,
  0xE5, 0x77, 0xF3, 0xEC, 0xD5, 0x3F, 0x20, 0x57, 0x66, 0x00, 0x58, 0x24,
  0xBF, 0xA4, 0xE7, 0xAA, 0xFE, 0xB9, 0x18, 0xC1, 0x00, 0x10, 0xC8, 0xCF,
  0xEB, 0x9E, 0xD5, 0x3F, 0x0B, 0xE3, 0x18, 0x00, 0x62, 0xF9, 0xD9, 0x75,
  0xF4, 0x19, 0xCB, 0x00, 0xB0, 0x91, 0xFC, 0x1C, 0x3B, 0xFA, 0x8C, 0x62,
  0x00, 0xD8, 0x57, 0x7E, 0xA9, 0x5D, 0x7C, 0xAE, 0xCD, 0x00, 0x70, 0x26,
  0xF9, 0x29, 0x77, 0xEE, 0xB9, 0x12, 0x03, 0xC0, 0xE9, 0xB9, 0xF5, 0xF0,
  0x33, 0x06, 0x80, 0x11, 0x1C, 0x77, 0xF8, 0x93, 0x01, 0x00, 0x18, 0xCA,
  0x00, 0x00, 0x0C, 0x65, 0x00, 0x00, 0x86, 0x32, 0x00, 0x00, 0x43, 0x19,
  0x00, 0x80, 0xA1, 0x0C, 0x00, 0xC0, 0x50, 0x06, 0x00, 0x60, 0x28, 0x03,
  0x00, 0x30, 0x94, 0x01, 0x00, 0x18, 0xCA, 0x00, 0x00, 0x0C, 0x65, 0x00,
  0x00, 0x86, 0x32, 0x00, 0x00, 0x43, 0x19, 0x00, 0x80, 0xA1, 0x0C, 0x00,
  0xC0, 0x50, 0x06, 0x00, 0x60, 0x28, 0x03, 0x00, 0x30, 0x94, 0x01, 0x00,
  0x18, 0xCA, 0x00, 0x00, 0x0C, 0x65, 0x00, 0x00, 0x86, 0x32, 0x00, 0x00,
  0x43, 0x19, 0x00, 0x80, 0xA1, 0x0C, 0x00, 0xC0, 0x50, 0x06, 0x00, 0x60,
  0x28, 0x03, 0x00, 0x30, 0x94, 0x01, 0x00, 0x18, 0xCA, 0x00, 0x00, 0x0C,
  0x65, 0x00, 0x00, 0x86, 0x32, 0x00, 0x00, 0x43, 0x19, 0x00, 0x80, 0xA1,
  0x0C, 0x00, 0xC0, 0x50, 0x06, 0x00, 0x60, 0x28, 0x03, 0x00, 0x30, 0x94,
  0x01, 0x00, 0x18, 0xCA, 0x00, 0x00, 0x0C, 0x65, 0x00, 0x00, 0x86, 0x32,
  0x00, 0x00, 0x43, 0x19, 0x00, 0x80, 0xA1, 0x0C, 0x00, 0xC0, 0x50, 0x06,
  0x00, 0x60, 0x28, 0x03, 0x00, 0x30, 0x94, 0x01, 0x00, 0x18, 0xCA, 0x00,
  0x00, 0x0C, 0x65, 0x00, 0x00, 0x86, 0x32, 0x00, 0x00, 0x43, 0x19, 0x00,
  0x80, 0xA1, 0x0C, 0x00, 0xC0, 0x50, 0x06, 0x00, 0x60, 0x28, 0x03, 0x00,
  0x30, 0x94, 0x01, 0x00, 0x18, 0xCA, 0x00, 0x00, 0x0C, 0x65, 0x00, 0x00,
  0x86, 0x32, 0x00, 0x00, 0x43, 0x19, 0x00, 0x80, 0xA1, 0x0C, 0x00, 0xC0,
  0x50, 0x06, 0x00, 0x60, 0x28, 0x03, 0x00, 0x30, 0x94, 0x01, 0x00, 0x18,
  0xCA, 0x00, 0x00, 0x0C, 0x65, 0x00, 0x00, 0x86, 0x32, 0x00, 0x00, 0x43,
  0x19, 0x00, 0x80, 0xA1, 0x0C, 0x00, 0xC0, 0x50, 0x06, 0x00, 0x60, 0x28,
  0x03, 0x00, 0x30, 0x94, 0x01, 0x00, 0x18, 0xCA, 0x00, 0x00, 0x0C, 0x65,
  0x00, 0x00, 0x86, 0x32, 0x00, 0x00, 0x43, 0x19, 0x00, 0x80, 0xA1, 0x0C,
  0x00, 0xC0, 0x50, 0x06, 0x00, 0x60, 0x28, 0x03, 0x00, 0x30, 0x94, 0x01,
  0x00, 0x18, 0xCA, 0x00, 0x00, 0x0C, 0x65, 0x00, 0x00, 0x86, 0x32, 0x00,
  0x00, 0x43, 0x19, 0x00, 0x80, 0xA1, 0x0C, 0x00, 0xC0, 0x50, 0x06, 0x00,
  0x60, 0x28, 0x03, 0x00, 0x30, 0x94, 0x01, 0x00, 0x18, 0xCA, 0x00, 0x00,
  0x0C, 0x65, 0x00, 0x00, 0x86, 0x32, 0x00, 0x00, 0x43, 0xFD, 0x1F, 0xD3,
  0x2F, 0x0A, 0xB6, 0x3A, 0x51, 0x10, 0xA6, 0x00, 0x00, 0x00, 0x00, 0x49,
  0x45, 0x4E, 0x44, 0xAE, 0x42, 0x60, 0x82,
};


//======================================================================
//9. WEB SERVER — HTML, JSON status, routes
//======================================================================

static void handleRoot() {
    static const char html[] = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<link rel="icon" type="image/png" href="/icon-192.png" sizes="96x96" />
<link rel="icon" type="image/svg+xml" href="/icon.svg" />
<link rel="apple-touch-icon" sizes="180x180" href="/icon-192.png" />
<meta name="apple-mobile-web-app-title" content="BenchPilot" />
<link rel="manifest" href="/manifest.json">
<title>BenchPilot</title>
<style>
/* Colors computed from TFT RGB565: AP=#410010 ADJ/NAV=#001062 FLUXSIM=#cca800 FLUXSIM_DIM=#555 HDGADJ=#cca800 HDG=#0ff TRG=#0f0 ALARM=#f00 */
*{box-sizing:border-box}
body{background:#0a0a0a;color:#ccc;font-family:'Courier New',monospace;margin:0;padding:10px;font-size:clamp(14px,1.6vw,20px);min-height:100vh;display:flex;flex-direction:column}
.wrap{max-width:960px;width:100%;margin:0 auto;display:flex;flex-direction:column;flex:1}
.ts{color:#6a8ba8}
.hx{color:#6a8b6a}
.sep{color:#7a6a7a}
.hi{color:#d9a040;font-weight:bold}
.log-line{white-space:pre-wrap;font-size:.85em;line-height:1.5;padding:1px 0}
.log-line.latest{background:#332200}
.log-line.latest .log-desc{color:#fda;font-weight:bold}
.log-cnt{display:inline-block;width:5%;color:#7a6a7a;vertical-align:top}
.log-ts{display:inline-block;width:8%;color:#6a8ba8;vertical-align:top}
.log-hx{display:inline-block;width:52%;color:#6a8b6a;vertical-align:top;word-break:break-all}
.log-desc{display:inline-block;color:#d9a040;vertical-align:top}
.log-desc-tx{display:inline-block;color:#d97010;vertical-align:top}
.log-line.latest .log-desc-tx{color:#ff9620;font-weight:bold}
.mode{font-size:1.2em;font-weight:bold;text-align:center;margin:4px 0 4px 0;min-height:1.2em}
.hdg{font-size:3.6em;font-weight:bold;text-align:center;color:#0ff;margin:0 0 4px 0;line-height:1.1}
.trg{font-size:1.4em;font-weight:bold;text-align:center;color:#0f0;margin:0 0 4px 0;display:none}
.rdr{font-size:2em;font-weight:bold;text-align:center;color:#0ff;margin:0 0 2px 0}
/* Rudder bar mirrors CYD: full button-width track, 1px grey outline, 5° ticks,
   white-bordered value box, fill inset 2px */
.rdrbar{position:relative;height:24px;margin:0 auto 4px auto;width:100%;border:1px solid #7b7d7b;box-sizing:border-box}
.rdrtick{position:absolute;top:4px;bottom:4px;width:1px;background:#7b7d7b;pointer-events:none}
.rdrfill{position:absolute;top:3px;height:18px}
.rdrnum{position:absolute;left:50%;top:50%;transform:translate(-50%,-50%);width:16.667%;height:24px;line-height:22px;background:#0a0a0a;border:1px solid #7b7d7b;color:#fff;font-weight:bold;font-size:0.8em;text-align:center;display:none}
.alarms{text-align:center;margin:0 0 6px 0;min-height:1.4em}
.alarm{display:inline-block;padding:2px 8px;border-radius:3px;margin:1px 3px;font-size:0.8em;font-weight:bold}
.alarm.red{background:#200008;color:#f00}
.alarm.org{background:#200008;color:#f60}
.sensors{display:flex;flex-wrap:wrap;gap:5px;margin:0 0 6px 0;justify-content:center;font-size:0.82em}
.sensors .s{background:#111;border:1px solid #222;padding:2px 7px;border-radius:3px;color:#ccc}
.btns{display:grid;grid-template-columns:100px 1fr 1fr 100px;gap:4px;margin-bottom:6px}
.btns button{padding:20px 8px;font-family:monospace;font-size:1.3em;font-weight:bold;border:0;border-radius:4px;cursor:pointer;color:#fff}
.btns button:active{filter:brightness(1.6)}
.btns .big{grid-row:span 2;display:flex;align-items:center;justify-content:center}
.btns .stby{background:#410010}.btns .auto{background:#410010}
.btns .stby.active,.btns .auto.active{background:#cc0020}
.btns .adj{background:#001062}.btns .adj10{background:#001062}
.btns .wind{background:#001062}.btns .track{background:#001062}
.btns .fluxsim{background:#cca800}
.btns .fluxsim.dim{background:#555;opacity:.6}
.btns .hdgadj{background:#555;opacity:.6}
.btns .hdgadj:active{background:#cca800;opacity:1;filter:none}
.btns .hdgadj.pulse{background:#cca800;opacity:1;filter:brightness(1.8)}
#frames{background:#111;border:1px solid #222;border-radius:4px;padding:6px 8px;flex:1;overflow-y:auto;font-size:.85em;min-height:1.8em;max-height:45vh}
.foot{width:100%;text-align:center;margin:3px 0;font-size:.85em}
.foot a{color:#6a8ba8;text-decoration:none}.foot a:hover{color:#d9a040}
.foot span{color:#444;margin:0 4px}#fl{text-align:left}#fr{text-align:right}
</style>
</head>
<body>

<div class="wrap">

<div class="mode" id="mode">MOD</div>
<div class="hdg" id="hdg">HDG</div>
<div class="trg" id="trg">TRG</div>
<div class="alarms" id="alarms"></div>
<div class="rdrbar" id="rdrbar">
  <div class="rdrfill" id="rdrfill" style="display:none"></div>
  <div class="rdrnum" id="rdrnum"></div>
</div>
<div class="sensors" id="sensors"></div>
<!-- Button grid — layout must match btns[] array in C++. -->
<div class="btns" style="grid-template-columns:96fr 52fr 52fr 96fr">
  <button class="big stby" id="stbybtn" onclick="apCmd('standby')">STBY</button>
  <button class="adj" onclick="apCmd('minus1')">-1&deg;</button>
  <button class="adj" onclick="apCmd('plus1')">+1&deg;</button>
  <button class="big auto" id="autobtn" onclick="apCmd('auto')">AUTO</button>
  <button class="adj10" onclick="apCmd('minus10')">-10&deg;</button>
  <button class="adj10" onclick="apCmd('plus10')">+10&deg;</button>
</div>
<div class="btns" style="grid-template-columns:48fr 48fr 52fr 52fr 48fr 48fr">
  <button class="wind" id="windbtn" onclick="apCmd('wind')" style="grid-column:span 2">WIND</button>
  <button class="hdgadj" id="hdg_m10" onclick="hdgAdj(-10)" style="display:none">-10</button>
  <button class="hdgadj" id="hdg_m1" onclick="hdgAdj(-1)" style="display:none">-1</button>
  <button class="fluxsim dim" id="simbtn" onclick="simAuto()">SIM</button>
  <button class="fluxsim dim" id="setbtn" onclick="simFixed()">SET</button>
  <button class="track" id="trackbtn" onclick="apCmd('track')" style="grid-column:span 2">TRACK</button>
  <button class="hdgadj" id="hdg_p1" onclick="hdgAdj(1)" style="display:none">+1</button>
  <button class="hdgadj" id="hdg_p10" onclick="hdgAdj(10)" style="display:none">+10</button>
</div>
<div class="foot" style="display:grid;grid-template-columns:1fr auto 1fr;align-items:center"><span id="fl"></span><span id="fc"><a href="#" onclick="showLive();return false" style="color:#d9a040">live</a><span style="color:#444">|</span><a href="#" onclick="showLogFiles();return false">logs</a></span><span id="fr"></span></div>
<div id="frames">LOG</div>
<script>
function beep(){
  try{
    let a=new AudioContext(), o=a.createOscillator(), g=a.createGain();
    o.type='sine'; o.frequency.value=1760;
    g.gain.setValueAtTime(0.15,a.currentTime);
    g.gain.exponentialRampToValueAtTime(0.001,a.currentTime+0.04);
    o.connect(g).connect(a.destination);
    o.start(); o.stop(a.currentTime+0.04);
  }catch(e){}
  try{navigator.vibrate(30)}catch(e){}
}
function simVis(active){
  document.getElementById('windbtn').style.display=active?'none':'';
  document.getElementById('trackbtn').style.display=active?'none':'';
  for(let id of ['hdg_m10','hdg_m1','hdg_p1','hdg_p10'])
    document.getElementById(id).style.display=active?'':'none';
}
async function simAuto(){
  beep();
  let sim=document.getElementById('simbtn');
  let active=!sim.classList.contains('dim');
  simVis(!active);
  if(active){
    await fetch('/api/heading/stop');
    sim.classList.add('dim');
  } else {
    document.getElementById('setbtn').classList.add('dim');
    await fetch('/api/heading?deg='+_lastHdg+'&interval=250');
    sim.classList.remove('dim');
  }
}
async function simFixed(){
  beep();
  let set=document.getElementById('setbtn');
  let active=!set.classList.contains('dim');
  simVis(!active);
  if(active){
    await fetch('/api/heading/stop');
    set.classList.add('dim');
  } else {
    document.getElementById('simbtn').classList.add('dim');
    await fetch('/api/heading?deg='+_lastHdg+'&drift=0&interval=250');
    set.classList.remove('dim');
  }
}
function hdgAdj(delta){
  beep();
  fetch('/api/heading/adj?d='+delta);
}
async function apCmd(cmd){
  beep();
  await fetch('/api/autopilot?cmd='+cmd);
}
let paused=false,pausedData=null,_lastHdg=180,_prevHdgPulse=-1,_newestKey='',gSlots=new Map();
function esc(s){return s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;')}
function sensor(v,u,s){return v>=0?'<span class="s">'+s+' '+v+u+'</span>':''}
function renderWidgets(d){
  let mode='', hdg='', rdr='', alhs='', trg='';
  if(d.display){
    for(let w of d.display){
      switch(w.w){
        case 'MODE': mode='<div class="mode" id="mode" style="color:'+(w.c=='yellow'?'#f0f':w.c=='green'?'#0f0':'#f0f')+'">'+w.t+'</div>'; break;
        case 'HDG': hdg='<div class="hdg" id="hdg" style="'+(w.sim?'color:#ffa500':'')+'">'+(w.t>=0?w.t:'HDG')+'</div>'; if(w.t>=0)_lastHdg=w.t; break;
        case 'RDR': if(w.num) rdr='<div class="rdr" id="rdr">'+(w.v?w.t:'RDR')+'</div>'; break;
        case 'ALARMS':{
          let sim=w.sim;
          if(w.a&&w.a.length){
            for(let a of w.a){
              let cls=a.includes('OFF')||a.includes('LARGE')||a.includes('AUTO')?'red':'org';
              alhs+='<span class="alarm '+cls+'">'+a+'</span>';
            }
          }
          break;
        }
        case 'TRG': if(w.t>=0){
          let a='',c='#0f0';
          if(w.s===3)      { a='';   c='#0ff'; }
          else if(w.s===1) { a='← '; c='#c00'; }
          else             { a='→ '; c='#0f0'; }
          trg='<div class="trg" id="trg" style="display:block;color:'+c+'">'+a+w.t+'\u00B0</div>';
        } break;
      }
    }
  }
  document.getElementById('mode').outerHTML=mode||'<div class="mode" id="mode">MOD</div>';
  document.getElementById('hdg').outerHTML=hdg||'<div class="hdg" id="hdg">HDG</div>';
  document.getElementById('alarms').innerHTML=alhs;
  document.getElementById('trg').outerHTML=trg||'<div class="trg" id="trg">TRG</div>';
}
// 1 tick every 5° — 5 sections per side, painted over the fill (mirrors CYD)
let rdrbar=document.getElementById('rdrbar'), rdrnumEl=document.getElementById('rdrnum');
for(let i=-25;i<=25;i+=5){
  if(!i)continue;
  let t=document.createElement('div');
  t.className='rdrtick';
  t.style.left='calc(50% + '+i+'*100%/60)';
  rdrbar.insertBefore(t,rdrnumEl);
}
function render(d){
  if(paused) return;
  renderWidgets(d);
  for(let w of d.display) if(w.w=='MODE'){
    document.getElementById('stbybtn').classList.toggle('active', w.v===0);
    document.getElementById('autobtn').classList.toggle('active', w.v===2);
  }
  let fb=document.getElementById('simbtn'),fixbtn=document.getElementById('setbtn');
  for(let w of d.display) if(w.w=='ALARMS'){
    fb.classList.toggle('dim',!(w.sim&&w.drift));
    fixbtn.classList.toggle('dim',!(w.sim&&!w.drift));
    simVis(w.sim);
  }
  // pulse hdg adj buttons when FLUXSIM drift changes heading
  for(let w of d.display) if(w.w=='HDG' && w.t>=0){
    if(_prevHdgPulse >= 0 && w.t != _prevHdgPulse){
      let dir=(w.t>_prevHdgPulse||(360+w.t-_prevHdgPulse)%360<180)?'hdg_p1':'hdg_m1';
      let btn=document.getElementById(dir);
      if(btn){btn.classList.add('pulse');setTimeout(()=>btn.classList.remove('pulse'),200);}
    }
    _prevHdgPulse=w.t;
  }
  // rudder bar — same geometry/colors as CYD: fill starts 1% off center, ±30° = 49%
  let rf=document.getElementById('rdrfill');
  if(d.rudder_valid){
    let r=d.rudder, av=Math.min(Math.abs(r),30), pct=av/30*49;
    rf.style.background=av>=30?'#f00':'#0ff';
    rf.style.left=r>=0?'51%':(49-pct)+'%';
    rf.style.width=pct+'%';
    rf.style.display='block';
    rdrnumEl.textContent=r;
  } else {
    rf.style.display='none';
    rdrnumEl.textContent='RDR';
  }
  rdrnumEl.style.display='block';
  // sensors
  let s='';
  if(d.wind_angle>=0||d.wind_speed>=0)
    s+=sensor(d.wind_angle,'\u00B0','Wind:')+sensor((d.wind_speed/10).toFixed(1),'kt','');
  s+=sensor(d.cog,'\u00B0','COG:');
  s+=sensor((d.stw/10).toFixed(1),'kt','STW:');
  s+=sensor((d.sog/10).toFixed(1),'kt','SOG:');
  s+=sensor((d.depth/10).toFixed(1),'ft','Depth:');
  s+=sensor(d.water_temp,'\u00B0F','Temp:');
  if(d.gps_sats >= 0) s += '<span class="s">SATS: '+d.gps_sats+' HDOP '+d.gps_hdop+'</span>';
  if(d.gps_lat_deg >= 0) s += '<span class="s">GPS: '+d.gps_lat_deg+'\u00B0 '+(d.gps_lat_min/100).toFixed(2)+'\' '+(d.gps_lat_n?'N':'S')+' '+d.gps_lon_deg+'\u00B0 '+(d.gps_lon_min/100).toFixed(2)+'\' '+(d.gps_lon_e?'E':'W')+'</span>';
  document.getElementById('sensors').innerHTML=s;
  // msgs — persistent slot dedup + static columns
  let h='';
  if(d.msgs&&d.msgs.length){
    let seen=new Set(), newestKey='';
    for(let m of d.msgs){
      let p=m.lastIndexOf(','),body=p>=0?m.slice(p+1):m;
      let k=body.replace(/[-\d]/g,'').replace(/\s+/g,' ').trim();
      if(!seen.has(k)){
        seen.add(k);
        if(!newestKey) newestKey=k;
        let s=gSlots.get(k);
        if(s){s.c=1;s.t=m;}else gSlots.set(k,{c:1,t:m});
      } else {
        let s=gSlots.get(k); if(s){s.c++;if(s.c>999)s.c=1;}
      }
    }
    for(let[k,s]of gSlots){
      if(!s.c) continue;
      h+=logLine(s.t,s.c,k===newestKey);
    }
    if(newestKey&&newestKey!==_newestKey){
      _newestKey=newestKey;
      try{let a=new AudioContext(),o=a.createOscillator(),g=a.createGain();o.type='square';o.frequency.value=1200;g.gain.setValueAtTime(0.07,a.currentTime);g.gain.exponentialRampToValueAtTime(0.001,a.currentTime+0.015);o.connect(g).connect(a.destination);o.start();o.stop(a.currentTime+0.015);}catch(e){}
    }
  }
  let frames=document.getElementById('frames');
  let st=frames.scrollTop, sh=frames.scrollHeight;
  frames.innerHTML=h||'';
  if(st>0) frames.scrollTop=st+(frames.scrollHeight-sh);
}
function logLine(msg,cnt,latest){
  let p=msg.indexOf(','),l=msg.lastIndexOf(',');
  if(p>=0&&l>p){
    let ts=esc(msg.slice(0,p)),hx=esc(msg.slice(p+1,l)),raw=msg.slice(l+1),isTx=raw.startsWith('> ');
    let desc=esc(isTx?raw.slice(2):raw);
    return '<div class="log-line'+(latest?' latest':'')+'"><span class="log-cnt">x'+cnt+'</span><span class="log-ts">'+ts+'</span><span class="log-hx">'+hx+'</span><span class="log-'+(isTx?'desc-tx':'desc')+'">'+desc+'</span></div>';
  }
  return '<div class="log-line'+(latest?' latest':'')+'"><span class="log-cnt">x'+cnt+'</span><span class="log-ts">--</span><span class="log-hx">--</span><span class="log-desc">'+esc(msg)+'</span></div>';
}
async function poll(){if(paused)return;let r=await fetch('/status');let d=await r.json();render(d)}
setInterval(poll,1000);poll();
document.getElementById('frames').onclick=function(){
  let t=[];
  this.querySelectorAll('div').forEach(d=>t.push(d.textContent));
  let s=t.join('\r\n'), ta=document.createElement('textarea');
  ta.value=s; ta.style.position='fixed'; ta.style.left='-9999px';
  document.body.appendChild(ta); ta.select();
  try{document.execCommand('copy')}catch(e){}
  document.body.removeChild(ta);
  showToast('Copied');
};
function showToast(msg){
  let el=document.createElement('div');
  el.textContent=msg;
  el.style.cssText='position:fixed;bottom:20px;left:50%;transform:translateX(-50%);background:#333;color:#fff;padding:6px 16px;border-radius:4px;font-size:14px;z-index:999;opacity:0;transition:opacity 0.3s';
  document.body.appendChild(el);
  requestAnimationFrame(()=>el.style.opacity='1');
  setTimeout(()=>{el.style.opacity='0';setTimeout(()=>el.remove(),300)},1500);
}
function setFooter(l,c,r){
  document.getElementById('fl').innerHTML=l||'';
  document.getElementById('fc').innerHTML=c;
  document.getElementById('fr').innerHTML=r||'';
}
function showLive(){ paused=false; setFooter(null,'<a href="#" onclick="showLive();return false" style="color:#d9a040">live</a><span style="color:#444">|</span><a href="#" onclick="showLogFiles();return false">logs</a>',null); poll(); }
async function showLogFiles(){
  paused=true; setFooter('<a href="#" onclick="showLive();return false" style="color:#6a8ba8">\u2190</a>','<a href="#" onclick="showLive();return false">live</a><span style="color:#444">|</span><a href="#" onclick="showLogFiles();return false" style="color:#d9a040">logs</a>','<a href="#" onclick="delLogs();return false" style="color:#6a8ba8">delete all logs</a>');
  let r=await fetch('/api/files'),files=await r.json();
  files.sort((a,b)=>b.name.localeCompare(a.name));
  let h='';
  for(let f of files)
    h+='<div><a href="#" onclick="viewFile(\''+esc(f.name)+'\');return false" style="color:#6a8ba8">'+esc(f.name)+'</a> <span style="color:#555">('+f.size+' bytes)</span></div>';
  document.getElementById('frames').innerHTML=h;
}
async function viewFile(name){
  paused=true; setFooter('<a href="#" onclick="showLogFiles();return false" style="color:#6a8ba8">\u2190</a>','<a href="#" onclick="showLive();return false">live</a><span style="color:#444">|</span><a href="#" onclick="showLogFiles();return false" style="color:#d9a040">logs</a>',null);
  let r=await fetch('/api/download?file='+name),txt=await r.text();
  let h='<pre style="white-space:pre-wrap;font-size:.85em;line-height:1.4;margin:0">'+esc(txt)+'</pre>';
  document.getElementById('frames').innerHTML=h;
}
async function delLogs(){
  if(!confirm('Delete all log files?'))return;
  await fetch('/api/logs/clear');
  showLogFiles();
}
</script>
</div>
</body>
</html>
)rawliteral";
    server.send(200, "text/html", html);
}

static void handleStatus() {
    unsigned long now = millis();
    unsigned long dt = (lastPacketTime > 0) ? now - lastPacketTime : 0;
    bool active = (lastPacketTime > 0 && dt < 3000);

    String json;
    json.reserve(8192);
    json = "{";
    json += "\"wifi\":" + String(wifiConnected ? "true" : "false") + ",";
    json += "\"ap\":\"" + String(AP_SSID) + "\",";
    json += "\"sta\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
    json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
    json += "\"uptime\":\"" + formatUptime(now) + "\",";
    json += "\"bytes\":" + String(totalBytes) + ",";
    json += "\"active\":" + String(active ? "true" : "false") + ",";
    json += "\"rudder\":" + String(g.rudder) + ",";
    json += "\"rudder_valid\":" + String(g.rudderValid ? "true" : "false") + ",";

    // Sensors
    auto valid = [](unsigned long t) { return millis() - t < g.SENSOR_TIMEOUT; };
    json += "\"wind_angle\":" + String(valid(g.sensorSeen[SI_WIND_ANGLE]) ? g.windAngle : -1) + ",";
    json += "\"wind_speed\":" + String(valid(g.sensorSeen[SI_WIND_SPEED]) ? g.windSpeed : -1) + ",";
    json += "\"stw\":" + String(valid(g.sensorSeen[SI_STW]) ? g.stw : -1) + ",";
    json += "\"sog\":" + String(valid(g.sensorSeen[SI_SOG]) ? g.sog : -1) + ",";
    json += "\"cog\":" + String(valid(g.sensorSeen[SI_COG]) ? g.cog : -1) + ",";
    json += "\"depth\":" + String(valid(g.sensorSeen[SI_DEPTH]) ? g.depth : -1) + ",";
    json += "\"water_temp\":" + String(valid(g.sensorSeen[SI_WATER_TEMP]) ? g.waterTemp : -1) + ",";
    bool gpsValid = (millis() - g.gpsSeen < g.SENSOR_TIMEOUT);
    json += "\"gps_lat_deg\":" + String(gpsValid ? g.gpsLatDeg : -1) + ",";
    json += "\"gps_lat_min\":" + String(gpsValid ? g.gpsLatMin : -1) + ",";
    json += "\"gps_lat_n\":" + String(gpsValid && g.gpsLatNorth ? "true" : "false") + ",";
    json += "\"gps_lon_deg\":" + String(gpsValid ? g.gpsLonDeg : -1) + ",";
    json += "\"gps_lon_min\":" + String(gpsValid ? g.gpsLonMin : -1) + ",";
    json += "\"gps_lon_e\":" + String(gpsValid && g.gpsLonEast ? "true" : "false") + ",";
    json += "\"gps_sats\":" + String(gpsValid ? g.gpsSats : -1) + ",";
    json += "\"gps_hdop\":" + String(gpsValid ? g.gpsHDOP : -1) + ",";
    bool apRecent = (millis() - g.apUpdated < AP_RECENT_MS);

    // Display widget array — both TFT and Web render from this
    json += "\"display\":[";
    bool first = true;
    for (auto w : widgetOrder) {
        if (w == W_END) break;
        if (w == W_BTNS) continue;  // buttons are static HTML
        if (!first) json += ",";
        first = false;
        switch (w) {
        case W_MODE:
            json += "{\"w\":\"MODE\",\"t\":\"";
            json += modeLabel(g.mode);
            json += "\",\"c\":\"";
            json += modeColorName(g.mode);
            json += "\",\"v\":" + String(g.mode) + "}";
            break;
        case W_HEADING:
            json += "{\"w\":\"HDG\",\"t\":" + String(g.heading)
                 + ",\"sim\":" + String(_hdgActive ? "true" : "false") + "}";
            break;
        case W_RUDDER:
            json += "{\"w\":\"RDR\",\"t\":" + String(g.rudderValid ? g.rudder : -1)
                 + ",\"v\":" + String(g.rudderValid ? "true" : "false") + ",\"num\":false}";
            break;
        case W_ALARMS: {
            json += "{\"w\":\"ALARMS\",\"a\":[";
            bool fa = true;
            if (apRecent && g.autoRelease) { json += "\"AUTO REL\""; fa = false; }
            if (apRecent && g.offCourse)   { if (!fa) json += ","; json += "\"OFF COURSE\""; fa = false; }
            if (apRecent && g.windShift)   { if (!fa) json += ","; json += "\"WIND SHIFT\""; fa = false; }
            if (apRecent && g.largeXTE)    { if (!fa) json += ","; json += "\"LARGE XTE\""; fa = false; }
            if (apRecent && g.noData)      { if (!fa) json += ","; json += "\"NO DATA\""; fa = false; }
            json += "],\"sim\":" + String(_hdgActive ? "true" : "false")
                 + ",\"drift\":" + String(_hdgAutoDrift ? "true" : "false") + "}";
            break;
        }
        case W_TARGET: {
            int s = 0;
            if (g.mode >= 2 && g.targetHeading >= 0) {
                if (g.heading >= 0) {
                    int diff = (g.targetHeading - g.heading + 540) % 360 - 180;
                    if (abs(diff) <= 3) s = 3;
                    else s = g.turnDirection == 2 ? 2 : 1;
                } else s = g.turnDirection;
            }
            json += "{\"w\":\"TRG\",\"t\":" + String((g.mode >= 2 && g.targetHeading >= 0) ? g.targetHeading : -1)
                 + ",\"s\":" + String(s) + "}";
            break;
        }
        }
    }
    json += "],";
    json += "\"msgs\":[";
    int lines = min(g.msgLogCount, 200);
    int newestIdx = (g.msgLogWriteIdx - 1 + MSG_LOG_SIZE) % MSG_LOG_SIZE;
    for (int i = 0; i < lines; i++) {
        if (i > 0) json += ",";
        int idx = (newestIdx - i + MSG_LOG_SIZE) % MSG_LOG_SIZE;
        json += "\"" + String(g.msgLog[idx]) + "\"";
    }
    json += "]}";
    server.send(200, "application/json", json);
}

static void setupWebServer() {
    server.on("/", handleRoot);
    server.on("/status", handleStatus);
    server.on("/api/heading", []() {
        String deg = server.arg("deg");
        if (deg.length() == 0) {
            server.send(400, "application/json", "{\"error\":\"missing deg\"}");
            return;
        }
        int h = constrain(deg.toInt(), 0, 359);
        String interval = server.arg("interval");
        if (interval.length() > 0)
            _hdgInterval = constrain(interval.toInt(), 200, 10000);
        _hdgDeg = h;
        _hdgActive = true;
        _hdgAutoDrift = (server.arg("drift") != "0");
        _lastHdgTx = 0;
        _hdgFrac = 0;
        gBtnsDrawn = false;
        char desc[MSG_TEXT_LEN], line[MSG_TEXT_LEN];
        snprintf(desc, sizeof(desc), "> HDG %d", h);
        uint8_t dg[] = {0x9C};
        buildLogLine(line, MSG_TEXT_LEN, millis(), dg, 1, desc);
        g.pushMsg(line);
        String json = "{\"status\":\"active\",\"heading\":" + String(h)
                    + ",\"interval\":" + String(_hdgInterval)
                    + ",\"msg\":\"0x9C\"}";
        server.send(200, "application/json", json);
    });
    server.on("/api/heading/stop", []() {
        _hdgActive = false;
        _hdgDeg = 0;
        _hdgFrac = 0;
        gBtnsDrawn = false;
        server.send(200, "application/json", "{\"status\":\"stopped\"}");
    });
    server.on("/api/heading/adj", []() {
        String d = server.arg("d");
        int delta = constrain(d.toInt(), -10, 10);
        _hdgDeg = (_hdgDeg + 360 + delta) % 360;
        _hdgFrac = 0;
        gBtnsDrawn = false;
        char desc[MSG_TEXT_LEN], line[MSG_TEXT_LEN];
        snprintf(desc, sizeof(desc), "> HDG %d", _hdgDeg);
        uint8_t dg[] = {0x9C};
        buildLogLine(line, MSG_TEXT_LEN, millis(), dg, 1, desc);
        g.pushMsg(line);
        server.send(200, "application/json", "{\"status\":\"ok\",\"heading\":" + String(_hdgDeg) + "}");
    });
    server.on("/api/autopilot", []() {
        String cmd = server.arg("cmd");
        for (int i = 0; i < NUM_BUTTONS; i++) {
            if (cmd.equalsIgnoreCase(btns[i].api)) {
                pressAutopilotButton(btns[i]);
                char buf[64];
                snprintf(buf, sizeof(buf), "{\"status\":\"ok\",\"cmd\":\"%s\"}", btns[i].label);
                server.send(200, "application/json", buf);
                return;
            }
        }
        // WIND and TRACK — not in touch buttons, handled only via web
        if (cmd.equalsIgnoreCase("wind")) {
            uint8_t dg[] = {0x86, 0x21, 0x23, 0xDC};
            if (sendDatagram(dg, 4)) {
                char line[MSG_TEXT_LEN];
                buildLogLine(line, MSG_TEXT_LEN, millis(), dg, 4, "> WIND");
                g.pushMsg(line);
                logDatagram(line);
                server.send(200, "application/json", "{\"status\":\"ok\",\"cmd\":\"WIND\"}");
            } else {
                server.send(500, "application/json", "{\"error\":\"TX fail\"}");
            }
            return;
        }
        if (cmd.equalsIgnoreCase("track")) {
            uint8_t dg[] = {0x86, 0x21, 0x28, 0xD7};
            if (sendDatagram(dg, 4)) {
                char line[MSG_TEXT_LEN];
                buildLogLine(line, MSG_TEXT_LEN, millis(), dg, 4, "> TRACK");
                g.pushMsg(line);
                logDatagram(line);
                server.send(200, "application/json", "{\"status\":\"ok\",\"cmd\":\"TRACK\"}");
            } else {
                server.send(500, "application/json", "{\"error\":\"TX fail\"}");
            }
            return;
        }
        server.send(400, "application/json", "{\"error\":\"unknown cmd\"}");
    });
    server.on("/api/logs/clear", HTTP_GET, []() {
        if (logFile) { logFile.close(); logFile = (File)nullptr; }
        int deleted = 0;
        File root = SD.open("/");
        if (root) {
            File entry = root.openNextFile();
            while (entry) {
                if (!entry.isDirectory()) {
                    String name = String(entry.name());
                    if (name.endsWith(".TXT")) { entry.close(); SD.remove("/" + name); deleted++; }
                    else entry.close();
                } else entry.close();
                entry = root.openNextFile();
            }
            root.close();
        }
        logNumber = findNextLogNumber();
        char buf[32];
        snprintf(buf, sizeof(buf), "/LOG_%04d.TXT", logNumber);
        logFile = SD.open(buf, FILE_WRITE);
        if (logFile) logFile.println(buf + 1);
        server.send(200, "application/json", "{\"status\":\"ok\",\"deleted\":" + String(deleted) + "}");
    });
    server.on("/logs", HTTP_GET, []() {
        String html = R"log(<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1"><style>
body{background:#0a0a0a;color:#ccc;font-family:'Courier New',monospace;margin:0;padding:10px;font-size:clamp(14px,1.6vw,20px)}
a{color:#d9a040;text-decoration:none}a:hover{color:#fff}
h2{color:#6a8ba8;margin:4px 0}ul{list-style:none;padding:0}
li{padding:3px 0}span{color:#555}.nav{margin-top:8px;font-size:.85em}
</style></head><body><h2>Log Files</h2><form style="margin:6px 0" onsubmit="return confirm('Delete all log files?')" action="/api/logs/clear" method="get"><button style="background:#410010;color:#f88;border:0;padding:4px 12px;border-radius:3px;font-family:monospace;font-size:inherit;cursor:pointer">Delete All Logs</button></form><ul>)log";
        String names[200];
        int count = 0;
        File root = SD.open("/");
        if (root) {
            File entry = root.openNextFile();
            while (entry && count < 200) {
                if (!entry.isDirectory()) {
                    String name = String(entry.name());
                    if (name.endsWith(".TXT")) names[count++] = name;
                }
                entry.close();
                entry = root.openNextFile();
            }
            root.close();
        }
        // Sort descending (newest first)
        for (int i = 0; i < count - 1; i++) {
            for (int j = i + 1; j < count; j++) {
                if (names[j] > names[i]) { String t = names[i]; names[i] = names[j]; names[j] = t; }
            }
        }
        for (int i = 0; i < count; i++) {
            File f = SD.open("/" + names[i]);
            int sz = f ? f.size() : 0; if (f) f.close();
            html += "<li><a href='/api/download?file=" + names[i] + "'>" + names[i] + "</a> <span>(" + String(sz) + " bytes)</span></li>";
        }
        html += "</ul><div class='nav'><a href='/'>Back to Helm</a></div></body></html>";
        server.send(200, "text/html", html);
    });
    server.on("/api/download", HTTP_GET, []() {
        if (!server.hasArg("file")) { server.send(400, "text/plain", "Missing file"); return; }
        String path = "/" + server.arg("file");
        File f = SD.open(path, FILE_READ);
        if (!f) { server.send(404, "text/plain", "Not found"); return; }
        server.streamFile(f, "text/plain");
        f.close();
    });
    server.on("/api/files", HTTP_GET, []() {
        String json = "[";
        bool first = true;
        File root = SD.open("/");
        if (root) {
            File entry = root.openNextFile();
            while (entry) {
                if (!entry.isDirectory()) {
                    String name = String(entry.name());
                    if (name.endsWith(".TXT")) {
                        if (!first) json += ",";
                        first = false;
                        json += "{\"name\":\"" + name + "\",\"size\":" + String(entry.size()) + "}";
                    }
                }
                entry.close();
                entry = root.openNextFile();
            }
            root.close();
        }
        json += "]";
        server.send(200, "application/json", json);
    });
    server.on("/manifest.json", HTTP_GET, []() {
        String json = "{";
        json += "\"name\":\"BenchPilot\",";
        json += "\"short_name\":\"BenchPilot\",";
        json += "\"display\":\"standalone\",";

        json += "\"icons\":[";
        json += "{\"src\":\"/icon-192.png\",\"sizes\":\"192x192\",\"type\":\"image/png\",\"purpose\":\"maskable\"},";
        json += "{\"src\":\"/icon-512.png\",\"sizes\":\"512x512\",\"type\":\"image/png\",\"purpose\":\"maskable\"}";
        json += "],";
        json += "\"background_color\":\"#000000\",";
        json += "\"theme_color\":\"#000000\"";
        json += "}";
        server.send(200, "application/json", json);
    });
    server.on("/icon.svg", HTTP_GET, []() {
        server.send(200, "image/svg+xml",
            "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 192 192'>"
            "<rect width='192' height='192' rx='32' fill='#0a0a0a'/>"
            "<text x='96' y='140' text-anchor='middle' font-size='128' font-weight='bold' font-family='monospace' fill='#0ff'>B</text>"
            "</svg>");
    });
    server.on("/icon-192.png", HTTP_GET, []() {
        server.send(200, "image/png", String((const char*)ICON_192, sizeof(ICON_192)));
    });
    server.on("/icon-512.png", HTTP_GET, []() {
        server.send(200, "image/png", String((const char*)ICON_512, sizeof(ICON_512)));
    });
    server.onNotFound([]() { handleRoot(); });
    server.begin();
}


//======================================================================
//10. SETUP & LOOP
//======================================================================

void setup() {
    pinMode(TFT_BL_PIN, OUTPUT);
    digitalWrite(TFT_BL_PIN, HIGH);

    tft.init();
    tft.setRotation(1);

    tsi.begin();
    touchscreen.begin(&tsi);
    touchscreen.setRotation(1);
    if (!initSD()) {
        tft.setTextSize(1);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.drawString("SD FAIL", 260, 4, 1);
    }

    drawStatusBar();

    WiFi.setHostname("benchpilot");
    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    dnsServer.start(53, "*", WiFi.softAPIP());

    ArduinoOTA.onStart([]() {
        gOtaActive = true;
        tft.fillScreen(TFT_BLACK);
        tft.setTextSize(2);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawCentreString("OTA Update", 160, 70, 2);
        tft.drawRect(30, 115, 260, 16, TFT_CYAN);
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int otaTotal) {
        int pct = (progress * 100L) / otaTotal;
        tft.fillRect(30, 115, (progress * 260L) / otaTotal, 16, TFT_CYAN);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawCentreString(String(pct) + "%", 160, 138, 2);
    });
    ArduinoOTA.onEnd([]() {
        gOtaActive = false;
        invalidateBands();
        tft.fillScreen(TFT_BLACK);
        tft.setTextSize(2);
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.drawCentreString("Done!", 160, 80, 2);
    });
    ArduinoOTA.onError([](ota_error_t error) {
        gOtaActive = false;
        invalidateBands();
        tft.fillScreen(TFT_BLACK);
        tft.setTextSize(1);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.drawCentreString("OTA Error: " + String(error), 160, 120, 1);
    });
    MDNS.begin("benchpilot");
    MDNS.addService("http", "tcp", 80);
    ArduinoOTA.setHostname("benchpilot");
    ArduinoOTA.begin();

    Seatalk.begin(SEATALK_BAUD, SERIAL_8E1, SEATALK_RX_PIN, 22);
    uart_set_line_inverse(UART_NUM_2, UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV);
    REG_CLR_BIT(UART_CONF0_REG(UART_NUM_2), UART_ERR_WR_MASK);

    setupWebServer();
    playMelody(STARTUP_MELODY, STARTUP_MELODY_LEN);
    g.tftRefresh = 0;
    updateDisplay();
}

void loop() {
    ArduinoOTA.handle();
    server.handleClient();
    dnsServer.processNextRequest();

    if (WiFi.status() == WL_CONNECTED) {
        if (!wifiConnected) {
            wifiConnected = true;
            drawStatusBar();
        }
    } else if (wifiConnected) {
        wifiConnected = false;
        drawStatusBar();
        WiFi.reconnect();
    }

    sendHeading();

    if (!gLogView && _driftFlashIdx >= 0) {
        flashButton(_driftFlashIdx);
        _driftFlashIdx = -1;
    }

    if (!_txActive) {
        while (Seatalk.available() > 0) {
            uint8_t b = Seatalk.read();
            totalBytes++;
            lastPacketTime = millis();
            _lastBusy = lastPacketTime;
            processByte(b);
        }
    }

    updateSpeaker();

    if (touchscreen.tirqTouched() && touchscreen.touched(&tsi)) {
        TS_Point p = touchscreen.getPoint(&tsi);
        touchX = map(p.x, 200, 3700, 1, 320);
        touchY = map(p.y, 240, 3800, 1, 240);
        unsigned long now = millis();
        if (now - lastBtnPress >= TOUCH_DEBOUNCE_MS) {
            if (gLogView) {
                if (touchX < LOG_SCROLL_LEFT || touchX >= LOG_SCROLL_RIGHT) {
                    logViewScroll((touchY < LOG_SCROLL_MID) ? 1 : -1);
                } else {
                    gLogView = false;
                    invalidateBands();
                    tft.fillScreen(TFT_BLACK);
                }
                lastBtnPress = now;
            } else {
                bool handled = false;
                for (int i = 0; i < NUM_BUTTONS; i++) {
                    auto &b = btns[i];
                    if (touchX >= b.x && touchX < b.x + b.w &&
                        touchY >= b.y && touchY < b.y + b.h) {
                        if (_hdgActive && (b.cmd == 0x23 || b.cmd == 0x28)) continue;
                        flashButton(i);
                        lastBtnPress = now;
                        if (b.cmd == 0xff) {
                            if (_hdgActive && _hdgAutoDrift) stopHdg_int();
                            else simHdg_auto(g.heading >= 0 ? g.heading : 180);
                        } else if (b.cmd == 0xF9) {
                            if (_hdgActive && !_hdgAutoDrift) stopHdg_int();
                            else simHdg_fixed(g.heading >= 0 ? g.heading : 180);
                        } else if (b.cmd == 0xFE) {
                            _hdgDeg = (_hdgDeg + 350) % 360; _hdgFrac = 0; gBtnsDrawn = false;
                        } else if (b.cmd == 0xFD) {
                            _hdgDeg = (_hdgDeg + 359) % 360; _hdgFrac = 0; gBtnsDrawn = false;
                        } else if (b.cmd == 0xFC) {
                            _hdgDeg = (_hdgDeg + 1) % 360; _hdgFrac = 0; gBtnsDrawn = false;
                        } else if (b.cmd == 0xFB) {
                            _hdgDeg = (_hdgDeg + 10) % 360; _hdgFrac = 0; gBtnsDrawn = false;
                        } else {
                            pressAutopilotButton(b);
                        }
                        handled = true;
                        break;
                    }
                }
                if (!handled && touchY < LOG_TAP_BOUNDARY) {
                    gLogView = true;
                    gLogScroll = 0;
                    gLogLastWriteIdx = -1;
                    tft.fillScreen(TFT_BLACK);
                    lastBtnPress = now;
                }
            }
        }
    }

    if (_flashIdx >= 0 && millis() > _flashEnd) {
        drawAllButtons();
        _flashIdx = -1;
    }
    updateDisplay();
}
