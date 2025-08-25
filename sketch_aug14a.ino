/********************************************************
  ESP32 - 8 Relay + RainMaker + EEPROM + TFT ST7735 UI
  - 8x Relay (active LOW)
  - TFT ST7735: Adafruit_GFX + Adafruit_ST7735
  - Indikator: Relay (Hijau=ON, Merah=OFF), WiFi (Hijau/Merah)
*********************************************************/

#include <EEPROM.h>
#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// ========= KONFIG EEPROM =========
#define ENABLE_EEPROM true
#define EEPROM_SIZE 16

// ========= PIN TFT (sesuai wiring kamu) =========
#define TFT_MISO 19  // tidak terpasang -1
#define TFT_MOSI 23  // pin (SDA)
#define TFT_SCLK 18  // pin SCK/SCLK
#define TFT_CS    5  // pin Chip select control pin
#define TFT_DC    2  // pin (AO )Data Command control pin (AO)
#define TFT_RST   4  // Reset pin (could connect to RST pin)

// Gunakan hardware SPI (VSPI: SCLK=18, MOSI=23, MISO=19)
Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);

// Pilih tab sesuai modulmu: INITR_BLACKTAB / INITR_GREENTAB / INITR_REDTAB
#define TFT_TAB_INIT INITR_BLACKTAB

// ========= RAINMAKER =========
const char *service_name = "PROV_12345";
const char *pop = "1234567";

// Nama device
char deviceName_1[] = "Switch1";
char deviceName_2[] = "Switch2";
char deviceName_3[] = "Switch3";
char deviceName_4[] = "Switch4";
char deviceName_5[] = "Switch5";
char deviceName_6[] = "Switch6";
char deviceName_7[] = "Switch7";
char deviceName_8[] = "Switch8";

// ========= GPIO =========
// Relay dipindah agar tidak bentrok dengan SPI TFT
uint8_t RelayPin[8] = { 21, 22, 27, 14, 13, 33, 32, 25 };

// LED WiFi pindah ke GPIO16 (HIGH = ON)
uint8_t wifiLed = 16;

// Tombol reset (LOW aktif). Pakai internal pull-up agar default HIGH
uint8_t gpio_reset = 0;

// State relay (true=ON)
bool toggleState[8] = { LOW };

Switch *my_switch[8];

// ========= Helper EEPROM =========
void writeEEPROM(int addr, bool state) {
  if (ENABLE_EEPROM) {
    EEPROM.write(addr, state ? 1 : 0);
    EEPROM.commit();
    Serial.printf("EEPROM saved: addr %d = %d\n", addr, state);
  }
}

bool readEEPROM(int addr) {
  if (ENABLE_EEPROM) {
    return EEPROM.read(addr) == 1;
  }
  return false;
}

// ========= TFT UI =========
const int SCREEN_W = 160; // landscape (setRotation(1))
const int SCREEN_H = 128;

uint16_t colorOn  = ST77XX_GREEN;
uint16_t colorOff = ST77XX_RED;
uint16_t colorBg  = ST77XX_BLACK;
uint16_t colorFg  = ST77XX_WHITE;

void drawHeader() {
  tft.fillScreen(colorBg);
  tft.setTextWrap(false);
  tft.setTextColor(colorFg);
  tft.setTextSize(2);
  tft.setCursor(4, 4);
  tft.print("8 Relay Status");
  // Garis bawah header
  tft.drawLine(0, 22, SCREEN_W-1, 22, ST77XX_WHITE);
}

void drawWiFiStatus(bool connected) {
  // Area WiFi di kanan header
  int x = SCREEN_W - 78; // area kotak 74x18
  int y = 2;
  tft.fillRect(x, y, 74, 18, colorBg);
  tft.drawRect(x, y, 74, 18, ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(x + 4, y + 5);
  tft.print("WiFi:");
  tft.setCursor(x + 34, y + 5);
  if (connected) {
    tft.setTextColor(colorOn);
    tft.print("Connected");
  } else {
    tft.setTextColor(colorOff);
    tft.print("No Link");
  }
  tft.setTextColor(colorFg);
}

void drawRelayTile(uint8_t idx, bool state) {
  // grid 4 kolom x 2 baris
  // tile size
  const int cols = 4;
  const int tileW = 38;      // ~ (160- (4+1)*4)/4 disederhanakan
  const int tileH = 40;
  const int marginX = 6;     // kiri-kanan margin kecil
  const int marginY = 26;    // di bawah header
  const int gapX = 4;
  const int gapY = 6;

  uint8_t col = idx % cols;
  uint8_t row = idx / cols;

  int x = marginX + col * (tileW + gapX);
  int y = marginY + row * (tileH + gapY);

  // Background tile
  tft.fillRoundRect(x, y, tileW, tileH, 6, ST77XX_BLUE);
  tft.drawRoundRect(x, y, tileW, tileH, 6, ST77XX_WHITE);

  // Label
  tft.setTextSize(1);
  tft.setCursor(x + 6, y + 4);
  tft.setTextColor(colorFg);
  tft.print("R");
  tft.print(idx + 1);

  // Lampu indikator (lingkaran)
  int cx = x + tileW/2;
  int cy = y + tileH/2 + 6;
  uint16_t c = state ? colorOn : colorOff;
  tft.fillCircle(cx, cy, 8, c);
  tft.drawCircle(cx, cy, 8, ST77XX_WHITE);
}

void drawAllRelayTiles() {
  for (int i = 0; i < 8; i++) {
    drawRelayTile(i, toggleState[i]);
  }
}

void updateRelayTile(uint8_t idx, bool state) {
  // Redraw hanya tile tertentu
  drawRelayTile(idx, state);
}

// ========= Relay Control =========
void setRelay(uint8_t pin, int addr, bool state) {
  // Active LOW relay board: LOW = ON, HIGH = OFF
  digitalWrite(pin, state ? LOW : HIGH);
  writeEEPROM(addr, state);
}

// ========= RainMaker Write Callback =========
void write_callback(Device *device, Param *param, const param_val_t val,
                    void *priv_data, write_ctx_t *ctx) {
  const char *device_name = device->getDeviceName();
  const char *param_name = param->getParamName();

  if (strcmp(param_name, "Power") == 0) {
    bool newState = val.val.b;
    for (int i = 0; i < 8; i++) {
      if (strcmp(device_name, my_switch[i]->getDeviceName()) == 0) {
        setRelay(RelayPin[i], i, newState);
        toggleState[i] = newState;
        my_switch[i]->updateAndReportParam(param_name, newState);
        updateRelayTile(i, newState); // ====> update TFT
        Serial.printf("Write callback: Relay %d = %d\n", i + 1, newState);
        break;
      }
    }
  }
}

// ========= Provisioning Events =========
void sysProvEvent(arduino_event_t *sys_event) {
  switch (sys_event->event_id) {
    case ARDUINO_EVENT_PROV_START:
      Serial.printf("Provisioning started: %s\n", service_name);
      printQR(service_name, pop, "ble");
      break;

    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("Connected to Wi-Fi");
      digitalWrite(wifiLed, HIGH);
      drawWiFiStatus(true); // ====> update TFT
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("Disconnected from Wi-Fi");
      digitalWrite(wifiLed, LOW);
      drawWiFiStatus(false); // ====> update TFT
      break;

    default:
      break;
  }
}

// ========= Setup =========
void setup() {
  Serial.begin(115200);
  if (ENABLE_EEPROM) EEPROM.begin(EEPROM_SIZE);

  // TFT init
  tft.initR(TFT_TAB_INIT);
  tft.setRotation(1); // 1 = landscape
  drawHeader();
  drawWiFiStatus(WiFi.status() == WL_CONNECTED);

  // Relay GPIO
  for (int i = 0; i < 8; i++) {
    pinMode(RelayPin[i], OUTPUT);
    // Ambil state awal dari EEPROM
    toggleState[i] = readEEPROM(i);
    // Terapkan ke relay (active LOW)
    setRelay(RelayPin[i], i, toggleState[i]);
  }
  drawAllRelayTiles();

  // LED WiFi & tombol reset
  pinMode(wifiLed, OUTPUT);
  digitalWrite(wifiLed, (WiFi.status() == WL_CONNECTED) ? HIGH : LOW);
  pinMode(gpio_reset, INPUT_PULLUP); // gunakan pull-up internal

  // RainMaker Setup
  Node my_node = RMaker.initNode("Madhonnnnn");

  char *deviceNames[] = {
    deviceName_1, deviceName_2, deviceName_3, deviceName_4,
    deviceName_5, deviceName_6, deviceName_7, deviceName_8
  };

  for (int i = 0; i < 8; i++) {
    my_switch[i] = new Switch(deviceNames[i], &RelayPin[i]);
    my_switch[i]->addCb(write_callback);
    my_node.addDevice(*my_switch[i]);
    my_switch[i]->updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState[i]);
  }

  RMaker.enableOTA(OTA_USING_PARAMS);
  RMaker.enableTZService();
  RMaker.enableSchedule();

  RMaker.start();
  WiFi.onEvent(sysProvEvent);
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM,
                          WIFI_PROV_SECURITY_1, pop, service_name);

  Serial.println("Setup selesai: 8 relay + TFT status + EEPROM (tanpa tombol manual).");
}

// ========= Loop =========
void loop() {
  // Tombol reset Wi-Fi / Factory (aktif LOW)
  if (digitalRead(gpio_reset) == LOW) {
    delay(100);
    unsigned long startTime = millis();
    while (digitalRead(gpio_reset) == LOW) delay(50);
    unsigned long duration = millis() - startTime;
    if (duration > 10000) {
      Serial.println("Factory reset triggered.");
      RMakerFactoryReset(2);
    } else if (duration > 3000) {
      Serial.println("WiFi reset triggered.");
      RMakerWiFiReset(2);
    }
  }

  // Auto reconnect Wi-Fi jika terputus
  static bool lastWiFi = false;
  bool nowWiFi = (WiFi.status() == WL_CONNECTED);
  if (!nowWiFi) {
    static unsigned long lastTry = 0;
    if (millis() - lastTry > 2000) {
      Serial.println("Wi-Fi terputus, mencoba reconnect...");
      WiFi.reconnect();
      lastTry = millis();
    }
  }
  // LED + TFT WiFi indikator (hanya update saat berubah)
  if (nowWiFi != lastWiFi) {
    digitalWrite(wifiLed, nowWiFi ? HIGH : LOW);
    drawWiFiStatus(nowWiFi);
    lastWiFi = nowWiFi;
  }

  // Kirim status refresh tiap 60 detik ke cloud
  static unsigned long lastReport = 0;
  if (millis() - lastReport > 60000) {
    for (int i = 0; i < 8; i++) {
      my_switch[i]->updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, toggleState[i]);
    }
    lastReport = millis();
    Serial.println("Status refresh ke cloud (Google & Alexa).");
  }
}
