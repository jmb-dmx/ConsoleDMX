/*
  Main project file for the ConsoleDMX AIO on ESP32 TTGO T-Display.
  This file is a port of the original ESP8266-based project.
*/

// --- Core ESP32 & WiFi Libraries ---
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <LittleFS.h>
#include <Preferences.h> // For storing configuration (replaces EEPROM on ESP32)

// --- DMX Libraries ---
#include "esp_dmx.h"      // ESP32 native DMX driver
#include <ArtnetWiFi.h>   // Using hideakitai/ArtNet v1.5.1 or later
#include <ESPAsyncE131.h>

// --- Other Libraries ---
#include <esp_now.h>
#include <FastLED.h>
#include <ArduinoJson.h>
#include <DNSServer.h>

// --- TTGO T-Display Specific ---
#include <TFT_eSPI.h>
#include <SPI.h>

TFT_eSPI tft = TFT_eSPI();

// =================================================================
// --- PINOUT & HARDWARE CONFIGURATION ---
// =================================================================

// DMX Configuration (using UART2)
const dmx_port_t dmx_port = DMX_NUM_2;
const int dmx_tx_pin = 17;
const int dmx_rx_pin = 16;
const int dmx_rts_pin = 21; // EN pin for MAX485

// LED Strip Configuration
#define NUM_LEDS 170
#define LED_DATA_PIN 22 // Example pin, adjust if needed
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB

// =================================================================
// --- GLOBAL CONSTANTS & DEFINITIONS ---
// =================================================================

#define DMX_CHANNELS 512
#define NUM_FADERS 96
#define MAX_SCENES 12
#define MAX_CHASER_STEPS 8
#define MAX_CHASER_PRESETS 4
#define MAX_XY_ASSIGN 16

const uint32_t CONFIG_MAGIC = 0xD0F2CCEE; // Magic number for config validation

enum Priority : uint8_t { PRIO_FADERS = 0, PRIO_ARTNET = 1, PRIO_HTP = 2 };

// =================================================================
// --- GLOBAL VARIABLES & DATA STRUCTURES ---
// =================================================================

// --- System & Network ---
WebServer server(80);
WebSocketsServer webSocket(81);
DNSServer dnsServer;
ArtnetWiFi artnet;
ESPAsyncE131 sacn(1); // Listen for 1 universe
e131_packet_t packet; // For sACN packet handling
Preferences preferences;
bool wifiAPMode = false;
uint32_t restartAt = 0;

// --- DMX Data Buffers ---
byte dmx_data[DMX_CHANNELS]; // Final merged output buffer
byte faderValues[DMX_CHANNELS];
byte artnetValues[DMX_CHANNELS];
byte sacnValues[DMX_CHANNELS]; // Added for sACN
byte scenesOut[DMX_CHANNELS];
byte outValues[DMX_CHANNELS]; // Final values before applying XY/Blackout
byte savedOut[DMX_CHANNELS]; // For blackout restore

// --- Scene Management ---
uint8_t sceneLevels[MAX_SCENES];
uint8_t sceneNonEmpty[MAX_SCENES];
uint8_t sceneDataCache[MAX_SCENES][DMX_CHANNELS];

// --- Chaser ---
bool chaser_running = false;
bool chaserPresetNonEmpty[MAX_CHASER_PRESETS] = {false};
uint8_t chaser_scenes[MAX_CHASER_STEPS] = {0, 1, 2, 3, 0, 0, 0, 0};
int chaser_bpm = 120;
int chaser_fade_ms = 0;
int chaser_step_duration_ms = 500;
uint8_t chaser_current_step = 0;
unsigned long chaser_last_step_time = 0;
bool chaser_is_fading = false;
unsigned long fade_start_time = 0;
uint8_t chaser_out_values[DMX_CHANNELS];
uint8_t fade_from_values[DMX_CHANNELS];
uint8_t fade_to_values[DMX_CHANNELS];

// --- XY Pad / Moving Heads (Robo) ---
struct XY_Channel_t {
  uint16_t ch;
  bool inverted;
};
XY_Channel_t xy_pad_x_channels[MAX_XY_ASSIGN];
XY_Channel_t xy_pad_y_channels[MAX_XY_ASSIGN];
uint8_t num_x_channels = 0;
uint8_t num_y_channels = 0;
uint8_t xy_pad_x_value = 127;
uint8_t xy_pad_y_value = 127;

// --- Blackout ---
bool blackoutActive = false;
bool ignoreBlackout[DMX_CHANNELS];

// --- LED Strip ---
CRGB leds[NUM_LEDS];
uint8_t ledArtnetValues[DMX_CHANNELS];

// --- Fader Customization ---
String faderNames[NUM_FADERS];
String faderColors[NUM_FADERS];

// --- Home Assistant ---
bool haAssignedChannels[DMX_CHANNELS];

// --- ESP-NOW ---
bool espNowReady = false;
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// --- Configuration Struct ---
struct Config {
  char ssid[32];
  char pass[64];
  uint8_t universe;
  uint8_t priority;
  uint8_t ap_only;
  uint8_t use_static_ip;
  char static_ip[16];
  char static_subnet[16];
  char static_gateway[16];
  char nodeName[32];
  uint8_t dmx_protocol;
  uint8_t espnow_mode;
  uint8_t espnow_channel;
  uint8_t espnow_intensity_only;
  uint8_t led_artnet_enable;
  uint8_t led_universe;
  uint8_t led_color_order;
};
Config cfg;

// =================================================================
// --- FUNCTION PROTOTYPES ---
// =================================================================

// --- Core ---
void applyOutput();
void setup();
void loop();

// --- WiFi & Web ---
void handleRoot();
void handleWifiPost();
void handleFactoryReset();
void sendInit(uint8_t num);
bool tryWiFiSTA(uint32_t timeoutMs);
void startAP();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length);
bool handleFileRead(String path);

// --- Config (Preferences) ---
void setDefaults(Config& c);
void loadConfig();
void saveConfig();

// --- File System (LittleFS) ---
bool fsInit();
String scenePath(uint8_t idx);
void loadAllScenesFromFS();
// ... other FS related functions will be ported later

// --- DMX & Protocols ---
void setupDMX();
void sendDMX();
void onArtNetDmx(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data);
void espNowOnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
void sendEspNowBlock(const uint8_t* data, uint16_t len_total);

// --- Logic ---
void updateDisplay() {
  tft.fillScreen(TFT_BLACK);
  tft.setCursor(0, 0);
  tft.setTextSize(2);
  tft.setTextColor(TFT_CYAN);
  tft.println("ConsoleDMX");

  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(1);

  if (wifiAPMode) {
    tft.println("Mode: AP");
    tft.print("SSID: ");
    tft.println(makeAPSSID());
  } else {
    tft.println("Mode: STA");
    tft.print("SSID: ");
    tft.println(cfg.ssid);
  }
  tft.print("IP: ");
  tft.println(wifiAPMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString());

  tft.println(""); // Spacer

  String proto = "DMX: ";
  switch(cfg.dmx_protocol) {
    case 1: proto += "Art-Net"; break;
    case 2: proto += "sACN"; break;
    default: proto += "Off"; break;
  }
  tft.println(proto);
  if (cfg.dmx_protocol > 0) {
    tft.print("Univers: ");
    tft.println(cfg.universe);
  }
}

void chaserTask() {
  if (!chaser_running) return;
  unsigned long now = millis();
  unsigned long beat_duration_ms = (chaser_bpm > 0) ? (60000 / chaser_bpm) : 1000;
  unsigned long step_duration = (chaser_step_duration_ms > 0) ? chaser_step_duration_ms : beat_duration_ms;

  if (chaser_is_fading) {
    float progress = (float)(now - fade_start_time) / (float)chaser_fade_ms;
    if (progress >= 1.0) {
      progress = 1.0;
      chaser_is_fading = false;
    }
    for (int i = 0; i < DMX_CHANNELS; i++) {
      chaser_out_values[i] = (uint8_t)((float)fade_from_values[i] + ((float)fade_to_values[i] - (float)fade_from_values[i]) * progress);
    }
    if (!chaser_is_fading) {
      memcpy(chaser_out_values, fade_to_values, DMX_CHANNELS);
    }
  } else if (now - chaser_last_step_time >= step_duration) {
    chaser_last_step_time = now;
    memcpy(fade_from_values, chaser_out_values, DMX_CHANNELS);

    int next_step = chaser_current_step;
    int loop_guard = 0;
    do {
      next_step = (next_step + 1) % MAX_CHASER_STEPS;
      if (loop_guard++ > MAX_CHASER_STEPS) {
        chaser_running = false; // All steps are invalid, stop chaser
        return;
      }
    } while (chaser_scenes[next_step] > MAX_SCENES);

    chaser_current_step = next_step;
    String step_msg = "chaser_step:" + String(chaser_current_step);
    webSocket.broadcastTXT(step_msg);

    uint8_t next_scene_idx = chaser_scenes[chaser_current_step];
    if (next_scene_idx == MAX_SCENES) { // "Black" scene
      memset(fade_to_values, 0, DMX_CHANNELS);
    } else if (sceneNonEmpty[next_scene_idx]) {
      memcpy(fade_to_values, sceneDataCache[next_scene_idx], DMX_CHANNELS);
    } else { // Scene is empty
      memset(fade_to_values, 0, DMX_CHANNELS);
    }

    if (chaser_fade_ms > 0) {
      chaser_is_fading = true;
      fade_start_time = now;
    } else {
      memcpy(chaser_out_values, fade_to_values, DMX_CHANNELS);
    }
  }
  applyOutput();
}

// ... other logic functions will be ported later
  for (int s = 0; s < MAX_SCENES; s++) {
    if (sceneLevels[s] == 0) continue;

    // Pointer to the cached scene data
    uint8_t* tempScene = sceneDataCache[s];

    for (int ch = 0; ch < DMX_CHANNELS; ch++) {
      if (tempScene[ch] == 0) continue;

      // Scale the scene's channel value by the scene's master level
      uint16_t v = (uint16_t)tempScene[ch] * (uint16_t)sceneLevels[s];
      v /= 255;

      // HTP merge with the output of other scenes
      if (v > scenesOut[ch]) {
        scenesOut[ch] = (uint8_t)(v > 255 ? 255 : v);
      }
    }
  }
}

void setBlackout(bool enable) {
  if (enable && !blackoutActive) {
    memcpy(savedOut, outValues, DMX_CHANNELS); // Save the current state before blackout
    blackoutActive = true;
    webSocket.broadcastTXT("ha_blackout:1");
    if (cfg.led_artnet_enable) {
      FastLED.clear();
      FastLED.show();
    }
  } else if (!enable && blackoutActive) {
    blackoutActive = false;
    webSocket.broadcastTXT("ha_blackout:0");
    if (cfg.led_artnet_enable) {
      for (int i = 0; i < NUM_LEDS; i++) {
        int dmx_index = i * 3;
        if (dmx_index + 2 < DMX_CHANNELS) {
          leds[i].setRGB(ledArtnetValues[dmx_index], ledArtnetValues[dmx_index+1], ledArtnetValues[dmx_index+2]);
        }
      }
      FastLED.show();
    }
  }
  applyOutput(); // Apply the new state
}

// =================================================================
// --- SETUP ---
// =================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("\nBooting ConsoleDMX AIO for ESP32...");

  // Initialize Display
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.println("Booting...");

  // Initialize Preferences (EEPROM replacement)
  preferences.begin("console-dmx", false);

  // Load configuration
  loadConfig();

  // Initialize Filesystem
  if (!fsInit()) {
    Serial.println("FS init failed. Halting.");
    while(1) delay(1000);
  }
  // provisionFileSystem(); // This will be implemented in the next step

  // Initialize DMX
  setupDMX();

  // Initialize WiFi
  if (cfg.ap_only) {
    startAP();
  } else if (!tryWiFiSTA(20000)) {
    startAP();
  }

  // TODO: mDNS responder
  // if (MDNS.begin("consoledmx")) {
  //   MDNS.addService("http", "tcp", 80);
  // }

  // Web Server Routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/wifi", HTTP_POST, handleWifiPost);
  server.on("/factory", HTTP_POST, handleFactoryReset);
  // Captive portal handler
  server.on("/generate_204", HTTP_GET, handleRoot);
  server.on("/gen_204", HTTP_GET, handleRoot);
  server.on("/hotspot-detect.html", HTTP_GET, handleRoot);
  server.onNotFound([]() {
    if (!handleFileRead(server.uri())) {
      server.send(404, "text/plain", "Not Found");
    }
  });
  server.begin();
  Serial.println("HTTP server started");

  // WebSocket Server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started");

  // Initialize ArtNet, sACN, ESP-NOW, FastLED based on config
  switch (cfg.dmx_protocol) {
    case 1: // Art-Net
      artnet.begin();
      artnet.setArtPollReplyConfigShortName(String(cfg.nodeName));
      artnet.setArtPollReplyConfigLongName(String(cfg.nodeName));
      artnet.setArtDmxData(onArtNetDmx);
      Serial.println("Protocol: Art-Net enabled.");
      break;
    case 2: // sACN
      if (sacn.begin(E131_MULTICAST, cfg.universe)) {
        Serial.printf("Protocol: sACN enabled, universe %u\n", cfg.universe);
      } else {
        Serial.println("Error: Unable to start sACN.");
      }
      break;
    default:
      Serial.println("Protocol: Network DMX disabled.");
      break;
  }

  // ESP-NOW Init
  if (cfg.espnow_mode > 0) {
    WiFi.mode(WIFI_AP_STA); // ESP-NOW works best in this mode
    if (cfg.espnow_channel > 0 && cfg.espnow_channel <= 14) {
      // To be implemented: esp_wifi_set_channel(cfg.espnow_channel, WIFI_SECOND_CHAN_NONE);
    }
    esp_now_init();
    if (cfg.espnow_mode == 1) { // Master
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, broadcastAddress, 6);
      peerInfo.channel = 0;
      peerInfo.encrypt = false;
      if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        Serial.println("ESP-NOW Master configured, broadcast peer added.");
        espNowReady = true;
      }
    } else { // Slave
      esp_now_register_recv_cb(espNowOnDataRecv);
      Serial.println("ESP-NOW Slave configured.");
      espNowReady = true;
    }
  }

  // FastLED Init
  if (cfg.led_artnet_enable) {
    switch (cfg.led_color_order) {
      case 0: FastLED.addLeds<LED_TYPE, LED_DATA_PIN, GRB>(leds, NUM_LEDS); break;
      case 1: FastLED.addLeds<LED_TYPE, LED_DATA_PIN, RGB>(leds, NUM_LEDS); break;
      case 2: FastLED.addLeds<LED_TYPE, LED_DATA_PIN, BGR>(leds, NUM_LEDS); break;
    }
    FastLED.setBrightness(255);
    Serial.println("FastLED Initialized for Art-Net output.");
  }

  // Initialize data buffers and fader names/colors
  memset(faderValues, 0, sizeof(faderValues));
  memset(artnetValues, 0, sizeof(artnetValues));
  memset(sacnValues, 0, sizeof(sacnValues));
  memset(scenesOut, 0, sizeof(scenesOut));
  memset(outValues, 0, sizeof(outValues));
  memset(savedOut, 0, sizeof(savedOut));
  memset(haAssignedChannels, 0, sizeof(haAssignedChannels));
  memset(ledArtnetValues, 0, sizeof(ledArtnetValues));
  memset(chaser_out_values, 0, sizeof(chaser_out_values));
  for(int i=0; i<NUM_FADERS; i++) {
    faderNames[i] = "CH " + String(i+1);
    faderColors[i] = ""; // Default color
  }

  // Load all data from FS
  loadAllScenesFromFS();
  loadFaderCustoms();
  loadSceneLevels();
  loadActiveChaser();
  loadAssignments();
  // Apply initial state
  recomputeScenes();
  applyOutput();

  updateDisplay();

  Serial.println("Setup complete. Ready.");
}


// =================================================================
// --- LOOP ---
// =================================================================

void loop() {
  webSocket.loop();
  server.handleClient();
  if(wifiAPMode) dnsServer.processNextRequest();

  if (cfg.dmx_protocol == 1) {
    artnet.parse();
  } else if (cfg.dmx_protocol == 2) {
    if (sacn.parsePacket(&packet)) {
        if (packet.universe == cfg.universe) {
            for (int i = 0; i < packet.property_value_count - 1 && i < DMX_CHANNELS; i++) {
                uint8_t val = packet.property_values[i + 1];
                if (val != artnetValues[i] && haAssignedChannels[i]) {
                    String msg = "ha_val:" + String(i + 1) + ":" + String(val);
                    webSocket.broadcastTXT(msg);
                }
                artnetValues[i] = val;
            }
            applyOutput();
        }
    }
  }

  chaserTask();
  // ledTask(); // Placeholder for status LED

  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 2000) { // Update display every 2 seconds
    updateDisplay();
    lastDisplayUpdate = millis();
  }

  static unsigned long lastDmxSend = 0;
  if (millis() - lastDmxSend > 25) { // ~40Hz
    sendDMX();
    lastDmxSend = millis();
  }

  if (restartAt && millis() > restartAt) {
    Serial.println("Restarting as requested...");
    delay(100);
    ESP.restart();
  }
}

// =================================================================
// --- FUNCTION IMPLEMENTATIONS ---
// =================================================================

// --- DMX Functions ---
void setupDMX() {
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_driver_install(dmx_port, &config, NULL, 0);
  dmx_set_pin(dmx_port, dmx_tx_pin, dmx_rx_pin, dmx_rts_pin);
  Serial.println("DMX driver installed.");
}

void sendDMX() {
    byte buffer[DMX_PACKET_SIZE];
    buffer[0] = 0; // DMX Start Code
    memcpy(buffer + 1, dmx_data, DMX_CHANNELS);

    dmx_write(dmx_port, buffer, DMX_CHANNELS + 1);
    dmx_send(dmx_port);
    // dmx_wait_sent is blocking, avoid in loop
}

void onArtNetDmx(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data)
{
  if (universe == cfg.universe) {
    for (int i = 0; i < length && i < DMX_CHANNELS; i++) {
      if (data[i] != artnetValues[i] && haAssignedChannels[i]) {
        String msg = "ha_val:" + String(i + 1) + ":" + String(data[i]);
        webSocket.broadcastTXT(msg);
      }
      artnetValues[i] = data[i];
    }
    applyOutput();
  }

  // Also handle LED strip universe if enabled
  if (cfg.led_artnet_enable && universe == cfg.led_universe) {
    for (int i = 0; i < length && i < DMX_CHANNELS; i++) {
      ledArtnetValues[i] = data[i];
    }
    for (int i = 0; i < NUM_LEDS; i++) {
      int dmx_index = i * 3;
      if (dmx_index + 2 < length) {
        leds[i].setRGB(data[dmx_index], data[dmx_index + 1], data[dmx_index + 2]);
      }
    }
    FastLED.show();
  }
}

void espNowOnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  const int max_data_len = 250 - sizeof(uint16_t); // ESP-NOW max payload is 250
  struct __attribute__((packed)) {
    uint16_t start_channel;
    uint8_t data[max_data_len];
  } packet;

  if (len > sizeof(packet)) return;
  memcpy(&packet, incomingData, len);

  if (packet.start_channel < DMX_CHANNELS) {
    uint16_t data_len = len - sizeof(packet.start_channel);
    if (packet.start_channel + data_len <= DMX_CHANNELS) {
      memcpy(&outValues[packet.start_channel], packet.data, data_len);
      applyOutput(); // Re-run applyOutput to process the new values
    }
  }
}

void sendEspNowBlock(const uint8_t* data, uint16_t len_total) {
  if (!(cfg.espnow_mode == 1 && espNowReady)) return;
  const int max_data_len = 250 - sizeof(uint16_t);
  struct __attribute__((packed)) {
    uint16_t start_channel;
    uint8_t data[max_data_len];
  } packet;

  for (uint16_t i = 0; i < len_total; i += max_data_len) {
    packet.start_channel = i;
    uint16_t len_to_send = len_total - i;
    if (len_to_send > max_data_len) len_to_send = max_data_len;

    memcpy(packet.data, data + i, len_to_send);
    esp_now_send(broadcastAddress, (uint8_t*)&packet, sizeof(packet.start_channel) + len_to_send);
  }
}

// --- Web & WiFi Functions ---
bool handleFileRead(String path) {
  if (path.endsWith("/")) path += "index.html";
  String contentType = "text/html";
  if (path.endsWith(".css")) contentType = "text/css";
  else if (path.endsWith(".js")) contentType = "application/javascript";
  else if (path.endsWith(".ico")) contentType = "image/x-icon";

  if (LittleFS.exists(path)) {
    File file = LittleFS.open(path, "r");
    server.streamFile(file, contentType);
    file.close();
    return true;
  }
  Serial.printf("File not found: %s\n", path.c_str());
  return false;
}

void handleRoot() {
  if(!handleFileRead(server.uri())) {
     server.send(200, "text/plain", "Welcome to ConsoleDMX. Please upload index.html to LittleFS.");
  }
}

void handleWifiPost() {
  bool changed = false;
  if (server.hasArg("ssid")) {
    strncpy(cfg.ssid, server.arg("ssid").c_str(), sizeof(cfg.ssid) - 1);
    changed = true;
  }
  if (server.hasArg("pass")) {
    strncpy(cfg.pass, server.arg("pass").c_str(), sizeof(cfg.pass) - 1);
    changed = true;
  }
  if (server.hasArg("aponly")) {
    cfg.ap_only = (uint8_t)server.arg("aponly").toInt();
    changed = true;
  }
  if (server.hasArg("use_static_ip")) {
     cfg.use_static_ip = (uint8_t)server.arg("use_static_ip").toInt();
     changed = true;
  } else {
    cfg.use_static_ip = 0;
    changed = true;
  }
  if (server.hasArg("static_ip")) {
    strncpy(cfg.static_ip, server.arg("static_ip").c_str(), sizeof(cfg.static_ip) - 1);
    changed = true;
  }
  if (server.hasArg("static_subnet")) {
    strncpy(cfg.static_subnet, server.arg("static_subnet").c_str(), sizeof(cfg.static_subnet) - 1);
    changed = true;
  }
  if (server.hasArg("static_gateway")) {
    strncpy(cfg.static_gateway, server.arg("static_gateway").c_str(), sizeof(cfg.static_gateway) - 1);
    changed = true;
  }

  if (changed) {
    saveConfig();
    server.send(200, "text/html", "<html><body><h3>Enregistré. Redémarrage dans 2s…</h3><script>setTimeout(()=>location.href='/',2000)</script></body></html>");
    restartAt = millis() + 2000;
  } else {
    server.send(200, "text/plain", "Aucun changement.");
  }
}

void handleFactoryReset() {
  Serial.println("Factory reset requested.");
  preferences.clear();

  // For now, just format the filesystem. A more nuanced approach might be needed.
  LittleFS.format();

  server.send(200, "text/html", "<html><body><h3>Remise d’usine effectuée. Redémarrage…</h3><script>setTimeout(()=>location.href='/',3000)</script></body></html>");
  restartAt = millis() + 3000;
}

String makeAPSSID() {
  String s = "ConsoleDMX-";
  uint64_t chipid = ESP.getEfuseMac();
  char chipid_str[13];
  snprintf(chipid_str, sizeof(chipid_str), "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
  s += chipid_str;
  return s;
}

bool tryWiFiSTA(uint32_t timeoutMs) {
  if (strlen(cfg.ssid) == 0) return false;
  WiFi.mode(WIFI_STA);
  // TODO: Handle static IP configuration from cfg
  WiFi.begin(cfg.ssid, cfg.pass);
  Serial.printf("WiFi STA: connecting to \"%s\"...\n", cfg.ssid);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeoutMs) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    wifiAPMode = false;
    Serial.printf("STA OK, IP: %s\n", WiFi.localIP().toString().c_str());
    return true;
  }
  Serial.println("STA connection failed.");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  return false;
}

void startAP() {
  wifiAPMode = true;
  String ap_ssid = makeAPSSID();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid.c_str(), "dmx12345");

  dnsServer.start(53, "*", WiFi.softAPIP());

  IPAddress ip = WiFi.softAPIP();
  Serial.printf("AP \"%s\" started. IP: %s\n", ap_ssid.c_str(), ip.toString().c_str());
}

// --- Config Functions ---
void setDefaults(Config& c) {
  memset(&c, 0, sizeof(c));
  strncpy(c.ssid, "BELL685", sizeof(c.ssid) - 1);
  strncpy(c.pass, "1739EC3C", sizeof(c.pass) - 1);
  c.universe = 0;
  c.priority = PRIO_FADERS;
  c.ap_only = 0;
  c.use_static_ip = 0;
  strncpy(c.static_ip, "192.168.1.123", sizeof(c.static_ip) - 1);
  strncpy(c.static_subnet, "255.255.255.0", sizeof(c.static_subnet) - 1);
  strncpy(c.static_gateway, "192.168.1.1", sizeof(c.static_gateway) - 1);
  strncpy(c.nodeName, "ESP32-ConsoleDMX", sizeof(c.nodeName) - 1);
  c.dmx_protocol = 1; // Art-Net default
  c.espnow_mode = 0;
  c.espnow_channel = 1;
  c.espnow_intensity_only = 0;
  c.led_artnet_enable = 0;
  c.led_universe = 1;
  c.led_color_order = 0; // 0=GRB, 1=RGB, 2=BGR
}

void loadConfig() {
  preferences.getBytes("config", &cfg, sizeof(cfg));
  if (preferences.getUInt("magic", 0) != CONFIG_MAGIC) {
    Serial.println("Config not found or invalid, loading defaults.");
    setDefaults(cfg);
    saveConfig();
  } else {
    Serial.println("Configuration loaded from Preferences.");
  }
}

void saveConfig() {
  preferences.putBytes("config", &cfg, sizeof(cfg));
  preferences.putUInt("magic", CONFIG_MAGIC);
  Serial.println("Configuration saved to Preferences.");
}

// --- FS Init ---
bool fsInit() {
  if (!LittleFS.begin(true)) { // true = format if mount failed
    Serial.println("LittleFS Mount Failed");
    return false;
  }
  Serial.println("LittleFS Mounted.");
  // Ensure directories exist
  LittleFS.mkdir("/scenes");
  LittleFS.mkdir("/chasers");
  return true;
}

void saveFaderCustoms() {
  StaticJsonDocument<4096> doc;
  JsonArray names = doc.createNestedArray("names");
  JsonArray colors = doc.createNestedArray("colors");
  for (int i = 0; i < NUM_FADERS; i++) {
    names.add(faderNames[i]);
    colors.add(faderColors[i]);
  }
  File f = LittleFS.open("/fader_customs.json", "w");
  if (f) {
    serializeJson(doc, f);
    f.close();
    Serial.println("Fader customizations saved.");
  } else {
    Serial.println("Failed to save fader customs.");
  }
}

void saveAssignments() {
  StaticJsonDocument<2048> doc;
  JsonArray x_ch = doc.createNestedArray("x_ch");
  for(int i=0; i<num_x_channels; i++) {
    JsonObject obj = x_ch.createNestedObject();
    obj["c"] = xy_pad_x_channels[i].ch;
    obj["i"] = xy_pad_x_channels[i].inverted;
  }
  JsonArray y_ch = doc.createNestedArray("y_ch");
  for(int i=0; i<num_y_channels; i++) {
    JsonObject obj = y_ch.createNestedObject();
    obj["c"] = xy_pad_y_channels[i].ch;
    obj["i"] = xy_pad_y_channels[i].inverted;
  }
  JsonArray bo_ignore = doc.createNestedArray("bo_ignore");
  for(int i=0; i<DMX_CHANNELS; i++) {
    if (ignoreBlackout[i]) {
      bo_ignore.add(i + 1);
    }
  }

  File f = LittleFS.open("/assignments.json", "w");
  if (f) {
    serializeJson(doc, f);
    f.close();
  }
}

void loadAssignments() {
  File f = LittleFS.open("/assignments.json", "r");
  if (f) {
    StaticJsonDocument<2048> doc;
    DeserializationError error = deserializeJson(doc, f);
    if (!error) {
      JsonArray x_ch = doc["x_ch"];
      num_x_channels = 0;
      for(JsonObject obj : x_ch) {
        if(num_x_channels < MAX_XY_ASSIGN) {
          xy_pad_x_channels[num_x_channels].ch = obj["c"];
          xy_pad_x_channels[num_x_channels].inverted = obj["i"];
          num_x_channels++;
        }
      }
      JsonArray y_ch = doc["y_ch"];
      num_y_channels = 0;
      for(JsonObject obj : y_ch) {
        if(num_y_channels < MAX_XY_ASSIGN) {
          xy_pad_y_channels[num_y_channels].ch = obj["c"];
          xy_pad_y_channels[num_y_channels].inverted = obj["i"];
          num_y_channels++;
        }
      }
      memset(ignoreBlackout, 0, sizeof(ignoreBlackout));
      JsonArray bo_ignore = doc["bo_ignore"];
      for(int ch : bo_ignore) {
        if(ch >= 1 && ch <= DMX_CHANNELS) {
          ignoreBlackout[ch - 1] = true;
        }
      }
    }
    f.close();
  }
}

// --- Scene File I/O ---
String scenePath(uint8_t idx) {
  char buf[20];
  snprintf(buf, sizeof(buf), "/scenes/s%u.bin", idx);
  return String(buf);
}

String haScenePath(uint8_t idx) {
  char buf[20];
  snprintf(buf, sizeof(buf), "/scenes/s_ha%u.json", idx);
  return String(buf);
}

String roboScenePath(uint8_t idx) {
  char buf[24];
  snprintf(buf, sizeof(buf), "/scenes/s_robo%u.json", idx);
  return String(buf);
}

bool loadSceneData(uint8_t idx, uint8_t* buffer) {
  if (idx >= MAX_SCENES) return false;
  memset(buffer, 0, DMX_CHANNELS);
  String p = scenePath(idx);
  if (LittleFS.exists(p)) {
    File f = LittleFS.open(p, "r");
    if (f) {
      f.readBytes((char*)buffer, DMX_CHANNELS);
      f.close();
      return true;
    }
  }
  return false;
}

void saveSceneToFS(uint8_t idx, const uint8_t* data) {
  if (idx >= MAX_SCENES) return;
  String p = scenePath(idx);
  File f = LittleFS.open(p, "w");
  if (!f) {
    Serial.printf("FS: open write fail %s\n", p.c_str());
    return;
  }
  f.write(data, DMX_CHANNELS);
  f.close();

  memcpy(sceneDataCache[idx], data, DMX_CHANNELS);

  uint8_t any = 0;
  for (int i = 0; i < DMX_CHANNELS; i++) {
    if (data[i]) {
      any = 1;
      break;
    }
  }
  sceneNonEmpty[idx] = any;
}

void clearSceneFS(uint8_t idx) {
  if (idx >= MAX_SCENES) return;
  String p = scenePath(idx);
  if (LittleFS.exists(p)) LittleFS.remove(p);

  p = haScenePath(idx);
  if (LittleFS.exists(p)) LittleFS.remove(p);

  p = roboScenePath(idx);
  if (LittleFS.exists(p)) LittleFS.remove(p);

  sceneNonEmpty[idx] = 0;
  memset(sceneDataCache[idx], 0, DMX_CHANNELS);
}

void loadAllScenesFromFS() {
  Serial.println("Loading all scenes from LittleFS...");
  for (int s = 0; s < MAX_SCENES; s++) {
    loadSceneData(s, sceneDataCache[s]);
    sceneNonEmpty[s] = 0;
    for (int i = 0; i < DMX_CHANNELS; i++) {
      if (sceneDataCache[s][i] != 0) {
        sceneNonEmpty[s] = 1;
        break;
      }
    }
  }
}

void setBlackout(bool enable) {
  if (enable && !blackoutActive) {
    memcpy(savedOut, outValues, DMX_CHANNELS); // Save the current state before blackout
    blackoutActive = true;
    webSocket.broadcastTXT("ha_blackout:1");
    if (cfg.led_artnet_enable) {
      FastLED.clear();
      FastLED.show();
    }
  } else if (!enable && blackoutActive) {
    blackoutActive = false;
    webSocket.broadcastTXT("ha_blackout:0");
    if (cfg.led_artnet_enable) {
      for (int i = 0; i < NUM_LEDS; i++) {
        int dmx_index = i * 3;
        if (dmx_index + 2 < DMX_CHANNELS) {
          leds[i].setRGB(ledArtnetValues[dmx_index], ledArtnetValues[dmx_index+1], ledArtnetValues[dmx_index+2]);
        }
      }
      FastLED.show();
    }
  }
  applyOutput(); // Apply the new state
}

void loadChaserPreset(uint8_t idx, uint8_t clientNum) {
  if (idx >= MAX_CHASER_PRESETS) return;
  String p = chaserPresetPath(idx);
  if (LittleFS.exists(p)) {
    File f = LittleFS.open(p, "r");
    if (f) {
      String line = f.readStringUntil('\n');
      f.close();
      int parts[11];
      int partIdx = 0;
      int currentPos = 0;
      int nextComma = -1;
      while(partIdx < 11 && currentPos < line.length()) {
          nextComma = line.indexOf(',', currentPos);
          if (nextComma == -1) nextComma = line.length();
          parts[partIdx] = line.substring(currentPos, nextComma).toInt();
          partIdx++;
          currentPos = nextComma + 1;
      }
      if (partIdx >= 11) {
          chaser_bpm = parts[0];
          chaser_fade_ms = parts[1];
          chaser_step_duration_ms = parts[2];
          for(int i=0; i<MAX_CHASER_STEPS; i++) {
              chaser_scenes[i] = (uint8_t)constrain(parts[3+i], -1, 12);
          }
          String payload = "chaser_config:" + String(idx) + ":";
          payload += String(chaser_bpm) + "," + String(chaser_fade_ms) + "," + String(chaser_step_duration_ms);
          for(int i=0; i<MAX_CHASER_STEPS; i++) { payload += "," + String(chaser_scenes[i]); }
          if (clientNum == 255) {
            webSocket.broadcastTXT(payload);
          } else {
            webSocket.sendTXT(clientNum, payload);
          }
      }
    }
  }
}

String chaserPresetPath(uint8_t idx) {
  char buf[24];
  snprintf(buf, sizeof(buf), "/chasers/c%u.cfg", idx);
  return String(buf);
}

void saveActiveChaser() {
  File f = LittleFS.open("/chaser_active.cfg", "w");
  if (f) {
    f.print(chaser_bpm); f.print(",");
    f.print(chaser_fade_ms); f.print(",");
    f.print(chaser_step_duration_ms);
    for (int i = 0; i < MAX_CHASER_STEPS; i++) {
      f.print(",");
      f.print(chaser_scenes[i]);
    }
    f.println();
    f.close();
  }
}

void loadActiveChaser() {
  File f = LittleFS.open("/chaser_active.cfg", "r");
  if (f) {
    String line = f.readStringUntil('\n');
    f.close();
    int parts[11];
    int partIdx = 0;
    int currentPos = 0;
    int nextComma = -1;
    while(partIdx < 11 && currentPos < line.length()) {
        nextComma = line.indexOf(',', currentPos);
        if (nextComma == -1) nextComma = line.length();
        parts[partIdx] = line.substring(currentPos, nextComma).toInt();
        partIdx++;
        currentPos = nextComma + 1;
    }
    if (partIdx >= 11) {
        chaser_bpm = parts[0];
        chaser_fade_ms = parts[1];
        chaser_step_duration_ms = parts[2];
        for(int i=0; i<MAX_CHASER_STEPS; i++) {
            chaser_scenes[i] = (uint8_t)constrain(parts[3+i], -1, 12);
        }
    }
  }
}

void saveSceneLevels() {
  File f = LittleFS.open("/scene_levels.bin", "w");
  if (f) {
    f.write(sceneLevels, sizeof(sceneLevels));
    f.close();
  }
}

void loadSceneLevels() {
  File f = LittleFS.open("/scene_levels.bin", "r");
  if (f) {
    f.readBytes((char*)sceneLevels, sizeof(sceneLevels));
    f.close();
  }
}

// --- WebSocket Event Handler ---
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[%u] WebSocket client connected from %s\n", num, webSocket.remoteIP(num).toString().c_str());
      sendInit(num);
      break;
    case WStype_DISCONNECTED:
      Serial.printf("[%u] WebSocket client disconnected.\n", num);
      break;
    case WStype_TEXT:
      {
        String msg = "";
        msg.reserve(length);
        for(size_t i = 0; i < length; i++) { msg += (char)payload[i]; }

        if (msg == "hello") {
          sendInit(num);
          return;
        }

        if (msg.startsWith("set:")) { // Format: "set:CH:VAL"
          int p1 = msg.indexOf(':', 4);
          if (p1 > 0) {
            int ch = msg.substring(4, p1).toInt();
            int val = msg.substring(p1 + 1).toInt();
            if (ch >= 1 && ch <= DMX_CHANNELS) {
              faderValues[ch - 1] = (uint8_t)constrain(val, 0, 255);
              applyOutput();
              webSocket.broadcastTXT(msg);
            }
          }
          return;
        }

        if (msg.startsWith("ha_links:")) {
          memset(haAssignedChannels, 0, sizeof(haAssignedChannels));
          String linksStr = msg.substring(9);
          int currentPos = 0;
          int nextComma = -1;
          while (currentPos < linksStr.length()) {
            nextComma = linksStr.indexOf(',', currentPos);
            if (nextComma == -1) {
              nextComma = linksStr.length();
            }
            String chStr = linksStr.substring(currentPos, nextComma);
            if (chStr.length() > 0) {
              int ch = chStr.toInt();
              if (ch >= 1 && ch <= DMX_CHANNELS) {
                haAssignedChannels[ch - 1] = true;
              }
            }
            currentPos = nextComma + 1;
          }
          return;
        }

        if (msg.startsWith("blackout:")) {
          int en = msg.substring(9).toInt();
          setBlackout(en != 0);
          return;
        }

        if (msg.startsWith("ignore_bo:")) { // ignore_bo:CH:STATE
          int p1 = msg.indexOf(':', 10);
          if (p1 > 0) {
            int ch = msg.substring(10, p1).toInt();
            int state = msg.substring(p1 + 1).toInt();
            if (ch >= 1 && ch <= DMX_CHANNELS) {
              ignoreBlackout[ch - 1] = (state != 0);
              webSocket.broadcastTXT(msg);
              saveAssignments();
            }
          }
          return;
        }

        if (msg.startsWith("xy_assign:")) {
          String data = msg.substring(10);
          num_x_channels = 0;
          num_y_channels = 0;
          int y_pos = data.indexOf('&');
          String x_data = (y_pos != -1) ? data.substring(0, y_pos) : data;
          String y_data = (y_pos != -1) ? data.substring(y_pos + 1) : "";

          int current_pos = 0;
          while(current_pos < x_data.length() && num_x_channels < MAX_XY_ASSIGN) {
            int next_comma = x_data.indexOf(',', current_pos);
            if (next_comma == -1) next_comma = x_data.length();
            String item = x_data.substring(current_pos, next_comma);
            if (item.length() > 0) {
              bool inverted = item.endsWith("i");
              if (inverted) item.remove(item.length() - 1);
              xy_pad_x_channels[num_x_channels].ch = item.toInt();
              xy_pad_x_channels[num_x_channels].inverted = inverted;
              num_x_channels++;
            }
            current_pos = next_comma + 1;
          }

          current_pos = 0;
          while(current_pos < y_data.length() && num_y_channels < MAX_XY_ASSIGN) {
            int next_comma = y_data.indexOf(',', current_pos);
            if (next_comma == -1) next_comma = y_data.length();
            String item = y_data.substring(current_pos, next_comma);
            if (item.length() > 0) {
              bool inverted = item.endsWith("i");
              if (inverted) item.remove(item.length() - 1);
              xy_pad_y_channels[num_y_channels].ch = item.toInt();
              xy_pad_y_channels[num_y_channels].inverted = inverted;
              num_y_channels++;
            }
            current_pos = next_comma + 1;
          }
          saveAssignments();
          return;
        }

        if (msg.startsWith("xy_values:")) {
          int p1 = msg.indexOf(':', 10);
          if (p1 > 0) {
            int x_val = msg.substring(10, p1).toInt();
            int y_val = msg.substring(p1 + 1).toInt();
            xy_pad_x_value = (uint8_t)constrain(x_val, 0, 255);
            xy_pad_y_value = (uint8_t)constrain(y_val, 0, 255);
            applyOutput();
          }
          return;
        }

        if (msg == "chaser_start") {
          if (!chaser_running) {
            chaser_current_step = MAX_CHASER_STEPS - 1;
            chaser_last_step_time = 0;
            memset(chaser_out_values, 0, DMX_CHANNELS);
          }
          chaser_running = true;
          return;
        }

        if (msg == "chaser_stop") {
          chaser_running = false;
          applyOutput();
          return;
        }

        if (msg.startsWith("chaser_params:")) {
          int p1 = msg.indexOf(':', 14);
          int p2 = (p1 > 0) ? msg.indexOf(':', p1 + 1) : -1;
          if (p1 > 0 && p2 > 0) {
            chaser_bpm = msg.substring(14, p1).toInt();
            chaser_fade_ms = msg.substring(p1 + 1, p2).toInt();
            chaser_step_duration_ms = msg.substring(p2 + 1).toInt();
            saveActiveChaser();
          }
          return;
        }

        if (msg.startsWith("chaser_scenes:")) {
          String scenes_str = msg.substring(14);
          int current_pos = 0;
          int next_colon = -1;
          for (int i = 0; i < MAX_CHASER_STEPS; i++) {
            next_colon = scenes_str.indexOf(':', current_pos);
            if (next_colon == -1) {
              if (current_pos < scenes_str.length()) {
                chaser_scenes[i] = scenes_str.substring(current_pos).toInt();
              }
              break;
            }
            chaser_scenes[i] = scenes_str.substring(current_pos, next_colon).toInt();
            current_pos = next_colon + 1;
          }
          saveActiveChaser();
          return;
        }

        if (msg.startsWith("chaser_save:")) {
          int idx = msg.substring(12).toInt();
          if (idx >= 0 && idx < MAX_CHASER_PRESETS) {
            String p = chaserPresetPath(idx);
            File f = LittleFS.open(p, "w");
            if (f) {
              f.print(chaser_bpm); f.print(",");
              f.print(chaser_fade_ms); f.print(",");
              f.print(chaser_step_duration_ms);
              for (int i = 0; i < MAX_CHASER_STEPS; i++) {
                f.print(",");
                f.print(chaser_scenes[i]);
              }
              f.println();
              f.close();
              chaserPresetNonEmpty[idx] = true;
            }
          }
          return;
        }

        if (msg.startsWith("chaser_load:")) {
          int idx = msg.substring(12).toInt();
          if (idx >= 0 && idx < 4) {
            loadChaserPreset(idx, num);
          }
          return;
        }

        if (msg.startsWith("scene_level_live:")) {
          int p1 = msg.indexOf(':', 17);
          if (p1 > 0) {
            int idx = msg.substring(17, p1).toInt();
            int val = msg.substring(p1 + 1).toInt();
            if (idx >= 0 && idx < MAX_SCENES) {
              sceneLevels[idx] = (uint8_t)constrain(val, 0, 255);
              recomputeScenes();
              applyOutput();
            }
          }
          return;
        }

        if (msg.startsWith("scene_level:")) {
          int p1 = msg.indexOf(':', 12);
          if (p1 > 0) {
            int idx = msg.substring(12, p1).toInt();
            int val = msg.substring(p1 + 1).toInt();
            if (idx >= 0 && idx < MAX_SCENES) {
              sceneLevels[idx] = (uint8_t)constrain(val, 0, 255);
              saveSceneLevels();
              recomputeScenes();
              applyOutput();
              pushLevels();
            }
          }
          return;
        }

        if (msg.startsWith("set_name:")) { // format: set_name:ch:name
          int p1 = msg.indexOf(':', 9);
          if (p1 > 0) {
            int ch = msg.substring(9, p1).toInt();
            String name = msg.substring(p1 + 1);
            if (ch >= 1 && ch <= NUM_FADERS) {
              faderNames[ch - 1] = name;
              saveFaderCustoms();
              webSocket.broadcastTXT(msg);
            }
          }
          return;
        }

        if (msg.startsWith("set_color:")) { // format: set_color:ch:color
          int p1 = msg.indexOf(':', 10);
          if (p1 > 0) {
            int ch = msg.substring(10, p1).toInt();
            String color = msg.substring(p1 + 1);
            if (ch >= 1 && ch <= NUM_FADERS) {
              faderColors[ch - 1] = color;
              saveFaderCustoms();
              webSocket.broadcastTXT(msg);
            }
          }
          return;
        }

        if (msg == "reset_fader_customs") {
          if(LittleFS.exists("/fader_customs.json")) {
            LittleFS.remove("/fader_customs.json");
          }
          for(int i=0; i<NUM_FADERS; i++) {
            faderNames[i] = "CH " + String(i+1);
            faderColors[i] = "";
          }
          webSocket.broadcastTXT("reinit"); // Tell clients to reload
          return;
        }

        if (msg == "dmx_zero") {
          memset(faderValues, 0, sizeof(faderValues));
          applyOutput();
          // To be fully implemented later with pushFaders()
          return;
        }

        if (msg == "all_zero") {
          memset(faderValues, 0, sizeof(faderValues));
          memset(sceneLevels, 0, sizeof(sceneLevels));
          recomputeScenes();
          applyOutput();
          // To be fully implemented later with pushFaders() and pushLevels()
          return;
        }

        if (msg.startsWith("scene_save_full:")) { // scene_save_full:idx|ha_json|robo_json
          int p1 = msg.indexOf('|', 16);
          int p2 = (p1 > 0) ? msg.indexOf('|', p1 + 1) : -1;
          if (p1 > 0 && p2 > 0) {
            int idx = msg.substring(16, p1).toInt();
            if (idx >= 0 && idx < MAX_SCENES) {
              // 1. Save DMX fader values
              saveSceneToFS((uint8_t)idx, faderValues);

              // 2. Save HA JSON
              String haJson = msg.substring(p1 + 1, p2);
              String haP = haScenePath(idx);
              File haF = LittleFS.open(haP, "w");
              if (haF) {
                haF.print(haJson);
                haF.close();
              }

              // 3. Save Robo JSON
              String roboJson = msg.substring(p2 + 1);
              String roboP = roboScenePath(idx);
              File roboF = LittleFS.open(roboP, "w");
              if (roboF) {
                roboF.print(roboJson);
                roboF.close();
              }
              pushSlots();
            }
          }
          return;
        }

        if (msg.startsWith("scene_clear:")) {
          int idx = msg.substring(12).toInt();
          if (idx >= 0 && idx < MAX_SCENES) {
            clearSceneFS((uint8_t)idx);
            sceneLevels[idx] = 0;
            recomputeScenes();
            applyOutput();
            pushSlots();
            pushLevels();
          }
          return;
        }

        if (msg.startsWith("scene_load:")) {
          int idx = msg.substring(11).toInt();
          if (idx >= 0 && idx < MAX_SCENES) {
            if (sceneNonEmpty[idx]) {
              memcpy(faderValues, sceneDataCache[idx], DMX_CHANNELS);
              pushFaders();
              applyOutput();

              String roboP = roboScenePath(idx);
              if (LittleFS.exists(roboP)) {
                File roboF = LittleFS.open(roboP, "r");
                if (roboF) {
                  String roboJson = roboF.readString();
                  roboF.close();
                  if (roboJson.length() > 2) {
                    String payload = "robo_load:" + roboJson;
                    webSocket.sendTXT(num, payload);
                  }
                }
              }
            }
          }
          return;
        }

        if (msg.startsWith("cfgx2:")) {
          int p1 = msg.indexOf(':', 6);
          int p2 = (p1 > 0) ? msg.indexOf(':', p1 + 1) : -1;
          int p3 = (p2 > 0) ? msg.indexOf(':', p2 + 1) : -1;
          int p4 = (p3 > 0) ? msg.indexOf(':', p3 + 1) : -1;
          int p5 = (p4 > 0) ? msg.indexOf(':', p4 + 1) : -1;
          int p6 = (p5 > 0) ? msg.indexOf(':', p5 + 1) : -1;
          if (p1 > 0 && p2 > 0 && p3 > 0) {
            uint8_t prevProto = cfg.dmx_protocol;
            bool needsRestart = false;

            cfg.dmx_protocol = (uint8_t)constrain(msg.substring(6, p1).toInt(), 0, 3);
            cfg.universe = (uint8_t)constrain(msg.substring(p1 + 1, p2).toInt(), 0, 255);
            cfg.priority = (uint8_t)constrain(msg.substring(p2 + 1, p3).toInt(), 0, 2);

            String nm = (p4 > 0) ? msg.substring(p3 + 1, p4) : msg.substring(p3 + 1);
            nm.trim();
            strncpy(cfg.nodeName, nm.c_str(), sizeof(cfg.nodeName) - 1);

            if (p4 > 0 && p5 > 0) {
              cfg.led_artnet_enable = (uint8_t)constrain(msg.substring(p4 + 1, p5).toInt(), 0, 1);
              cfg.led_universe = (uint8_t)constrain((p6 > 0) ? msg.substring(p5 + 1, p6).toInt() : msg.substring(p5 + 1).toInt(), 0, 255);
              if (p6 > 0) {
                cfg.led_color_order = (uint8_t)constrain(msg.substring(p6 + 1).toInt(), 0, 2);
              }
            }

            saveConfig();
            if (prevProto != cfg.dmx_protocol) {
              needsRestart = true;
            }
            // Further checks for LED strip changes etc. can be added here.

            if (needsRestart) {
              restartAt = millis() + 800;
            }
            sendInit(num); // Send updated config back to the client
          }
          return;
        }

        if (msg.startsWith("espnow_mode:")) {
          int mode = msg.substring(12).toInt();
          if (mode >= 0 && mode <= 2) {
            cfg.espnow_mode = (uint8_t)mode;
            saveConfig();
            restartAt = millis() + 800;
          }
          return;
        }
        if (msg.startsWith("espnow_cfg:")) {
          int p1 = msg.indexOf(':', 11);
          if (p1 > 0) {
            cfg.espnow_channel = (uint8_t)constrain(msg.substring(11, p1).toInt(), 0, 14);
            cfg.espnow_intensity_only = (msg.substring(p1 + 1).toInt() != 0) ? 1 : 0;
            saveConfig();
            restartAt = millis() + 800;
          }
          return;
        }
      }
      break;
    case WStype_ERROR:
      Serial.printf("[%u] WebSocket error: %s\n", num, (char*)payload);
      break;
    default:
      break;
  }
}

void sendInit(uint8_t num) {
  String out = "init:";

  for (int i = 0; i < DMX_CHANNELS; i++) {
    out += String(faderValues[i]);
    if (i < DMX_CHANNELS - 1) out += ",";
  }
  out += "|";

  IPAddress ip = wifiAPMode ? WiFi.softAPIP() : WiFi.localIP();
  out += String((int)cfg.dmx_protocol) + "," + String(cfg.universe) + "," + String((int)cfg.priority) + "," + ip.toString() + "," +
         String((int)cfg.ap_only) + "," + String(cfg.nodeName) + "," + String((int)cfg.use_static_ip) + "," + String(cfg.static_ip) + "," +
         String(cfg.static_subnet) + "," + String(cfg.static_gateway) + "," + String((int)cfg.espnow_mode) + "," +
         String(cfg.espnow_channel) + "," + String((int)cfg.espnow_intensity_only) + "," + String((int)blackoutActive) + "," +
         String((int)cfg.led_artnet_enable) + "," + String(cfg.led_universe) + "," + String((int)cfg.led_color_order) + "," +
         String((int)chaser_running) + "," + String(chaser_bpm) + "," + String(chaser_fade_ms) + "," + String(chaser_step_duration_ms);
  for (int i = 0; i < MAX_CHASER_STEPS; i++) {
    out += "," + String(chaser_scenes[i]);
  }
  out += ",&|"; // XY data placeholder

  for (int s = 0; s < MAX_SCENES; s++) {
    out += String(sceneLevels[s]);
    if (s < MAX_SCENES - 1) out += ",";
  }
  out += "|";

  for (int s = 0; s < MAX_SCENES; s++) {
    out += String(sceneNonEmpty[s]);
    if (s < MAX_SCENES - 1) out += ",";
  }
  out += "|";

  for (int i = 0; i < MAX_CHASER_PRESETS; i++) {
    out += String(chaserPresetNonEmpty[i] ? 1 : 0);
    if (i < MAX_CHASER_PRESETS - 1) out += ",";
  }

  StaticJsonDocument<4096> doc;
  JsonArray names = doc.createNestedArray("names");
  JsonArray colors = doc.createNestedArray("colors");
  for (int i = 0; i < NUM_FADERS; i++) {
    names.add(faderNames[i]);
    colors.add(faderColors[i]);
  }
  String customs_json;
  serializeJson(doc, customs_json);
  out += "§" + customs_json;

  String bo_ignore_str = "~";
  for (int i = 0; i < DMX_CHANNELS; i++) {
    bo_ignore_str += ignoreBlackout[i] ? "1" : "0";
  }
  out += bo_ignore_str;

  webSocket.sendTXT(num, out);
}

void pushSlots() {
  String sl = "slots:";
  for (int s = 0; s < MAX_SCENES; s++) {
    sl += String(sceneNonEmpty[s]);
    if (s < MAX_SCENES - 1) sl += ",";
  }
  webSocket.broadcastTXT(sl);
}

void pushLevels() {
  String lv = "levels:";
  for (int s = 0; s < MAX_SCENES; s++) {
    lv += String(sceneLevels[s]);
    if (s < MAX_SCENES - 1) lv += ",";
  }
  webSocket.broadcastTXT(lv);
}

void pushFaders() {
  String fv = "faders:";
  for (int i = 0; i < DMX_CHANNELS; i++) {
    fv += String(faderValues[i]);
    if (i < DMX_CHANNELS - 1) fv += ",";
  }
  webSocket.broadcastTXT(fv);
}

void applyOutput() {
  if (blackoutActive) {
    byte blackout_values[DMX_CHANNELS];
    memset(blackout_values, 0, DMX_CHANNELS);

    // Passthrough for channels marked as ignore
    for (int i = 0; i < DMX_CHANNELS; i++) {
      if (ignoreBlackout[i]) {
        blackout_values[i] = savedOut[i];
      }
    }

    // Passthrough for live XY channels
    for (uint8_t i = 0; i < num_x_channels; i++) {
      uint16_t ch = xy_pad_x_channels[i].ch;
      if (ch > 0 && ch <= DMX_CHANNELS) {
        blackout_values[ch - 1] = xy_pad_x_channels[i].inverted ? (255 - xy_pad_x_value) : xy_pad_x_value;
      }
    }
    for (uint8_t i = 0; i < num_y_channels; i++) {
      uint16_t ch = xy_pad_y_channels[i].ch;
      if (ch > 0 && ch <= DMX_CHANNELS) {
        blackout_values[ch - 1] = xy_pad_y_channels[i].inverted ? (255 - xy_pad_y_value) : xy_pad_y_value;
      }
    }

    memcpy(dmx_data, blackout_values, DMX_CHANNELS);
    return;
  }

  if (chaser_running) {
    memcpy(outValues, chaser_out_values, DMX_CHANNELS);
  } else {
    for (int i = 0; i < DMX_CHANNELS; i++) {
    uint8_t base = 0;
    switch (cfg.priority) {
      case PRIO_FADERS:
        base = faderValues[i];
        break;
      case PRIO_ARTNET:
        base = artnetValues[i]; // artnetValues is used for both Art-Net and sACN
        break;
      case PRIO_HTP:
        base = max(faderValues[i], artnetValues[i]);
        break;
    }

    // HTP merge the result with the scenes output
    uint8_t final_val = max(base, scenesOut[i]);

    // If a channel is assigned to HA, it should not be controlled by the console's faders/scenes.
    if (haAssignedChannels[i]) {
      final_val = 0;
    }
    outValues[i] = final_val;
  }

  // Apply XY pad values, which have ultimate priority over everything else
  for (uint8_t i = 0; i < num_x_channels; i++) {
    uint16_t ch = xy_pad_x_channels[i].ch;
    if (ch > 0 && ch <= DMX_CHANNELS) {
      outValues[ch - 1] = xy_pad_x_channels[i].inverted ? (255 - xy_pad_x_value) : xy_pad_x_value;
    }
  }
  for (uint8_t i = 0; i < num_y_channels; i++) {
    uint16_t ch = xy_pad_y_channels[i].ch;
    if (ch > 0 && ch <= DMX_CHANNELS) {
      outValues[ch - 1] = xy_pad_y_channels[i].inverted ? (255 - xy_pad_y_value) : xy_pad_y_value;
    }
  }

  // Copy the final computed values to the DMX buffer
  memcpy(dmx_data, outValues, DMX_CHANNELS);

  // Send data via ESP-NOW if in master mode
  sendEspNowBlock(outValues, DMX_CHANNELS);
}

void recomputeScenes() {
  memset(scenesOut, 0, DMX_CHANNELS);
  for (int s = 0; s < MAX_SCENES; s++) {
    if (sceneLevels[s] == 0) continue;

    // Pointer to the cached scene data
    uint8_t* tempScene = sceneDataCache[s];

    for (int ch = 0; ch < DMX_CHANNELS; ch++) {
      if (tempScene[ch] == 0) continue;

      // Scale the scene's channel value by the scene's master level
      uint16_t v = (uint16_t)tempScene[ch] * (uint16_t)sceneLevels[s];
      v /= 255;

      // HTP merge with the output of other scenes
      if (v > scenesOut[ch]) {
        scenesOut[ch] = (uint8_t)(v > 255 ? 255 : v);
      }
    }
  }
}