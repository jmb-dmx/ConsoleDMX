#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <EEPROM.h>
#include "esp_dmx.h"
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>
#include <SPI.h>

// DMX Configuration
const dmx_port_t dmx_port = DMX_NUM_2;
const int dmx_tx_pin = 17;
const int dmx_rx_pin = 16;
const int dmx_rts_pin = 21;

// TTGO T-Display
TFT_eSPI tft = TFT_eSPI();

#define NUM_FADERS 96
#define DMX_CHANNELS 512
#define MAX_SCENES 12
#define EEPROM_SIZE 4096

const uint32_t CONFIG_MAGIC = 0xD0F1CCEE;

struct Config {
  uint32_t magic;
  char ssid[32];
  char pass[64];
  uint8_t ap_only;
  uint8_t use_static_ip;
  char static_ip[16];
  char static_subnet[16];
  char static_gateway[16];
  char nodeName[32];
};
const int ADDR_CONFIG = 0;
WebServer server(80);
WebSocketsServer webSocket(81);
uint8_t dmx_buffer[DMX_PACKET_SIZE] = {0};
uint8_t faderValues[DMX_CHANNELS];
uint8_t sceneLevels[MAX_SCENES];
uint8_t scenesOut[DMX_CHANNELS];
uint8_t outValues[DMX_CHANNELS];
uint8_t sceneNonEmpty[MAX_SCENES];
uint8_t sceneDataCache[MAX_SCENES][DMX_CHANNELS];
bool blackoutActive = false;
bool ignoreBlackout[DMX_CHANNELS];
uint8_t savedOut[DMX_CHANNELS];
#define MAX_CHASER_STEPS 8
#define MAX_CHASER_PRESETS 4
bool chaser_running = false;
bool chaserPresetNonEmpty[MAX_CHASER_PRESETS] = {false, false, false, false};
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

#define MAX_XY_ASSIGN 16
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

String faderNames[NUM_FADERS];
String faderColors[NUM_FADERS];

Config cfg;
bool wifiAPMode = false;
uint32_t restartAt = 0;

// ---- Screen Status Color System ----
uint16_t baseBgColor = TFT_BLACK;
uint16_t flashBgColor = TFT_BLACK;
uint32_t flashEndMillis = 0;

void setStatusColor(uint8_t r, uint8_t g, uint8_t b) {
    baseBgColor = tft.color565(r, g, b);
}

void flashStatusColor(uint8_t r, uint8_t g, uint8_t b, uint16_t ms) {
    flashBgColor = tft.color565(r, g, b);
    flashEndMillis = millis() + ms;
}

void statusDisplayTask() {
    static uint16_t lastColor = 0xFFFF; // Force initial update
    uint32_t now = millis();

    uint16_t targetColor = (now < flashEndMillis) ? flashBgColor : baseBgColor;

    if (targetColor != lastColor) {
        lastColor = targetColor;

        // Determine contrasting text color based on background brightness
        uint8_t r8 = (targetColor >> 11) & 0x1F;
        uint8_t g8 = (targetColor >> 5) & 0x3F;
        uint8_t b8 = targetColor & 0x1F;
        // Scale to 8-bit for brightness calculation
        r8 = (r8 * 255) / 31;
        g8 = (g8 * 255) / 63;
        b8 = (b8 * 255) / 31;
        // Luma formula for perceived brightness
        uint16_t brightness = (r8 * 299 + g8 * 587 + b8 * 114) / 1000;
        uint16_t textColor = (brightness > 128) ? TFT_BLACK : TFT_WHITE;

        tft.fillScreen(targetColor);
        tft.setTextColor(textColor, targetColor);

        updateDisplay(); // Redraw the text content
    }
}

void updateDisplay() {
  tft.setCursor(0, 0);
  tft.setTextSize(2);
  tft.println("ConsoleDMX");
  tft.setTextSize(1);
  if (wifiAPMode) {
    tft.println("Mode: AP");
    tft.print("IP: ");
    tft.println(WiFi.softAPIP());
  } else {
    tft.println("Mode: STA");
    tft.print("IP: ");
    tft.println(WiFi.localIP());
  }
}

void applyOutput();
void recomputeScenes();
void chaserTask();
void pushFaders();
void handleRoot();
void handleWifiPost();
void handleFactoryReset();
void sendInit(uint8_t num);
bool tryWiFiSTA(uint32_t timeoutMs);
void startAP();
bool fsInit();
String scenePath(uint8_t idx);
void setBlackout(bool enable);
void saveAssignments();
void loadAssignments();
void saveSceneLevels();
void loadSceneLevels();
void saveActiveChaser();
void loadActiveChaser();

void setDefaults(Config& c) {
  memset(&c, 0, sizeof(c));
  c.magic = CONFIG_MAGIC;
  strncpy(c.ssid, "BELL685", sizeof(c.ssid) - 1);
  strncpy(c.pass, "1739EC3C", sizeof(c.pass) - 1);
  c.ap_only = 0;
  c.use_static_ip = 0;
  strncpy(c.nodeName, "ConsoleDMX", sizeof(c.nodeName) - 1);
}
void loadConfig() {
  EEPROM.get(ADDR_CONFIG, cfg);
  if (cfg.magic != CONFIG_MAGIC) {
    setDefaults(cfg);
    EEPROM.put(ADDR_CONFIG, cfg);
    EEPROM.commit();
  }
}
void saveConfig() {
  cfg.magic = CONFIG_MAGIC;
  EEPROM.put(ADDR_CONFIG, cfg);
  EEPROM.commit();
}
bool fsInit() {
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
    return false;
  }
  LittleFS.mkdir("/scenes");
  LittleFS.mkdir("/chasers");
  return true;
}
String chaserPresetPath(uint8_t idx) {
  char buf[24];
  snprintf(buf, sizeof(buf), "/chasers/c%u.cfg", idx);
  return String(buf);
}
String scenePath(uint8_t idx) {
  char buf[20];
  snprintf(buf, sizeof(buf), "/scenes/s%u.bin", idx);
  return String(buf);
}
String roboScenePath(uint8_t idx) {
  char buf[24];
  snprintf(buf, sizeof(buf), "/scenes/s_robo%u.json", idx);
  return String(buf);
}
void checkChaserPresets() {
  for (int i = 0; i < MAX_CHASER_PRESETS; i++) {
    chaserPresetNonEmpty[i] = LittleFS.exists(chaserPresetPath(i));
  }
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

void loadFaderCustoms() {
  File f = LittleFS.open("/fader_customs.json", "r");
  if (f) {
    StaticJsonDocument<4096> doc;
    DeserializationError error = deserializeJson(doc, f);
    if (!error) {
      JsonArray names = doc["names"];
      JsonArray colors = doc["colors"];
      for (int i = 0; i < NUM_FADERS; i++) {
        faderNames[i] = names[i].as<String>();
        faderColors[i] = colors[i].as<String>();
      }
    }
    f.close();
  }
}

void loadAllScenesFromFS() {
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
  size_t n = f.write(data, DMX_CHANNELS);
  f.close();
  memcpy(sceneDataCache[idx], data, DMX_CHANNELS);
  uint8_t any = 0;
  for (int i = 0; i < DMX_CHANNELS; i++)
    if (data[i]) {
      any = 1;
      break;
    }
  sceneNonEmpty[idx] = any;
  Serial.printf("Scene %u saved to %s (%u bytes)\n", idx, p.c_str(), (unsigned)n);
}
void clearSceneFS(uint8_t idx) {
  if (idx >= MAX_SCENES) return;
  String p = scenePath(idx);
  if (LittleFS.exists(p)) LittleFS.remove(p);
  
  p = roboScenePath(idx);
  if (LittleFS.exists(p)) LittleFS.remove(p);

  sceneNonEmpty[idx] = 0;
  memset(sceneDataCache[idx], 0, DMX_CHANNELS);
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
        chaser_running = false;
        return;
      }
    } while (chaser_scenes[next_step] > MAX_SCENES);
    chaser_current_step = next_step;
    String step_msg = "chaser_step:" + String(chaser_current_step);
    webSocket.broadcastTXT(step_msg);
    uint8_t next_scene_idx = chaser_scenes[chaser_current_step];
    if (next_scene_idx == MAX_SCENES) {
      memset(fade_to_values, 0, DMX_CHANNELS);
    } else if (sceneNonEmpty[next_scene_idx]) {
      memcpy(fade_to_values, sceneDataCache[next_scene_idx], DMX_CHANNELS);
    } else {
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
void recomputeScenes() {
  memset(scenesOut, 0, DMX_CHANNELS);
  for (int s = 0; s < MAX_SCENES; s++) {
    if (sceneLevels[s] == 0) continue;
    uint8_t* tempScene = sceneDataCache[s];
    for (int ch = 0; ch < DMX_CHANNELS; ch++) {
      if (tempScene[ch] == 0) continue;
      uint16_t v = (uint16_t)tempScene[ch] * (uint16_t)sceneLevels[s];
      v /= 255;
      if (v > scenesOut[ch]) { scenesOut[ch] = (uint8_t)(v > 255 ? 255 : v); }
    }
  }
}

void setBlackout(bool enable) {
  if (enable && !blackoutActive) {
    memcpy(savedOut, outValues, DMX_CHANNELS);
    blackoutActive = true;
    flashStatusColor(0, 0, 255, 150);
    applyOutput();
  } else if (!enable && blackoutActive) {
    blackoutActive = false;
    flashStatusColor(0, 255, 0, 150);
    applyOutput();
  }
}
void applyOutput() {
  if (blackoutActive) {
    uint8_t blackout_values[DMX_CHANNELS];
    memset(blackout_values, 0, DMX_CHANNELS);

    for (int i = 0; i < DMX_CHANNELS; i++) {
      if (ignoreBlackout[i]) {
        blackout_values[i] = savedOut[i];
      }
    }

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

    dmx_write(dmx_port, blackout_values, DMX_PACKET_SIZE);
    dmx_send(dmx_port);
    return;
  }
  if (chaser_running) {
    memcpy(outValues, chaser_out_values, DMX_CHANNELS);
  } else {
    for (int i = 0; i < DMX_CHANNELS; i++) {
      uint8_t base = faderValues[i];
      uint8_t v = max(base, scenesOut[i]);
      outValues[i] = v;
    }
  }
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

  dmx_buffer[0] = 0; // Start code
  for (int i = 0; i < DMX_CHANNELS; i++) {
    dmx_buffer[i+1] = outValues[i];
  }
  dmx_write(dmx_port, dmx_buffer, DMX_PACKET_SIZE);
  dmx_send(dmx_port);
}
void zeroAllFaders() {
  memset(faderValues, 0, sizeof(faderValues));
}
void zeroAllScenesLevels() {
  memset(sceneLevels, 0, sizeof(sceneLevels));
  recomputeScenes();
}
void handleRoot() {
  File f = LittleFS.open("/index.html", "r");
  if (!f) {
    server.send(404, "text/plain", "File not found: /index.html");
    return;
  }
  server.streamFile(f, "text/html");
  f.close();
}
void handleWifiPost() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Méthode non autorisée");
    return;
  }
  if (server.hasArg("ssid")) strncpy(cfg.ssid, server.arg("ssid").c_str(), sizeof(cfg.ssid) - 1);
  if (server.hasArg("pass")) strncpy(cfg.pass, server.arg("pass").c_str(), sizeof(cfg.pass) - 1);
  if (server.hasArg("aponly")) cfg.ap_only = (uint8_t)server.arg("aponly").toInt();
  if (server.hasArg("use_static_ip")) cfg.use_static_ip = (uint8_t)server.arg("use_static_ip").toInt();
  if (server.hasArg("static_ip")) strncpy(cfg.static_ip, server.arg("static_ip").c_str(), sizeof(cfg.static_ip) - 1);
  if (server.hasArg("static_subnet")) strncpy(cfg.static_subnet, server.arg("static_subnet").c_str(), sizeof(cfg.static_subnet) - 1);
  if (server.hasArg("static_gateway")) strncpy(cfg.static_gateway, server.arg("static_gateway").c_str(), sizeof(cfg.static_gateway) - 1);
  saveConfig();
  server.send(200, "text/html", "<html><body><h3>Enregistré. Redémarrage dans 2s…</h3><script>setTimeout(()=>location.href='/',2000)</script></body></html>");
  delay(500);
  ESP.restart();
}
void handleFactoryReset() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Méthode non autorisée");
    return;
  }
  setDefaults(cfg);
  EEPROM.put(ADDR_CONFIG, cfg);
  EEPROM.commit();
  for (int s = 0; s < MAX_SCENES; s++) clearSceneFS((uint8_t)s);
  if (LittleFS.exists("/assignments.json")) {
    LittleFS.remove("/assignments.json");
  }
  if (LittleFS.exists("/scene_levels.bin")) {
    LittleFS.remove("/scene_levels.bin");
  }
  if (LittleFS.exists("/chaser_active.cfg")) {
    LittleFS.remove("/chaser_active.cfg");
  }
  for (int i = 0; i < MAX_CHASER_PRESETS; i++) {
    String p = chaserPresetPath(i);
    if (LittleFS.exists(p)) {
      LittleFS.remove(p);
    }
  }
  if (LittleFS.exists("/fader_customs.json")) {
    LittleFS.remove("/fader_customs.json");
  }
  memset(faderValues, 0, sizeof(faderValues));
  memset(ignoreBlackout, 0, sizeof(ignoreBlackout));
  memset(sceneLevels, 0, sizeof(sceneLevels));
  memset(scenesOut, 0, sizeof(scenesOut));
  memset(outValues, 0, sizeof(outValues));
  memset(sceneNonEmpty, 0, sizeof(sceneNonEmpty));
  blackoutActive = false;
  recomputeScenes();
  applyOutput();
  flashStatusColor(255, 0, 0, 400);
  server.send(200, "text/html",
              "<html><body><h3>Remise d’usine effectuée. Redémarrage…</h3>"
              "<script>setTimeout(()=>location.href='/',3000)</script></body></html>");
  delay(600);
  ESP.restart();
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
void pushSlots() {
  String sl = "slots:";
  for (int s = 0; s < MAX_SCENES; s++) {
    sl += String(sceneNonEmpty[s]);
    if (s < MAX_SCENES - 1) sl += ",";
  }
  webSocket.broadcastTXT(sl);
}
void sendInit(uint8_t num) {
  String out = "init:";
  for (int i = 0; i < DMX_CHANNELS; i++) {
    out += String(faderValues[i]);
    if (i < DMX_CHANNELS - 1) out += ",";
  }
  out += "|";
  IPAddress ip = wifiAPMode ? WiFi.softAPIP() : WiFi.localIP();
  String ipStr = ip.toString();
  out += ipStr;
  out += ",";
  out += String((int)cfg.ap_only);
  out += ",";
  out += String(cfg.nodeName);
  out += ",";
  out += String((int)cfg.use_static_ip);
  out += ",";
  out += String(cfg.static_ip);
  out += ",";
  out += String(cfg.static_subnet);
  out += ",";
  out += String(cfg.static_gateway);
  out += ",";
  out += String((int)blackoutActive);
  out += ",";
  out += String((int)chaser_running);
  out += ",";
  out += String(chaser_bpm);
  out += ",";
  out += String(chaser_fade_ms);
  out += ",";
  out += String(chaser_step_duration_ms);
  out += ",";
  for (int i = 0; i < MAX_CHASER_STEPS; i++) {
    out += String(chaser_scenes[i]);
    if (i < MAX_CHASER_STEPS - 1) out += ",";
  }
  out += ",";
  String xy_assign_str = "";
  for(uint8_t i=0; i < num_x_channels; i++) {
    xy_assign_str += String(xy_pad_x_channels[i].ch);
    if(xy_pad_x_channels[i].inverted) xy_assign_str += "i";
    if(i < num_x_channels - 1) xy_assign_str += ",";
  }
  xy_assign_str += "&";
  for(uint8_t i=0; i < num_y_channels; i++) {
    xy_assign_str += String(xy_pad_y_channels[i].ch);
    if(xy_pad_y_channels[i].inverted) xy_assign_str += "i";
    if(i < num_y_channels - 1) xy_assign_str += ",";
  }
  out += xy_assign_str;
  out += "|";
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

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connecté depuis %s\n", num, ip.toString().c_str());
        flashStatusColor(255, 255, 255, 120);
        sendInit(num);
      }
      break;
    case WStype_TEXT:
      {
        String msg;
        msg.reserve(length);
        for (size_t i = 0; i < length; i++) msg += (char)payload[i];
        if (msg == "reboot") {
          Serial.println("Reboot command received, restarting...");
          delay(100);
          ESP.restart();
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
              Serial.printf("Chaser preset %d saved.\n", idx);
              if (!chaserPresetNonEmpty[idx]) {
                chaserPresetNonEmpty[idx] = true;
              }
            }
          }
          return;
        }
        if (msg.startsWith("chaser_load:")) {
          int idx = msg.substring(12).toInt();
          if (idx >= 0 && idx < MAX_CHASER_PRESETS) {
            String p = chaserPresetPath(idx);
            if (LittleFS.exists(p)) {
              File f = LittleFS.open(p, "r");
              if (f) {
                String line = f.readStringUntil('\n');
                f.close();
                line.trim();

                // Update server-side state from the loaded preset
                int parts[11];
                int partIdx = 0;
                int currentPos = 0;
                int nextComma = -1;
                while(partIdx < 11 && currentPos < line.length()) {
                    nextComma = line.indexOf(',', currentPos);
                    if (nextComma == -1) nextComma = line.length();
                    String part = line.substring(currentPos, nextComma);
                    part.trim();
                    if (part.length() > 0) {
                      parts[partIdx] = part.toInt();
                    } else {
                      parts[partIdx] = 0;
                    }
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

                // Send the loaded config back to the client to update UI
                String response = "chaser_config:" + String(idx) + ":" + line;
                webSocket.sendTXT(num, response);
              }
            }
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
        if (msg == "hello") {
          sendInit(num);
          return;
        }
        if (msg == "dmx_zero") {
          memset(faderValues, 0, sizeof(faderValues));
          applyOutput();
          pushFaders();
          return;
        }
        if (msg == "all_zero") {
          memset(faderValues, 0, sizeof(faderValues));
          pushFaders();
          memset(sceneLevels, 0, sizeof(sceneLevels));
          pushLevels();
          recomputeScenes();
          applyOutput();
          return;
        }
        if (msg == "erase_all") {
          zeroAllFaders();
          String z = "setv:";
          for (int i = 1; i <= DMX_CHANNELS; i++) {
            z += String(i) + ":0";
            if (i < DMX_CHANNELS) z += ",";
          }
          webSocket.broadcastTXT(z);
          applyOutput();
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
        if (msg.startsWith("set:")) {
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
        if (msg.startsWith("set_name:")) {
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
        if (msg.startsWith("set_color:")) {
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
            faderColors[i] = "#262626";
          }
          webSocket.broadcastTXT("reinit");
          return;
        }
        if (msg.startsWith("blackout:")) {
          int en = msg.substring(9).toInt();
          setBlackout(en != 0);
          return;
        }
        if (msg.startsWith("ignore_bo:")) {
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
        if (msg.startsWith("scene_save_full:")) {
          int p1 = msg.indexOf('|', 16);
          int p2 = (p1 > 0) ? msg.indexOf('|', p1 + 1) : -1;
          if (p1 > 0 && p2 > 0) {
            int idx = msg.substring(16, p1).toInt();
            if (idx >= 0 && idx < MAX_SCENES) {
              saveSceneToFS((uint8_t)idx, faderValues);
              String roboJson = msg.substring(p2 + 1);
              String roboP = roboScenePath(idx);
              File roboF = LittleFS.open(roboP, "w");
              if (roboF) {
                roboF.print(roboJson);
                roboF.close();
              }
              recomputeScenes();
              applyOutput();
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
            pushLevels();
            pushSlots();
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
      }
      break;
    default: break;
  }
}
String makeAPSSID() {
  String s = "ConsoleDMX-";
  s += String((uint32_t)ESP.getEfuseMac(), HEX);
  s.toUpperCase();
  return s;
}
bool tryWiFiSTA(uint32_t timeoutMs) {
  WiFi.mode(WIFI_STA);
  if (cfg.use_static_ip) {
    IPAddress staticIP, staticGateway, staticSubnet;
    if (staticIP.fromString(cfg.static_ip) && staticGateway.fromString(cfg.static_gateway) && staticSubnet.fromString(cfg.static_subnet)) {
      WiFi.config(staticIP, staticGateway, staticSubnet);
      Serial.printf("WiFi: Tentative avec IP Statique: %s\n", cfg.static_ip);
    } else {
      Serial.println("WiFi: Configuration IP statique invalide. Passage en DHCP.");
    }
  } else {
    Serial.println("WiFi: Utilisation de DHCP.");
  }
  WiFi.begin(cfg.ssid, cfg.pass);
  Serial.printf("WiFi STA: connexion à \"%s\"…\n", cfg.ssid);
  ledSetBase(255, 200, 0);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeoutMs) {
    delay(300);
    Serial.print(".");
    statusDisplayTask();
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    wifiAPMode = false;
    Serial.printf("STA OK, IP: %s\n", WiFi.localIP().toString().c_str());
    setStatusColor(0, 255, 0);
    return true;
  }
  Serial.println("STA échec.");
  return false;
}
void startAP() {
  wifiAPMode = true;
  String ap = makeAPSSID();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap.c_str(), "dmx12345");
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("AP \"%s\" démarré. IP: %s\n", ap.c_str(), ip.toString().c_str());
  setStatusColor(0, 0, 255);
}
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("\nBoot DMX Web + 12 Scenes");
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.println("Booting...");

  setStatusColor(255, 0, 255);
  EEPROM.begin(EEPROM_SIZE);
  loadConfig();
  if (!fsInit()) {
    Serial.println("FS init failed, formatting again…");
    tft.println("FS Error!");
    while(true);
  }

  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_driver_install(dmx_port, &config, NULL, 0);
  dmx_set_pin(dmx_port, dmx_tx_pin, dmx_rx_pin, dmx_rts_pin);

  memset(faderValues, 0, sizeof(faderValues));
  loadSceneLevels();
  memset(scenesOut, 0, sizeof(scenesOut));
  memset(outValues, 0, sizeof(outValues));
  memset(sceneNonEmpty, 0, sizeof(sceneNonEmpty));
  memset(savedOut, 0, sizeof(savedOut));
  memset(chaser_out_values, 0, sizeof(chaser_out_values));
  memset(fade_from_values, 0, sizeof(fade_from_values));
  memset(fade_to_values, 0, sizeof(fade_to_values));
  for(int i=0; i<NUM_FADERS; i++) {
    faderNames[i] = "CH " + String(i+1);
    faderColors[i] = "";
  }
  loadFaderCustoms();
  loadAssignments();
  num_x_channels = 0;
  num_y_channels = 0;
  xy_pad_x_value = 127;
  xy_pad_y_value = 127;
  blackoutActive = false;
  loadAllScenesFromFS();
  checkChaserPresets();
  loadActiveChaser();
  recomputeScenes();
  applyOutput();
  if (cfg.ap_only) startAP();
  else if (!tryWiFiSTA(20000)) startAP();

  if (MDNS.begin("consoledmx")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("mDNS responder started: http://consoledmx.local");
  } else {
    Serial.println("Error setting up MDNS responder!");
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/wifi", HTTP_POST, handleWifiPost);
  server.on("/factory", HTTP_POST, handleFactoryReset);
  server.begin();
  Serial.println("HTTP serveur :80");
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket :81");

  statusDisplayTask(); // Initial draw
  Serial.println("Prêt.");
}
void loop() {
  server.handleClient();
  webSocket.loop();
  chaserTask();
  statusDisplayTask();
  if (restartAt && millis() > restartAt) {
    Serial.println("Redémarrage demandé...");
    delay(100);
    ESP.restart();
  }
}