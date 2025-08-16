#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WebSocketsServer.h>
#include <EEPROM.h>
#include <ESPDMX.h>
#include <ArtnetWiFi.h>
#include <ESPAsyncE131.h>
#include <ESP8266mDNS.h>
#include <espnow.h>
#include <FS.h>
#include <LittleFS.h>
#include <FastLED.h>
#include <ArduinoJson.h>

extern "C" {
#include "user_interface.h"
}

#define NUM_FADERS 96
#define NUM_LEDS 170
#define LED_DATA_PIN D5
#define LED_TYPE    WS2812B
#define DMX_CHANNELS 512
#define MAX_SCENES 12
#define EEPROM_SIZE 4096
#define LED_R D6
#define LED_G D7
#define LED_B D8
const uint32_t CONFIG_MAGIC = 0xD0F1CCEE;
enum Priority : uint8_t { PRIO_FADERS = 0,
                          PRIO_ARTNET = 1,
                          PRIO_HTP = 2 };
struct Config {
  uint32_t magic;
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
const int ADDR_CONFIG = 0;
ESP8266WebServer server(80);
WebSocketsServer webSocket(81);
DMXESPSerial dmx;
ArtnetWiFi artnet;
ESPAsyncE131 sacn(1);
uint8_t faderValues[DMX_CHANNELS];
uint8_t artnetValues[DMX_CHANNELS];
uint8_t sceneLevels[MAX_SCENES];
uint8_t scenesOut[DMX_CHANNELS];
uint8_t outValues[DMX_CHANNELS];
uint8_t sceneNonEmpty[MAX_SCENES];
uint8_t sceneDataCache[MAX_SCENES][DMX_CHANNELS];
bool haAssignedChannels[DMX_CHANNELS];
CRGB leds[NUM_LEDS];
uint8_t ledArtnetValues[DMX_CHANNELS];
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
bool espNowReady = false;
uint32_t restartAt = 0;
uint8_t baseR = 0, baseG = 0, baseB = 0;
uint8_t flashR = 0, flashG = 0, flashB = 0;
uint32_t flashUntil = 0;
void ledApply(uint8_t r, uint8_t g, uint8_t b) {
  analogWrite(LED_R, 255 - r);
  analogWrite(LED_G, 255 - g);
  analogWrite(LED_B, 255 - b);
}
void ledSetBase(uint8_t r, uint8_t g, uint8_t b) {
  baseR = r;
  baseG = g;
  baseB = b;
}
void ledFlash(uint8_t r, uint8_t g, uint8_t b, uint16_t ms) {
  flashR = r;
  flashG = g;
  flashB = b;
  flashUntil = millis() + ms;
}
void ledTask() {
  if (millis() < flashUntil) ledApply(flashR, flashG, flashB);
  else ledApply(baseR, baseG, baseB);
}
void ledInit() {
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  analogWriteRange(255);
  ledSetBase(0, 0, 0);
  ledApply(0, 0, 0);
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
void sendEspNowBlock(const uint8_t* data, uint16_t len_total);
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
  c.universe = 0;
  c.priority = PRIO_FADERS;
  c.ap_only = 0;
  c.use_static_ip = 0;
  strncpy(c.nodeName, "ConsoleDMX", sizeof(c.nodeName) - 1);
  c.dmx_protocol = 1;
  c.espnow_mode = 0;
  c.espnow_channel = 1;
  c.espnow_intensity_only = 0;
  c.led_artnet_enable = 0;
  c.led_universe = 1;
  c.led_color_order = 0; // 0=GRB, 1=RGB, 2=BGR
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
  if (!LittleFS.begin()) {
    Serial.println("LittleFS: format…");
    if (!LittleFS.format()) {
      Serial.println("LittleFS format FAIL");
      return false;
    }
    if (!LittleFS.begin()) {
      Serial.println("LittleFS mount FAIL");
      return false;
    }
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
  } else {
    // No active chaser config, load preset 0 by default
    loadChaserPreset(0, 255);
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
void espNowOnDataRecv(uint8_t* mac, uint8_t* incomingData, uint8_t len) {
  const int max_data_len = 248;
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
      for (uint16_t i = 0; i < data_len; i++) {
        dmx.write(packet.start_channel + i + 1, outValues[packet.start_channel + i]);
      }
      ledFlash(0, 0, 255, 50);
    }
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
  if (cfg.espnow_mode == 2) return;
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
void sendEspNowBlock(const uint8_t* data, uint16_t len_total) {
  if (!(cfg.espnow_mode == 1 && espNowReady)) return;
  const int max_data_len = 248;
  struct __attribute__((packed)) {
    uint16_t start_channel;
    uint8_t data[max_data_len];
  } packet;
  uint8_t broadcastAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
  for (uint16_t i = 0; i < len_total; i += max_data_len) {
    packet.start_channel = i;
    uint16_t len_to_send = len_total - i;
    if (len_to_send > max_data_len) len_to_send = max_data_len;
    memcpy(packet.data, data + i, len_to_send);
    esp_now_send(broadcastAddress, (uint8_t*)&packet, sizeof(packet.start_channel) + len_to_send);
  }
}
void setBlackout(bool enable) {
  if (enable && !blackoutActive) {
    memcpy(savedOut, outValues, DMX_CHANNELS);
    blackoutActive = true;
    webSocket.broadcastTXT("ha_blackout:1");
    if (cfg.led_artnet_enable) {
      FastLED.clear();
      FastLED.show();
    }
    ledFlash(0, 0, 255, 150);
    applyOutput();
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
    ledFlash(0, 255, 0, 150);
    applyOutput();
  }
}
void applyOutput() {
  if (cfg.espnow_mode == 2) return;
  if (blackoutActive) {
    uint8_t blackout_values[DMX_CHANNELS];
    memset(blackout_values, 0, DMX_CHANNELS);

    // Handle "B" button passthrough first
    for (int i = 0; i < DMX_CHANNELS; i++) {
      if (ignoreBlackout[i]) {
        blackout_values[i] = savedOut[i];
      }
    }

    // Then, handle live X/Y passthrough (overwriting if necessary)
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

    for (int i = 0; i < DMX_CHANNELS; i++) {
      dmx.write(i + 1, blackout_values[i]);
    }
    dmx.update();
    sendEspNowBlock(blackout_values, DMX_CHANNELS);
    return;
  }
  if (chaser_running) {
    memcpy(outValues, chaser_out_values, DMX_CHANNELS);
  } else {
    for (int i = 0; i < DMX_CHANNELS; i++) {
      uint8_t base = 0;
      switch (cfg.priority) {
        case PRIO_FADERS: base = faderValues[i]; break;
        case PRIO_ARTNET: base = artnetValues[i]; break;
        case PRIO_HTP: base = max(faderValues[i], artnetValues[i]); break;
      }
      uint8_t v = max(base, scenesOut[i]);
      if (haAssignedChannels[i]) { v = 0; }
      outValues[i] = v;
    }
  }
  // Apply XY pad values
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
  for (int i = 0; i < DMX_CHANNELS; i++) {
    dmx.write(i + 1, outValues[i]);
  }
  if (cfg.espnow_mode == 1 && espNowReady) {
    if (cfg.espnow_intensity_only == 1) {
      uint8_t espnow_values[DMX_CHANNELS];
      memcpy(espnow_values, outValues, DMX_CHANNELS);
      for (uint8_t i = 0; i < num_x_channels; i++) {
        uint16_t ch = xy_pad_x_channels[i].ch;
        if (ch > 0 && ch <= DMX_CHANNELS) {
          espnow_values[ch - 1] = 0;
        }
      }
      for (uint8_t i = 0; i < num_y_channels; i++) {
        uint16_t ch = xy_pad_y_channels[i].ch;
        if (ch > 0 && ch <= DMX_CHANNELS) {
          espnow_values[ch - 1] = 0;
        }
      }
      sendEspNowBlock(espnow_values, DMX_CHANNELS);
    } else {
      sendEspNowBlock(outValues, DMX_CHANNELS);
    }
  }
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
  memset(artnetValues, 0, sizeof(artnetValues));
  memset(ignoreBlackout, 0, sizeof(ignoreBlackout));
  memset(sceneLevels, 0, sizeof(sceneLevels));
  memset(scenesOut, 0, sizeof(scenesOut));
  memset(outValues, 0, sizeof(outValues));
  memset(sceneNonEmpty, 0, sizeof(sceneNonEmpty));
  blackoutActive = false;
  recomputeScenes();
  applyOutput();
  ledFlash(255, 0, 0, 400);
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
  out += String((int)cfg.dmx_protocol);
  out += ",";
  out += String(cfg.universe);
  out += ",";
  out += String((int)cfg.priority);
  out += ",";
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
  out += String((int)cfg.espnow_mode);
  out += ",";
  out += String(cfg.espnow_channel);
  out += ",";
  out += String((int)cfg.espnow_intensity_only);
  out += ",";
  out += String((int)blackoutActive);
  out += ",";
  out += String((int)cfg.led_artnet_enable);
  out += ",";
  out += String(cfg.led_universe);
  out += ",";
  out += String((int)cfg.led_color_order);
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
  
  // Add fader customizations
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
void loadChaserPreset(uint8_t idx, uint8_t clientNum) {
  if (idx >= 4) return;
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
          Serial.printf("Chaser preset %d loaded.\n", idx);
      }
    }
  }
}
void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connecté depuis %s\n", num, ip.toString().c_str());
        ledFlash(255, 255, 255, 120);
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
                // Maybe send an update to all clients? For now, not needed.
              }
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
            int channel = msg.substring(11, p1).toInt();
            int intensity_only = msg.substring(p1 + 1).toInt();
            if (channel >= 0 && channel <= 14) {
              cfg.espnow_channel = (uint8_t)channel;
            }
            cfg.espnow_intensity_only = (intensity_only != 0) ? 1 : 0;
            saveConfig();
            restartAt = millis() + 800;
          }
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
          for (int i = 0; i < DMX_CHANNELS; i++) {
            if (haAssignedChannels[i]) {
              String msg = "ha_val:" + String(i + 1) + ":0";
              webSocket.broadcastTXT(msg);
            }
          }
          return;
        }
        if (msg == "all_zero") {
          memset(faderValues, 0, sizeof(faderValues));
          pushFaders();
          for (int i = 0; i < DMX_CHANNELS; i++) {
            if (haAssignedChannels[i]) {
              String msg = "ha_val:" + String(i + 1) + ":0";
              webSocket.broadcastTXT(msg);
            }
          }
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
              // Broadcast the change to all clients
              webSocket.broadcastTXT(msg);
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
          // Reset in-memory arrays to default
          for(int i=0; i<NUM_FADERS; i++) {
            faderNames[i] = "CH " + String(i+1);
            faderColors[i] = "#262626";
          }
          // Tell clients to reload
          webSocket.broadcastTXT("reinit");
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
            clearSceneFS((uint8_t)idx); // This now also clears robo file
            String p = haScenePath(idx);
            if (LittleFS.exists(p)) LittleFS.remove(p);
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
              // 1. Load and send DMX faders
              memcpy(faderValues, sceneDataCache[idx], DMX_CHANNELS);
              pushFaders();
              applyOutput();

              // 2. Load and send Robo data if it exists
              String roboP = roboScenePath(idx);
              if (LittleFS.exists(roboP)) {
                File roboF = LittleFS.open(roboP, "r");
                if (roboF) {
                  String roboJson = roboF.readString();
                  roboF.close();
                  if (roboJson.length() > 2) { // check for non-empty json
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
              String p = haScenePath(idx);
              if (LittleFS.exists(p)) {
                File f = LittleFS.open(p, "r");
                if (f) {
                  String haJson = f.readString();
                  f.close();
                  if (haJson.length() > 2) {
                    String payload = "ha_load:" + String(idx) + ":";
                    payload += haJson;
                    webSocket.broadcastTXT(payload);
                  }
                }
              }
            }
          }
          return;
        }
        if (msg.startsWith("cfg:")) {
          int p1 = msg.indexOf(':', 4);
          if (p1 > 0) {
            int univ = msg.substring(4, p1).toInt();
            int prio = msg.substring(p1 + 1).toInt();
            cfg.universe = (uint8_t)constrain(univ, 0, 255);
            cfg.priority = (uint8_t)constrain(prio, 0, 2);
            saveConfig();
            Serial.printf("Config maj: universe=%u priority=%u\n", cfg.universe, cfg.priority);
            if (cfg.dmx_protocol == 1) {
              artnet.begin();
              artnet.subscribeArtDmxUniverse((uint16_t)cfg.universe,
                                             [&](const uint8_t* data, uint16_t size, const ArtDmxMetadata& metadata, const ArtNetRemoteInfo& remote) {
                                               for (int i = 0; i < DMX_CHANNELS && i < size; i++) artnetValues[i] = data[i];
                                               applyOutput();
                                             });
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
            bool prevLedEn = cfg.led_artnet_enable;
            uint8_t prevLedUniv = cfg.led_universe;
            uint8_t prevLedColorOrder = cfg.led_color_order;
            int proto = msg.substring(6, p1).toInt();
            int unv = msg.substring(p1 + 1, p2).toInt();
            int pr = msg.substring(p2 + 1, p3).toInt();
            String nm = (p4 > 0) ? msg.substring(p3 + 1, p4) : msg.substring(p3 + 1);
            nm.trim();
            nm.replace("|", " ");
            nm.replace(",", " ");
            nm.replace(":", " ");
            cfg.dmx_protocol = (uint8_t)constrain(proto, 0, 3);
            cfg.universe = (uint8_t)constrain(unv, 0, 255);
            cfg.priority = (uint8_t)constrain(pr, 0, 2);
            strncpy(cfg.nodeName, nm.c_str(), sizeof(cfg.nodeName) - 1);
            cfg.nodeName[sizeof(cfg.nodeName) - 1] = 0;
            if (p4 > 0 && p5 > 0) {
              int ledEn = msg.substring(p4 + 1, p5).toInt();
              int ledUnv = (p6 > 0) ? msg.substring(p5 + 1, p6).toInt() : msg.substring(p5 + 1).toInt();
              cfg.led_artnet_enable = (uint8_t)constrain(ledEn, 0, 1);
              cfg.led_universe = (uint8_t)constrain(ledUnv, 0, 255);
              if (p6 > 0) {
                int ledOrder = msg.substring(p6 + 1).toInt();
                cfg.led_color_order = (uint8_t)constrain(ledOrder, 0, 2);
              }
            }
            saveConfig();
            bool needsRestart = (prevProto != cfg.dmx_protocol) ||
                                (cfg.dmx_protocol > 0 && (prevLedEn != cfg.led_artnet_enable || prevLedUniv != cfg.led_universe || prevLedColorOrder != cfg.led_color_order));
            if (needsRestart) {
              restartAt = millis() + 800;
            } else {
            }
            sendInit(num);
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
  s += String(ESP.getChipId(), HEX);
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
    ledTask();
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    wifiAPMode = false;
    Serial.printf("STA OK, IP: %s\n", WiFi.localIP().toString().c_str());
    ledSetBase(0, 255, 0);
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
  ledSetBase(0, 0, 255);
}
void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.println("\nBoot DMX Web + Art-Net (hideakitai) + 12 Scenes + RGB LED + ON/OFF Art-Net + LittleFS + BLACKOUT");
  ledInit();
  FastLED.setBrightness(255);
  ledSetBase(255, 0, 255);
  EEPROM.begin(EEPROM_SIZE);
  loadConfig();

  switch (cfg.led_color_order) {
    default:
    case 0: FastLED.addLeds<LED_TYPE, LED_DATA_PIN, GRB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip); break;
    case 1: FastLED.addLeds<LED_TYPE, LED_DATA_PIN, RGB>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip); break;
    case 2: FastLED.addLeds<LED_TYPE, LED_DATA_PIN, BGR>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip); break;
  }

  if (!fsInit()) {
    Serial.println("FS init failed, formatting again…");
    LittleFS.format();
    fsInit();
  }
  dmx.init(DMX_CHANNELS);
  memset(faderValues, 0, sizeof(faderValues));
  memset(artnetValues, 0, sizeof(artnetValues));
  loadSceneLevels();
  memset(scenesOut, 0, sizeof(scenesOut));
  memset(outValues, 0, sizeof(outValues));
  memset(sceneNonEmpty, 0, sizeof(sceneNonEmpty));
  memset(savedOut, 0, sizeof(savedOut));
  memset(haAssignedChannels, 0, sizeof(haAssignedChannels));
  memset(ledArtnetValues, 0, sizeof(ledArtnetValues));
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
  server.on("/", handleRoot);
  server.on("/wifi", HTTP_POST, handleWifiPost);
  server.on("/factory", HTTP_POST, handleFactoryReset);
  server.begin();
  Serial.println("HTTP serveur :80");
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket :81");
  switch (cfg.dmx_protocol) {
    case 1:
      artnet.begin();
      artnet.setArtPollReplyConfigShortName(String(cfg.nodeName));
      artnet.setArtPollReplyConfigLongName(String(cfg.nodeName));
      artnet.subscribeArtDmxUniverse((uint16_t)cfg.universe,
                                     [&](const uint8_t* data, uint16_t size, const ArtDmxMetadata& metadata, const ArtNetRemoteInfo& remote) {
                                       if (blackoutActive) return;
                                       for (int i = 0; i < DMX_CHANNELS && i < size; i++) {
                                         if (data[i] != artnetValues[i] && haAssignedChannels[i]) {
                                           String msg = "ha_val:" + String(i + 1) + ":" + String(data[i]);
                                           webSocket.broadcastTXT(msg);
                                         }
                                         artnetValues[i] = data[i];
                                       }
                                       applyOutput();
                                       ledFlash(0, 255, 255, 120);
                                     });
      if (cfg.led_artnet_enable) {
        artnet.subscribeArtDmxUniverse((uint16_t)cfg.led_universe,
          [&](const uint8_t* data, uint16_t size, const ArtDmxMetadata& metadata, const ArtNetRemoteInfo& remote) {
            if (blackoutActive) return;
            for (int i = 0; i < DMX_CHANNELS && i < size; i++) ledArtnetValues[i] = data[i];
            for (int i = 0; i < NUM_LEDS; i++) {
              int dmx_index = i * 3;
              if (dmx_index + 2 < size && dmx_index + 2 < DMX_CHANNELS) {
                leds[i].setRGB(ledArtnetValues[dmx_index], ledArtnetValues[dmx_index + 1], ledArtnetValues[dmx_index + 2]);
              }
            }
            FastLED.show();
            ledFlash(255, 0, 255, 40);
          });
        Serial.printf("Art-Net: Subscribed to LED universe %u\n", cfg.led_universe);
      }
      Serial.println("Protocole: Art-Net activé.");
      break;
    case 2:
      if (sacn.begin(E131_MULTICAST, cfg.universe)) {
        Serial.printf("Protocole: sACN E1.31 activé, univers %u\n", cfg.universe);
      } else {
        Serial.println("Erreur: Impossible de démarrer sACN.");
      }
      break;
    default:
      Serial.println("Protocole: DMX réseau désactivé.");
      break;
  }

  if (cfg.espnow_mode > 0) {
    if (cfg.espnow_channel > 0 && cfg.espnow_channel <= 14) {
      Serial.printf("ESP-NOW: Setting channel to %d\n", cfg.espnow_channel);
      wifi_set_channel(cfg.espnow_channel);
    }
  }
  if (cfg.espnow_mode == 1) {
    Serial.println("ESP-NOW: Init Master...");
    if (esp_now_init() == 0) {
      esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
      uint8_t broadcastAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
      esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 0, 0, 0);
      Serial.println("ESP-NOW: Master Ready. Broadcast peer added.");
      espNowReady = true;
    } else {
      Serial.println("ESP-NOW: Init Master Failed.");
    }
  } else if (cfg.espnow_mode == 2) {
    Serial.println("ESP-NOW: Init Slave...");
    if (esp_now_init() == 0) {
      esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
      esp_now_register_recv_cb(espNowOnDataRecv);
      Serial.println("ESP-NOW: Slave Ready.");
      espNowReady = true;
    } else {
      Serial.println("ESP-NOW: Init Slave Failed.");
    }
  }
  Serial.println("Prêt.");
}
void loop() {
  server.handleClient();
  webSocket.loop();
  if (cfg.dmx_protocol == 1) {
    artnet.parse();
  } else if (cfg.dmx_protocol == 2) {
    while (!sacn.isEmpty()) {
      e131_packet_t packet;
      sacn.pull(&packet);
      if (!blackoutActive) {
        for (int i = 0; i < DMX_CHANNELS && i < packet.property_value_count - 1; i++) {
          uint8_t val = packet.property_values[i + 1];
          if (val != artnetValues[i] && haAssignedChannels[i]) {
            String msg = "ha_val:" + String(i + 1) + ":" + String(val);
            webSocket.broadcastTXT(msg);
          }
          artnetValues[i] = val;
        }
        applyOutput();
        ledFlash(0, 255, 255, 120);
      }
    }
  }
  chaserTask();
  MDNS.update();
  dmx.update();
  ledTask();
  if (restartAt && millis() > restartAt) {
    Serial.println("Redémarrage demandé (ON/OFF Art-Net)...");
    delay(100);
    ESP.restart();
  }
}