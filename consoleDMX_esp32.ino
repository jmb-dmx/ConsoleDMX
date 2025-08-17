#include <Arduino.h>
#include "esp_dmx.h"
#include <WiFi.h>
#include <WebServer.h>

// --- Configuration DMX ---
// Utilisation du port DMX 2 (UART2 de l'ESP32)
const dmx_port_t dmx_port = DMX_NUM_2;
const int tx_pin = 17;
const int rx_pin = 16;
const int rts_pin = 21;

// --- Configuration du point d'accès WiFi ---
const char* ssid = "ESP32-DMX-Control";
const char* password = "dmx12345";

// --- Serveur Web ---
// Le serveur écoutera sur le port 80 (HTTP standard)
WebServer server(80);

// --- Tampon de données DMX ---
// Un tableau pour stocker les valeurs des 512 canaux
byte dmx_data[DMX_PACKET_SIZE] = {0};

// --- Variables pour le rafraîchissement DMX ---
unsigned long last_dmx_update = 0;
// Intervalle d'envoi DMX en millisecondes (40Hz, une bonne fréquence pour le DMX)
const int dmx_update_interval = 25;

// --- Fonction pour générer la page HTML ---
String getHtmlPage() {
  // On construit la page HTML dans une chaîne de caractères
  String page = "<!DOCTYPE html><html><head><title>ESP32 DMX Control</title><meta name='viewport' content='width=device-width, initial-scale=1'>";
  page += "<style>body{font-family: Arial, sans-serif; background-color: #2c2c2c; color: #eee;} h1{text-align: center;}";
  page += ".slider-container{display: flex; align-items: center; margin-bottom: 12px; padding: 5px; background-color: #3a3a3a; border-radius: 5px;}";
  page += "label{width: 100px; text-align: right; margin-right: 15px;} input[type=range]{flex-grow: 1;} span{margin-left: 15px; font-weight: bold; width: 30px;}</style></head>";
  page += "<body><h1>Contrôleur DMX sur ESP32</h1>";

  // Boucle pour créer 16 sliders (canaux 1 à 16)
  for (int i = 1; i <= 16; i++) {
    page += "<div class='slider-container'>";
    page += "<label for='ch" + String(i) + "'>Canal " + String(i) + "</label>";
    page += "<input type='range' id='ch" + String(i) + "' min='0' max='255' value='" + String(dmx_data[i]) + "' oninput='updateDMX(" + String(i) + ", this.value)'>";
    page += "<span id='val" + String(i) + "'>" + String(dmx_data[i]) + "</span>";
    page += "</div>";
  }

  // Le script JavaScript qui envoie les données au serveur quand on bouge un curseur
  page += "<script>";
  page += "function updateDMX(channel, value) {";
  page += "document.getElementById('val' + channel).innerText = value;";
  page += "fetch('/set?channel=' + channel + '&value=' + value);"; // Envoie une requête GET au serveur
  page += "}</script>";
  page += "</body></html>";
  return page;
}

// --- Fonctions de gestion des requêtes web ---

// Fonction appelée quand on accède à la racine "/"
void handleRoot() {
  server.send(200, "text/html", getHtmlPage());
}

// Fonction appelée par le JavaScript pour mettre à jour une valeur DMX
void handleSet() {
  if (server.hasArg("channel") && server.hasArg("value")) {
    int channel = server.arg("channel").toInt();
    int value = server.arg("value").toInt();

    // On vérifie que le canal est valide (entre 1 et 512)
    if (channel >= 1 && channel < DMX_PACKET_SIZE) {
      dmx_data[channel] = value; // On met à jour la valeur dans notre tableau
      server.send(200, "text/plain", "OK"); // On répond au navigateur que tout va bien
    } else {
      server.send(400, "text/plain", "Canal Invalide");
    }
  } else {
    server.send(400, "text/plain", "Requete Invalide");
  }
}

void setup() {
  Serial.begin(115200);

  // --- Initialisation du DMX ---
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  dmx_driver_install(dmx_port, &config, NULL, 0);
  dmx_set_pin(dmx_port, tx_pin, rx_pin, rts_pin);
  Serial.println("Pilote DMX installé.");

  // --- Initialisation du point d'accès WiFi ---
  Serial.println("Configuration du point d'accès WiFi...");
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("Connectez-vous au WiFi '");
  Serial.print(ssid);
  Serial.println("'");
  Serial.print("Et rendez-vous sur l'adresse IP : http://");
  Serial.println(myIP);

  // --- Configuration des routes du serveur web ---
  server.on("/", HTTP_GET, handleRoot);   // Quand on va sur "/", on appelle handleRoot
  server.on("/set", HTTP_GET, handleSet); // Quand on reçoit une requête "/set", on appelle handleSet
  server.begin(); // On démarre le serveur
  Serial.println("Serveur Web démarré.");
}

void loop() {
  // Gère les requêtes des clients web
  server.handleClient();

  // Envoie les données DMX à intervalle régulier pour un signal stable
  if (millis() - last_dmx_update > dmx_update_interval) {
    last_dmx_update = millis();

    dmx_data[0] = 0; // On s'assure que le start code est toujours à 0

    dmx_write(dmx_port, dmx_data, DMX_PACKET_SIZE);
    dmx_send(dmx_port);
    // On n'attend pas la fin de l'envoi pour ne pas bloquer le serveur web,
    // la boucle est assez rapide pour que ça ne pose pas de problème.
    // dmx_wait_sent(dmx_port, 0);
  }
}