#include <WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <vector>

// ------------------- Konstanta Pin -------------------
#define DHTPIN 32
#define RELAY_PIN 26
#define RELAY_PIN2 27
#define RELAY_PIN3 14
#define DHTTYPE DHT11

//-------------------- STRUCK -------------------------
struct JadwalPenyiraman {
  std::vector<String> hari;
  String waktu;
};

// ------------------- WiFi Manager -------------------
class WiFiManager {
public:
  WiFiManager(const char* ssid, const char* password)
    : _ssid(ssid), _password(password) {}

  void connect() {
    WiFi.begin(_ssid, _password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nWiFi connected, IP address: ");
    Serial.println(WiFi.localIP());
  }

private:
  const char* _ssid;
  const char* _password;
};

// ------------------- Supabase Client -------------------
class SupabaseClient {
public:
  JadwalPenyiraman jadwal;
  SupabaseClient(const String& url, const String& apiKey)
    : _url(url), _apiKey(apiKey) {}

  String fetchData(const String& endpoint) {
    return makeRequest(endpoint, "GET");
  }

  float fetchThreshold(const String& table, const String& column) {
    String endpoint = "/rest/v1/" + table + "?select=" + column + "&limit=1";
    String response = fetchData(endpoint);

    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, response);
    if (error) {
      Serial.print("JSON error: ");
      Serial.println(error.f_str());
      return -1.0;
    }

    if (doc[0].containsKey(column)) {
      return doc[0][column].as<int>();
    }

    Serial.println("Key not found in JSON");
    return -1.0;
  }



  JadwalPenyiraman fetchJadwalPenyiramanById(const String& table, int id) {
    JadwalPenyiraman jadwal;
    String endpoint = "/rest/v1/" + table + "?id_jadwal_penyiraman=eq." + String(id) + "&select=hari,waktu&limit=1";

    String response = fetchData(endpoint);

    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, response);

    if (error) {
      Serial.print("JSON error: ");
      Serial.println(error.f_str());
      return jadwal;
    }

    if (doc.is<JsonArray>() && doc.size() > 0) {
      JsonObject obj = doc[0];
      if (obj.containsKey("hari") && obj.containsKey("waktu")) {
        JsonArray hariArray = obj["hari"];
        for (JsonVariant h : hariArray) {
          jadwal.hari.push_back(h.as<String>());
        }
        jadwal.waktu = obj["waktu"].as<String>();
      }
    } else {
      Serial.println("Invalid JSON or empty result");
    }

    return jadwal;
  }

  void insertRiwayatPendingin(float suhu) {
    StaticJsonDocument<64> doc;
    doc["suhu"] = suhu;

    String body;
    serializeJson(doc, body);

    String endpoint = "/rest/v1/riwayat_pendingin";
    String response = makeRequest(endpoint, "POST", body);

    Serial.print("Insert riwayat_pendingin response: ");
    Serial.println(response);
  }

  void insertRiwayatPenyiraman(int idJalurPenyiraman) {
    StaticJsonDocument<128> doc;
    doc["id_jalur_penyiraman"] = idJalurPenyiraman; 

    String body;
    serializeJson(doc, body);

    String endpoint = "/rest/v1/riwayat_penyiraman";
    String response = makeRequest(endpoint, "POST", body);

    Serial.print("Insert riwayat_penyiraman response (relay ");
    Serial.print(idJalurPenyiraman);
    Serial.print("): ");
    Serial.println(response);
  }


private:
  String _url;
  String _apiKey;

  String makeRequest(const String& endpoint, const String& method = "GET", const String& body = "") {
    HTTPClient http;
    http.begin(_url + endpoint);
    http.addHeader("apikey", _apiKey);
    http.addHeader("Authorization", "Bearer " + _apiKey);
    http.addHeader("Content-Type", "application/json");

    int httpCode = -1;
    if (method == "GET") httpCode = http.GET();
    else if (method == "POST") httpCode = http.POST(body);
    else if (method == "PATCH") httpCode = http.PATCH(body);

    String payload;
    if (httpCode > 0) {
      payload = http.getString();wqa
    } else {
      Serial.print("HTTP Request failed: ");
      Serial.println(httpCode);
    }
    http.end();
    return payload;
  }
};

// ------------------- Relay Controller -------------------
class RelayController {
public:
  RelayController(uint8_t pin)
    : _pin(pin) {}

  void begin() {
    pinMode(_pin, OUTPUT);
    turnOff();
  }

  uint8_t getPin() const {
    return _pin;
  }

  void setMQTTClient(PubSubClient* client, const char* topic) {
    _mqttClient = client;
    _mqttTopic = topic;
  }

  void setSupabaseClient(SupabaseClient* client) {
    _supabaseClient = client;
  }

  void turnOn() {
    digitalWrite(_pin, LOW);
    if (!_status) {
      Serial.println("Relay ON");
      _status = true;
    }
  }

  void turnOff() {
    digitalWrite(_pin, HIGH);
    if (_status) {
      Serial.println("Relay OFF");
      _status = false;
    }
  }

  void setManualOverride(bool override) {
    _manualOverride = override;
    Serial.print("Manual override set to: ");
    Serial.println(_manualOverride ? "ON" : "OFF");
  }

  void controlByTempWithFeedback(float temp, int threshold) {
    if (_manualOverride) {
      Serial.println("Otomatisasi dinonaktifkan (Relay on)");
      return;
    }
    bool prevStatus = _status;
    if (temp >= threshold) {
      turnOn();

    } else {
      turnOff();
    }

    if (_mqttClient && (_status != prevStatus)) {
      StaticJsonDocument<64> doc;
      doc["status"] = _status ? "On" : "Off";
      doc["source"] = "auto";

      char buffer[64];
      size_t len = serializeJson(doc, buffer);
      _mqttClient->publish(_mqttTopic, (const uint8_t*)buffer, len, true);

      Serial.print("Auto status published: ");
      Serial.println(buffer);
    }
    if (!_manualOverride && !_inserted && _status && temp >= threshold) {
      _inserted = true;
      if (_supabaseClient) {
        _supabaseClient->insertRiwayatPendingin(temp);
      }
    }

    if (temp < threshold && _inserted) {
      _inserted = false;
    }
  }

  bool isOn() const {
    return _status;
  }

private:
  uint8_t _pin;
  bool _status = false;
  bool _inserted = false;
  bool _manualOverride = false;
  PubSubClient* _mqttClient = nullptr;
  const char* _mqttTopic = nullptr;
  SupabaseClient* _supabaseClient = nullptr;
};


//-------------------- JADWAL PENYIRAMAN --------------
class JadwalPenyiramanManager {
private:
  SupabaseClient& supabase;
  struct Penyiram {
    RelayController* relay;
    int jalurPenyiraman;
    int jadwalId;
    JadwalPenyiraman jadwal;
    bool isActive = false;
    bool hasWateredToday = false;
    unsigned long lastTurnOnTime = 0;
    unsigned long scheduledTime = 0;  
  };
  std::vector<Penyiram> penyirams;
  unsigned long lastPollTime = 0;
  unsigned long pollInterval = 5000; 
  String lastCheckedDay = "";

  int getCurrentTimeInSeconds() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) return -1;
    return timeinfo.tm_hour * 3600 + timeinfo.tm_min * 60 + timeinfo.tm_sec;
  }

  int convertTimeStringToSeconds(const String& timeStr) {
    int h = 0, m = 0, s = 0;
    int result = sscanf(timeStr.c_str(), "%d:%d:%d", &h, &m, &s);
    if (result != 3) {
      Serial.printf("Error parsing waktu: %s\n", timeStr.c_str());
      return -1;
    }
    return h * 3600 + m * 60 + s;
  }

  String getCurrentDay() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) return "";
    const char* days[] = { "Minggu", "Senin", "Selasa", "Rabu", "Kamis", "Jumat", "Sabtu" };
    return String(days[timeinfo.tm_wday]);
  }

  void handleAllPenyiraman() {
    String currentDay = getCurrentDay();
    int currentSeconds = getCurrentTimeInSeconds();

    if (currentSeconds == -1) return;

    for (auto& penyiram : penyirams) {
      if (penyiram.isActive || penyiram.hasWateredToday) {
        continue;
      }

      int jadwalSeconds = convertTimeStringToSeconds(penyiram.jadwal.waktu);
      if (jadwalSeconds == -1) continue;

      // Cek apakah hari ini sesuai jadwal
      bool hariSesuai = false;
      for (const String& hari : penyiram.jadwal.hari) {
        if (currentDay == hari) {
          hariSesuai = true;
          break;
        }
      }

      if (hariSesuai) {
        int selisih = abs(currentSeconds - jadwalSeconds);
        if (selisih <= 60) {
          activatePenyiram(penyiram, currentDay);
        }
      }
    }

    handleActivePenyiram();
  }

  void activatePenyiram(Penyiram& penyiram, const String& currentDay) {
    penyiram.relay->turnOn();
    penyiram.isActive = true;
    penyiram.hasWateredToday = true;
    penyiram.lastTurnOnTime = millis();

    Serial.printf("Relay %d ON (jadwal id %d) - Hari: %s, Waktu: %s\n",
                  penyiram.relay->getPin(), penyiram.jadwalId,
                  currentDay.c_str(), penyiram.jadwal.waktu.c_str());

    supabase.insertRiwayatPenyiraman(penyiram.jalurPenyiraman);
  }

  void handleActivePenyiram() {
    for (auto& penyiram : penyirams) {
      if (penyiram.isActive && millis() - penyiram.lastTurnOnTime >= 5000) {
        penyiram.relay->turnOff();
        penyiram.isActive = false;
        Serial.printf("Relay %d OFF (selesai menyiram)\n", penyiram.relay->getPin());
      }
    }
  }

public:
  JadwalPenyiramanManager(SupabaseClient& supabaseRef)
    : supabase(supabaseRef) {}

  void addPenyiram(RelayController* relay, int jadwalId, int jalurPenyiraman) {
    penyirams.push_back({ relay, jadwalId, jalurPenyiraman });
  }

  void pollJadwalIfNeeded() {
    if (millis() - lastPollTime >= pollInterval) {
      lastPollTime = millis();

      String currentDay = getCurrentDay();
      if (currentDay != lastCheckedDay) {
        for (auto& penyiram : penyirams) {
          penyiram.hasWateredToday = false;
        }
        lastCheckedDay = currentDay;
        Serial.println("Hari berganti, reset status penyiraman.");
      }

      // Update jadwal dari Supabase
      for (auto& penyiram : penyirams) {
        JadwalPenyiraman newJadwal = supabase.fetchJadwalPenyiramanById("jadwal_penyiraman", penyiram.jadwalId);

        // Cek apakah jadwal berubah
        if (newJadwal.waktu != penyiram.jadwal.waktu || newJadwal.hari.size() != penyiram.jadwal.hari.size()) {
          penyiram.jadwal = newJadwal;
          Serial.printf("Jadwal pompa %d diperbarui: %s\n",
                        penyiram.relay->getPin(), newJadwal.waktu.c_str());
        }
      }
    }
  }

  void update() {
    pollJadwalIfNeeded();
    handleAllPenyiraman();  
  }

  void printStatus() {
    Serial.println("=== Status Jadwal Penyiraman ===");
    for (const auto& penyiram : penyirams) {
      Serial.printf("Relay %d (ID:%d) - Active: %s, Watered Today: %s, Waktu: %s\n",
                    penyiram.relay->getPin(), penyiram.jadwalId,
                    penyiram.isActive ? "Yes" : "No",
                    penyiram.hasWateredToday ? "Yes" : "No",
                    penyiram.jadwal.waktu.c_str());
    }
    Serial.println("==============================");
  }
};

// ------------------- MQTT Manager -------------------
class MQTTManager {
public:
  MQTTManager(const char* broker, int port,
              const char* controlTopic, const char* sensorTopic,
              const char* penyiraman1Topic, const char* penyiraman2Topic,
              RelayController* relay, RelayController* relay2, RelayController* relay3, DHT* dht)
    : _broker(broker), _port(port),
      _controlTopic(controlTopic), _sensorTopic(sensorTopic),
      _penyiraman1Topic(penyiraman1Topic), _penyiraman2Topic(penyiraman2Topic),
      _relay(relay), _relay2(relay2), _relay3(relay3), _dht(dht), _client(_wifiClient) {
    instance = this;
  }

  void begin() {
    _client.setServer(_broker, _port);
    _client.setCallback(callbackWrapper);
    _relay->setMQTTClient(&_client, _controlTopic);
    _relay2->setMQTTClient(&_client, _penyiraman1Topic);
    _relay3->setMQTTClient(&_client, _penyiraman2Topic);
    connect();
  }

  void loop() {
    if (!_client.connected()) {
      connect();
    }
    _client.loop();

    if (millis() - _lastPublish >= 10000) {
      _lastPublish = millis();
      publishSensorData();
    }
  }

  float getCurrentTemperature() {
    return _dht->readTemperature();
  }

private:
  WiFiClient _wifiClient;
  PubSubClient _client;

  const char* _broker;
  int _port;
  const char* _controlTopic;
  const char* _sensorTopic;
  const char* _penyiraman1Topic;
  const char* _penyiraman2Topic;

  RelayController* _relay;
  RelayController* _relay2;
  RelayController* _relay3;
  DHT* _dht;
  unsigned long _lastPublish = 0;

  static MQTTManager* instance;

  void connect() {
    while (!_client.connected()) {
      Serial.print("Connecting to MQTT...");
      String clientId = "ESP32Client-" + String((uint32_t)ESP.getEfuseMac(), HEX);
      if (_client.connect(clientId.c_str())) {
        Serial.println("connected");
        _client.subscribe(_controlTopic);
        _client.subscribe(_penyiraman1Topic);
        _client.subscribe(_penyiraman2Topic);
        Serial.print("Subscribed to topic: ");
        Serial.println(_controlTopic);
      } else {
        Serial.print("Failed, rc=");
        Serial.print(_client.state());
        Serial.println(" try again in 5 seconds");
        delay(5000);
      }
    }
  }

  static void callbackWrapper(char* topic, byte* payload, unsigned int length) {
    if (instance) {
      instance->handleCallback(topic, payload, length);
    }
  }

  void handleCallback(char* topic, byte* payload, unsigned int length) {
    Serial.print("MQTT Message [");
    Serial.print(topic);
    Serial.print("]: ");

    String message;
    for (unsigned int i = 0; i < length; i++) {
      message += (char)payload[i];
    }
    Serial.println(message);

    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, message);

    if (error) {
      Serial.print("JSON Parsing failed: ");
      Serial.println(error.f_str());
      return;
    }

    RelayController* targetRelay = nullptr;

    if (strcmp(topic, _controlTopic) == 0) {
      targetRelay = _relay;
    } else if (strcmp(topic, _penyiraman1Topic) == 0) {
      targetRelay = _relay2;
    } else if (strcmp(topic, _penyiraman2Topic) == 0) {
      targetRelay = _relay3;
    }

    if (targetRelay && doc.containsKey("status") && doc.containsKey("source")) {
      String status = doc["status"];
      String source = doc["source"];

      if (source == "manual") {
        targetRelay->setManualOverride(true);
        if (status == "On") {
          targetRelay->turnOn();
        } else if (status == "Off") {
          targetRelay->setManualOverride(false);
          targetRelay->turnOff();
        }
      }
    }
  }

  void publishSensorData() {
    float temp = _dht->readTemperature();
    float hum = _dht->readHumidity();

    if (!isnan(temp) && !isnan(hum)) {
      int humidityInt = (int)hum;
      String payload = "{\"suhu\": " + String(temp) + ", \"kelembaban\": " + String(humidityInt) + "}";
      _client.publish(_sensorTopic, payload.c_str());
      Serial.print("Published sensor data: ");
      Serial.println(payload);
    } else {
      Serial.println("Failed to read DHT sensor!");
    }
  }
};

MQTTManager* MQTTManager::instance = nullptr;

// ------------------- Global Instance -------------------
const char* ssid = "RUMAH KITA";
const char* password = "lulus_cepat";

const char* mqttBroker = "broker.hivemq.com";
const int mqttPort = 1883;
const char* mqttStatusTopic = "orchitech/status";
const char* mqttSensorTopic = "orchitech/sensor";
const char* mqttPenyiraman1Topic = "orchitech/penyiraman1";
const char* mqttPenyiraman2Topic = "orchitech/penyiraman2";

const char* supabaseUrl = "https://lyxsovhvhalnlxizeqyc.supabase.co";
const char* supabaseApiKey = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6Imx5eHNvdmh2aGFsbmx4aXplcXljIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NDUwMzU4MTIsImV4cCI6MjA2MDYxMTgxMn0.RPOlZz58zP_UaYqSO5fOaSl_Iu99pZwNmFEQ5ODWafs";



struct Penyiram {
  RelayController* relay;
  int jadwalId;
  JadwalPenyiraman jadwal;
  bool isActive = false;
  bool hasWateredToday = false;
  unsigned long lastTurnOnTime = 0;
};


WiFiManager wifiManager(ssid, password);
RelayController relay(RELAY_PIN);
RelayController relay2(RELAY_PIN2);
RelayController relay3(RELAY_PIN3);
DHT dht(DHTPIN, DHTTYPE);
MQTTManager mqttManager(mqttBroker, mqttPort,
                        mqttStatusTopic, mqttSensorTopic,
                        mqttPenyiraman1Topic, mqttPenyiraman2Topic,
                        &relay, &relay2, &relay3, &dht);
SupabaseClient supabase(supabaseUrl, supabaseApiKey);
JadwalPenyiramanManager jadwalManager(supabase);

const unsigned long thresholdPollInterval = 10000;
unsigned long lastThresholdCheck = 0;
int latestThreshold = 30;

unsigned long lastJadwalCheck = 0;



// ------------------- Setup & Loop -------------------
void setup() {
  Serial.begin(115200);
  relay.begin();
  relay.setSupabaseClient(&supabase);
  relay2.begin();
  relay3.begin();
  dht.begin();
  wifiManager.connect();
  configTime(7 * 3600, 0, "pool.ntp.org");

  mqttManager.begin();

  jadwalManager.addPenyiram(&relay2, 1, 1);
  jadwalManager.addPenyiram(&relay3, 2, 2);
}

void loop() {
  mqttManager.loop();

  unsigned long currentMillis = millis();
  if (currentMillis - lastThresholdCheck >= thresholdPollInterval) {
    lastThresholdCheck = currentMillis;
    latestThreshold = supabase.fetchThreshold("status_aktivasi_pendingin", "batas_suhu");
  }

  float suhu = mqttManager.getCurrentTemperature();
  if (latestThreshold >= 0) {
    relay.controlByTempWithFeedback(suhu, latestThreshold);
  }

  jadwalManager.update();
  delay(3000);
}
