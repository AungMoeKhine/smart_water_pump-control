/*
 * Automatic Pump Control System (ESP32-S3 Migration)
 * Features: TFT Touch UI, Local Web Control, Cloud Control (MQTT)
 * 
 * Target Hardware: ESP32-S3, TFT (Touch), Ultrasonic (Upper Tank Only), Flow Sensor
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <time.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <ZMPT101B.h>
#include <Adafruit_NeoPixel.h>

// ============================================================================
// CONFIGURATION & PINOUT
// ============================================================================

// Pin Assignments (ESP32-S3)
const int VOLTAGE_SENSOR_PIN = 4;
const int UPPER_TANK_TRIG_PIN = 5;
const int UPPER_TANK_ECHO_PIN = 6;
const int BUZZER_PIN = 7;
const int MOTOR_PIN = 8;
const int FLOW_SENSOR_PIN = 18;
const int RGB_LED_PIN = 48;

// MQTT Configuration
const char* mqtt_server = "210195b635414206adcd944325fe6f59.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "my_switch";       
const char* mqtt_pass = "My_password123";  

// Constants
#define SAMPLE_BUFFER_SIZE 20
#define MAX_DISTANCE 84
#define ULTRASONIC_INTERVAL 4000 
#define GENERAL_INTERVAL 1000

// ============================================================================
// DATA STRUCTURES
// ============================================================================

struct VoltageConfig {
  int HIGH_THRESHOLD = 250;
  int LOW_THRESHOLD = 170;
  int WAIT_SECONDS_SET = 10;
  int waitSeconds = 10;
  int status = 1;                // 1 = normal, 0 = abnormal
  float currentVoltage = 0.0f;
  unsigned long lastCheck = 0;
};

struct TankConfig {
  static const int LOW_THRESHOLD = 20;     
  int FULL_THRESHOLD = 80;                 
  static constexpr float MIN_HEIGHT = 12.0f;   
  static constexpr float MAX_HEIGHT = 84.0f;   
  static constexpr float BUFFER_HEIGHT = 6.0f; 
  float upperHeight = MIN_HEIGHT;
  int rawUpperPercentage = 0;
  int displayUpperPercentage = 0;
  float upperDistance = 0;
  int upperInvalidCount = 0;
  static const int MAX_INVALID_COUNT = 10;
  bool errorAck = false; // Tracks if sensor alarm was silenced

  void updateFullThreshold() {
    float effectiveHeight = upperHeight + BUFFER_HEIGHT;
    FULL_THRESHOLD = (upperHeight * 0.8 / effectiveHeight) * 100;
  }
};

struct PumpConfig {
  int motorStatus = 0;             // 0 = OFF, 1 = ON
  bool isRunning = false;
  bool flowDetected = false;
  bool manualOverride = false;
  bool wasRunningBeforeVoltageError = false;
};

struct DryRunConfig {
  int WAIT_SECONDS_SET = 60;
  int waitSeconds = 60;
  int error = 0;                   // 0 = No error, 1 = Alarm, 2 = Locked
  unsigned long lastUpdate = 0;
  unsigned long alarmStartTime = 0;
};

enum class PumpState {
  IDLE, PUMPING, DRY_RUN_ALARM, DRY_RUN_LOCKED, SENSOR_ERROR, VOLTAGE_ERROR, VOLTAGE_WAIT
};

String getStateString(PumpState state) {
  switch(state) {
    case PumpState::IDLE: return "IDLE";
    case PumpState::PUMPING: return "PUMPING";
    case PumpState::DRY_RUN_ALARM: return "DRY_ALARM";
    case PumpState::DRY_RUN_LOCKED: return "LOCKED";
    case PumpState::SENSOR_ERROR: return "SENSOR_ERROR";
    case PumpState::VOLTAGE_ERROR: return "VOLTAGE_ERROR";
    case PumpState::VOLTAGE_WAIT: return "VOLTAGE_WAIT";
    default: return "UNKNOWN";
  }
}

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

VoltageConfig voltageConfig;
TankConfig tankConfig;
PumpConfig pumpConfig;
DryRunConfig dryRunConfig;
PumpState currentState = PumpState::IDLE;

Preferences preferences;
WiFiMulti wifiMulti;
WebServer server(80);
WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);
TFT_eSPI tft = TFT_eSPI();
Adafruit_NeoPixel rgbLed(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
ZMPT101B voltageSensor(VOLTAGE_SENSOR_PIN, 50.0);

String deviceID = "";
String subTopic = "";
String statusTopic = "";
String onlineTopic = ""; 
String devicePin = "123456"; 

WiFiUDP dnsUdp;
const byte DNS_PORT = 53;

// --- WiFi & DNS Helper ---
void processDNS() {
  int packetSize = dnsUdp.parsePacket();
  if (packetSize > 0) {
    unsigned char buf[512];
    dnsUdp.read(buf, 512);
    if ((buf[2] & 0x80) == 0) {
      buf[2] = 0x81; buf[3] = 0x80;
      buf[7] = 0x01;
      int pos = 12;
      while (buf[pos] != 0 && pos < packetSize) pos += buf[pos] + 1;
      pos += 5;
      buf[pos++] = 0xC0; buf[pos++] = 0x0C;
      buf[pos++] = 0x00; buf[pos++] = 0x01;
      buf[pos++] = 0x00; buf[pos++] = 0x01;
      buf[pos++] = 0x00; buf[pos++] = 0x00; buf[pos++] = 0x00; buf[pos++] = 0x3C;
      buf[pos++] = 0x00; buf[pos++] = 0x04;
      IPAddress ip = WiFi.softAPIP();
      buf[pos++] = ip[0]; buf[pos++] = ip[1]; buf[pos++] = ip[2]; buf[pos++] = ip[3];
      dnsUdp.beginPacket(dnsUdp.remoteIP(), dnsUdp.remotePort());
      dnsUdp.write(buf, pos);
      dnsUdp.endPacket();
    }
  }
}

// --- Web Interface ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Smart Pump Dashboard</title>
<style>
  body{font-family:sans-serif;background:#121212;color:white;text-align:center;padding:20px;margin:0;}
  .card{background:#1e1e1e;border-radius:12px;padding:20px;max-width:400px;margin:auto;box-shadow:0 4px 15px rgba(0,0,0,0.5);border:1px solid #333;position:relative;}
  .conn-dot{width:10px;height:10px;background:#28a745;border-radius:50%;display:inline-block;margin-right:5px;}
  .off{background:#dc3545;}
  .refresh-btn{position:absolute;top:15px;right:15px;background:none;border:none;color:#03ef;font-size:1.2rem;cursor:pointer;padding:5px;}
  .row{display:flex;justify-content:space-between;font-size:1.1rem;margin:12px 0;border-bottom:1px solid #333;padding-bottom:10px;}
  .btn{width:100%;padding:15px;color:white;background:#03ef;border:none;border-radius:8px;margin-top:10px;font-weight:bold;cursor:pointer;font-size:1.1rem; transition: background 0.3s;}
  .btn:active, .refresh-btn:active{transform:scale(0.96);opacity:0.85;}
  .btn-green{background:#28a745;}
  .btn-red{background:#dc3545;}
  @keyframes spin { 100% { transform: rotate(360deg); } }
  .spinning { animation: spin 1s linear infinite; opacity: 1 !important; color: #03ef62 !important; }
  .tank-wrap { width: 140px; height: 180px; border: 4px solid #333; border-radius: 15px; margin: 30px auto; position: relative; background: linear-gradient(90deg, #1a1a1a 0%, #333 50%, #1a1a1a 100%); overflow: visible; }
  .tank-wrap::before { content: ''; position: absolute; top: -14px; left: 50%; transform: translateX(-50%); width: 80px; height: 14px; background: linear-gradient(90deg, #1a1a1a 0%, #333 50%, #1a1a1a 100%); border-left: 4px solid #333; border-right: 4px solid #333; z-index: 1; }
  .tank-wrap::after { content: ''; position: absolute; top: -24px; left: 50%; transform: translateX(-50%); width: 100px; height: 10px; background: linear-gradient(90deg, #1a1a1a 0%, #333 50%, #1a1a1a 100%); border: 3px solid #333; border-radius: 4px; z-index: 5; }
  .tank-inner { position: absolute; top: 0; left: 0; right: 0; bottom: 0; border-radius: 10px; overflow: hidden; z-index: 2; }
  .tank-ridges { position: absolute; top: 0; left: 0; width: 100%; height: 100%; pointer-events: none; z-index: 3; }
  .tank-ridges::after { content: ''; position: absolute; left: -4px; right: -4px; top: 25%; height: 2px; background: rgba(0, 0, 0, 0.35); box-shadow: 0 45px 0 rgba(0, 0, 0, 0.35), 0 90px 0 rgba(0, 0, 0, 0.35); }
  .tank-fill { position: absolute; bottom: 0; left: 0; width: 100%; background: linear-gradient(90deg, #0277bd 0%, #039be5 50%, #0277bd 100%); transition: height 0.8s cubic-bezier(0.4, 0, 0.2, 1); height: 0%; }
  .tank-text { position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); font-weight: bold; font-size: 1.6rem; color: #fff; z-index: 4; text-shadow: 2px 2px 4px rgba(0,0,0,0.8); }
  a{color:#888;text-decoration:none;font-size:0.9rem;}
  .modal { display: none; position: fixed; z-index: 1000; left: 0; top: 0; width: 100%; height: 100%; background-color: rgba(0,0,0,0.8); backdrop-filter: blur(4px); }
  .modal-content { background-color: #1e1e1e; position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); padding: 25px; border-left: 5px solid #ff4d4d; border-radius: 8px; width: 85%; max-width: 320px; box-shadow: 0 10px 30px rgba(0,0,0,0.8); text-align: left; box-sizing: border-box; }
  .modal-title { color: #ff4d4d; font-size: 1.3rem; margin: 0 0 10px 0; display: flex; align-items: center; gap: 8px; }
  .modal-text { color: #ddd; margin-bottom: 25px; font-size: 1.05rem; line-height: 1.5; }
  .modal-close { background: #333; color: white; padding: 12px; border: none; border-radius: 6px; cursor: pointer; font-weight: bold; width: 100%; font-size: 1.1rem; transition: background 0.2s; }
  .modal-close:hover { background: #555; }
</style></head><body>
  <div id="warnModal" class="modal"><div class="modal-content"><h3 class="modal-title"><span>⚠️</span> Action Blocked</h3><div id="warnText" class="modal-text">Safety protocols prevented this action.</div><button class="modal-close" onclick="closeModal()">Understood</button></div></div>
  <div style="margin-bottom:15px;"><a href="/settings">⚙️ Settings</a></div>
  <div class="card">
    <button id="rfb" class="refresh-btn" onclick="rfr()" title="Refresh Status">🔄</button>
    <div style="font-size:0.8rem;text-align:left;margin-bottom:10px;"><span id="dot" class="conn-dot"></span><span id="cStat">Device: Online</span></div>
    <h2 style="color:#03ef;margin-top:0;">💧 Pump Dashboard</h2>
    <div class="tank-wrap"><div class="tank-inner"><div class="tank-fill" id="tankFill"></div></div><div class="tank-ridges"></div><div class="tank-text" id="tankVal">-- %</div></div>
    <div class="row"><span>Voltage:</span><span id="volt">-- V</span></div>
    <div class="row"><span>Volt Status:</span><span id="vstat">--</span></div>
    <div class="row"><span>Pump:</span><span id="state">--</span></div>
    <div class="row"><span>System Info:</span><span id="info">--</span></div>
    <div class="row" id="cdRow" style="display:none;color:#ff4d4d;font-weight:bold;"><span>Dry-Run in:</span><span id="cd">--</span></div>
    <button class="btn" id="btnToggle" onclick="togglePump()">PUMP: OFF</button>
    <button class="btn btn-red" id="btnReset" onclick="resetPump()" style="display:none;">Reset Alarm</button>
  </div>
  <div style="margin-top:25px;color:#666;font-size:0.75rem;">
    <div style="margin-bottom:8px;">Cloud ID: <span id="cid" style="color:#888;">--</span></div>
    Device IP: <span id="dip">--</span>
  </div>
<script>
  function showModal(msg) { document.getElementById('warnText').innerText = msg; document.getElementById('warnModal').style.display = 'block'; }
  function closeModal() { document.getElementById('warnModal').style.display = 'none'; }
  function togglePump() { fetch('/toggle').then(r=>r.json()).then(d=>{ if(d.status === 'blocked') showModal(d.reason); else upd(); }).catch(e=>{}); }
  function resetPump() { fetch('/reset').then(r=>r.json()).then(d=>{ if(d.status === 'blocked') showModal(d.reason); else upd(); }).catch(e=>{}); }
  function rfr() { let b=document.getElementById('rfb'); b.classList.add('spinning'); upd().finally(()=>b.classList.remove('spinning')); }
  function upd() { return fetch('/status').then(r=>r.json()).then(d=>{
      document.getElementById('dot').className='conn-dot'; document.getElementById('cStat').innerText='Device: Online';
      document.getElementById('dip').innerText = d.ip; document.getElementById('tankFill').style.height = d.tank + '%';
      document.getElementById('tankVal').innerText = d.tStr; if (document.getElementById('cid')) document.getElementById('cid').innerText = d.id;
      document.getElementById('volt').innerText=d.volt+' V'; document.getElementById('vstat').innerText=d.vStat;
      document.getElementById('state').innerText=d.pStat; document.getElementById('info').innerText=d.info;
      let btn=document.getElementById('btnToggle'); let rst=document.getElementById('btnReset');
      // FIXED: Show "Reset Alarm" for Sensor Error (sErr) if not acknowledged (ack)
      if(d.err){rst.style.display='block';btn.style.display='none';} 
      else if(d.sErr && !d.ack){btn.style.display='block';rst.style.display='none';btn.innerText='Reset Alarm';btn.className='btn btn-red';}
      else{rst.style.display='none';btn.style.display='block'; btn.innerText=d.pStat=="ON"?'Stop the Pump':'Start the Pump'; btn.className=d.pStat=="ON"?'btn btn-red':'btn btn-green';}
      let cd=document.getElementById('cdRow'); if(d.pStat=="ON"&&d.info=="FLOW_CHECKING!"){cd.style.display='flex';document.getElementById('cd').innerText=d.cd+'s';}else{cd.style.display='none';}
    }).catch(e=>{ document.getElementById('dot').className='conn-dot off'; document.getElementById('cStat').innerText='Device: Offline (Connecting...)'; }); }
  setInterval(upd,1000);
</script></body></html>
)rawliteral";

const char settings_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
<style>
  body{font-family:sans-serif;background:#121212;color:white;padding:15px;text-align:center;}
  .card{background:#1e1e1e;border-radius:12px;padding:20px;max-width:400px;margin:auto;border:1px solid #333;text-align:left;box-shadow: 0 10px 25px rgba(0,0,0,0.5);}
  .lbl-wrap { display: flex; justify-content: space-between; align-items: flex-end; margin: 15px 0 5px; }
  label{font-weight:bold;color:#aaa; font-size: 0.9rem;}
  .range { color: #555; font-size: 0.75rem; font-weight: normal; }
  input, select{width:100%;padding:12px;background:#2a2a2a;border:1px solid #444;color:white;border-radius:6px;box-sizing:border-box; font-size: 1rem;}
  .pass-row { position: relative; }
  .show-pass { margin-top: 8px; font-size: 0.85rem; color: #888; display: flex; align-items: center; cursor: pointer; }
  .show-pass input { width: auto; margin-right: 8px; }
  .btn{width:100%;padding:15px;background:#28a745;color:white;border:none;border-radius:8px;margin-top:25px;font-weight:bold;cursor:pointer;font-size:1.1rem;}
  .btn:active { transform: scale(0.98); opacity: 0.9; }
  h2{color:#03ef;margin:0 0 20px 0;text-align:center; font-size: 1.5rem;}
  hr { border: 0; border-top: 1px solid #333; margin: 20px 0; }
</style></head><body>
  <div class="card">
    <h2>⚙️ Device Settings</h2>
    <form action="/save" method="POST">
      <div class="lbl-wrap"><label>WiFi SSID</label></div>
      <select name="ssid" required>%WIFI_LIST%</select>
      <div class="lbl-wrap"><label>WiFi Password</label></div>
      <div class="pass-row"><input type="password" id="p" name="pass" placeholder="Leave empty to keep current"><label class="show-pass"><input type="checkbox" onclick="togglePass()"> Show Password</label></div>
      <hr>
      <div class="lbl-wrap"><label>Tank Height</label><span class="range">1.0 - 7.0 ft</span></div><select name="uH">%TANK_LIST%</select>
      <div class="lbl-wrap"><label>High Voltage Set</label><span class="range">230 - 260 V</span></div><input type="number" name="vH" value="%VHIGH%" min="230" max="260">
      <div class="lbl-wrap"><label>Low Voltage Set</label><span class="range">150 - 190 V</span></div><input type="number" name="vL" value="%VLOW%" min="150" max="190">
      <div class="lbl-wrap"><label>Dry-Run Delay</label><span class="range">60 - 180 s</span></div><input type="number" name="dD" value="%DRY%" min="60" max="180">
      <button type="submit" class="btn">Save & Reboot</button>
    </form>
    <div style="text-align:center;margin-top:15px;"><a href="/" style="color:#888;text-decoration:none;font-size:0.9rem;">← Back to Dashboard</a></div>
  </div>
<script>
  function togglePass() { var x = document.getElementById("p"); if (x.type === "password") x.type = "text"; else x.type = "password"; }
</script></body></html>
)rawliteral";

// --- Sensor Implementation ---

class NonBlockingUltrasonic {
private:
  int trigPin, echoPin;
  static const int NUM_SAMPLES = 10;
  static const unsigned long SAMPLE_INTERVAL_MS = 60;
  static const unsigned long MAX_PULSE_DURATION = 70000;
  float samples[NUM_SAMPLES];
  int sampleIndex = 0;
  unsigned long lastSampleTime = 0;
  bool collecting = false;
  float lastMedian = -1.0;

  float median(float arr[], int n) {
    float temp[n];
    memcpy(temp, arr, sizeof(float) * n);
    for (int i = 0; i < n - 1; i++) {
      for (int j = i + 1; j < n; j++) {
        if (temp[j] < temp[i]) { float t = temp[i]; temp[i] = temp[j]; temp[j] = t; }
      }
    }
    return (n % 2 == 0) ? (temp[n/2-1] + temp[n/2]) / 2.0 : temp[n/2];
  }

public:
  NonBlockingUltrasonic(int t, int e) : trigPin(t), echoPin(e) {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW);
  }

  void start() { if (!collecting) { sampleIndex = 0; collecting = true; lastSampleTime = millis() - SAMPLE_INTERVAL_MS; } }
  
  void update() {
    unsigned long now = millis();
    if (collecting && (now - lastSampleTime >= SAMPLE_INTERVAL_MS)) {
      lastSampleTime = now;
      digitalWrite(trigPin, LOW); delayMicroseconds(9);
      digitalWrite(trigPin, HIGH); delayMicroseconds(140);
      digitalWrite(trigPin, LOW);
      long duration = pulseIn(echoPin, HIGH, MAX_PULSE_DURATION);
      float inches = (duration == 0) ? -1.0 : (duration * 0.01356 / 2.0);
      samples[sampleIndex++] = inches;
      if (sampleIndex >= NUM_SAMPLES) {
        collecting = false;
        float v[NUM_SAMPLES]; int vc = 0;
        for (int i=0; i<NUM_SAMPLES; i++) if (samples[i] > 0 && samples[i] <= MAX_DISTANCE) v[vc++] = samples[i];
        lastMedian = (vc > NUM_SAMPLES/2) ? median(v, vc) : -1.0;
      }
    }
  }

  float getDistance() { return lastMedian; }
  bool isBusy() { return collecting; }
};

NonBlockingUltrasonic upperSensor(UPPER_TANK_TRIG_PIN, UPPER_TANK_ECHO_PIN);

void monitorSensors() {
  static unsigned long lastScan = 0;
  unsigned long now = millis();

  // Ultrasonic Scan (Kept at 4s for stability)
  if (now - lastScan >= ULTRASONIC_INTERVAL) {
    if (!upperSensor.isBusy()) upperSensor.start();
  }
  upperSensor.update();
  
  if (!upperSensor.isBusy() && now - lastScan >= ULTRASONIC_INTERVAL) {
    float dist = upperSensor.getDistance();
    if (dist > 0) {
      tankConfig.upperDistance = dist;
      tankConfig.upperInvalidCount = 0;
      // FIX: Reset Sensor Error Acknowledgement if sensor works again
      tankConfig.errorAck = false; 
      
      float effectiveHeight = tankConfig.upperHeight + TankConfig::BUFFER_HEIGHT;
      tankConfig.rawUpperPercentage = ((effectiveHeight - dist) * 100) / effectiveHeight;
      tankConfig.rawUpperPercentage = constrain(tankConfig.rawUpperPercentage, 0, 100);
      tankConfig.displayUpperPercentage = map(tankConfig.rawUpperPercentage, 0, tankConfig.FULL_THRESHOLD, 0, 100);
      tankConfig.displayUpperPercentage = constrain(tankConfig.displayUpperPercentage, 0, 100);
    } else {
      tankConfig.upperInvalidCount++;
    }
    lastScan = now;
  }

  // Voltage Monitoring (RMS via ZMPT101B)
  static unsigned long lastVoltSample = 0;
  if (now - lastVoltSample >= 2) { 
    float v = voltageSensor.getRmsVoltage();
    voltageConfig.currentVoltage = (0.2 * v) + (0.8 * voltageConfig.currentVoltage);
    lastVoltSample = now;
  }
}

// Global Vars
String ssid_saved = "";
String pass_saved = "";

void setup() {
  Serial.begin(115200);
  
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  digitalWrite(MOTOR_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  preferences.begin("pump-control", false);
  tankConfig.upperHeight = preferences.getFloat("upperH", TankConfig::MIN_HEIGHT);
  voltageConfig.HIGH_THRESHOLD = preferences.getInt("vHigh", 250);
  voltageConfig.LOW_THRESHOLD = preferences.getInt("vLow", 170);
  dryRunConfig.WAIT_SECONDS_SET = preferences.getInt("dryDelay", 60);
  devicePin = preferences.getString("pin", "123456");
  ssid_saved = preferences.getString("ssid", "");
  pass_saved = preferences.getString("pass", "");
  pumpConfig.motorStatus = preferences.getInt("motor", 0);
  preferences.end();
  
  tankConfig.updateFullThreshold();
  
  rgbLed.begin();
  rgbLed.setBrightness(30);
  voltageSensor.setSensitivity(526.25);
  
  tft.init();
  tft.setRotation(1);
  drawDashboard();

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("Auto-Pump-Config", "12345678");
  dnsUdp.begin(DNS_PORT);
  Serial.println("\nHotspot Started: Auto-Pump-Config");

  if (ssid_saved != "") {
    wifiMulti.addAP(ssid_saved.c_str(), pass_saved.c_str());
    Serial.print("Connecting to WiFi: " + ssid_saved);
    unsigned long startAttempt = millis();
    while (wifiMulti.run() != WL_CONNECTED && millis() - startAttempt < 10000) {
      delay(500); Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
        // FIX: Turn OFF AP Mode after successful connection
        WiFi.mode(WIFI_STA);
        Serial.println("AP Mode Disabled.");
    }
    else Serial.println("\nFailed to connect saved WiFi.");
  } else {
    Serial.println("\nNo WiFi credentials saved. Use AP to configure.");
  }

  deviceID = getDeviceID();
  subTopic = "smartpump/" + deviceID + "/set";
  statusTopic = "smartpump/" + deviceID + "/status";
  onlineTopic = "smartpump/" + deviceID + "/online";

  server.on("/", handleRoot);
  server.on("/toggle", handleToggle);
  server.on("/status", handleStatus);
  server.on("/reset", handleReset);
  server.on("/settings", handleSettings);
  server.on("/save", HTTP_POST, handleSave);
  server.onNotFound([]() {
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/plain", "Redirect");
  });
  server.begin();

  espClient.setInsecure();
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(2048);
  
  Serial.println("System Initialized. Device ID: " + deviceID);
}

void loop() {
  processDNS();
  monitorSensors();
  updatePumpLogic();
  checkTouch();
  
  server.handleClient();
  
  if (WiFi.status() == WL_CONNECTED) {
    if (!mqttClient.connected()) {
      static unsigned long lastReconnectAttempt = 0;
      unsigned long now = millis();
      if (now - lastReconnectAttempt > 5000) {
        lastReconnectAttempt = now;
        if (reconnectMQTT()) lastReconnectAttempt = 0;
      }
    } else {
      mqttClient.loop();
    }
  }

  static unsigned long lastPublish = 0;
  if (millis() - lastPublish >= 30000) {
    lastPublish = millis();
    publishState();
  }

  updateTFT();
  updateLEDStatus();
  
  delay(1);
}

// ----------------------------------------------------------------------------
// HELPERS & LOGIC
// ----------------------------------------------------------------------------
String getDeviceID() {
  uint64_t chipid = ESP.getEfuseMac();
  uint16_t chip = (uint16_t)(chipid >> 32);
  char id[23];
  snprintf(id, 23, "%04X%08X", chip, (uint32_t)chipid);
  return String(id);
}

void saveMotorStatus() {
  preferences.begin("pump-control", false);
  preferences.putInt("motor", pumpConfig.motorStatus);
  preferences.end();
}

void setLedColor(uint8_t r, uint8_t g, uint8_t b) {
  rgbLed.setPixelColor(0, rgbLed.Color(r, g, b));
  rgbLed.show();
}

void updateLEDStatus() {
  static unsigned long lastLED = 0;
  if (millis() - lastLED < 500) return;
  lastLED = millis();

  if (currentState == PumpState::DRY_RUN_ALARM || currentState == PumpState::DRY_RUN_LOCKED || 
      currentState == PumpState::SENSOR_ERROR || currentState == PumpState::VOLTAGE_ERROR) {
    static bool flash = false; flash = !flash;
    setLedColor(flash ? 255 : 0, 0, 0); 
  } else if (pumpConfig.isRunning) {
    setLedColor(255, 200, 0); 
  } else if (WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
    setLedColor(0, 255, 0); 
  } else if (WiFi.status() == WL_CONNECTED) {
    setLedColor(0, 255, 255); 
  } else {
    static int bright = 0; static int dir = 5; bright += dir;
    if (bright >= 50 || bright <= 0) dir = -dir;
    setLedColor(0, 0, bright);
  }
}

// ----------------------------------------------------------------------------
// EXPLICIT SAFETY CHECKS FOR MANUAL STARTS
// ----------------------------------------------------------------------------
String getStartBlockReason() {
  if (voltageConfig.currentVoltage > voltageConfig.HIGH_THRESHOLD) 
    return "Voltage is too HIGH (" + String((int)voltageConfig.currentVoltage) + "V). Safe limit is " + String(voltageConfig.HIGH_THRESHOLD) + "V.";
  if (voltageConfig.currentVoltage < voltageConfig.LOW_THRESHOLD) 
    return "Voltage is too LOW (" + String((int)voltageConfig.currentVoltage) + "V). Safe limit is " + String(voltageConfig.LOW_THRESHOLD) + "V.";
  if (voltageConfig.status == 0) 
    return "Voltage stabilization is in progress. Please wait a moment.";
  if (tankConfig.rawUpperPercentage >= tankConfig.FULL_THRESHOLD) 
    return "The Tank is already FULL (" + String(tankConfig.displayUpperPercentage) + "%). Pumping is not needed.";
  if (tankConfig.upperInvalidCount >= TankConfig::MAX_INVALID_COUNT) 
    return "Sensor Error! Unable to verify water level safely. Please check the ultrasonic sensor.";
  return "";
}

// --- Pump Controller Implementation ---

void updatePumpLogic() {
  unsigned long currentMillis = millis();
  static PumpState lastState = PumpState::IDLE;
  static int lastPercentage = -1;
  static int lastVoltStatus = -1;
  static bool lastFlow = false;
  static int lastErr = -1;
  static int lastMotorStatus = -1;
  static bool lastAck = false; // Track Ack state changes
  
  pumpConfig.flowDetected = (digitalRead(FLOW_SENSOR_PIN) == LOW);
  bool sensorError = (tankConfig.upperInvalidCount >= TankConfig::MAX_INVALID_COUNT);

  bool voltAbnormal = (voltageConfig.currentVoltage > voltageConfig.HIGH_THRESHOLD || 
                       voltageConfig.currentVoltage < voltageConfig.LOW_THRESHOLD);

  if (voltAbnormal) {
    if (voltageConfig.status == 1) { 
        if (pumpConfig.motorStatus == 1) {
            pumpConfig.wasRunningBeforeVoltageError = true;
        } else {
            pumpConfig.wasRunningBeforeVoltageError = false;
        }
    }
    voltageConfig.status = 0;
    voltageConfig.lastCheck = currentMillis;
  } else if (voltageConfig.status == 0) {
    if (voltageConfig.currentVoltage >= voltageConfig.LOW_THRESHOLD && voltageConfig.currentVoltage <= voltageConfig.HIGH_THRESHOLD) {
      if (currentMillis - voltageConfig.lastCheck >= GENERAL_INTERVAL) {
        voltageConfig.waitSeconds--;
        voltageConfig.lastCheck = currentMillis;
        
        publishState();
        
        if (voltageConfig.waitSeconds <= 0) {
          voltageConfig.status = 1;
          
          if (pumpConfig.wasRunningBeforeVoltageError) {
             if (tankConfig.rawUpperPercentage < tankConfig.FULL_THRESHOLD && 
                 tankConfig.upperInvalidCount < TankConfig::MAX_INVALID_COUNT) {
                 pumpConfig.motorStatus = 1;
                 dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET;
                 saveMotorStatus();
             }
             pumpConfig.wasRunningBeforeVoltageError = false;
          }

          voltageConfig.waitSeconds = voltageConfig.WAIT_SECONDS_SET;
        }
      }
    } else {
      voltageConfig.lastCheck = currentMillis;
    }
  }

  if (tankConfig.rawUpperPercentage >= tankConfig.FULL_THRESHOLD || sensorError || voltageConfig.status == 0) {
    if (pumpConfig.motorStatus == 1) {
      pumpConfig.motorStatus = 0;
      saveMotorStatus();
    }
  }

  if (tankConfig.rawUpperPercentage <= TankConfig::LOW_THRESHOLD && !sensorError) {
    if (voltageConfig.status == 1 && dryRunConfig.error == 0) {
      if (pumpConfig.motorStatus == 0) {
        pumpConfig.motorStatus = 1;
        dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET;
        saveMotorStatus();
      }
    }
  }

  if (sensorError) currentState = PumpState::SENSOR_ERROR;
  else if (dryRunConfig.error == 1) currentState = PumpState::DRY_RUN_ALARM;
  else if (dryRunConfig.error == 2) currentState = PumpState::DRY_RUN_LOCKED;
  else if (voltAbnormal) currentState = PumpState::VOLTAGE_ERROR;
  else if (voltageConfig.status == 0) currentState = PumpState::VOLTAGE_WAIT;
  else if (pumpConfig.motorStatus == 1) currentState = PumpState::PUMPING;
  else currentState = PumpState::IDLE;

  switch (currentState) {
    case PumpState::PUMPING:
      digitalWrite(MOTOR_PIN, HIGH);
      pumpConfig.isRunning = true;
      if (pumpConfig.flowDetected) {
        if (dryRunConfig.waitSeconds != dryRunConfig.WAIT_SECONDS_SET) {
          dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET;
          publishState();
        }
      } else {
        if (currentMillis - dryRunConfig.lastUpdate >= GENERAL_INTERVAL) {
          dryRunConfig.waitSeconds--;
          dryRunConfig.lastUpdate = currentMillis;
          publishState();
          if (dryRunConfig.waitSeconds <= 0) {
            dryRunConfig.error = 1;
            dryRunConfig.alarmStartTime = currentMillis;
            pumpConfig.motorStatus = 0;
            saveMotorStatus();
          }
        }
      }
      break;

    case PumpState::DRY_RUN_ALARM:
      digitalWrite(MOTOR_PIN, LOW);
      pumpConfig.isRunning = false;
      digitalWrite(BUZZER_PIN, (currentMillis / 500) % 2);
      if (currentMillis - dryRunConfig.alarmStartTime >= 60000) {
        digitalWrite(BUZZER_PIN, LOW);
        dryRunConfig.error = 2;
        publishState();
      }
      break;

    default:
      digitalWrite(MOTOR_PIN, LOW);
      pumpConfig.isRunning = false;
      if (currentState == PumpState::SENSOR_ERROR && !tankConfig.errorAck) {
          digitalWrite(BUZZER_PIN, (currentMillis / 500) % 2);
      } else {
          digitalWrite(BUZZER_PIN, LOW);
      }
      break;
  }

  if (currentState != lastState || 
      tankConfig.displayUpperPercentage != lastPercentage || 
      voltageConfig.status != lastVoltStatus || 
      pumpConfig.flowDetected != lastFlow || 
      dryRunConfig.error != lastErr ||
      pumpConfig.motorStatus != lastMotorStatus ||
      tankConfig.errorAck != lastAck) { // Added Ack check
      
    lastState = currentState;
    lastPercentage = tankConfig.displayUpperPercentage;
    lastVoltStatus = voltageConfig.status;
    lastFlow = pumpConfig.flowDetected;
    lastErr = dryRunConfig.error;
    lastMotorStatus = pumpConfig.motorStatus;
    lastAck = tankConfig.errorAck;
    
    publishState();
  }
}

// --- TFT UI Colors ---
#define COLOR_BG      TFT_BLACK
#define COLOR_PANEL   0x2104 
#define COLOR_TEXT    TFT_WHITE
#define COLOR_PUMP_ON 0x07E0 
#define COLOR_PUMP_OFF 0xF800 
#define COLOR_VOLT    TFT_YELLOW
#define COLOR_TANK    0x03EF 

void drawDashboard() {
  tft.fillScreen(COLOR_BG);
  tft.fillRect(0, 0, 240, 28, COLOR_PANEL);
  tft.setTextColor(TFT_WHITE);
  tft.drawCentreString("PUMP DASHBOARD", 120, 7, 2);

  uint16_t TANK_OUTLINE = 0x3186;
  int tx = 55, ty = 55, tw = 130, th = 165, tr = 8;
  tft.drawRoundRect(tx, ty, tw, th, tr, TANK_OUTLINE);
  tft.drawRoundRect(tx+1, ty+1, tw-2, th-2, tr, TANK_OUTLINE);

  tft.fillRect(tx + 15, ty - 14, tw - 30, 14, TANK_OUTLINE);
  tft.fillRect(tx + 5, ty - 24, tw - 10, 10, TANK_OUTLINE);
  tft.drawRect(tx + 5, ty - 24, tw - 10, 10, TANK_OUTLINE);

  uint16_t RIDGE = 0x2945; 
  tft.drawFastHLine(tx, ty + (th * 25 / 100), tw, RIDGE);
  tft.drawFastHLine(tx, ty + (th * 50 / 100), tw, RIDGE);
  tft.drawFastHLine(tx, ty + (th * 75 / 100), tw, RIDGE);

  tft.drawFastHLine(5, 240, 230, TANK_OUTLINE);
  tft.drawFastHLine(5, 260, 230, TANK_OUTLINE);
  tft.drawFastHLine(5, 280, 230, TANK_OUTLINE);
  tft.setTextColor(0xAD55);
  tft.drawString("Voltage:", 8, 244, 2);
  tft.drawString("Pump:", 8, 264, 2);
  tft.drawString("Info:", 8, 284, 2);
}

void updateTFT() {
  static unsigned long lastUIUpdate = 0;
  if (millis() - lastUIUpdate < 500) return; 
  lastUIUpdate = millis();

  String tStr;
  if (tankConfig.upperDistance < 0 && tankConfig.upperInvalidCount < TankConfig::MAX_INVALID_COUNT) tStr = "---";
  else if (tankConfig.upperInvalidCount >= TankConfig::MAX_INVALID_COUNT) tStr = "ERR";
  else if (tankConfig.rawUpperPercentage > tankConfig.FULL_THRESHOLD) tStr = "FULL";
  else if (tankConfig.rawUpperPercentage < TankConfig::LOW_THRESHOLD) tStr = "LOW";
  else tStr = String(tankConfig.displayUpperPercentage) + "%";

  String vS;
  if (currentState == PumpState::VOLTAGE_WAIT) vS = "DELAY (" + String(voltageConfig.waitSeconds) + ")";
  else if (voltageConfig.currentVoltage > voltageConfig.HIGH_THRESHOLD) vS = "OVER";
  else if (voltageConfig.currentVoltage < voltageConfig.LOW_THRESHOLD) vS = "UNDER";
  else vS = "NORMAL";

  String info;
  if (dryRunConfig.error >= 1) info = "DRY_RUN_ERROR!";
  else if (tankConfig.upperInvalidCount >= TankConfig::MAX_INVALID_COUNT) info = "SENSOR_ERROR!";
  else if (pumpConfig.isRunning) {
    if (pumpConfig.flowDetected) info = "FLOW_DETECTED!";
    else info = "FLOW_CHECKING!";
  } else info = "SYSTEM_STANDBY!";

  int fillPct = constrain(tankConfig.displayUpperPercentage, 0, 100);
  int tx = 55, ty = 55, tw = 130, th = 165;
  int barH = map(fillPct, 0, 100, 0, th - 4);
  uint16_t TANK_OUTLINE = 0x3186;
  uint16_t RIDGE = 0x2945;

  tft.fillRoundRect(tx + 2, ty + 2, tw - 4, th - 4, 6, COLOR_BG);
  if (barH > 0) tft.fillRect(tx + 2, ty + (th - 4) - barH + 2, tw - 4, barH, COLOR_TANK);
  
  tft.drawFastHLine(tx, ty + (th * 25 / 100), tw, RIDGE);
  tft.drawFastHLine(tx, ty + (th * 50 / 100), tw, RIDGE);
  tft.drawFastHLine(tx, ty + (th * 75 / 100), tw, RIDGE);

  tft.drawRoundRect(tx, ty, tw, th, 8, TANK_OUTLINE);
  tft.drawRoundRect(tx+1, ty+1, tw-2, th-2, 8, TANK_OUTLINE);

  tft.setTextColor(TFT_WHITE);
  tft.drawCentreString(tStr, tx + tw / 2, ty + th / 2 - 8, 4);

  uint16_t pColor = pumpConfig.isRunning ? COLOR_PUMP_ON : TFT_BLUE;
  if (dryRunConfig.error >= 1) pColor = TFT_RED;
  else if (voltageConfig.status == 0) pColor = TFT_YELLOW;

  tft.fillRect(90, 70, 140, 60, pColor);
  tft.setTextColor(TFT_WHITE, pColor);
  tft.drawCentreString(pumpConfig.isRunning ? "ON" : "OFF", 160, 80, 4);
  tft.drawCentreString(info, 160, 110, 2);

  tft.setTextColor(voltageConfig.status ? TFT_GREEN : TFT_RED, COLOR_BG);
  tft.fillRect(255, 60, 50, 60, COLOR_BG);
  tft.drawCentreString(String((int)voltageConfig.currentVoltage) + "V", 280, 75, 4);
  tft.drawCentreString(vS, 280, 105, 1);
  
  tft.setTextColor(TFT_WHITE, COLOR_PANEL);
  tft.fillRect(200, 5, 110, 20, COLOR_PANEL);
  if (WiFi.status() == WL_CONNECTED) {
    tft.drawRightString(WiFi.localIP().toString(), 310, 8, 2);
  } else {
    tft.drawRightString("OFFLINE", 310, 8, 2);
  }
}

void checkTouch() {
  uint16_t x, y;
  if (tft.getTouch(&x, &y)) {
    if (x > 90 && x < 230 && y > 70 && y < 130) {
      
      // FIX: Handle Sensor Alarm Silence before Safety Check
      if (currentState == PumpState::SENSOR_ERROR) {
          if (!tankConfig.errorAck) {
              tankConfig.errorAck = true; // Silence
              Serial.println("TFT: Sensor Alarm Silenced");
              delay(300);
              return;
          }
      }

      bool wantsToStart = (pumpConfig.motorStatus == 0) || (dryRunConfig.error == 2);
      
      if (wantsToStart && dryRunConfig.error != 1) {
        String blockReason = getStartBlockReason();
        if (blockReason != "") {
          Serial.println("TFT Toggle Blocked: " + blockReason);
          
          tft.fillRoundRect(20, 70, 280, 100, 8, TFT_RED);
          tft.setTextColor(TFT_WHITE, TFT_RED);
          tft.drawCentreString("ACTION BLOCKED!", 160, 90, 4);
          
          int splitIdx = blockReason.indexOf(" ", 25);
          if (splitIdx == -1) splitIdx = 30; 
          tft.drawCentreString(blockReason.substring(0, splitIdx), 160, 120, 2);
          if (blockReason.length() > splitIdx) {
              tft.drawCentreString(blockReason.substring(splitIdx + 1), 160, 140, 2);
          }
          delay(2500); 
          
          tft.fillScreen(COLOR_BG);
          drawDashboard(); 
          return;
        }
      }

      if (dryRunConfig.error == 1) {
        dryRunConfig.error = 2;
        digitalWrite(BUZZER_PIN, LOW);
        Serial.println("TFT: Alarm Silenced");
      } else if (dryRunConfig.error == 2) {
        dryRunConfig.error = 0;
        dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET;
        pumpConfig.motorStatus = 1;
        Serial.println("TFT: Alarm Reset & Started");
      } else {
        pumpConfig.motorStatus = !pumpConfig.motorStatus;
        
        if (pumpConfig.motorStatus == 1) {
          dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET;
        }
        
        Serial.println("TFT: Manual Toggle");
      }
      saveMotorStatus();
      delay(300); 
    }
  }
}

// --- MQTT & Web Callbacks ---
bool reconnectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return false;
  String clientId = "Pump-" + getDeviceID();
  if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_pass, onlineTopic.c_str(), 1, true, "0")) {
    Serial.println("MQTT Connected");
    mqttClient.publish(onlineTopic.c_str(), "1", true); 
    mqttClient.subscribe(subTopic.c_str());
    publishState(); 
    return true;
  }
  return false;
}

void handleRoot() {
  server.send_P(200, "text/html", index_html);
}

void handleSettings() {
  String wifiList = "";
  int n = WiFi.scanNetworks();
  if (n <= 0) {
    wifiList = "<option value=''>No networks found</option>";
  } else {
    for (int i = 0; i < n; ++i) {
      String ssid = WiFi.SSID(i);
      String sel = (ssid == ssid_saved) ? " selected" : "";
      wifiList += "<option value='" + ssid + "'" + sel + ">" + ssid + " (" + String(WiFi.RSSI(i)) + "dBm)</option>";
    }
  }
  WiFi.scanDelete();

  String tankList = "";
  for (float f = 1.0; f <= 7.01; f += 0.5) {
    float inches = f * 12.0;
    String sel = (abs(tankConfig.upperHeight - inches) < 0.1) ? " selected" : "";
    String lbl = (f == (int)f) ? String((int)f) : String(f, 1);
    tankList += "<option value='" + String(inches, 1) + "'" + sel + ">" + lbl + " ft</option>";
  }

  String s = settings_html;
  s.replace("%WIFI_LIST%", wifiList);
  s.replace("%TANK_LIST%", tankList);
  s.replace("%VHIGH%", String(voltageConfig.HIGH_THRESHOLD));
  s.replace("%VLOW%", String(voltageConfig.LOW_THRESHOLD));
  s.replace("%DRY%", String(dryRunConfig.WAIT_SECONDS_SET));
  server.send(200, "text/html", s);
}

void handleSave() {
  preferences.begin("pump-control", false);
  if (server.hasArg("ssid")) preferences.putString("ssid", server.arg("ssid"));
  if (server.hasArg("pass") && server.arg("pass") != "") preferences.putString("pass", server.arg("pass"));
  if (server.hasArg("uH")) {
    float h = server.arg("uH").toFloat();
    tankConfig.upperHeight = constrain(h, TankConfig::MIN_HEIGHT, TankConfig::MAX_HEIGHT);
    preferences.putFloat("upperH", tankConfig.upperHeight);
  }
  if (server.hasArg("vH")) {
    voltageConfig.HIGH_THRESHOLD = server.arg("vH").toInt();
    preferences.putInt("vHigh", voltageConfig.HIGH_THRESHOLD);
  }
  if (server.hasArg("vL")) {
    voltageConfig.LOW_THRESHOLD = server.arg("vL").toInt();
    preferences.putInt("vLow", voltageConfig.LOW_THRESHOLD);
  }
  if (server.hasArg("dD")) {
    dryRunConfig.WAIT_SECONDS_SET = server.arg("dD").toInt();
    preferences.putInt("dryDelay", dryRunConfig.WAIT_SECONDS_SET);
  }
  preferences.end();
  
  String html = "<!DOCTYPE html><html><head><meta http-equiv=\"refresh\" content=\"8;url=/\" ><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"><title>Rebooting</title><style>body{background:#121212;color:white;font-family:sans-serif;text-align:center;margin-top:50px;}</style></head><body><h2>Settings Saved!</h2><p>Rebooting device. You will be redirected to the dashboard shortly...</p></body></html>";
  server.send(200, "text/html", html);
  delay(1000);
  ESP.restart();
}

void handleStatus() {
  server.send(200, "application/json", generateStatusJson());
}

void handleToggle() {
  // FIX: Handle Sensor Alarm Silence before Safety Check
  if (currentState == PumpState::SENSOR_ERROR) {
      if (!tankConfig.errorAck) {
          tankConfig.errorAck = true; // Silence
          updatePumpLogic(); // Update buzzer state immediately
          publishState();
          server.send(200, "application/json", "{\"status\":\"success\", \"msg\":\"Silenced\"}");
          return;
      }
  }

  bool wantsToStart = (pumpConfig.motorStatus == 0) || (dryRunConfig.error == 2);
  
  if (wantsToStart && dryRunConfig.error != 1) { 
    String blockReason = getStartBlockReason();
    if (blockReason != "") {
      server.send(200, "application/json", "{\"status\":\"blocked\", \"reason\":\"" + blockReason + "\"}");
      return;
    }
  }

  if (dryRunConfig.error == 1) {
    dryRunConfig.error = 2; // Silence
    digitalWrite(BUZZER_PIN, LOW);
  } else if (dryRunConfig.error == 2) {
    dryRunConfig.error = 0;
    dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET;
    pumpConfig.motorStatus = 1;
  } else {
    pumpConfig.motorStatus = !pumpConfig.motorStatus;
    
    if (pumpConfig.motorStatus == 1) {
      dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET;
    }
  }
  
  saveMotorStatus();
  updatePumpLogic(); 
  publishState();
  
  server.send(200, "application/json", "{\"status\":\"success\"}");
}

void handleReset() {
  dryRunConfig.error = 0;
  dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET;
  digitalWrite(BUZZER_PIN, LOW);
  saveMotorStatus();
  publishState();
  server.send(200, "application/json", "{\"status\":\"success\"}");
}

// --- MQTT Functions ---

String generateStatusJson() {
  DynamicJsonDocument doc(1024);
  doc["pump"] = pumpConfig.motorStatus;
  
  String tStr;
  if (tankConfig.upperDistance < 0 && tankConfig.upperInvalidCount < TankConfig::MAX_INVALID_COUNT) tStr = "---";
  else if (tankConfig.upperInvalidCount >= TankConfig::MAX_INVALID_COUNT) tStr = "ERR";
  else if (tankConfig.rawUpperPercentage > tankConfig.FULL_THRESHOLD) tStr = "FULL";
  else if (tankConfig.rawUpperPercentage < TankConfig::LOW_THRESHOLD) tStr = "LOW";
  else tStr = String(tankConfig.displayUpperPercentage) + "%";
  
  String vS;
  if (currentState == PumpState::VOLTAGE_WAIT) vS = "DELAY (" + String(voltageConfig.waitSeconds) + "s)";
  else if (voltageConfig.currentVoltage > voltageConfig.HIGH_THRESHOLD) vS = "OVER";
  else if (voltageConfig.currentVoltage < voltageConfig.LOW_THRESHOLD) vS = "UNDER";
  else vS = "NORMAL";

  String info;
  if (dryRunConfig.error >= 1) info = "DRY_RUN_ERROR!";
  else if (tankConfig.upperInvalidCount >= TankConfig::MAX_INVALID_COUNT) info = "SENSOR_ERROR!";
  else if (pumpConfig.isRunning) {
    if (pumpConfig.flowDetected) info = "FLOW_DETECTED!";
    else info = "FLOW_CHECKING!";
  } else info = "SYSTEM_STANDBY!";

  doc["tank"] = tankConfig.displayUpperPercentage;
  doc["tStr"] = tStr;
  doc["volt"] = (int)voltageConfig.currentVoltage;
  doc["vStat"] = vS;
  doc["pStat"] = pumpConfig.isRunning ? "ON" : "OFF";
  doc["info"] = info;
  doc["err"] = dryRunConfig.error;
  doc["cd"] = dryRunConfig.waitSeconds;
  doc["id"] = deviceID;
  doc["ip"] = WiFi.localIP().toString();
  
  // NEW: Add Sensor Error status and Ack status for HTML logic
  doc["sErr"] = (currentState == PumpState::SENSOR_ERROR) ? 1 : 0;
  doc["ack"] = tankConfig.errorAck ? 1 : 0;

  String json;
  serializeJson(doc, json);
  return json;
}

void publishState() {
  if (WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
    String json = generateStatusJson();
    mqttClient.publish(statusTopic.c_str(), json.c_str());
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (int i = 0; i < length; i++) msg += (char)payload[i];
  Serial.println("MQTT Received: " + msg);

  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, msg);
  if (error) return;

  if (doc.containsKey("toggle")) {
    
    // FIX: Handle Sensor Alarm Silence before Safety Check
    if (currentState == PumpState::SENSOR_ERROR) {
        if (!tankConfig.errorAck) {
            tankConfig.errorAck = true; // Silence
            updatePumpLogic(); // Update buzzer
            publishState();
            return;
        }
    }

    bool wantsToStart = (pumpConfig.motorStatus == 0) || (dryRunConfig.error == 2);
    
    if (wantsToStart && dryRunConfig.error != 1) {
      String blockReason = getStartBlockReason();
      if (blockReason != "") {
        Serial.println("MQTT Start Blocked: " + blockReason);
        DynamicJsonDocument resp(256);
        resp["alert"] = "Action Blocked";
        resp["reason"] = blockReason;
        String respStr;
        serializeJson(resp, respStr);
        mqttClient.publish(statusTopic.c_str(), respStr.c_str());
        return; 
      }
    }

    if (dryRunConfig.error == 1) {
      dryRunConfig.error = 2; // Silence
      digitalWrite(BUZZER_PIN, LOW);
    } else if (dryRunConfig.error == 2) {
      dryRunConfig.error = 0;
      dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET;
      pumpConfig.motorStatus = 1;
    } else {
      pumpConfig.motorStatus = !pumpConfig.motorStatus;
      
      if (pumpConfig.motorStatus == 1) {
        dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET;
      }
    }
    saveMotorStatus();
    
    updatePumpLogic();
    publishState();
    
  } else if (doc.containsKey("reset")) {
    dryRunConfig.error = 0;
    dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET;
    digitalWrite(BUZZER_PIN, LOW);
    saveMotorStatus();
  } else if (doc.containsKey("get")) {
    publishState();
  }
}