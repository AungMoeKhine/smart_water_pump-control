/*
 * Automatic Pump Control System (ESP32-S3) - DUAL CORE FINAL EDITION
 * Features: LCD (I2C), Local Web Control, Cloud Control (MQTT), Physical Button
 * Added: License System (File Upload + Text Paste + Admin Backdoor)
 * 
 * Core 1 (Main Loop): Real-time Hardware Control (Sensors, Pump, Button, LCD)
 * Core 0 (Task):      Network Control (WiFi, HiveMQ MQTT, OTA)
 */

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <time.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <ZMPT101B.h>
#include <Adafruit_NeoPixel.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <MD5Builder.h>
#include <mbedtls/base64.h>
#include "logo.h"

// ============================================================================
// CONFIGURATION & PINOUT
// ============================================================================

// Pin Assignments (ESP32-S3)
const int VOLTAGE_SENSOR_PIN = 4;
const int UPPER_TANK_TRIG_PIN = 5;
const int UPPER_TANK_ECHO_PIN = 6;
const int BUZZER_PIN = 7;
const int MOTOR_PIN = 8;
const int MANUAL_BTN_PIN = 9;   // Physical Push Button (Connect to GND)
const int FLOW_SENSOR_PIN = 18;
const int RGB_LED_PIN = 48;
const int SDA_PIN = 1;
const int SCL_PIN = 2;

// LCD Configuration
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Big Digit Custom Characters
byte bar1[8] = { B11100, B11110, B11110, B11110, B11110, B11110, B11110, B11100 };
byte bar2[8] = { B00111, B01111, B01111, B01111, B01111, B01111, B01111, B00111 };
byte bar3[8] = { B11111, B11111, B00000, B00000, B00000, B00000, B11111, B11111 };
byte bar4[8] = { B11110, B11100, B00000, B00000, B00000, B00000, B11000, B11100 };
byte bar5[8] = { B01111, B00111, B00000, B00000, B00000, B00000, B00011, B01111 };
byte bar6[8] = { B00000, B00000, B00000, B00000, B00000, B00000, B11111, B11111 };
byte bar7[8] = { B00000, B00000, B00000, B00000, B00000, B00000, B00111, B01111 };
byte bar8[8] = { B11111, B11111, B00000, B00000, B00000, B00000, B00000, B00000 };

// MQTT Configuration
const char* mqtt_server = "210195b635414206adcd944325fe6f59.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "my_switch";
const char* mqtt_pass = "My_password123";

// --- FIRMWARE VERSIONING ---
const int FIRMWARE_VERSION = 1;  
const char* FW_URL_BASE = "https://raw.githubusercontent.com/AungMoeKhine/smart_water_pump-control/main/";

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
  const int WAIT_SECONDS_SET = 10; 
  int waitSeconds = 10;
  int status = 1;  
  volatile float currentVoltage = 0.0f; // Volatile for Dual-Core
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
  volatile int displayUpperPercentage = 0; // Volatile for Dual-Core
  float upperDistance = 0;
  int upperInvalidCount = 0;
  static const int MAX_INVALID_COUNT = 10;
  bool errorAck = false;         
  bool firstReadingDone = false; 

  void updateFullThreshold() {
    float effectiveHeight = upperHeight + BUFFER_HEIGHT;
    FULL_THRESHOLD = (upperHeight * 0.8 / effectiveHeight) * 100;
  }
};

struct PumpConfig {
  volatile int motorStatus = 0;  // 0 = OFF, 1 = ON
  volatile bool isRunning = false;
  bool flowDetected = false;
  bool manualOverride = false;
  bool wasRunningBeforeVoltageError = false;
};

struct DryRunConfig {
  int WAIT_SECONDS_SET = 60;
  int waitSeconds = 60;
  volatile int error = 0;  // 0 = No error, 1 = Alarm, 2 = Locked
  unsigned long lastUpdate = 0;
  unsigned long alarmStartTime = 0;
  int autoRetryMinutes = 30; // 0 = Disabled
  int retryCountdown = 0; 
  unsigned long lastRetryUpdate = 0;
};

struct OTAConfig {
  bool updateAvailable;
  int newVersion;
  int remoteVersion; 
} otaConfig = { false, 0, 0 };

struct ScheduleConfig {
  int dndStart = 22;  
  int dndEnd = 6;     
  bool enabled = false;
  float timezoneOffset = 6.5;  
};

ScheduleConfig scheduleConfig;

enum class PumpState {
  IDLE,
  PUMPING,
  DRY_RUN_ALARM,
  DRY_RUN_LOCKED,
  SENSOR_ERROR,
  VOLTAGE_ERROR,
  VOLTAGE_WAIT
};

// ============================================================================
// GLOBAL OBJECTS & PROTOTYPES
// ============================================================================

// --- LICENSE SYSTEM VARIABLES ---
unsigned long installDate = 0;
unsigned long lastTokenTime = 0;
int validDays = 10;               // Default trial period
bool isSystemExpired = false;
String uploadedLicenseToken = ""; // Buffer for Web Upload
// --------------------------------

VoltageConfig voltageConfig;
TankConfig tankConfig;
PumpConfig pumpConfig;
DryRunConfig dryRunConfig;
PumpState currentState = PumpState::IDLE;
bool currentDndActive = false; 

Preferences preferences;
WiFiMulti wifiMulti;
WebServer server(80);
WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);
Adafruit_NeoPixel rgbLed(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
ZMPT101B voltageSensor(VOLTAGE_SENSOR_PIN, 50.0);

String deviceID = "";
String subTopic = "";
String statusTopic = "";
String onlineTopic = "";
String devicePin = "123456";
String webAlertMsg = "";
unsigned long webAlertTime = 0;
WiFiUDP dnsUdp;
const byte DNS_PORT = 53;

// Task Handle for Dual Core
TaskHandle_t NetworkTaskHandle;

// WiFi Credentials Global
String ssid_saved = "";
String pass_saved = "";

// Explicit Function Prototypes 
void checkExpiry(); // New Prototype
void networkTask(void * parameter);
void publishState();
void updatePumpLogic();
void saveMotorStatus();
void checkOTA();
void startOTA();
void handleUpdatePage();  
void mqttCallback(char* topic, byte* payload, unsigned int length); 
void handleRoot();
void handleSettings();
void handleSave();
void handleStatus();
void handleToggle();
void handleReset();
void handleScan();
void handleLogo();
String generateStatusJson();
bool reconnectMQTT();
void updateLCD();
void updateLEDStatus();
void monitorSensors();
void monitorButton();
String processManualToggle();
void setLedColor(uint8_t r, uint8_t g, uint8_t b);
String getDeviceID(); 
// New Handlers
void handleAdmin();
void handleApplyLicenseText();
void handleLicenseUpload();
void handleLicenseUploadData();

// --- Time Sync Helper ---
void waitForTimeSync() {
  Serial.println("Syncing Time via NTP...");
  configTime(scheduleConfig.timezoneOffset * 3600, 0, "pool.ntp.org", "time.google.com", "time.nist.gov");
  
  time_t now = time(nullptr);
  int retry = 0;
  while (now < 1600000000 && retry < 15) { 
    delay(300);
    Serial.print(".");
    now = time(nullptr);
    retry++;
  }
  Serial.println();
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    Serial.print("Time Synced: ");
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

    // --- LICENSE ACTIVATION LOGIC ---
    time(&now);
    if (installDate == 0 && now > 1600000000) {
      installDate = (unsigned long)now;
      preferences.begin("pump-control", false);
      preferences.putULong("installDate", installDate);
      preferences.end();
      Serial.println("License Timer Activated!");
    }
    checkExpiry();
    // --------------------------------

  } else {
    Serial.println("Time Sync Failed (Timeout). Proceeding...");
  }
}

// --- WiFi & DNS Helper ---
void processDNS() {
  if (WiFi.getMode() == WIFI_STA) return;

  int packetSize = dnsUdp.parsePacket();
  if (packetSize > 0) {
    unsigned char buf[512];
    dnsUdp.read(buf, 512);
    if ((buf[2] & 0x80) == 0) {
      buf[2] = 0x81;
      buf[3] = 0x80;
      buf[7] = 0x01;
      int pos = 12;
      while (buf[pos] != 0 && pos < packetSize) pos += buf[pos] + 1;
      pos += 5;
      buf[pos++] = 0xC0;
      buf[pos++] = 0x0C;
      buf[pos++] = 0x00;
      buf[pos++] = 0x01;
      buf[pos++] = 0x00;
      buf[pos++] = 0x01;
      buf[pos++] = 0x00;
      buf[pos++] = 0x00;
      buf[pos++] = 0x00;
      buf[pos++] = 0x3C;
      buf[pos++] = 0x00;
      buf[pos++] = 0x04;
      IPAddress ip = WiFi.softAPIP();
      buf[pos++] = ip[0];
      buf[pos++] = ip[1];
      buf[pos++] = ip[2];
      buf[pos++] = ip[3];
      dnsUdp.beginPacket(dnsUdp.remoteIP(), dnsUdp.remotePort());
      dnsUdp.write(buf, pos);
      dnsUdp.endPacket();
    }
  }
}

// --- Web Interface HTML (Full UI) ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Smart Pump Dashboard</title>
<style>
  body{font-family:sans-serif;background:#121212;color:white;text-align:center;padding:20px;margin:0;}
  .logo { width: 80px; height: auto; margin-bottom: 10px; border-radius: 50%; border: 2px solid #333; }
  .card{background:#1e1e1e;border-radius:12px;padding:20px;max-width:400px;margin:auto;box-shadow:0 4px 15px rgba(0,0,0,0.5);border:1px solid #333;position:relative;}
  
  .tabs { display: flex; max-width: 440px; margin: 0 auto 15px auto; gap: 10px; }
  .tab { flex: 1; padding: 12px; text-decoration: none; border-radius: 12px; font-weight: bold; font-size: 1.05rem; transition: 0.3s; border: 1px solid transparent; }
  .tab-active { background: #1e1e1e; color: #03ef; border: 1px solid #333; box-shadow: 0 4px 15px rgba(0,0,0,0.5); }
  .tab-inactive { background: #121212; color: #888; }
  .tab-inactive:hover { background: #1a1a1a; color: #ccc; }

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
  .modal { display: none; position: fixed; z-index: 1000; left: 0; top: 0; width: 100%; height: 100%; background-color: rgba(0,0,0,0.8); backdrop-filter: blur(4px); }
  .modal-content { background-color: #1e1e1e; position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); padding: 25px; border-left: 5px solid #ff4d4d; border-radius: 8px; width: 85%; max-width: 320px; box-shadow: 0 10px 30px rgba(0,0,0,0.8); text-align: left; box-sizing: border-box; }
  .modal-title { color: #ff4d4d; font-size: 1.3rem; margin: 0 0 10px 0; display: flex; align-items: center; gap: 8px; }
  .modal-text { color: #ddd; margin-bottom: 25px; font-size: 1.05rem; line-height: 1.5; }
  .modal-close { background: #333; color: white; padding: 12px; border: none; border-radius: 6px; cursor: pointer; font-weight: bold; width: 100%; font-size: 1.1rem; transition: background 0.2s; }
  .modal-close:hover { background: #555; }
</style></head><body>
  <div id="warnModal" class="modal"><div class="modal-content"><h3 class="modal-title"><span>⚠️</span> Action Blocked</h3><div id="warnText" class="modal-text">Safety protocols prevented this action.</div><button class="modal-close" onclick="closeModal()">Understood</button></div></div>
  
  <div id="expiryBanner" style="display:none; background:#dc3545; color:white; padding:12px; border-radius:12px; margin-bottom:15px; font-weight:bold; max-width:440px; margin:0 auto 15px auto;">
      ⚠️ SYSTEM EXPIRED ⚠️<br><span style="font-size:0.8rem; font-weight:normal">Pump operations are disabled.</span>
  </div>
  <div id="warnBanner" style="display:none; background:#ffc107; color:#333; padding:12px; border-radius:12px; margin-bottom:15px; font-weight:bold; max-width:440px; margin:0 auto 15px auto;">
      ⚠️ SUBSCRIPTION ENDING SOON ⚠️<br><span style="font-size:0.8rem; font-weight:normal" id="warnMsg"></span>
  </div>

  <div class="tabs">
    <a href="/" class="tab tab-active">🏠 Home</a>
    <a href="/settings" class="tab tab-inactive">⚙️ Settings</a>
  </div>

  <div class="card">
    <button id="rfb" class="refresh-btn" onclick="rfr()" title="Refresh Status">🔄</button>
    <div style="font-size:0.8rem;text-align:left;margin-bottom:10px;"><span id="dot" class="conn-dot"></span><span id="cStat">Device: Online</span></div>
    <h2 style="color:#03ef;margin-top:0;">
      <img src="/logo.png" class="logo"><br>
      💧Tank Water Level
      <div id="dndBadge" style="display:none; font-size: 0.9rem; background: #6f42c1; color: white; padding: 4px 10px; border-radius: 12px; margin: 5px auto; width: fit-content;">
        🌙 DND Active
      </div>
    </h2>
    <div class="tank-wrap"><div class="tank-inner"><div class="tank-fill" id="tankFill"></div></div><div class="tank-ridges"></div><div class="tank-text" id="tankVal">-- %</div></div>
    <div class="row"><span>Voltage:</span><span id="volt">-- V</span></div>
    <div class="row"><span>Volt Status:</span><span id="vstat">--</span></div>
    <div class="row"><span>Pump:</span><span id="state">--</span></div>
    <div class="row"><span>System Info:</span><span id="info">--</span></div>
    <div class="row" id="cdRow" style="display:none;color:#ff4d4d;font-weight:bold;"><span>Dry-Run in:</span><span id="cd">--</span></div>
    <div id="otaHub" style="display:none; margin:15px 0; padding:15px; border:1px solid #03ef; border-radius:12px; background:rgba(3,239,98,0.05);">
        <div style="color:#03ef; font-weight:bold; margin-bottom:10px;" id="otaMsg">New Version Available!</div>
        <button class="btn btn-green" onclick="startOTA()">Update Now</button>
    </div>
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
  function checkOTA() { window.location.href = '/update_github'; }
  function startOTA() { if(confirm('Are you sure you want to update? The system will reboot.')) fetch('/start-ota').then(()=>{ document.body.innerHTML='<h2 style="color:white;text-align:center;margin-top:50px;">Updating... Please wait.</h2>'; }); }
  function rfr() { let b=document.getElementById('rfb'); b.classList.add('spinning'); upd().finally(()=>b.classList.remove('spinning')); }
  
  let lastWebAlert = ""; 
  function upd() { return fetch('/status').then(r=>r.json()).then(d=>{
      document.getElementById('dot').className='conn-dot'; document.getElementById('cStat').innerText='Device: Online';
      document.getElementById('dip').innerText = d.ip; document.getElementById('tankFill').style.height = d.tank + '%';
      document.getElementById('tankVal').innerText = d.tStr; if (document.getElementById('cid')) document.getElementById('cid').innerText = d.id;
      document.getElementById('volt').innerText=d.volt+' V'; document.getElementById('vstat').innerText=d.vStat;
      document.getElementById('state').innerText=d.pStat; document.getElementById('info').innerText=d.info;
      document.getElementById('dndBadge').style.display = d.dndAct ? 'block' : 'none';
      let btn=document.getElementById('btnToggle'); let rst=document.getElementById('btnReset');
      if(d.err){rst.style.display='block';btn.style.display='none';} 
      else if(d.sErr && !d.ack){btn.style.display='block';rst.style.display='none';btn.innerText='Reset Alarm';btn.className='btn btn-red';}
      else{rst.style.display='none';btn.style.display='block'; btn.innerText=d.pStat=="ON"?'Stop the Pump':'Start the Pump'; btn.className=d.pStat=="ON"?'btn btn-red':'btn btn-green';}
      
      let cd=document.getElementById('cdRow');
      if(d.err == 2 && d.rM > 0) {
         cd.style.display='flex';
         cd.children[0].innerText = 'Retry in:';
         let m = Math.floor(d.rCd / 60); let s = d.rCd % 60;
         document.getElementById('cd').innerText = m + 'm ' + s + 's';
      } else if(d.pStat=="ON" && d.info=="FLOW_CHECKING!") {
         cd.style.display='flex';
         cd.children[0].innerText = 'Dry-Run in:';
         document.getElementById('cd').innerText = d.cd + 's';
      } else {
         cd.style.display='none';
      }

      let ota=document.getElementById('otaHub');
      if(d.ota){ ota.style.display='block'; document.getElementById('otaMsg').innerText='New Version ' + d.nVer + ' Available!'; }
      else{ ota.style.display='none'; }

      if(d.expired) {
          document.getElementById('expiryBanner').style.display = 'block';
          document.getElementById('warnBanner').style.display = 'none';
      } else {
          document.getElementById('expiryBanner').style.display = 'none';
          if(d.daysLeft <= 7 && d.daysLeft >= 0) {
              document.getElementById('warnBanner').style.display = 'block';
              document.getElementById('warnMsg').innerText = d.daysLeft + " days remaining.";
          } else document.getElementById('warnBanner').style.display = 'none';
      }

      if(d.alertMsg && d.alertMsg !== lastWebAlert) {
         showModal(d.alertMsg);
         lastWebAlert = d.alertMsg;
      } else if (!d.alertMsg) {
         lastWebAlert = "";
      }
    }).catch(e=>{ document.getElementById('dot').className='conn-dot off'; document.getElementById('cStat').innerText='Device: Offline (Connecting...)'; }); }
  setInterval(upd,1000);
</script></body></html>
)rawliteral";

const char settings_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
<style>
  body{font-family:sans-serif;background:#121212;color:white;padding:20px;text-align:center;margin:0;}
  .card{background:#1e1e1e;border-radius:12px;padding:20px;max-width:400px;margin:auto;border:1px solid #333;text-align:left;box-shadow: 0 10px 25px rgba(0,0,0,0.5); position:relative;}
  
  .tabs { display: flex; max-width: 440px; margin: 0 auto 15px auto; gap: 10px; }
  .tab { flex: 1; padding: 12px; text-decoration: none; border-radius: 12px; font-weight: bold; font-size: 1.05rem; transition: 0.3s; border: 1px solid transparent; text-align: center; }
  .tab-active { background: #1e1e1e; color: #fff; border: 1px solid #333; box-shadow: 0 4px 15px rgba(0,0,0,0.5); }
  .tab-inactive { background: #121212; color: #888; }
  .tab-inactive:hover { background: #1a1a1a; color: #ccc; }

  .conn-dot{width:10px;height:10px;background:#28a745;border-radius:50%;display:inline-block;margin-right:5px;}
  .off{background:#dc3545;}
  .refresh-btn{position:absolute;top:15px;right:15px;background:none;border:none;color:#03ef;font-size:1.2rem;cursor:pointer;padding:5px;}
  .refresh-btn:active{transform:scale(0.96);opacity:0.85;}
  @keyframes spin { 100% { transform: rotate(360deg); } }
  .spinning { animation: spin 1s linear infinite; opacity: 1 !important; color: #03ef62 !important; }

  .lbl-wrap { display: flex; justify-content: space-between; align-items: flex-end; margin: 15px 0 5px; }
  label{font-weight:bold;color:#aaa; font-size: 0.9rem;}
  .logo { width: 80px; height: auto; margin: 0 auto 20px; display: block; border-radius: 50%; border: 2px solid #333; }
  .range { color: #555; font-size: 0.75rem; font-weight: normal; }
  input, select{width:100%;padding:12px;background:#2a2a2a;border:1px solid #444;color:white;border-radius:6px;box-sizing:border-box; font-size: 1rem;}
  .pass-row { position: relative; }
  .show-pass { margin-top: 8px; font-size: 0.85rem; color: #888; display: flex; align-items: center; cursor: pointer; }
  .show-pass input { width: auto; margin-right: 8px; }
  .btn{width:100%;padding:15px;background:#28a745;color:white;border:none;border-radius:8px;margin-top:25px;font-weight:bold;cursor:pointer;font-size:1.1rem;}
  .btn:active { transform: scale(0.98); opacity: 0.9; }
  hr { border: 0; border-top: 1px solid #333; margin: 25px 0; }
</style></head><body>

  <div class="tabs">
    <a href="/" class="tab tab-inactive">🏠 Home</a>
    <a href="/settings" class="tab tab-active">⚙️ Settings</a>
  </div>

  <div class="card">
    <button id="rfb" class="refresh-btn" onclick="rfr()" title="Refresh Status">🔄</button>
    <div style="font-size:0.8rem;text-align:left;margin-bottom:10px;"><span id="dot" class="conn-dot"></span><span id="cStat">Device: Online</span></div>

    <img src="/logo.png" class="logo">
    
    <form action="/save" method="POST">
      <div class="lbl-wrap"><label>WiFi SSID</label><button type="button" onclick="scn()" style="font-size:0.7rem; color:#03ef; background:none; border:1px solid #333; border-radius:4px; cursor:pointer; padding:2px 5px;">Scan for Networks</button></div>
      <select id="ss" name="ssid_sel" onchange="chSS(this)">
        <option value="" selected>-- Keep Current (%CUR_SSID%) --</option>
        %WIFI_LIST%
        <option value="__man__">Enter SSID Manually...</option>
      </select>
      <input type="text" id="mi" name="ssid_man" placeholder="Type Network Name" style="display:none; margin-top:10px;">
      
      <div class="lbl-wrap"><label>WiFi Password</label></div>
      <div class="pass-row"><input type="password" id="p" name="pass" placeholder="Leave empty to keep current"><label class="show-pass"><input type="checkbox" onclick="togglePass('p')"> Show WiFi Password</label></div>
      <div class="lbl-wrap"><label>Device PIN (for Cloud)</label></div>
      <div class="pass-row"><input type="password" id="pin" name="pin" value="%PIN%" required><label class="show-pass"><input type="checkbox" onclick="togglePass('pin')"> Show PIN</label></div>
      <hr>
      <div class="lbl-wrap"><label>Tank Height</label><span class="range">1.0 - 7.0 ft</span></div><select name="uH">%TANK_LIST%</select>
      <div class="lbl-wrap"><label>Over Voltage Set</label><span class="range">230 - 260 V</span></div><select name="vH">%VH_LIST%</select>
      <div class="lbl-wrap"><label>Under Voltage Set</label><span class="range">150 - 190 V</span></div><select name="vL">%VL_LIST%</select>
      
      <div class="lbl-wrap"><label>Dry-Run Delay</label><span class="range">60 - 180 s</span></div><select name="dD">%DRY_LIST%</select>
      
      <div class="lbl-wrap"><label>Auto-Retry Wait</label><span class="range">Disable / 30 / 60</span></div>
      <select name="rM"><option value="0" %RM0%>Disabled</option><option value="30" %RM30%>30 Minutes</option><option value="60" %RM60%>60 Minutes</option></select>
      
      <hr>
      <div class="lbl-wrap"><label>🌙 Smart Scheduling (DND)</label></div>
      <select name="dndEn"><option value="0" %DND_OFF%>Disabled</option><option value="1" %DND_ON%>Enabled</option></select>
      <div style="display:flex; gap:10px; margin-top:10px;">
        <div style="flex:1;"><label>Start Hour</label><select name="dndS">%START_LIST%</select></div>
        <div style="flex:1;"><label>End Hour</label><select name="dndE">%END_LIST%</select></div>
      </div>
      <div class="lbl-wrap"><label>📍 Home Time Zone (GMT)</label></div>
      <select name="tzOf">%TZ_LIST%</select>
      <button type="submit" class="btn">Save & Reboot</button>
    </form>
    <hr>
    <div style="text-align:center;">
      <h3 style="color:#6f42c1; margin:0 0 15px 0;">🛠️ Maintenance</h3>
      <div id="otaHub" style="display:none; margin-bottom:15px; padding:15px; border:1px solid #6f42c1; border-radius:12px; background:rgba(111, 66, 193, 0.05); text-align:left;">
          <div style="color:#6f42c1; font-weight:bold; margin-bottom:10px;" id="otaMsg">Update Available!</div>
          <button class="btn" style="background:#28a745; margin-top:0;" onclick="startOTA()">Update Now</button>
      </div>
      <button class="btn" style="background:#6f42c1; margin-top:0;" onclick="checkOTA()">Check for Updates</button>
      <p style="font-size:0.8rem; color:#555; margin-top:10px;">Current Version: v%VER%</p>
    </div>
    
    <hr>
    <div style="text-align:center;">
      <h3 style="color:#28a745; margin:0 0 15px 0;">🔑 License Management</h3>
      <div style="background:#2a2a2a; padding:10px; border-radius:8px; font-size:0.85rem; margin-bottom:15px; word-break:break-all;">
          <strong>Device ID:</strong> <span id="did" style="color:#03ef;">%DID%</span>
      </div>
      
      <!-- Option 1: File Upload -->
      <form method='POST' action='/upload_license' enctype='multipart/form-data'>
         <input type='file' name='license' accept='.key,.txt' style="background:#1e1e1e; padding:10px; margin-bottom:5px; width:100%; box-sizing:border-box;">
         <button type='submit' class="btn" style="margin-top:0; margin-bottom:15px;">Upload Token File</button>
      </form>

      <div style="color:#666; margin-bottom:15px;">- OR -</div>

      <!-- Option 2: Text Paste -->
      <form method='POST' action='/apply_license'>
         <input type='text' name='key' placeholder="Paste Token String (MTc...)" style="padding:12px; margin-bottom:5px; width:100%; box-sizing:border-box;">
         <button type='submit' class="btn" style="margin-top:0; background:#6f42c1;">Activate via Text</button>
      </form>
    </div>

  </div>

  <div style="margin-top:25px;color:#666;font-size:0.75rem;">
    <div style="margin-bottom:8px;">Cloud ID: <span id="cid" style="color:#888;">--</span></div>
    Device IP: <span id="dip">--</span>
  </div>

<script>
  function togglePass(id) { var x = document.getElementById(id); if (x.type === "password") x.type = "text"; else x.type = "password"; }
  function chSS(s) { 
    var m = document.getElementById('mi');
    if(s.value === '__man__') { m.style.display='block'; m.required=true; }
    else { m.style.display='none'; m.required=false; }
  }
  function scn() { 
    if(!confirm('Scan for WiFi networks?')) return;
    fetch('/scan').then(()=>alert('Scan started. This takes about 5-10 seconds. Refreshing list...')).then(()=>setTimeout(()=>location.reload(),6000));
  }
  function checkOTA() { window.location.href = '/update_github'; }
  function startOTA() { if(confirm('Are you sure you want to update? The system will reboot.')) fetch('/start-ota').then(()=>{ document.body.innerHTML='<h2 style="color:white;text-align:center;margin-top:50px;">Updating...</h2>'; }); }
  
  function rfr() { let b=document.getElementById('rfb'); b.classList.add('spinning'); upd().finally(()=>b.classList.remove('spinning')); }
  function upd() { 
    return fetch('/status').then(r=>r.json()).then(d=>{
      document.getElementById('dot').className='conn-dot'; 
      document.getElementById('cStat').innerText='Device: Online';
      document.getElementById('dip').innerText = d.ip; 
      if (document.getElementById('cid')) document.getElementById('cid').innerText = d.id;
      
      let ota=document.getElementById('otaHub');
      if(d.ota){ ota.style.display='block'; document.getElementById('otaMsg').innerText='New Version ' + d.nVer + ' Available!'; }
      else{ ota.style.display='none'; }
    }).catch(e=>{ 
      document.getElementById('dot').className='conn-dot off'; 
      document.getElementById('cStat').innerText='Device: Offline (Connecting...)'; 
    }); 
  }
  
  setInterval(upd, 1000); 
  upd();
</script></body></html>
)rawliteral";

// --- SENSOR IMPLEMENTATION ---
class NonBlockingUltrasonic {
private:
  int trigPin, echoPin;
  static const int NUM_SAMPLES = 10;
  static const unsigned long SAMPLE_INTERVAL_MS = 60;
  static const unsigned long MAX_PULSE_DURATION = 20000; 
  
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
        if (temp[j] < temp[i]) {
          float t = temp[i];
          temp[i] = temp[j];
          temp[j] = t;
        }
      }
    }
    return (n % 2 == 0) ? (temp[n / 2 - 1] + temp[n / 2]) / 2.0 : temp[n / 2];
  }

public:
  NonBlockingUltrasonic(int t, int e)
    : trigPin(t), echoPin(e) {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW);
  }

  void start() {
    if (!collecting) {
      sampleIndex = 0;
      collecting = true;
      lastSampleTime = millis() - SAMPLE_INTERVAL_MS;
    }
  }

  void update() {
    unsigned long now = millis();
    if (collecting && (now - lastSampleTime >= SAMPLE_INTERVAL_MS)) {
      lastSampleTime = now;
      digitalWrite(trigPin, LOW);
      delayMicroseconds(9);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(140);
      digitalWrite(trigPin, LOW);
      long duration = pulseIn(echoPin, HIGH, MAX_PULSE_DURATION);
      float inches = (duration == 0) ? -1.0 : (duration * 0.01356 / 2.0);
      samples[sampleIndex++] = inches;
      if (sampleIndex >= NUM_SAMPLES) {
        collecting = false;
        float v[NUM_SAMPLES];
        int vc = 0;
        for (int i = 0; i < NUM_SAMPLES; i++)
          if (samples[i] > 0 && samples[i] <= MAX_DISTANCE) v[vc++] = samples[i];
        lastMedian = (vc > NUM_SAMPLES / 2) ? median(v, vc) : -1.0;
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

  if (now - lastScan >= ULTRASONIC_INTERVAL) {
    if (!upperSensor.isBusy()) upperSensor.start();
  }
  upperSensor.update();

  if (!upperSensor.isBusy() && now - lastScan >= ULTRASONIC_INTERVAL) {
    float dist = upperSensor.getDistance();
    if (dist > 0) {
      tankConfig.firstReadingDone = true; 
      tankConfig.upperDistance = dist;
      tankConfig.upperInvalidCount = 0;
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

  static unsigned long lastVoltSample = 0;
  if (now - lastVoltSample >= 500) {
    float v = voltageSensor.getRmsVoltage();
    if (v < 40.0f) v = 0.0f;
    if (v < 350.0f) {
        if (voltageConfig.currentVoltage < 10.0f && v > 50.0f) {
            voltageConfig.currentVoltage = v;
        } else {
            voltageConfig.currentVoltage = (0.2f * v) + (0.8f * voltageConfig.currentVoltage);
        }
    }
    lastVoltSample = now;
  }
}

// ============================================================================
// SYSTEM SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== DUAL CORE SMART PUMP SYSTEM BOOTING ===");

  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  pinMode(MANUAL_BTN_PIN, INPUT_PULLUP);  
  
  digitalWrite(MOTOR_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  preferences.begin("pump-control", false);
  tankConfig.upperHeight = preferences.getFloat("upperH", TankConfig::MIN_HEIGHT);
  voltageConfig.HIGH_THRESHOLD = preferences.getInt("vHigh", 250);
  voltageConfig.LOW_THRESHOLD = preferences.getInt("vLow", 170);
  dryRunConfig.WAIT_SECONDS_SET = preferences.getInt("dryDelay", 60);
  dryRunConfig.autoRetryMinutes = preferences.getInt("retryMins", 30);
  voltageConfig.status = 0;         
  voltageConfig.waitSeconds = 10;    
  devicePin = preferences.getString("pin", "123456");
  ssid_saved = preferences.getString("ssid", "");
  pass_saved = preferences.getString("pass", "");
  pumpConfig.motorStatus = preferences.getInt("motor", 0);
  pumpConfig.manualOverride = preferences.getBool("override", false);

  scheduleConfig.enabled = preferences.getBool("dndEn", false);
  scheduleConfig.dndStart = preferences.getInt("dndS", 22);
  scheduleConfig.dndEnd = preferences.getInt("dndE", 6);
  scheduleConfig.timezoneOffset = preferences.getFloat("tzOf", 6.5);
  
  // Load License Details
  installDate = preferences.getULong("installDate", 0);
  validDays = preferences.getInt("validDays", 10);
  lastTokenTime = preferences.getULong("lastTokenTime", 0);
  
  preferences.end();

  tankConfig.updateFullThreshold();
  checkExpiry(); // Check license validity right away

  rgbLed.begin();
  rgbLed.setBrightness(30);
  voltageSensor.setSensitivity(473.5f); // FIXED: SENSITIVITY RESTORED TO 473.5f

  Serial.println("Running Initial Hardware Safety Check...");
  for(int i=0; i<10; i++) { monitorSensors(); delay(50); }
  updatePumpLogic(); 

  Wire.begin(SDA_PIN, SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcd.createChar(1, bar1); lcd.createChar(2, bar2); lcd.createChar(3, bar3);
  lcd.createChar(4, bar4); lcd.createChar(5, bar5); lcd.createChar(6, bar6);
  lcd.createChar(7, bar7); lcd.createChar(8, bar8);
  lcd.setCursor(0, 0); lcd.print("********************");
  lcd.setCursor(0, 1); lcd.print("*  AUTOMATIC PUMP  *");
  lcd.setCursor(0, 2); lcd.print("*  CONTROL SYSTEM  *");
  lcd.setCursor(0, 3); lcd.print("********************");
  
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("Auto-Pump-Config", "12345678");
  Serial.print("Initial Scan Start: ");
  int nS = WiFi.scanNetworks(true); 
  Serial.println(nS == -1 ? "OK" : "Error");
  dnsUdp.begin(DNS_PORT);
  Serial.println("\nHotspot Started: Auto-Pump-Config");

  if (ssid_saved != "") {
    wifiMulti.addAP(ssid_saved.c_str(), pass_saved.c_str());
    Serial.print("Connecting to WiFi: " + ssid_saved);
    unsigned long startAttempt = millis();
    while (wifiMulti.run() != WL_CONNECTED && millis() - startAttempt < 10000) {
      delay(500); Serial.print(".");
    }
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("\nWiFiMulti failed. Trying forced login to: " + ssid_saved);
      WiFi.begin(ssid_saved.c_str(), pass_saved.c_str());
      unsigned long startWait = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - startWait < 8000) {
        delay(500); Serial.print("!");
      }
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
      waitForTimeSync(); 
      WiFi.mode(WIFI_STA);
      dnsUdp.stop(); 
      Serial.println("AP Mode Disabled.");
    } else {
      Serial.println("\nAll connection attempts failed. Use AP to configure.");
    }
  } else {
    Serial.println("\nNo WiFi credentials saved. Use AP to configure.");
  }

  deviceID = getDeviceID();
  subTopic = "smartpump/" + deviceID + "/set";
  statusTopic = "smartpump/" + deviceID + "/status";
  onlineTopic = "smartpump/" + deviceID + "/online";

  server.on("/", handleRoot);
  server.on("/settings", handleSettings);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/status", handleStatus);
  server.on("/toggle", handleToggle);
  server.on("/reset", handleReset);
  server.on("/scan", handleScan);
  server.on("/logo.png", handleLogo);
  server.on("/update_github", handleUpdatePage);  
  server.on("/check-ota", []() { checkOTA(); server.send(200, "application/json", "{\"status\":\"checking\"}"); });
  server.on("/start-ota", []() { server.send(200, "application/json", "{\"status\":\"starting\"}"); startOTA(); });
  
  // NEW ROUTES
  server.on("/upload_license", HTTP_POST, handleLicenseUpload, handleLicenseUploadData);
  server.on("/apply_license", HTTP_POST, handleApplyLicenseText);
  server.on("/admin", handleAdmin); // Restored Admin Backdoor

  server.onNotFound([]() { server.sendHeader("Location", "http://192.168.4.1/", true); server.send(302, "text/plain", "Redirect"); });
  server.begin();
  
  if (MDNS.begin("smartpump")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("mDNS responder started: http://smartpump.local");
  }

  espClient.setInsecure();
  espClient.setHandshakeTimeout(60); 
  espClient.setTimeout(60000);       

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(4096); 
  mqttClient.setSocketTimeout(60);
  mqttClient.setKeepAlive(60);       

  Serial.println("System Initialized. Device ID: " + deviceID);

  // --- LAUNCH NETWORK TASK ON CORE 0 ---
  xTaskCreatePinnedToCore(
    networkTask,      
    "NetworkTask",    
    12000,            
    NULL,             
    1,                
    &NetworkTaskHandle, 
    0);               
    
  Serial.println("Network Task Started on Core 0");
}

// ----------------------------------------------------------------------------
// CORE 0: NETWORK TASK (WiFi, MQTT, OTA)
// ----------------------------------------------------------------------------
void networkTask(void * parameter) {
  while(true) {
    if (ssid_saved != "" && WiFi.status() != WL_CONNECTED) {
       WiFi.disconnect();
       WiFi.reconnect();
       vTaskDelay(5000 / portTICK_PERIOD_MS); 
    }
    if (WiFi.status() == WL_CONNECTED) {
      if (!mqttClient.connected()) {
        static unsigned long lastReconnectAttempt = 0;
        if (millis() - lastReconnectAttempt > 10000) {
          if (!reconnectMQTT()) lastReconnectAttempt = millis(); 
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
    static unsigned long lastOTACheck = 0;
    if (millis() - lastOTACheck >= 3600000) {  
      lastOTACheck = millis();
      checkOTA();
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}

// ----------------------------------------------------------------------------
// CORE 1: MAIN LOOP (Sensors, Pump Logic, UI)
// ----------------------------------------------------------------------------
void loop() {
  processDNS();
  monitorSensors();
  monitorButton();  
  updatePumpLogic();

  server.handleClient();

  updateLCD();
  updateLEDStatus();

  delay(1);
}

// ----------------------------------------------------------------------------
// HELPERS & LOGIC
// ----------------------------------------------------------------------------
String getStartBlockReason() {
  if (voltageConfig.currentVoltage > voltageConfig.HIGH_THRESHOLD)
    return "Voltage is OVER (" + String((int)voltageConfig.currentVoltage) + "V). Limit: " + String(voltageConfig.HIGH_THRESHOLD) + "V.";
  if (voltageConfig.currentVoltage < voltageConfig.LOW_THRESHOLD)
    return "Voltage is UNDER (" + String((int)voltageConfig.currentVoltage) + "V). Limit: " + String(voltageConfig.LOW_THRESHOLD) + "V.";
  if (voltageConfig.status == 0)
    return "Voltage stabilization is in progress. Please wait a moment.";
  if (tankConfig.rawUpperPercentage >= tankConfig.FULL_THRESHOLD)
    return "The Tank is already FULL (" + String(tankConfig.displayUpperPercentage) + "%). Pumping is not needed.";
  if (tankConfig.upperInvalidCount >= TankConfig::MAX_INVALID_COUNT)
    return "Sensor Error! Unable to verify water level safely. Please check the ultrasonic sensor.";
  return "";
}

String processManualToggle() {
  if (isSystemExpired) return "Blocked:License Expired! Please renew to operate.";

  bool sensorErrorActive = (tankConfig.upperInvalidCount >= TankConfig::MAX_INVALID_COUNT);

  if (sensorErrorActive) {
    if (!tankConfig.errorAck) {
      tankConfig.errorAck = true;  
      updatePumpLogic();           
      publishState();
      return "Silenced";
    }
  }

  if (dryRunConfig.error != 0) {
    dryRunConfig.error = 0;
    dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET;
    digitalWrite(BUZZER_PIN, LOW);
    pumpConfig.motorStatus = 0; 
    saveMotorStatus();
    updatePumpLogic();
    publishState();
    return "Success";
  }

  bool wantsToStart = (pumpConfig.motorStatus == 0);
  if (wantsToStart) {
    String blockReason = getStartBlockReason();
    if (blockReason != "") return "Blocked:" + blockReason;
  }

  pumpConfig.motorStatus = !pumpConfig.motorStatus;

  if (pumpConfig.motorStatus == 1) {
    dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET;
    pumpConfig.manualOverride = false; 
  } else {
    pumpConfig.manualOverride = true;  
  }

  saveMotorStatus();
  updatePumpLogic();
  publishState();
  return "Success";
}

void monitorButton() {
  static int lastReading = HIGH;
  static unsigned long lastDebounceTime = 0;
  
  int reading = digitalRead(MANUAL_BTN_PIN);
  
  if (reading != lastReading) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > 20) {  
    static int buttonState = HIGH;
    if (reading != buttonState) {
      buttonState = reading;
      
      if (buttonState == LOW) { 
        Serial.println("Physical Manual Button Pressed!");
        String res = processManualToggle();
        
        if (res.startsWith("Blocked:")) {
          String reason = res.substring(8);
          Serial.println("Manual Action Blocked: " + reason);
          
          setLedColor(255, 0, 0); 
          
          if (mqttClient.connected()) {
            DynamicJsonDocument resp(256);
            resp["alert"] = "Action Blocked (Device Button)";
            resp["reason"] = reason;
            String respStr;
            serializeJson(resp, respStr);
            mqttClient.publish(statusTopic.c_str(), respStr.c_str());
          }

          webAlertMsg = "Device Button: " + reason;
          webAlertTime = millis();
        }
      }
    }
  }
  lastReading = reading;
}

void rebootSystem() {
  Serial.println("Rebooting system in 2 seconds...");
  espClient.stop();
  server.stop();
  delay(2000);
  ESP.restart();
}

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
  preferences.putBool("override", pumpConfig.manualOverride); 
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

  if (currentState == PumpState::DRY_RUN_ALARM || currentState == PumpState::DRY_RUN_LOCKED || currentState == PumpState::SENSOR_ERROR || currentState == PumpState::VOLTAGE_ERROR) {
    static bool flash = false;
    flash = !flash;
    setLedColor(flash ? 255 : 0, 0, 0);
  } else if (pumpConfig.isRunning) {
    setLedColor(255, 200, 0);
  } else if (WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
    setLedColor(0, 255, 0);
  } else if (WiFi.status() == WL_CONNECTED) {
    setLedColor(0, 255, 255);
  } else {
    static int bright = 0;
    static int dir = 5;
    bright += dir;
    if (bright >= 50 || bright <= 0) dir = -dir;
    setLedColor(0, 0, bright);
  }
}

// --- LICENSE SYSTEM FUNCTIONS ---
void checkExpiry() {
  if (installDate == 0) return;
  time_t now;
  time(&now);
  unsigned long nowEpoch = (unsigned long)now;
  unsigned long expiryDate = installDate + (validDays * 86400UL);
  isSystemExpired = (nowEpoch > expiryDate);
}

bool verifyAndApplyLicense(String tokenBase64, String& outMsg) {
  tokenBase64.trim();
  if (tokenBase64.length() == 0) { outMsg = "Empty Token"; return false; }

  unsigned char decoded[512];
  size_t outLen = 0;
  int ret = mbedtls_base64_decode(decoded, sizeof(decoded), &outLen, (const unsigned char*)tokenBase64.c_str(), tokenBase64.length());
  if (ret != 0) { outMsg = "Invalid Base64"; return false; }

  String raw = "";
  for (size_t i = 0; i < outLen; i++) raw += (char)decoded[i];

  int firstPipe = raw.indexOf('|');
  int lastPipe = raw.lastIndexOf('|');
  if (firstPipe == -1 || lastPipe == -1 || firstPipe == lastPipe) { outMsg = "Invalid Token"; return false; }

  String tsStr = raw.substring(0, firstPipe);
  String daysStr = raw.substring(firstPipe + 1, lastPipe);
  String signature = raw.substring(lastPipe + 1);

  unsigned long tokenTs = strtoul(tsStr.c_str(), NULL, 10);
  int addDays = daysStr.toInt();

  if (tokenTs <= lastTokenTime) { outMsg = "Token Already Used"; return false; }

  time_t now; time(&now);
  unsigned long nowEpoch = (unsigned long)now;
  if (nowEpoch > tokenTs + 604800) { outMsg = "Token Expired (>7 days old)"; return false; }

  String secret = "ACER123";
  String payload = tsStr + "|" + daysStr + "|" + secret + "|" + getDeviceID();

  MD5Builder md5;
  md5.begin();
  md5.add(payload);
  md5.calculate();
  if (!md5.toString().equalsIgnoreCase(signature)) { outMsg = "Invalid Signature"; return false; }

  preferences.begin("pump-control", false);
  validDays += addDays;
  lastTokenTime = tokenTs;
  preferences.putInt("validDays", validDays);
  preferences.putULong("lastTokenTime", lastTokenTime);
  preferences.end();
  
  checkExpiry();
  outMsg = "Success! Extended by " + String(addDays) + " days.";
  return true;
}

// --- LICENSE SYSTEM FUNCTIONS ---
// ... (keep checkExpiry and verifyAndApplyLicense as they are) ...

void processLicenseTokenString(String token) {
  String msg;
  bool success = verifyAndApplyLicense(token, msg);
  
  if (success) {
    Serial.println("License Success: " + msg);
  } else {
    Serial.println("License Error: " + msg);
  }

  // Send an immediate MQTT alert back to the app
  if (mqttClient.connected()) {
    DynamicJsonDocument doc(256);
    doc["alert"] = success ? "License Updated" : "License Failed";
    doc["reason"] = msg;
    String alertJson;
    serializeJson(doc, alertJson);
    mqttClient.publish(statusTopic.c_str(), alertJson.c_str());
  }
}

// ... (keep the other license functions as they are) ...

void sendLicenseResponse(int httpCode, String title, String color, String message) {
  String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'><style>body{font-family:sans-serif;text-align:center;padding:50px;background:#121212;color:white;}</style></head><body>";
  html += "<h2 style='color:" + color + ";'>" + title + "</h2><p>" + message + "</p>";
  html += "<br><a href='/settings' style='color:#03ef;text-decoration:none;font-weight:bold;padding:10px;border:1px solid #03ef;border-radius:8px;'>Back to Settings</a></body></html>";
  server.send(httpCode, "text/html", html);
}

void handleLicenseUpload() {
  server.sendHeader("Connection", "close");
  String msg;
  if (verifyAndApplyLicense(uploadedLicenseToken, msg)) sendLicenseResponse(200, "Success!", "#28a745", msg);
  else sendLicenseResponse(403, "Failed", "#dc3545", "Error: " + msg);
}

void handleLicenseUploadData() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    uploadedLicenseToken = ""; 
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    for (size_t i = 0; i < upload.currentSize; i++) uploadedLicenseToken += (char)upload.buf[i];
  }
}

// Handle Text-Based License Application
void handleApplyLicenseText() {
  if (server.hasArg("key")) {
    String token = server.arg("key");
    String msg;
    if (verifyAndApplyLicense(token, msg)) {
      sendLicenseResponse(200, "Activated!", "#28a745", msg);
    } else {
      sendLicenseResponse(403, "Activation Failed", "#dc3545", "Error: " + msg);
    }
  } else {
    sendLicenseResponse(400, "Error", "#dc3545", "No token key provided.");
  }
}

void handleAdmin() {
  if (!server.hasArg("secret") || server.arg("secret") != "ACER123") {
    server.send(403, "text/plain", "Forbidden");
    return;
  }

  preferences.begin("pump-control", false);

  if (server.hasArg("extend")) {
    validDays += server.arg("extend").toInt();
    preferences.putInt("validDays", validDays);
    server.send(200, "text/plain", "Success! Added days. Total Valid Days: " + String(validDays));
  } 
  else if (server.hasArg("reset")) {
    // Reset to "Not Activated" state
    installDate = 0;
    preferences.putULong("installDate", 0);
    server.send(200, "text/plain", "License Reset. Device will re-lock on next reboot/sync.");
  } 
  else {
    // Just show status
    String s = "Admin Mode OK\n";
    s += "Install Date (Epoch): " + String(installDate) + "\n";
    s += "Valid Days: " + String(validDays) + "\n";
    s += "Expired: " + String(isSystemExpired ? "YES" : "NO");
    server.send(200, "text/plain", s);
  }

  preferences.end();
  checkExpiry(); // Update status immediately
}
// ----------------------------------------

void updatePumpLogic() {
  unsigned long currentMillis = millis();

  // License Enforcer
  if (isSystemExpired && pumpConfig.motorStatus == 1) {
    pumpConfig.motorStatus = 0;
    saveMotorStatus();
    publishState();
  }

  currentDndActive = false;
  if (scheduleConfig.enabled) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      int hour = timeinfo.tm_hour;
      if (scheduleConfig.dndStart > scheduleConfig.dndEnd) {
        if (hour >= scheduleConfig.dndStart || hour < scheduleConfig.dndEnd) currentDndActive = true;
      } else {
        if (hour >= scheduleConfig.dndStart && hour < scheduleConfig.dndEnd) currentDndActive = true;
      }
    }
  }

  static PumpState lastState = PumpState::IDLE;
  static int lastPercentage = -1;
  static int lastVoltStatus = -1;
  static bool lastFlow = false;
  static int lastErr = -1;
  static int lastMotorStatus = -1;
  static bool lastAck = false;
  static int lastRawVolt = -1;            
  static unsigned long lastPushTime = 0;
  static bool lastDndState = false;  

  pumpConfig.flowDetected = (digitalRead(FLOW_SENSOR_PIN) == LOW);
  bool sensorError = (tankConfig.upperInvalidCount >= TankConfig::MAX_INVALID_COUNT);

  bool voltAbnormal = (voltageConfig.currentVoltage > voltageConfig.HIGH_THRESHOLD || voltageConfig.currentVoltage < voltageConfig.LOW_THRESHOLD);

  if (voltAbnormal) {
    if (voltageConfig.status == 1) {
      pumpConfig.wasRunningBeforeVoltageError = (pumpConfig.motorStatus == 1);
      voltageConfig.status = 0; 
    }
    voltageConfig.waitSeconds = voltageConfig.WAIT_SECONDS_SET; 
    voltageConfig.lastCheck = currentMillis;                    
  } 
  else if (voltageConfig.status == 0) {
    if (currentMillis - voltageConfig.lastCheck >= GENERAL_INTERVAL) {
      int secondsPassed = (currentMillis - voltageConfig.lastCheck) / 1000;
      voltageConfig.waitSeconds -= secondsPassed;
      voltageConfig.lastCheck = currentMillis;

      if (voltageConfig.waitSeconds < 0) voltageConfig.waitSeconds = 0;

      publishState(); 

      if (voltageConfig.waitSeconds <= 0) {
        voltageConfig.status = 1;                                  
        voltageConfig.waitSeconds = voltageConfig.WAIT_SECONDS_SET; 

        if (pumpConfig.wasRunningBeforeVoltageError) {
          if (tankConfig.rawUpperPercentage < tankConfig.FULL_THRESHOLD && tankConfig.upperInvalidCount < TankConfig::MAX_INVALID_COUNT && !isSystemExpired) {
            pumpConfig.motorStatus = 1;
            dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET; 
            saveMotorStatus();
          }
          pumpConfig.wasRunningBeforeVoltageError = false;
        }
      }
    }
  }

  if (tankConfig.rawUpperPercentage >= tankConfig.FULL_THRESHOLD || sensorError || voltageConfig.status == 0) {
    if (pumpConfig.motorStatus == 1) {
      pumpConfig.motorStatus = 0;
      saveMotorStatus();
    }
    if (tankConfig.rawUpperPercentage >= tankConfig.FULL_THRESHOLD && pumpConfig.manualOverride) {
      pumpConfig.manualOverride = false;
      saveMotorStatus();
    }
  }

  if (tankConfig.firstReadingDone && tankConfig.rawUpperPercentage <= TankConfig::LOW_THRESHOLD && !sensorError && !isSystemExpired) {
    if (voltageConfig.status == 1 && dryRunConfig.error == 0 && !currentDndActive && !pumpConfig.manualOverride) {
      if (pumpConfig.motorStatus == 0) {
        pumpConfig.motorStatus = 1;
        dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET;
        saveMotorStatus();
      }
    }
  }

  if (voltAbnormal) currentState = PumpState::VOLTAGE_ERROR;
  else if (voltageConfig.status == 0) currentState = PumpState::VOLTAGE_WAIT;
  else if (sensorError) currentState = PumpState::SENSOR_ERROR;
  else if (dryRunConfig.error == 1) currentState = PumpState::DRY_RUN_ALARM;
  else if (dryRunConfig.error == 2) currentState = PumpState::DRY_RUN_LOCKED;
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
        dryRunConfig.retryCountdown = dryRunConfig.autoRetryMinutes * 60;
        dryRunConfig.lastRetryUpdate = currentMillis;
        publishState();
      }
      break;

    case PumpState::DRY_RUN_LOCKED:
      digitalWrite(MOTOR_PIN, LOW);
      pumpConfig.isRunning = false;
      digitalWrite(BUZZER_PIN, LOW);

      if (tankConfig.rawUpperPercentage >= tankConfig.FULL_THRESHOLD) {
        dryRunConfig.error = 0;             
        dryRunConfig.retryCountdown = 0;    
        dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET; 
        saveMotorStatus();                  
        publishState();                     
        break;                              
      }
      
      if (dryRunConfig.autoRetryMinutes > 0 && currentMillis - dryRunConfig.lastRetryUpdate >= 1000) {
        dryRunConfig.retryCountdown--;
        dryRunConfig.lastRetryUpdate = currentMillis;
        
        if (dryRunConfig.retryCountdown <= 0) {
          dryRunConfig.error = 0;
          dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET;
          
          bool safetyLock = (tankConfig.rawUpperPercentage >= tankConfig.FULL_THRESHOLD) || 
                            (tankConfig.upperInvalidCount >= TankConfig::MAX_INVALID_COUNT) || 
                            (voltageConfig.status == 0) || isSystemExpired;
          if (!safetyLock) {
            pumpConfig.motorStatus = 1;
            saveMotorStatus();
          }
          publishState();
        }
      }
      break;

    default:
      digitalWrite(MOTOR_PIN, LOW);
      pumpConfig.isRunning = false;
      if (sensorError && !tankConfig.errorAck) {
        digitalWrite(BUZZER_PIN, (currentMillis / 500) % 2);
      } else {
        digitalWrite(BUZZER_PIN, LOW);
      }
      break;
  }

  if (currentState != lastState || tankConfig.displayUpperPercentage != lastPercentage || voltageConfig.status != lastVoltStatus || pumpConfig.flowDetected != lastFlow || dryRunConfig.error != lastErr || pumpConfig.motorStatus != lastMotorStatus || tankConfig.errorAck != lastAck || currentDndActive != lastDndState) {  

    lastState = currentState;
    lastPercentage = tankConfig.displayUpperPercentage;
    lastVoltStatus = voltageConfig.status;
    lastFlow = pumpConfig.flowDetected;
    lastErr = dryRunConfig.error;
    lastMotorStatus = pumpConfig.motorStatus;
    lastAck = tankConfig.errorAck;
    lastRawVolt = (int)voltageConfig.currentVoltage;
    lastPushTime = currentMillis;
    lastDndState = currentDndActive;

    publishState();
  } else {
    int currentVolt = (int)voltageConfig.currentVoltage;
    if (abs(currentVolt - lastRawVolt) >= 2) {     
      if (currentMillis - lastPushTime >= 2000) {  
        lastRawVolt = currentVolt;
        lastPushTime = currentMillis;
        publishState();
      }
    }
  }
}

void custom0(int col, int r) { lcd.setCursor(col, r); lcd.write(2); lcd.write(8); lcd.write(1); lcd.setCursor(col, r + 1); lcd.write(2); lcd.write(6); lcd.write(1); }
void custom1(int col, int r) { lcd.setCursor(col, r); lcd.write(32); lcd.write(32); lcd.write(1); lcd.setCursor(col, r + 1); lcd.write(32); lcd.write(32); lcd.write(1); }
void custom2(int col, int r) { lcd.setCursor(col, r); lcd.write(5); lcd.write(3); lcd.write(1); lcd.setCursor(col, r + 1); lcd.write(2); lcd.write(6); lcd.write(6); }
void custom3(int col, int r) { lcd.setCursor(col, r); lcd.write(5); lcd.write(3); lcd.write(1); lcd.setCursor(col, r + 1); lcd.write(7); lcd.write(6); lcd.write(1); }
void custom4(int col, int r) { lcd.setCursor(col, r); lcd.write(2); lcd.write(6); lcd.write(1); lcd.setCursor(col, r + 1); lcd.write(32); lcd.write(32); lcd.write(1); }
void custom5(int col, int r) { lcd.setCursor(col, r); lcd.write(2); lcd.write(3); lcd.write(4); lcd.setCursor(col, r + 1); lcd.write(7); lcd.write(6); lcd.write(1); }
void custom6(int col, int r) { lcd.setCursor(col, r); lcd.write(2); lcd.write(3); lcd.write(4); lcd.setCursor(col, r + 1); lcd.write(2); lcd.write(6); lcd.write(1); }
void custom7(int col, int r) { lcd.setCursor(col, r); lcd.write(2); lcd.write(8); lcd.write(1); lcd.setCursor(col, r + 1); lcd.write(32); lcd.write(32); lcd.write(1); }
void custom8(int col, int r) { lcd.setCursor(col, r); lcd.write(2); lcd.write(3); lcd.write(1); lcd.setCursor(col, r + 1); lcd.write(2); lcd.write(6); lcd.write(1); }
void custom9(int col, int r) { lcd.setCursor(col, r); lcd.write(2); lcd.write(3); lcd.write(1); lcd.setCursor(col, r + 1); lcd.write(7); lcd.write(6); lcd.write(1); }

void printNumber(int value, int col, int r) {
  switch (value) {
    case 0: custom0(col, r); break;
    case 1: custom1(col, r); break;
    case 2: custom2(col, r); break;
    case 3: custom3(col, r); break;
    case 4: custom4(col, r); break;
    case 5: custom5(col, r); break;
    case 6: custom6(col, r); break;
    case 7: custom7(col, r); break;
    case 8: custom8(col, r); break;
    case 9: custom9(col, r); break;
  }
}

void updateLCD() {
  static unsigned long lastLCD = 0;
  if (millis() - lastLCD < 500) return;
  lastLCD = millis();

  int val = tankConfig.displayUpperPercentage;
  printNumber(val / 100, 0, 0);
  printNumber((val / 10) % 10, 3, 0);
  printNumber(val % 10, 6, 0);
  lcd.setCursor(9, 0); lcd.print("%");
  lcd.setCursor(9, 1); lcd.print(" ");

  bool voltAbnormal = (voltageConfig.currentVoltage > voltageConfig.HIGH_THRESHOLD || voltageConfig.currentVoltage < voltageConfig.LOW_THRESHOLD);
  bool isWait = (!voltAbnormal && voltageConfig.status == 0);
  
  int vVal = isWait ? voltageConfig.waitSeconds : (int)voltageConfig.currentVoltage;
  char vUnit = isWait ? 's' : 'V';
  printNumber(vVal / 100, 0, 2);
  printNumber((vVal / 10) % 10, 3, 2);
  printNumber(vVal % 10, 6, 2);
  lcd.setCursor(9, 2); lcd.print(vUnit);
  lcd.setCursor(9, 3); lcd.print(" ");
  
  lcd.setCursor(10, 0);
  if(isSystemExpired) {
      lcd.print(" EXPIRED  ");
  } else {
      lcd.print(pumpConfig.isRunning ? " PUMP ON  " : " PUMP OFF ");
  }

  String info;
  if (isSystemExpired) info = " CHECK LC ";
  else if (dryRunConfig.error == 1) info = " DRY ALRM ";
  else if (dryRunConfig.error == 2) {
    if (dryRunConfig.autoRetryMinutes == 0) {
      info = " LOCKED   ";
    } else {
      int m = (dryRunConfig.retryCountdown + 59) / 60; 
      char buf[11];
      snprintf(buf, sizeof(buf), " WAIT %02dM", m);
      info = String(buf);
    }
  }
  else if (tankConfig.upperInvalidCount >= TankConfig::MAX_INVALID_COUNT) info = " SNR ERR  ";
  else if (pumpConfig.isRunning) info = pumpConfig.flowDetected ? " FLOW OK  " : " FLOW CHK ";
  else info = " STANDBY  ";
  
  while(info.length() < 10) info += " ";
  lcd.setCursor(10, 1); lcd.print(info.substring(0, 10));

  // --- CLOUD STATUS DISPLAY ---
  lcd.setCursor(10, 2); 
  if (WiFi.status() != WL_CONNECTED) {
    lcd.print(" NO WIFI  ");   
  } else if (!mqttClient.connected()) {
    lcd.print(" WAITING  ");   
  } else {
    lcd.print(" ONLINE   ");   
  }

  lcd.setCursor(10, 3); 
  if (voltAbnormal) {
      if (voltageConfig.currentVoltage > voltageConfig.HIGH_THRESHOLD) lcd.print(" OVER     ");
      else lcd.print(" UNDER    ");
  } else if (isWait) {
      lcd.print(" DELAY    ");
  } else {
      lcd.print(" NORMAL   ");
  }
}

// --- MQTT & Web Callbacks ---
bool reconnectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return false;

  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 0) || timeinfo.tm_year < 120) {
    Serial.println("MQTT Blocked: System time invalid (1970). Forcing Sync...");
    waitForTimeSync(); 
    if (!getLocalTime(&timeinfo, 0) || timeinfo.tm_year < 120) {
       Serial.println("MQTT Failed: Time still invalid.");
       return false;
    }
  }

  String clientId = "Pump-" + getDeviceID();
  
  espClient.stop(); 

  bool isConnected = mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_pass, onlineTopic.c_str(), 1, true, "0");

  if (isConnected) {
    Serial.println("[Core 0] MQTT Connected!");
    mqttClient.publish(onlineTopic.c_str(), "1", true);
    mqttClient.subscribe(subTopic.c_str());
    publishState();
    return true;
  } else {
    Serial.print("[Core 0] MQTT Connect Failed, rc=");
    Serial.println(mqttClient.state());
  }
  
  return false;
}

void handleScan() {
  Serial.println("Manual Scan Requested...");
  WiFi.mode(WIFI_AP_STA);
  int nS = WiFi.scanNetworks(true);
  Serial.println(nS == -1 ? "Scan Triggered Successfully" : "Scan Trigger Failed");
  server.send(200, "text/plain", "OK");
}

void handleLogo() {
  server.send_P(200, "image/png", (const char*)logo_png, sizeof(logo_png));
}

void handleRoot() {
  if (WiFi.scanComplete() == -2) {
    WiFi.scanNetworks(true);
  }
  server.send(200, "text/html", index_html);
}

void handleSettings() {
  Serial.println("Settings Page Request...");
  
  String wifiList = "";
  wifiList.reserve(512);
  
  int n = WiFi.scanComplete();

  if (n == -1) {
    wifiList = "<option value='' disabled>Scanning in progress... Please wait.</option>";
  } else if (n == -2 || n == 0) {
    wifiList = "<option value='' disabled>No networks found. Try scanning again.</option>";
    WiFi.scanNetworks(true);
  } else {
    for (int i = 0; i < n; ++i) {
      String ssid = WiFi.SSID(i);
      String sel = (ssid == ssid_saved) ? " selected" : "";
      wifiList += "<option value='" + ssid + "'" + sel + ">" + ssid + " (" + String(WiFi.RSSI(i)) + "dBm)</option>";
    }
  }

  String tankList = "";
  tankList.reserve(512);
  for (float f = 1.0; f <= 7.01; f += 0.5) {
    float inches = f * 12.0;
    String sel = (abs(tankConfig.upperHeight - inches) < 0.1) ? " selected" : "";
    String lbl = (f == (int)f) ? String((int)f) : String(f, 1);
    tankList += "<option value='" + String(inches, 1) + "'" + sel + ">" + lbl + " ft</option>";
  }

  String startList = "";
  startList.reserve(512);
  String endList = "";
  endList.reserve(512);
  for (int i = 0; i < 24; i++) {
    String hourStr = (i < 10 ? "0" : "") + String(i) + ":00";
    String selS = (i == scheduleConfig.dndStart) ? " selected" : "";
    String selE = (i == scheduleConfig.dndEnd) ? " selected" : "";
    startList += "<option value='" + String(i) + "'" + selS + ">" + hourStr + "</option>";
    endList += "<option value='" + String(i) + "'" + selE + ">" + hourStr + "</option>";
  }

  String tzList = "";
  tzList.reserve(256);
  for (float f = -12.0; f <= 14.0; f += 0.5) {
    String sel = (abs(scheduleConfig.timezoneOffset - f) < 0.1) ? " selected" : "";
    String lbl = (f >= 0 ? "+" : "") + (f == (int)f ? String((int)f) : String(f, 1));
    tzList += "<option value='" + String(f, 1) + "'" + sel + ">GMT " + lbl + "</option>";
  }

  String vHList = "";
  vHList.reserve(256);
  for (int i = 230; i <= 260; i++) {
    String sel = (i == voltageConfig.HIGH_THRESHOLD) ? " selected" : "";
    vHList += "<option value='" + String(i) + "'" + sel + ">" + String(i) + " Volts</option>";
  }
  
  String vLList = "";
  vLList.reserve(256);
  for (int i = 150; i <= 190; i++) {
    String sel = (i == voltageConfig.LOW_THRESHOLD) ? " selected" : "";
    vLList += "<option value='" + String(i) + "'" + sel + ">" + String(i) + " Volts</option>";
  }
  
  String dryList = "";
  dryList.reserve(256);
  for (int i = 60; i <= 180; i += 5) {
    String sel = (i == dryRunConfig.WAIT_SECONDS_SET) ? " selected" : "";
    dryList += "<option value='" + String(i) + "'" + sel + ">" + String(i) + " Seconds</option>";
  }

  String s = settings_html;
  s.replace("%CUR_SSID%", ssid_saved == "" ? "None" : ssid_saved);
  s.replace("%WIFI_LIST%", wifiList);
  s.replace("%TANK_LIST%", tankList);
  s.replace("%VH_LIST%", vHList);
  s.replace("%VL_LIST%", vLList);
  s.replace("%DRY_LIST%", dryList);
  s.replace("%RM0%", dryRunConfig.autoRetryMinutes == 0 ? "selected" : "");
  s.replace("%RM30%", dryRunConfig.autoRetryMinutes == 30 ? "selected" : "");
  s.replace("%RM60%", dryRunConfig.autoRetryMinutes == 60 ? "selected" : "");
  s.replace("%PIN%", devicePin);
  s.replace("%DND_ON%", scheduleConfig.enabled ? "selected" : "");
  s.replace("%DND_OFF%", !scheduleConfig.enabled ? "selected" : "");
  s.replace("%START_LIST%", startList);
  s.replace("%END_LIST%", endList);
  s.replace("%TZ_LIST%", tzList);
  s.replace("%VER%", String(FIRMWARE_VERSION));
  s.replace("%DID%", getDeviceID()); // INJECT DEVICE ID
  server.send(200, "text/html", s);
}

void handleSave() {
  preferences.begin("pump-control", false);
  String ssid = "";
  if (server.hasArg("ssid_sel") && server.arg("ssid_sel") != "__man__") ssid = server.arg("ssid_sel");
  else if (server.hasArg("ssid_man")) ssid = server.arg("ssid_man");
  
  if (ssid != "") {
    ssid_saved = ssid;
    preferences.putString("ssid", ssid_saved);
  }
  
  if (server.hasArg("pass") && server.arg("pass") != "") {
    pass_saved = server.arg("pass");
    preferences.putString("pass", pass_saved);
  }
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
  if (server.hasArg("rM")) {
    dryRunConfig.autoRetryMinutes = server.arg("rM").toInt();
    preferences.putInt("retryMins", dryRunConfig.autoRetryMinutes);
  }
  if (server.hasArg("pin")) {
    devicePin = server.arg("pin");
    preferences.putString("pin", devicePin);
  }
  if (server.hasArg("dndEn")) {
    scheduleConfig.enabled = (server.arg("dndEn").toInt() == 1);
    preferences.putBool("dndEn", scheduleConfig.enabled);
  }
  if (server.hasArg("dndS")) {
    scheduleConfig.dndStart = server.arg("dndS").toInt();
    preferences.putInt("dndS", scheduleConfig.dndStart);
  }
  if (server.hasArg("dndE")) {
    scheduleConfig.dndEnd = server.arg("dndE").toInt();
    preferences.putInt("dndE", scheduleConfig.dndEnd);
  }
  if (server.hasArg("tzOf")) {
    scheduleConfig.timezoneOffset = server.arg("tzOf").toFloat();
    preferences.putFloat("tzOf", scheduleConfig.timezoneOffset);
  }
  preferences.end();

  String html = "<!DOCTYPE html><html><head><meta http-equiv=\"refresh\" content=\"15;url=/\" ><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"><title>Rebooting</title><style>body{background:#121212;color:white;font-family:sans-serif;text-align:center;margin-top:50px;}</style></head><body><h2>Settings Saved!</h2><p>Rebooting device. Please wait about 15 seconds...</p></body></html>";
  server.send(200, "text/html", html);
  rebootSystem();
}

void handleUpdatePage() {
  checkOTA();  

  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>OTA Update</title>";
  html += "<style>";
  html += "body{font-family:sans-serif; background:#121212; color:white; text-align:center; padding:40px 20px; margin:0;}";
  html += ".card{background:#1e1e1e; border-radius:12px; padding:30px 20px; max-width:400px; margin:auto; border:1px solid #333; box-shadow:0 10px 25px rgba(0,0,0,0.5);}";
  html += "h1{font-size:1.8rem; margin:0 0 15px 0; color:#03ef;}";
  html += "p{font-size:1.1rem; color:#ddd; margin-bottom:10px;}";
  html += ".info{color:#888; font-size:0.9rem; margin-top:25px; line-height:1.6; padding:15px; background:#121212; border-radius:8px; border:1px solid #333;}";
  html += ".btn{width:100%; padding:15px; background:#28a745; color:white; border:none; border-radius:8px; margin-top:20px; font-weight:bold; cursor:pointer; font-size:1.1rem;}";
  html += ".btn:active{transform:scale(0.98); opacity:0.9;}";
  html += ".back-link{display:inline-block; margin-top:25px; color:#aaa; text-decoration:none; font-size:1rem; padding:10px 20px; border:1px solid #333; border-radius:8px; background:#121212; transition:0.3s;}";
  html += ".back-link:hover{color:#fff; background:#2a2a2a;}";
  html += "</style></head><body>";

  html += "<div class='card'>";

  if (otaConfig.updateAvailable) {
    html += "<h1>Update Available!</h1>";
    html += "<p>A newer version (<strong>v" + String(otaConfig.remoteVersion) + "</strong>) is available.</p>";
    html += "<p style='font-size:0.95rem; color:#aaa;'>Current version: v" + String(FIRMWARE_VERSION) + "</p>";
    html += "<button class='btn' onclick=\"fetch('/start-ota').then(()=>document.body.innerHTML='<h2 style=\\'color:white;text-align:center;margin-top:50px;\\'>Updating...<br><span style=\\'font-size:1rem;color:#888;\\'>Please wait, device will reboot.</span></h2>');\">Start Update Now</button>";
  } else {
    html += "<h1 style='color:#28a745;'>No Updates</h1>";
    html += "<p>You are on the latest version (v" + String(FIRMWARE_VERSION) + ").</p>";
  }

  html += "<div class='info'>";
  html += "Remote Version read: " + String(otaConfig.remoteVersion) + "<br>";
  html += "Raw: " + String(otaConfig.remoteVersion) + "<br>";
  html += "Free Heap: " + String(ESP.getFreeHeap()) + " bytes<br>";
  html += "</div>";

  html += "<a href='/settings' class='back-link'>← Back to Settings</a>";
  html += "</div>";  
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleStatus() {
  server.send(200, "application/json", generateStatusJson());
}

void handleToggle() {
  String res = processManualToggle();

  if (res == "Silenced") {
    server.send(200, "application/json", "{\"status\":\"success\", \"msg\":\"Silenced\"}");
  } else if (res.startsWith("Blocked:")) {
    String reason = res.substring(8); 
    server.send(200, "application/json", "{\"status\":\"blocked\", \"reason\":\"" + reason + "\"}");
  } else {
    server.send(200, "application/json", "{\"status\":\"success\"}");
  }
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
  if (voltageConfig.currentVoltage > voltageConfig.HIGH_THRESHOLD) vS = "OVER";
  else if (voltageConfig.currentVoltage < voltageConfig.LOW_THRESHOLD) vS = "UNDER";
  else if (voltageConfig.status == 0) vS = "DELAY (" + String(voltageConfig.waitSeconds) + "s)";
  else vS = "NORMAL";

  String info;
  if (dryRunConfig.error == 1) info = "DRY_RUN_ALARM!";
  else if (dryRunConfig.error == 2) info = (dryRunConfig.autoRetryMinutes == 0) ? "PUMP_LOCKED!" : "WAITING_RETRY!";
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
  doc["rM"] = dryRunConfig.autoRetryMinutes;
  doc["rCd"] = dryRunConfig.retryCountdown;

  doc["id"] = deviceID;
  doc["ip"] = WiFi.localIP().toString();

  doc["uH"] = tankConfig.upperHeight;
  doc["vH"] = voltageConfig.HIGH_THRESHOLD;
  doc["vL"] = voltageConfig.LOW_THRESHOLD;
  doc["dD"] = dryRunConfig.WAIT_SECONDS_SET;
  doc["ssid"] = ssid_saved;
  doc["dndEn"] = scheduleConfig.enabled ? 1 : 0;
  doc["dndS"] = scheduleConfig.dndStart;
  doc["dndE"] = scheduleConfig.dndEnd;
  doc["tzOf"] = scheduleConfig.timezoneOffset;
  
  doc["dndAct"] = currentDndActive ? 1 : 0;

  doc["sErr"] = (tankConfig.upperInvalidCount >= TankConfig::MAX_INVALID_COUNT) ? 1 : 0;
  doc["ack"] = tankConfig.errorAck ? 1 : 0;
  doc["dnd"] = scheduleConfig.enabled ? 1 : 0;
  doc["ota"] = otaConfig.updateAvailable ? 1 : 0;
  doc["ver"] = FIRMWARE_VERSION;
  doc["nVer"] = otaConfig.newVersion;

  // --- INJECT LICENSE EXPIRY INTO JSON ---
  long daysLeft = 999;
  if (installDate > 0) {
    time_t now; time(&now);
    unsigned long nowEpoch = (unsigned long)now;
    unsigned long expiryDate = installDate + (validDays * 86400UL);
    if (nowEpoch < expiryDate) daysLeft = (expiryDate - nowEpoch + 86399) / 86400;
    else daysLeft = 0;
  }
  doc["expired"] = isSystemExpired ? 1 : 0;
  doc["daysLeft"] = daysLeft;
  // ---------------------------------------

  if (webAlertMsg != "") {
    if (millis() - webAlertTime < 4000) {
      doc["alertMsg"] = webAlertMsg;
    } else {
      webAlertMsg = ""; 
    }
  }

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

  // URGENT: Allow License Update even if expired
  if (doc.containsKey("lic")) {
    processLicenseTokenString(doc["lic"].as<String>());
    publishState();
    return;
  }

  if (doc.containsKey("toggle")) {

    if (!doc.containsKey("pin") || String(doc["pin"].as<const char*>()) != devicePin) {
      Serial.println("MQTT Sync: Wrong or Missing PIN");
      DynamicJsonDocument resp(256);
      resp["alert"] = "Access Denied";
      resp["reason"] = "Wrong or Missing Device PIN!";
      String respStr;
      serializeJson(resp, respStr);
      mqttClient.publish(statusTopic.c_str(), respStr.c_str());
      return;
    }

    String res = processManualToggle();

    if (res.startsWith("Blocked:")) {
      String reason = res.substring(8);
      Serial.println("MQTT Start Blocked: " + reason);
      DynamicJsonDocument resp(256);
      resp["alert"] = "Action Blocked";
      resp["reason"] = reason;
      String respStr;
      serializeJson(resp, respStr);
      mqttClient.publish(statusTopic.c_str(), respStr.c_str());
    }

  } else if (doc.containsKey("otaCheck")) {
    if (doc.containsKey("pin") && String(doc["pin"].as<const char*>()) == devicePin) {
      checkOTA();
      if (!otaConfig.updateAvailable) {
        DynamicJsonDocument resp(256);
        resp["alert"] = "System Up to Date";
        resp["reason"] = "You are already using the latest version: v" + String(FIRMWARE_VERSION);
        String respStr;
        serializeJson(resp, respStr);
        mqttClient.publish(statusTopic.c_str(), respStr.c_str());
      }
    }
  } else if (doc.containsKey("otaStart")) {
    if (doc.containsKey("pin") && String(doc["pin"].as<const char*>()) == devicePin) {
      startOTA();
    }
  } else if (doc.containsKey("reset")) {
    if (!doc.containsKey("pin") || String(doc["pin"].as<const char*>()) != devicePin) {
      return;  
    }
    dryRunConfig.error = 0;
    dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET;
    digitalWrite(BUZZER_PIN, LOW);
    saveMotorStatus();
    publishState();
  } else if (doc.containsKey("get")) {
    if (doc.containsKey("pin") && String(doc["pin"].as<const char*>()) == devicePin) {
      publishState();
    } else {
      Serial.println("MQTT Sync: Unauthorized Status Request");
      DynamicJsonDocument resp(256);
      resp["alert"] = "Access Denied";
      resp["reason"] = "Unauthorized Request - Wrong PIN!";
      String respStr;
      serializeJson(resp, respStr);
      mqttClient.publish(statusTopic.c_str(), respStr.c_str());
    }
  } else if (doc.containsKey("save")) {
    if (!doc.containsKey("pin") || String(doc["pin"].as<const char*>()) != devicePin) {
      return;
    }

    preferences.begin("pump-control", false);
    if (doc.containsKey("ssid") && doc["ssid"].as<String>() != "") {
      String newSsid = doc["ssid"].as<String>();
      preferences.putString("ssid", newSsid);
      Serial.println("MQTT Save: SSID updated (Length: " + String(newSsid.length()) + ")");
    }
    if (doc.containsKey("pass") && doc["pass"].as<String>() != "") {
      String newPass = doc["pass"].as<String>();
      preferences.putString("pass", newPass);
      Serial.println("MQTT Save: Password updated (Length: " + String(newPass.length()) + ")");
    }

    if (doc.containsKey("uH")) {
      float h = doc["uH"].as<float>();
      tankConfig.upperHeight = constrain(h, TankConfig::MIN_HEIGHT, TankConfig::MAX_HEIGHT);
      preferences.putFloat("upperH", tankConfig.upperHeight);
    }
    if (doc.containsKey("vH")) {
      voltageConfig.HIGH_THRESHOLD = doc["vH"].as<int>();
      preferences.putInt("vHigh", voltageConfig.HIGH_THRESHOLD);
    }
    if (doc.containsKey("vL")) {
      voltageConfig.LOW_THRESHOLD = doc["vL"].as<int>();
      preferences.putInt("vLow", voltageConfig.LOW_THRESHOLD);
    }
    if (doc.containsKey("dD")) {
      dryRunConfig.WAIT_SECONDS_SET = doc["dD"].as<int>();
      preferences.putInt("dryDelay", dryRunConfig.WAIT_SECONDS_SET);
    }
    if (doc.containsKey("rM")) {
      dryRunConfig.autoRetryMinutes = doc["rM"].as<int>();
      preferences.putInt("retryMins", dryRunConfig.autoRetryMinutes);
    }
    if (doc.containsKey("newPin")) {
      devicePin = doc["newPin"].as<String>();
      preferences.putString("pin", devicePin);
    }
    if (doc.containsKey("dndEn")) {
      scheduleConfig.enabled = (doc["dndEn"].as<int>() == 1);
      preferences.putBool("dndEn", scheduleConfig.enabled);
    }
    if (doc.containsKey("dndS")) {
      scheduleConfig.dndStart = doc["dndS"].as<int>();
      preferences.putInt("dndS", scheduleConfig.dndStart);
    }
    if (doc.containsKey("dndE")) {
      scheduleConfig.dndEnd = doc["dndE"].as<int>();
      preferences.putInt("dndE", scheduleConfig.dndEnd);
    }
    if (doc.containsKey("tzOf")) {
      scheduleConfig.timezoneOffset = doc["tzOf"].as<float>();
      preferences.putFloat("tzOf", scheduleConfig.timezoneOffset);
    }
    preferences.end();

    Serial.println("MQTT: Settings saved via Cloud.");
    rebootSystem();
  }
}

// --- OTA Implementation ---
void checkOTA() {
  if (WiFi.status() != WL_CONNECTED) return;

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(10000);
  HTTPClient http;

  String verUrl = String(FW_URL_BASE) + "version.txt";
  Serial.println("Checking for updates: " + verUrl);

  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setUserAgent("ESP32-Auto-Pump");

  if (http.begin(client, verUrl)) {
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();

      String cleanVer = "";
      for (int i = 0; i < payload.length(); i++) {
        if (isdigit(payload[i])) cleanVer += payload[i];
      }

      if (cleanVer.length() > 0) {
        int remoteVer = cleanVer.toInt();
        otaConfig.remoteVersion = remoteVer;  

        if (remoteVer > FIRMWARE_VERSION) {
          otaConfig.updateAvailable = true;
          otaConfig.newVersion = remoteVer;
          Serial.println("New update found: " + String(remoteVer));
        } else {
          otaConfig.updateAvailable = false;
          Serial.println("System is up to date.");
        }
      }
      publishState();
    } else {
      Serial.println("Update Check Failed, HTTP: " + String(httpCode));
    }
    http.end();
  }
}

void startOTA() {
  if (WiFi.status() != WL_CONNECTED || !otaConfig.updateAvailable) return;

  Serial.println("Starting OTA Update...");

  if (mqttClient.connected()) {
    mqttClient.disconnect();
  }

  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(15000);

  httpUpdate.setLedPin(RGB_LED_PIN, LOW);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("UPDATING SYSTEM");
  lcd.setCursor(0, 1);
  lcd.print("Do not turn off");

  String binUrl = String(FW_URL_BASE) + "firmware.bin";
  t_httpUpdate_return ret = httpUpdate.update(client, binUrl);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("UPDATE FAILED!");
      delay(5000);
      ESP.restart();
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      break;
    case HTTP_UPDATE_OK:
      Serial.println("HTTP_UPDATE_OK");
      break;
  }
}