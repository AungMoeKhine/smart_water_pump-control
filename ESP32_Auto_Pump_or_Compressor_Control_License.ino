/*
 * Automatic Pump Control System (ESP32-S3) - DUAL CORE FINAL EDITION
 * Features: LCD, Local Web, Cloud Control, Cool-Down
 * Added: Compressor Solenoid Valve Delay (Pre/Post Start Venting)
 */

// --- Required Libraries ---
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
#include <esp_task_wdt.h>

// ============================================================================
// CONFIGURATION & PINOUT
// ============================================================================

const int SDA_PIN = 1;
const int SCL_PIN = 2;
const int VOLTAGE_SENSOR_PIN = 4;
const int UPPER_TANK_TRIG_PIN = 5;
const int UPPER_TANK_ECHO_PIN = 6;
const int BUZZER_PIN = 7;
const int MOTOR_PIN = 8;
const int MANUAL_BTN_PIN = 9;
const int SOLENOID_PIN = 10;
const int FLOW_SENSOR_PIN = 18;
const int RGB_LED_PIN = 48;

LiquidCrystal_I2C lcd(0x27, 20, 4);

byte bar1[8] = { B11100, B11110, B11110, B11110, B11110, B11110, B11110, B11100 };
byte bar2[8] = { B00111, B01111, B01111, B01111, B01111, B01111, B01111, B00111 };
byte bar3[8] = { B11111, B11111, B00000, B00000, B00000, B00000, B11111, B11111 };
byte bar4[8] = { B11110, B11100, B00000, B00000, B00000, B00000, B11000, B11100 };
byte bar5[8] = { B01111, B00111, B00000, B00000, B00000, B00000, B00011, B01111 };
byte bar6[8] = { B00000, B00000, B00000, B00000, B00000, B00000, B11111, B11111 };
byte bar7[8] = { B00000, B00000, B00000, B00000, B00000, B00000, B00111, B01111 };
byte bar8[8] = { B11111, B11111, B00000, B00000, B00000, B00000, B00000, B00000 };

const char* mqtt_server = "210195b635414206adcd944325fe6f59.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char* mqtt_user = "my_switch";
const char* mqtt_pass = "My_password123";

const int FIRMWARE_VERSION = 1;
const char* FW_URL_BASE = "https://raw.githubusercontent.com/AungMoeKhine/smart_water_pump-control/main/";

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
  float currentVoltage = 0.0f;
  unsigned long lastCheck = 0;
};

struct TankConfig {
  static const int LOW_THRESHOLD = 20;
  int FULL_THRESHOLD = 80;
  static constexpr float MIN_HEIGHT = 12.0f;
  static constexpr float MAX_HEIGHT = 84.0f;
  static constexpr float BUFFER_HEIGHT = 9.0f;
  float upperHeight = MIN_HEIGHT;
  int rawUpperPercentage = 0;
  int displayUpperPercentage = 0;
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
  int motorStatus = 0;
  bool isRunning = false;
  bool flowDetected = false;
  bool manualOverride = false;
  bool wasRunningBeforeVoltageError = false;
  bool wasRunningBeforeCoolDown = false;
  unsigned long lastFlowTime = 0;
};

struct DryRunConfig {
  int WAIT_SECONDS_SET = 60;
  int waitSeconds = 60;
  int error = 0;
  unsigned long lastUpdate = 0;
  unsigned long alarmStartTime = 0;
  int autoRetryMinutes = 30;
  int retryCountdown = 0;
  unsigned long lastRetryUpdate = 0;
};

struct CoolDownConfig {
  int restMinutes = 0;
  unsigned long runStartTime = 0;
  unsigned long restStartTime = 0;
  bool isResting = false;
};

struct CompressorConfig {
  int opMode = 0;      // 0 = Water Pump, 1 = Air Compressor
  int valveDelay = 5;  // Starts from 5 Seconds
  bool isPreVenting = false;
  bool isPostVenting = false;
  unsigned long ventStartTime = 0;
  bool lastTargetStatus = false;
} compConfig;

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
  PRE_START_VALVE,
  PUMPING,
  POST_STOP_VALVE,
  DRY_RUN_ALARM,
  DRY_RUN_LOCKED,
  SENSOR_ERROR,
  VOLTAGE_ERROR,
  VOLTAGE_WAIT,
  COOLING_DOWN
};

// ============================================================================
// GLOBAL OBJECTS & PROTOTYPES
// ============================================================================

SemaphoreHandle_t systemMutex;
volatile bool pendingMqttPublish = false;

unsigned long installDate = 0;
unsigned long lastTokenTime = 0;
int validDays = 10;
bool isSystemExpired = false;
String uploadedLicenseToken = "";

VoltageConfig voltageConfig;
TankConfig tankConfig;
PumpConfig pumpConfig;
DryRunConfig dryRunConfig;
CoolDownConfig coolDownConfig;
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
int sysLang = 0;
String webAlertMsg = "";
unsigned long webAlertTime = 0;

WiFiUDP dnsUdp;
const byte DNS_PORT = 53;

TaskHandle_t NetworkTaskHandle;

String ssid_saved = "";
String pass_saved = "";

// Prototypes
void checkExpiry();
void networkTask(void* parameter);
void publishState();
void updatePumpLogic();
void saveMotorStatus();
void checkOTA();
void startOTA();
void handleUpdatePage();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void handleRoot();
void handleSettings();
void handleConfig();
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
void handleAdmin();
void handleApplyLicenseText();
void handleLicenseUpload();
void handleLicenseUploadData();

// --- Time Sync Helper (NON-BLOCKING) ---
void requestTimeSync() {
  Serial.println("[NTP] Requesting Time Sync in background...");
  configTime(scheduleConfig.timezoneOffset * 3600, 0, "pool.ntp.org", "time.google.com", "time.nist.gov");
}

void checkAndSaveInstallDate() {
  if (installDate != 0) return;  // Already saved

  time_t now = time(nullptr);
  if (now > 1600000000) {  // If time is successfully synced
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      Serial.print("Time Synced: ");
      Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

      if (xSemaphoreTakeRecursive(systemMutex, portMAX_DELAY)) {
        installDate = (unsigned long)now;
        preferences.begin("pump-control", false);
        preferences.putULong("installDate", installDate);
        preferences.end();
        xSemaphoreGiveRecursive(systemMutex);
        checkExpiry();
      }
    }
  }
}

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

// --- Web Interface HTML ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Smart Pump Dashboard</title>
<style>
  body{font-family:sans-serif;background:#121212;color:white;text-align:center;padding:20px;margin:0;}
  .logo { width: 80px; height: auto; margin-bottom: 10px; border-radius: 50%; border: 2px solid #333; }
  .card{background:#1e1e1e;border-radius:12px;padding:20px;max-width:400px;margin:auto;box-shadow:0 4px 15px rgba(0,0,0,0.5);border:1px solid #333;position:relative;}
  .tabs { display: flex; max-width: 440px; margin: 0 auto 15px auto; gap: 10px; }
  .tab { flex: 1; padding: 12px; text-decoration: none; border-radius: 12px; font-weight: bold; font-size: 1.05rem; transition: 0.3s; border: 1px solid transparent; }
  .tab-active { background: #1e1e1e; color: #fff; border: 1px solid #333; box-shadow: 0 4px 15px rgba(0,0,0,0.5); }
  .tab-inactive { background: #121212; color: #888; }
  .tab-inactive:hover { background: #1a1a1a; color: #ccc; }
  .conn-dot{width:10px;height:10px;background:#28a745;border-radius:50%;display:inline-block;margin-right:5px;}
  .off{background:#dc3545;}
  .refresh-btn{position:absolute;top:15px;right:15px;background:none;border:none;color:#03ef;font-size:1.2rem;cursor:pointer;padding:5px;}
  .row{display:flex;justify-content:space-between;font-size:1.1rem;margin:12px 0;border-bottom:1px solid #333;padding-bottom:10px;}
  .btn{width:100%;padding:15px;color:white;background:#03ef;border:none;border-radius:8px;margin-top:10px;font-weight:bold;cursor:pointer;font-size:1.1rem; transition: background 0.3s;}
  .btn:active, .refresh-btn:active{transform:scale(0.96);opacity:0.85;}
  .btn-green{background:#28a745 !important;}
  .btn-red{background:#dc3545 !important;}
  .btn-grey{background:#555 !important;color:#aaa !important;cursor:not-allowed;}
  .lang-mm .row { font-size: 1rem; }
  .lang-mm .row span:first-child { flex: 2; text-align: left; white-space: nowrap; }
  .lang-mm .row span:last-child { flex: 1; text-align: right; }
  @media screen and (max-width: 440px) { .lang-mm .row { font-size: 0.88rem; letter-spacing: -0.2px; } .lang-mm .tab { font-size: 0.85rem; padding: 10px 5px; } }
  @keyframes spin { 100% { transform: rotate(360deg); } }
  .spinning { animation: spin 1s linear infinite; opacity: 1 !important; color: #03ef62 !important; }
  
  .tank-wrap { width: 140px; height: 180px; border: 4px solid #333; border-radius: 15px; margin: 30px auto; position: relative; background: linear-gradient(90deg, #1a1a1a 0%, #333 50%, #1a1a1a 100%); overflow: visible; }
  .tank-wrap::before { content: ''; position: absolute; top: -14px; left: 50%; transform: translateX(-50%); width: 80px; height: 14px; background: linear-gradient(90deg, #1a1a1a 0%, #333 50%, #1a1a1a 100%); border-left: 4px solid #333; border-right: 4px solid #333; z-index: 1; }
  .tank-wrap::after { content: ''; position: absolute; top: -24px; left: 50%; transform: translateX(-50%); width: 100px; height: 10px; background: linear-gradient(90deg, #1a1a1a 0%, #333 50%, #1a1a1a 100%); border: 3px solid #333; border-radius: 4px; z-index: 5; }
  .tank-inner { position: absolute; top: 0; left: 0; right: 0; bottom: 0; border-radius: 10px; overflow: hidden; z-index: 2; }
  .tank-ridges { position: absolute; top: 0; left: 0; width: 100%; height: 100%; pointer-events: none; z-index: 3; }
  .tank-ridges::after { content: ''; position: absolute; left: -4px; right: -4px; top: 25%; height: 2px; background: rgba(0, 0, 0, 0.35); box-shadow: 0 45px 0 rgba(0, 0, 0, 0.35), 0 90px 0 rgba(0, 0, 0, 0.35); }
  .tank-fill { position: absolute; bottom: 0; left: 0; width: 100%; background: #039be5; transition: height 0.8s cubic-bezier(0.4, 0, 0.2, 1); height: 0%; overflow: visible; }
  .tank-fill::before, .tank-fill::after { content: ''; position: absolute; left: 0; width: 200%; height: 25px; opacity: 0; transition: opacity 0.5s; background-repeat: repeat-x; background-size: 50% 100%; top: -15px; }
  .tank-fill::before { background-image: url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 800 50'%3E%3Cpath d='M0,25 Q100,5 200,25 T400,25 T600,25 T800,25 v35 h-800 z' fill='%2387CEFA' opacity='0.7'/%3E%3C/svg%3E"); animation: wave 4s linear infinite; z-index: 1; }
  .tank-fill::after { background-image: url("data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 800 50'%3E%3Cpath d='M0,25 Q100,45 200,25 T400,25 T600,25 T800,25 v35 h-800 z' fill='%23039be5'/%3E%3C/svg%3E"); animation: wave 3s linear infinite reverse; z-index: 2; margin-top: 2px; }
  .tank-fill.pumping::before, .tank-fill.pumping::after { opacity: 1; }
  @keyframes wave { 0% { transform: translateX(0); } 100% { transform: translateX(-50%); } }
  .tank-glaze { position: absolute; top: 0; left: 0; width: 100%; height: 100%; z-index: 5; pointer-events: none; background: linear-gradient(90deg, transparent 0%, rgba(255,255,255,0.15) 50%, transparent 100%); border-radius: 10px; }
  .tank-text { position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); font-weight: bold; font-size: 1.6rem; color: #fff; z-index: 6; text-shadow: 2px 2px 4px rgba(0,0,0,0.8); }
  
  .modal { display: none; position: fixed; z-index: 1000; left: 0; top: 0; width: 100%; height: 100%; background-color: rgba(0,0,0,0.8); backdrop-filter: blur(4px); }
  .modal-content { background-color: #1e1e1e; position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); padding: 25px; border-left: 5px solid #ffc107; border-radius: 8px; width: 85%; max-width: 320px; box-shadow: 0 10px 30px rgba(0,0,0,0.8); text-align: left; box-sizing: border-box; }
  .modal-title { color: #ffc107; font-size: 1.3rem; margin: 0 0 10px 0; display: flex; align-items: center; gap: 8px; }
  .modal-text { color: #ddd; margin-bottom: 25px; font-size: 1.05rem; line-height: 1.5; }
  .modal-close { background: #333; color: white; padding: 12px; border: none; border-radius: 6px; cursor: pointer; font-weight: bold; width: 100%; font-size: 1.1rem; transition: background 0.2s; }
</style></head><body>
  <div id="warnModal" class="modal"><div class="modal-content"><h3 class="modal-title"><span>🔔</span> System Notice</h3><div id="warnText" class="modal-text">System processing...</div><button class="modal-close" onclick="closeModal()">Understood</button></div></div>
  
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
      <div id="dndBadge" style="display:none; font-size: 0.9rem; background: #6f42c1; color: white; padding: 4px 10px; border-radius: 12px; margin: 5px auto; width: fit-content;">🌙 DND Active</div>
    </h2>
    <div class="tank-wrap"><div class="tank-inner"><div class="tank-fill" id="tankFill"></div><div class="tank-glaze"></div></div><div class="tank-ridges"></div><div class="tank-text" id="tankVal">-- %</div></div>
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
  const showModal = (msg) => { document.getElementById('warnText').innerText = msg; document.getElementById('warnModal').style.display = 'block'; };
  const closeModal = () => { document.getElementById('warnModal').style.display = 'none'; };
  const togglePump = () => { fetch('/toggle').then(r=>r.json()).then(d=>{ if(d.status === 'blocked') showModal(d.reason); else upd(); }).catch(e=>{}); };
  const resetPump = () => { fetch('/reset').then(r=>r.json()).then(d=>{ if(d.status === 'blocked') showModal(d.reason); else upd(); }).catch(e=>{}); };
  const checkOTA = () => { window.location.href = '/update_github'; };
  const startOTA = () => { if(confirm('Are you sure you want to update? The system will reboot.')) fetch('/start-ota').then(()=>{ document.body.innerHTML='<h2 style="color:white;text-align:center;margin-top:50px;">Updating... Please wait.</h2>'; }); };
  const rfr = () => { let b=document.getElementById('rfb'); b.classList.add('spinning'); upd().finally(()=>b.classList.remove('spinning')); };
  
  let lastWebAlert = ""; 
  const dict = { "Smart Pump Dashboard": "ရေမော်တာ ထိန်းချုပ်စနစ်", "🏠 Home": "🏠 ပင်မစာမျက်နှာ", "⚙️ Settings": "⚙️ ဆက်တင်များ", "💧Tank Water Level": "💧ရေတိုင်ကီ ရေအမှတ်", "Voltage:": "လျှပ်စစ်အား-", "Volt Status:": "ဗို့အား အခြေအနေ-", "Pump:": "ရေမော်တာ-", "System Info:": "စက် အချက်အလက်-", "Cloud ID:": "ကလောက် အိုင်ဒီ-", "Device IP:": "စက် အိုင်ပီ-", "Update Now": "ယခု အဆင့်မြှင့်မည်", "System Notice": "စနစ် အသိပေးချက်", "System processing...": "စနစ် အလုပ်လုပ်နေပါသည်...", "Understood": "နားလည်ပါသည်" };
  const applyDict = () => { if(window.lSet) return; document.title = dict["Smart Pump Dashboard"]; const w = document.createTreeWalker(document.body, NodeFilter.SHOW_TEXT, null, false); let n; while(n = w.nextNode()){ let t=n.nodeValue.trim(); if(dict[t]) n.nodeValue = n.nodeValue.replace(t, dict[t]); } document.getElementById('expiryBanner').innerHTML="⚠️ စနစ် သက်တမ်းကုန်ဆုံးသွားပါပြီ ⚠️<br><span style='font-size:0.8rem; font-weight:normal'>ရေမော်တာ အသုံးပြုခွင့် ပိတ်ထားပါသည်။</span>"; document.getElementById('warnBanner').innerHTML="⚠️ သက်တမ်းကုန်ဆုံးရန် နီးကပ်နေပါပြီ ⚠️<br><span style='font-size:0.8rem; font-weight:normal' id='warnMsg'></span>"; window.lSet = true; };
  const upd = () => { return fetch('/status').then(r=>r.json()).then(d=>{
    if(d.lang == 1) { document.body.classList.add('lang-mm'); applyDict(); } else document.body.classList.remove('lang-mm');
      document.getElementById('dot').className='conn-dot'; document.getElementById('cStat').innerText=d.lang==1?'စက် အွန်လိုင်း':'Device: Online';
      document.getElementById('dip').innerText = d.ip; document.getElementById('tankFill').style.height = d.tank + '%';
      
      let vS = d.vStat; let iF = d.info; let tS = d.tStr; let pS = d.pStat;
      if (d.lang==1) {
        if(vS=="NORMAL") vS="ပုံမှန်"; else if(vS=="OVER") vS="ကျော်လွန်"; else if(vS=="UNDER") vS="လျော့နည်း"; else if(vS=="DELAY") vS="စောင့်ဆိုင်း";
        if(iF=="DRY_RUN_ALARM!") iF="ရေမရှိ အချက်ပေး!"; else if(iF=="PUMP_LOCKED!") iF="ပိတ်သိမ်းထားသည်!"; else if(iF=="WAITING_RETRY!") iF="ပြန်စရန်စောင့်နေသည်!"; else if(iF=="SENSOR_ERROR!") iF="ဆင်ဆာ ချို့ယွင်းချက်!"; else if(iF=="SYSTEM_STANDBY!") iF="အသင့်အနေအထား!"; else if(iF=="FLOW_DETECTED!") iF="ရေစီးဆင်းမှုရှိသည်!"; else if(iF=="FLOW_CHECKING!") iF="ရေစီးဆင်းမှုစစ်နေ!"; else if(iF=="COOLING_DOWN!") iF="အအေးခံနေသည်!"; else if(iF=="VENTING_VALVE!") iF="လေလျှော့နေသည်!";
        if(tS=="FULL") tS="ပြည့်"; else if(tS=="LOW") tS="နည်း"; else if(tS=="ERR") tS="ချို့";
        if(pS=="ON") pS="ဖွင့်"; else if(pS=="OFF") pS="ပိတ်"; else if(pS=="PUMPING") pS="ရေတင်နေသည်"; else if(pS=="STANDBY") pS="အသင့်အနေအထား"; else if(pS=="DRY ALRM") pS="ရေမရှိ အချက်ပေး"; else if(pS=="LOCKED") pS="ပိတ်သိမ်းထားသည်";
      }
      document.getElementById('tankVal').innerText = tS; if (document.getElementById('cid')) document.getElementById('cid').innerText = d.id;
      document.getElementById('volt').innerText=d.volt+' V'; document.getElementById('vstat').innerText=vS;
      document.getElementById('state').innerText=pS; document.getElementById('info').innerText=iF;
      document.getElementById('dndBadge').style.display = d.dndAct ? 'block' : 'none';
      let btn=document.getElementById('btnToggle'); let rst=document.getElementById('btnReset');
      
      if(d.err){
         rst.style.display='block'; btn.style.display='none';
         rst.innerText = d.lang==1 ? 'အချက်ပေး ပြန်ပိတ်မည်' : 'Reset Alarm';
      } else if(d.sErr && !d.ack){
         btn.style.display='block';rst.style.display='none';
         btn.innerText=d.lang==1?'အချက်ပေး ပြန်ပိတ်မည်':'Reset Alarm'; btn.className='btn btn-red';
      } else if(iF == "VENTING_VALVE!" || iF == "လေလျှော့နေသည်!") {
         btn.style.display='block'; rst.style.display='none';
         btn.innerText = d.lang==1 ? "စောင့်ဆိုင်းပါ..." : "PLEASE WAIT...";
         btn.className = 'btn btn-grey';
         btn.disabled = true;
      } else {
         rst.style.display='none'; btn.style.display='block'; 
         btn.disabled = false;
         if(d.lang==1) {
            btn.innerText = (d.pStat == "ON") ? 'ရေမော်တာ ပိတ်မည်' : 'ရေမော်တာ ဖွင့်မည်';
         } else {
            btn.innerText = (d.pStat == "ON") ? 'Stop the Pump' : 'Start the Pump';
         }
         // This line applies the Red class when ON and Green class when OFF
         btn.className = (d.pStat == "ON") ? "btn btn-red" : "btn btn-green";
      }
      
      let tf = document.getElementById('tankFill');
      if (d.pStat == "ON") tf.classList.add('pumping'); else tf.classList.remove('pumping');
      
      let cd=document.getElementById('cdRow');
      // Case 1: Locked, waiting for Auto-Retry
      if(d.err == 2 && d.rM > 0) {
         cd.style.display='flex'; 
         cd.children[0].innerText = d.lang==1?'ပြန်လည်စတင်ရန်-':'Retry in:';
         let m = Math.floor(d.rCd / 60); let s = d.rCd % 60; 
         document.getElementById('cd').innerText = m + 'm ' + s + 's';
      } 
      // Case 2: Actively counting down for Dry Run (Only after 1-min grace)
      else if(d.isDR) {
         cd.style.display='flex'; 
         cd.children[0].innerText = d.lang==1?'ရေမရှိ အချက်ပေးရန်-':'Dry-Run in:';
         document.getElementById('cd').innerText = d.cd + 's';
      } 
      // Case 3: Cooling down (PUMP REST)
      else if(d.info=="COOLING_DOWN!" || d.info=="အအေးခံနေသည်!") {
         cd.style.display='flex'; 
         cd.children[0].innerText = d.lang==1?'အအေးခံရန် ကျန်ချိန်-':'Cooling in:';
         let m = Math.floor(d.rstCd / 60); let s = d.rstCd % 60; 
         document.getElementById('cd').innerText = m + 'm ' + s + 's';
      } 
      // Case 4: Everything is fine, hide the countdown row
      else { 
         cd.style.display='none'; 
      }

      let ota=document.getElementById('otaHub');
      if(d.ota){ ota.style.display='block'; document.getElementById('otaMsg').innerText = d.lang==1 ? 'ဗားရှင်းသစ် v' + d.nVer + ' ရနိုင်ပါပြီ!' : 'New Version v' + d.nVer + ' Available!'; } else { ota.style.display='none'; }

      document.getElementById('expiryBanner').style.display = 'none';
      document.getElementById('warnBanner').style.display = 'none';

      if(d.alertMsg && d.alertMsg !== lastWebAlert) { showModal(d.alertMsg); lastWebAlert = d.alertMsg; } else if (!d.alertMsg) { lastWebAlert = ""; }
    }).catch(e=>{ document.getElementById('dot').className='conn-dot off'; document.getElementById('cStat').innerText=(document.body.classList.contains('lang-mm') ? 'စက် အော့ဖ်လိုင်း (ချိတ်ဆက်နေသည်...)' : 'Device: Offline (Connecting...)'); }); }
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
  .lang-mm label { font-size: 0.85rem; }
  .lang-mm .lbl-wrap { flex-direction: column; align-items: flex-start; }
  @media screen and (max-width: 440px) { .lang-mm .tab { font-size: 0.85rem; padding: 12px 5px; } }
</style></head><body>

  <div id="expiryBanner" style="display:none; background:#dc3545; color:white; padding:12px; border-radius:12px; margin-bottom:15px; font-weight:bold; max-width:440px; margin:0 auto 15px auto;">
      ⚠️ SYSTEM EXPIRED ⚠️<br><span style="font-size:0.8rem; font-weight:normal">Pump operations are disabled.</span>
  </div>
  <div id="warnBanner" style="display:none; background:#ffc107; color:#333; padding:12px; border-radius:12px; margin-bottom:15px; font-weight:bold; max-width:440px; margin:0 auto 15px auto;">
      ⚠️ SUBSCRIPTION ENDING SOON ⚠️<br><span style="font-size:0.8rem; font-weight:normal" id="warnMsg"></span>
  </div>

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
      <select id="ss" name="ssid_sel" onchange="chSS(this)"><option value="">Loading...</option></select>
      <input type="text" id="mi" name="ssid_man" placeholder="Type Network Name" style="display:none; margin-top:10px;">
      
      <div class="lbl-wrap"><label>WiFi Password</label></div>
      <div class="pass-row"><input type="password" id="p" name="pass" placeholder="Leave empty to keep current"><label class="show-pass"><input type="checkbox" onclick="togglePass('p')"> Show WiFi Password</label></div>
      <div class="lbl-wrap"><label>Device PIN (for Cloud)</label></div>
      <div class="pass-row"><input type="password" id="pin" name="pin" required><label class="show-pass"><input type="checkbox" onclick="togglePass('pin')"> Show PIN</label></div>
      <hr>
      <div class="lbl-wrap"><label>Interface Language</label></div>
      <select name="sysLang" id="sysLang">
         <option value="0">English</option>
         <option value="1">Myanmar (မြန်မာ)</option>
      </select>
      <hr>

      <div class="lbl-wrap"><label>Operating Mode</label></div>
      <select name="opM" id="opM" onchange="toggleVDly()">
         <option value="0">Water Pump</option>
         <option value="1">Air Compressor</option>
      </select>
      <div class="lbl-wrap" id="vWrap" style="display:none;"><label>Compressor Valve Delay</label><span class="range">5 - 15s</span></div>
      <select name="vDly" id="vDly" style="display:none;"></select>
      <hr>

      <div class="lbl-wrap"><label>Tank Height</label><span class="range">1.0 - 7.0 ft</span></div><select name="uH" id="uH_s"></select>
      <div class="lbl-wrap"><label>Over Voltage Set</label><span class="range">230 - 260 V</span></div><select name="vH" id="vH_s"></select>
      <div class="lbl-wrap"><label>Under Voltage Set</label><span class="range">150 - 190 V</span></div><select name="vL" id="vL_s"></select>
      <div class="lbl-wrap"><label>Dry-Run Delay</label><span class="range">60 - 180 s</span></div><select name="dD" id="dD_s"></select>
      
      <div class="lbl-wrap"><label>Pump Cool-down (After 1Hr)</label><span class="range">Disable / 5 - 15m</span></div>
      <select name="rstM" id="rstM_s"></select>
      <div class="lbl-wrap"><label>Auto-Retry Wait</label><span class="range">Disable / 30 / 60</span></div>
      <select name="rM" id="rM_s"></select>
      <hr>

      <div class="lbl-wrap"><label>🌙 Smart Scheduling (DND)</label></div>
      <select name="dndEn" id="dndEn_s"><option value="0">Disabled</option><option value="1">Enabled</option></select>
      <div style="display:flex; gap:10px; margin-top:10px;">
        <div style="flex:1;"><label>Start Hour</label><select name="dndS" id="dndS_s"></select></div>
        <div style="flex:1;"><label>End Hour</label><select name="dndE" id="dndE_s"></select></div>
      </div>
      <div class="lbl-wrap"><label>📍 Home Time Zone (GMT)</label></div>
      <select name="tzOf" id="tz_s"></select>
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
      <p style="font-size:0.8rem; color:#555; margin-top:10px;">Current Version: <span id="curVer">loading...</span></p>
    </div>
    
    <hr>
    <div style="text-align:center;">
      <h3 style="color:#28a745; margin:0 0 15px 0;">🔑 License Management</h3>
      <div style="background:#2a2a2a; padding:10px; border-radius:8px; font-size:0.85rem; margin-bottom:15px; word-break:break-all;">
          <strong>Device ID:</strong> <span id="did" style="color:#03ef;">loading...</span>
      </div>
      
      <form method='POST' action='/upload_license' enctype='multipart/form-data'>
         <label style="display:inline-block; padding: 10px; background:#1e1e1e; border:1px solid #333; cursor:pointer; color: #aaa; border-radius:6px; margin-bottom:5px; width:100%; box-sizing:border-box; text-align:left;">
            <span style="background:#007bff; color:white; padding:5px 10px; border-radius:4px; margin-right:10px;">Choose File</span>
            <span id="fileName">No file chosen</span>
            <input type="file" name="license" accept=".key,.txt" style="display:none;" onchange="document.getElementById('fileName').innerText = this.files[0] ? this.files[0].name : (document.getElementById('sysLang').value == '1' ? 'ဖိုင်ရွေးချယ်ထားခြင်းမရှိပါ' : 'No file chosen')">
         </label>
         <button type='submit' class="btn" style="margin-top:0; margin-bottom:15px;">Upload Token File</button>
      </form>

      <div style="color:#666; margin-bottom:15px;">- OR -</div>

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
  function toggleVDly() { let m = document.getElementById('opM').value; document.getElementById('vWrap').style.display = (m=="1")?"flex":"none"; document.getElementById('vDly').style.display = (m=="1")?"block":"none"; }
  const togglePass = (id) => { let x = document.getElementById(id); x.type = x.type==="password"?"text":"password"; };
  const chSS = (s) => { let m = document.getElementById('mi'); if(s.value === '__man__') { m.style.display='block'; m.required=true; } else { m.style.display='none'; m.required=false; } };
  const scn = () => { if(!confirm('Scan for WiFi networks?')) return; fetch('/scan').then(()=>alert('Scan started. This takes about 5-10 seconds. Refreshing list...')).then(()=>setTimeout(()=>location.reload(),6000)); };
  const checkOTA = () => { window.location.href = '/update_github'; };
  const startOTA = () => { if(confirm('Are you sure you want to update? The system will reboot.')) fetch('/start-ota').then(()=>{ document.body.innerHTML='<h2 style="color:white;text-align:center;margin-top:50px;">Updating...</h2>'; }); };
  const rfr = () => { let b=document.getElementById('rfb'); b.classList.add('spinning'); upd().finally(()=>b.classList.remove('spinning')); };

  const dict = { "⚙️ Settings": "⚙️ ဆက်တင်များ", "🏠 Home": "🏠 ပင်မစာမျက်နှာ", "WiFi SSID": "ဝိုင်ဖိုင် အမည်", "WiFi Password": "ဝိုင်ဖိုင် စကားဝှက်", "Device PIN (for Cloud)": "လုံခြုံရေး ပင်နံပါတ်", "Interface Language": "ဘာသာစကား", "Tank Height": "ရေတိုင်ကီ အမြင့်", "Over Voltage Set": "ဗို့အားလွန် သတ်မှတ်ချက်", "Under Voltage Set": "ဗို့အားလျော့ သတ်မှတ်ချက်", "Dry-Run Delay": "ရေမရှိ အချက်ပေးချိန်", "Operating Mode": "စက် အမျိုးအစား", "Compressor Valve Delay": "အဆို့ရှင် ဖွင့်ချိန်", "Water Pump": "ရေမော်တာ", "Air Compressor": "လေကွန်ပရက်ဆာ", "Pump Cool-down (After 1Hr)": "၁နာရီမောင်းပြီး အနားပေးချိန်", "Auto-Retry Wait": "ပြန်လည်စတင်ရန် စောင့်ချိန်", "🌙 Smart Scheduling (DND)": "ညဘက် အသံပိတ်စနစ် (DND)", "Start Hour": "စတင်ရန် အချိန်", "End Hour": "ပြီးဆုံးရန် အချိန်", "📍 Home Time Zone (GMT)": "အချိန်ဇုန် (GMT)", "Save & Reboot": "သိမ်းဆည်း၍ ပြန်ဖွင့်မည်", "🛠️ Maintenance": "🛠️ ပြုပြင်ထိန်းသိမ်းမှု", "Check for Updates": "ဗားရှင်း အသစ်စစ်ရန်", "🔑 License Management": "🔑 လိုင်စင် စီမံခန့်ခွဲမှု", "Upload Token File": "ဖိုင်ဖြင့် သက်တမ်းတိုးမည်", "Activate via Text": "စာသားဖြင့် သက်တမ်းတိုးမည်", "Leave empty to keep current": "မပြောင်းလိုပါက အလွတ်ထားပါ", "Show WiFi Password": "ဝိုင်ဖိုင် စကားဝှက် ပြမည်", "Show PIN": "ပင်နံပါတ် ပြမည်", "Scan for Networks": "ဝိုင်ဖိုင် ရှာမည်", "Type Network Name": "ဝိုင်ဖိုင် အမည် ရိုက်ထည့်ပါ", "Enter SSID Manually...": "ဝိုင်ဖိုင် အမည် ကိုယ်တိုင်ပေးမည်...", "Paste Token String (MTc...)": "ဖုန်းဖြင့်ရသော တိုကင်စာသားကို ဤနေရာတွင် ထည့်ပါ", "Choose File": "ဖိုင်ရွေးမည်", "No file chosen": "ဖိုင်ရွေးချယ်ထားခြင်းမရှိပါ", "Current Version:": "လက်ရှိ ဗားရှင်း-", "Device ID:": "စက် အမှတ်စဉ်-", "Cloud ID:": "ကလောက် အိုင်ဒီ-", "Device IP:": "စက် အိုင်ပီ-", "Update Now": "ယခု အဆင့်မြှင့်မည်" };
  
  window.onload = () => {
    fetch('/config').then(r=>r.json()).then(c=>{
       let so = `<option value="">-- Keep Current (${c.ssid==""?"None":c.ssid}) --</option>`;
       if(c.ws==-1) so+=`<option value="" disabled>Scanning in progress...</option>`;
       else if(c.ws==0||c.ws==-2) so+=`<option value="" disabled>No networks found.</option>`;
       else if(c.nets) { for(let n of c.nets) so+=`<option value="${n.s}" ${n.s==c.ssid?"selected":""}>${n.s} (${n.r}dBm)</option>`; }
       so+=`<option value="__man__">Enter SSID Manually...</option>`;
       document.getElementById('ss').innerHTML = so;
       
       document.getElementById('pin').value = c.pin; document.getElementById('sysLang').value = c.lang;
       document.getElementById('opM').value = c.opM;
       
       let vDs=""; for(let i=5; i<=15; i++) vDs+=`<option value="${i}" ${c.vDly==i?"selected":""}>${i} Seconds</option>`; document.getElementById('vDly').innerHTML = vDs;
       
       let tH=""; for(let f=1.0; f<=7.01; f+=0.5){ let inc=f*12.0; tH+=`<option value="${inc.toFixed(1)}" ${Math.abs(c.uH-inc)<0.1?"selected":""}>${f} ft</option>`;} document.getElementById('uH_s').innerHTML = tH;
       let vH=""; for(let i=230; i<=260; i++) vH+=`<option value="${i}" ${c.vH==i?"selected":""}>${i} Volts</option>`; document.getElementById('vH_s').innerHTML = vH;
       let vL=""; for(let i=150; i<=190; i++) vL+=`<option value="${i}" ${c.vL==i?"selected":""}>${i} Volts</option>`; document.getElementById('vL_s').innerHTML = vL;
       let dD=""; for(let i=60; i<=180; i+=5) dD+=`<option value="${i}" ${c.dD==i?"selected":""}>${i} Seconds</option>`; document.getElementById('dD_s').innerHTML = dD;
       let rM=""; for(let i of [0,5,10,15]) rM+=`<option value="${i}" ${c.rstM==i?"selected":""}>${i==0?"Disabled":i+" Minutes"}</option>`; document.getElementById('rstM_s').innerHTML = rM;
       let rmOps=""; for(let i of [0,30,60]) rmOps+=`<option value="${i}" ${c.rM==i?"selected":""}>${i==0?"Disabled":i+" Minutes"}</option>`; document.getElementById('rM_s').innerHTML = rmOps;
       
       document.getElementById('dndEn_s').value = c.dndEn;
       let h=""; for(let i=0; i<24; i++) h+=`<option value="${i}">${(i<10?'0':'')+i+":00"}</option>`;
       document.getElementById('dndS_s').innerHTML = h; document.getElementById('dndS_s').value = c.dndS;
       document.getElementById('dndE_s').innerHTML = h; document.getElementById('dndE_s').value = c.dndE;
       let tz=""; for(let f=-12.0; f<=14.0; f+=0.5) tz+=`<option value="${f}" ${Math.abs(c.tzOf-f)<0.1?"selected":""}>GMT ${f>=0?'+':''}${f}</option>`; document.getElementById('tz_s').innerHTML = tz;
       
       document.getElementById('curVer').innerText = c.ver; document.getElementById('did').innerText = c.did;
       toggleVDly();
       
       if(c.lang==1 && !window.lSet) {
          document.body.classList.add('lang-mm');
          const w = document.createTreeWalker(document.body, NodeFilter.SHOW_TEXT, null, false);
          let n; while(n = w.nextNode()){ let t=n.nodeValue.trim(); if(dict[t]) n.nodeValue = n.nodeValue.replace(t, dict[t]); }
          if(document.getElementById('ss').options[0]) document.getElementById('ss').options[0].text = `-- လက်ရှိ အတိုင်းထားမည် (${c.ssid===""?"None":c.ssid}) --`;
          window.lSet = true;
       }
    });
  };

  const upd = () => { return fetch('/status').then(r=>r.json()).then(d=>{
      document.getElementById('dot').className='conn-dot'; document.getElementById('cStat').innerText=d.lang==1?'စက် အွန်လိုင်း':'Device: Online';
      document.getElementById('dip').innerText = d.ip; if (document.getElementById('cid')) document.getElementById('cid').innerText = d.id;
      let ota=document.getElementById('otaHub');
      if(d.ota){ ota.style.display='block'; document.getElementById('otaMsg').innerText=d.lang==1?'ဗားရှင်းသစ် v'+d.nVer+' ရနိုင်ပါပြီ!':'New Version v'+d.nVer+' Available!'; } else { ota.style.display='none'; }
    }).catch(e=>{ document.getElementById('dot').className='conn-dot off'; document.getElementById('cStat').innerText=document.body.classList.contains('lang-mm')?'စက် အော့ဖ်လိုင်း (ချိတ်ဆက်နေသည်...)':'Device: Offline (Connecting...)'; }); 
  };
  setInterval(upd, 1000); upd();
</script></body></html>
)rawliteral";

// --- SENSOR IMPLEMENTATION ---
class NonBlockingUltrasonic {
private:
  int trigPin, echoPin;
  static const int NUM_SAMPLES = 20;
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
  float getDistance() {
    return lastMedian;
  }
  bool isBusy() {
    return collecting;
  }
};

NonBlockingUltrasonic upperSensor(UPPER_TANK_TRIG_PIN, UPPER_TANK_ECHO_PIN);

// --- Sensor Polling Logic ---
void monitorSensors() {
  static unsigned long lastScan = 0;
  unsigned long now = millis();

  if (now - lastScan >= ULTRASONIC_INTERVAL) {
    if (!upperSensor.isBusy()) upperSensor.start();
  }
  upperSensor.update();

  if (!upperSensor.isBusy() && now - lastScan >= ULTRASONIC_INTERVAL) {
    float dist = upperSensor.getDistance();
    if (xSemaphoreTakeRecursive(systemMutex, portMAX_DELAY)) {
      if (dist > 0) {
        tankConfig.upperInvalidCount = 0;
        tankConfig.errorAck = false;
        float diff = abs(dist - tankConfig.upperDistance);
        if (!tankConfig.firstReadingDone) {
          tankConfig.upperDistance = dist;
          tankConfig.firstReadingDone = true;
        } else if (diff > 0.3f) {
          tankConfig.upperDistance = (0.9f * dist) + (0.1f * tankConfig.upperDistance);
        }
        float effectiveHeight = tankConfig.upperHeight + TankConfig::BUFFER_HEIGHT;
        tankConfig.rawUpperPercentage = ((effectiveHeight - tankConfig.upperDistance) * 100) / effectiveHeight;
        tankConfig.rawUpperPercentage = constrain(tankConfig.rawUpperPercentage, 0, 100);
        tankConfig.displayUpperPercentage = map(tankConfig.rawUpperPercentage, 0, tankConfig.FULL_THRESHOLD, 0, 100);
        tankConfig.displayUpperPercentage = constrain(tankConfig.displayUpperPercentage, 0, 100);
      } else {
        tankConfig.upperInvalidCount++;
      }
      xSemaphoreGiveRecursive(systemMutex);
    }
    lastScan = now;
  }

  static unsigned long lastVoltSample = 0;
  if (now - lastVoltSample >= 500) {
    float v = voltageSensor.getRmsVoltage();
    if (isnan(v)) v = 0.0f;
    if (xSemaphoreTakeRecursive(systemMutex, portMAX_DELAY)) {
      if (v < 40.0f) {
        voltageConfig.currentVoltage = 0.0f;
      } else if (v < 350.0f) {
        if (voltageConfig.currentVoltage < 10.0f) voltageConfig.currentVoltage = v;
        else voltageConfig.currentVoltage = (0.2f * v) + (0.8f * voltageConfig.currentVoltage);
      }
      xSemaphoreGiveRecursive(systemMutex);
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

  systemMutex = xSemaphoreCreateRecursiveMutex();

  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  pinMode(MANUAL_BTN_PIN, INPUT_PULLUP);

  digitalWrite(MOTOR_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(SOLENOID_PIN, LOW);

  preferences.begin("pump-control", false);
  tankConfig.upperHeight = preferences.getFloat("upperH", TankConfig::MIN_HEIGHT);
  voltageConfig.HIGH_THRESHOLD = preferences.getInt("vHigh", 250);
  voltageConfig.LOW_THRESHOLD = preferences.getInt("vLow", 170);
  dryRunConfig.WAIT_SECONDS_SET = preferences.getInt("dryDelay", 60);
  dryRunConfig.autoRetryMinutes = preferences.getInt("retryMins", 30);
  coolDownConfig.restMinutes = preferences.getInt("restM", 0);
  compConfig.opMode = preferences.getInt("opM", 0);
  compConfig.valveDelay = preferences.getInt("vDly", 5);
  voltageConfig.status = 0;
  voltageConfig.waitSeconds = 10;
  devicePin = preferences.getString("pin", "123456");
  sysLang = preferences.getInt("sysLang", 0);
  ssid_saved = preferences.getString("ssid", "");
  pass_saved = preferences.getString("pass", "");
  pumpConfig.motorStatus = preferences.getInt("motor", 0);
  pumpConfig.manualOverride = preferences.getBool("override", false);
  pumpConfig.wasRunningBeforeVoltageError = preferences.getBool("wasRunV", false);

  scheduleConfig.enabled = preferences.getBool("dndEn", false);
  scheduleConfig.dndStart = preferences.getInt("dndS", 22);
  scheduleConfig.dndEnd = preferences.getInt("dndE", 6);
  scheduleConfig.timezoneOffset = preferences.getFloat("tzOf", 6.5);

  installDate = preferences.getULong("installDate", 0);
  validDays = preferences.getInt("validDays", 10);
  lastTokenTime = preferences.getULong("lastTokenTime", 0);
  preferences.end();

  tankConfig.updateFullThreshold();
  checkExpiry();

  rgbLed.begin();
  rgbLed.setBrightness(30);
  voltageSensor.setSensitivity(526.2500000000f);

  Serial.println("Running Initial Hardware Safety Check...");
  for (int i = 0; i < 60; i++) {
    monitorSensors();
    delay(50);
  }
  updatePumpLogic();

  Wire.begin(SDA_PIN, SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcd.createChar(1, bar1);
  lcd.createChar(2, bar2);
  lcd.createChar(3, bar3);
  lcd.createChar(4, bar4);
  lcd.createChar(5, bar5);
  lcd.createChar(6, bar6);
  lcd.createChar(7, bar7);
  lcd.createChar(8, bar8);

  lcd.setCursor(0, 0);
  lcd.print("********************");
  lcd.setCursor(0, 1);
  lcd.print("*  AUTOMATIC PUMP  *");
  lcd.setCursor(0, 2);
  lcd.print("*  CONTROL SYSTEM  *");
  lcd.setCursor(0, 3);
  lcd.print("********************");

  WiFi.mode(WIFI_AP_STA);
  WiFi.setAutoReconnect(true);  
  WiFi.setSleep(false);         
  
  if (ssid_saved == "") {
    // Brand new install! Force AP mode actively!
    Serial.println("No SSID saved. Starting AP mode immediately.");
    WiFi.softAP("Auto-Pump-Config", "12345678");
    dnsUdp.begin(DNS_PORT);
  }
  // Note: Initial scan removed to prevent the AP from crashing on the ESP32-S3

  if (ssid_saved != "") {
    Serial.print("Connecting to WiFi: " + ssid_saved);
    WiFi.begin(ssid_saved.c_str(), pass_saved.c_str());

    // Only wait a maximum of 5 seconds during boot.
    // If it fails, we move on instantly so the pump can operate offline!
    unsigned long startWait = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startWait < 5000) {
      delay(250);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nConnected! IP: " + WiFi.localIP().toString());
      requestTimeSync();  // Request time without blocking

      // Turn OFF AP Mode because we successfully connected to the router
      WiFi.mode(WIFI_STA);
      dnsUdp.stop();
    } else {
      Serial.println("\nOffline Boot. System running locally. Will reconnect in background.");
    }
  }

  deviceID = getDeviceID();
  subTopic = "smartpump/" + deviceID + "/set";
  statusTopic = "smartpump/" + deviceID + "/status";
  onlineTopic = "smartpump/" + deviceID + "/online";

  server.on("/", handleRoot);
  server.on("/settings", handleSettings);
  server.on("/config", handleConfig);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/status", handleStatus);
  server.on("/toggle", handleToggle);
  server.on("/reset", handleReset);
  server.on("/scan", handleScan);
  server.on("/logo.png", handleLogo);
  server.on("/update_github", handleUpdatePage);
  server.on("/check-ota", []() {
    checkOTA();
    server.send(200, "application/json", "{\"status\":\"checking\"}");
  });
  server.on("/start-ota", []() {
    server.send(200, "application/json", "{\"status\":\"starting\"}");
    startOTA();
  });
  server.on("/upload_license", HTTP_POST, handleLicenseUpload, handleLicenseUploadData);
  server.on("/apply_license", HTTP_POST, handleApplyLicenseText);
  server.on("/admin", handleAdmin);
  server.onNotFound([]() {
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/plain", "Redirect");
  });
  server.begin();

  if (MDNS.begin("smartpump")) MDNS.addService("http", "tcp", 80);

  espClient.setInsecure();
  espClient.setHandshakeTimeout(30);  // Increased to 30s
  espClient.setTimeout(15000);        // Increased to 15s
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(4096);
  mqttClient.setSocketTimeout(10);
  mqttClient.setKeepAlive(60);

  // --- NEW WATCHDOG CONFIG ---
  esp_task_wdt_deinit();  // Crucial: Remove default Arduino 5s WDT so our 30s config applies
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 30000,   // 30 seconds
    .idle_core_mask = 0,   // Core IDLE checks
    .trigger_panic = true  // Hard reboot on freeze
  };
  esp_task_wdt_init(&wdt_config);  // Pass the address of the config
  esp_task_wdt_add(NULL);          // Add the current task to WDT
  // ------------------------------------

  xTaskCreatePinnedToCore(networkTask, "NetworkTask", 12000, NULL, 1, &NetworkTaskHandle, 0);
}
void networkTask(void* parameter) {
  // --- WDT REMOVED FOR NETWORK TASK ---
  // We intentionally do NOT monitor this task with the Watchdog.
  // It is allowed to safely block for 30+ seconds during offline MQTT TLS
  // reconnects without crashing the main pump system.

  static unsigned long lastPublish = 0;
  static unsigned long lastWifiAttempt = 0;
  static unsigned long lastMqttAttempt = 0;
  static unsigned long noInternetTimer = millis();
  static unsigned long heartbeatTimer = 0;

  // FIX 1: Start as 'true' because setup() turns AP on by default
  static bool apModeActive = true;

  while (true) {
    bool hasWiFi = (WiFi.status() == WL_CONNECTED);
    bool hasCloud = mqttClient.connected();
    unsigned long now = millis();

    // --- 1. DYNAMIC AP MODE (TRUE INTERNET FALLBACK) ---
    if (hasWiFi && hasCloud) {
      noInternetTimer = now;  // Reset timer because internet is healthy!
      if (apModeActive) {
        Serial.println("[NET] Fully connected to Cloud! Hiding AP Mode.");
        dnsUdp.stop();
        WiFi.mode(WIFI_STA);  // Turns AP OFF, keeps local network connection
        apModeActive = false;
      }
    } else {
      // Start AP Mode if we lose connection OR if it's a brand new install (No SSID saved)
      if (!apModeActive && (ssid_saved == "" || !hasWiFi)) {
        Serial.println("[NET] No Router Connection! Starting AP Mode for local access/setup.");
        WiFi.mode(WIFI_AP_STA);
        WiFi.softAP("Auto-Pump-Config", "12345678");
        dnsUdp.begin(DNS_PORT);
        apModeActive = true;
        lastWifiAttempt = now;  // Reset timer so it doesn't instantly reconnect
      }
    }

    // --- 2. BACKGROUND RECONNECT (NO REBOOTS) ---
    if (!hasWiFi && ssid_saved != "") {
      int stations = WiFi.softAPgetStationNum();  // Check if someone is connected to the AP
      static int lastStations = 0;

      if (stations > 0 && lastStations == 0) {
        // Someone just connected! Give them a fresh 5 minutes without interruption.
        lastWifiAttempt = now;
        Serial.println("[WIFI] Client connected to AP mode. Pausing background reconnects for 5 mins.");
      }
      lastStations = stations;

      if (stations > 0) {
        // User is setting up WiFi. Pause reconnects for 5 mins so web page doesn't lag.
        if (now - lastWifiAttempt > 300000UL) {
          lastWifiAttempt = now;
          Serial.println("[WIFI] AP Setup Timeout. Checking old router...");
          WiFi.disconnect();                                   // <--- ADD THIS
          WiFi.begin(ssid_saved.c_str(), pass_saved.c_str());  // <--- CHANGE TO THIS
        }
      } else {
        // Nobody is on the AP. Aggressively try to reconnect to router every 60s.
        if (now - lastWifiAttempt > 60000UL) {
          lastWifiAttempt = now;
          Serial.println("[WIFI] Offline. Attempting background reconnect...");
          WiFi.disconnect();                                   // <--- ADD THIS
          WiFi.begin(ssid_saved.c_str(), pass_saved.c_str());  // <--- CHANGE TO THIS
        }
      }
    } else if (hasWiFi) {
      // FIX 2: Keep timer fresh while connected, preventing premature disconnects
      lastWifiAttempt = now;
    }

    // --- 3. MQTT & CLOUD MANAGEMENT ---
    if (hasWiFi) {
      if (!hasCloud) {
        if (now - lastMqttAttempt > 10000) {
          reconnectMQTT();  // Try to reach HiveMQ every 10 seconds
          lastMqttAttempt = now;
        }
      } else {
        mqttClient.loop();

        // HEARTBEAT: Force publish every 30s to keep mobile App connected
        if (now - heartbeatTimer > 30000) {
          pendingMqttPublish = true;
          heartbeatTimer = now;
        }

        // NON-BLOCKING MUTEX: Send data safely
        if (pendingMqttPublish && (now - lastPublish >= 500)) {
          if (xSemaphoreTakeRecursive(systemMutex, pdMS_TO_TICKS(100))) {
            publishState();
            pendingMqttPublish = false;
            xSemaphoreGiveRecursive(systemMutex);
            lastPublish = now;
          }
        }
      }
    }

    // GENTLE DELAY: Ensures Core 1 (Pump Logic) and Web Server never freeze
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void loop() {
  processDNS();
  monitorSensors();
  monitorButton();
  updatePumpLogic();
  server.handleClient();
  updateLCD();
  updateLEDStatus();
  delay(1);
  esp_task_wdt_reset();
}

String getStartBlockReason() {
  if (voltageConfig.currentVoltage > voltageConfig.HIGH_THRESHOLD) return "Voltage is OVER (" + String((int)voltageConfig.currentVoltage) + "V).";
  if (voltageConfig.currentVoltage < voltageConfig.LOW_THRESHOLD) return "Voltage is UNDER (" + String((int)voltageConfig.currentVoltage) + "V).";
  if (voltageConfig.status == 0) return "Voltage stabilization in progress. Please wait.";
  if (coolDownConfig.isResting) return "Pump is cooling down. Please wait.";
  if (tankConfig.displayUpperPercentage >= 100) return "The Tank is already FULL.";
  if (tankConfig.upperInvalidCount >= TankConfig::MAX_INVALID_COUNT) return "Sensor Error! Check the ultrasonic sensor.";

  // NEW: Block starting if the compressor valve is currently busy
  if (compConfig.opMode == 1) {
    if (compConfig.isPreVenting) return "Compressor is already starting (Pre-Venting)...";
    if (compConfig.isPostVenting) return "System is still venting pressure. Please wait.";
  }

  return "";
}

String processManualToggle() {
  String res = "Success";
  // Use a 500ms timeout for the lock to prevent the system from freezing
  if (xSemaphoreTakeRecursive(systemMutex, pdMS_TO_TICKS(500))) {

    // (LICENSE CHECK REMOVED FROM HERE - ONLY CLOUD IS BLOCKED NOW)

    // 1. SENSOR ERROR SILENCE (Highest Priority)
    // If the buzzer is beeping because of a sensor error, the first press stops the noise.
    bool sensorErrorActive = (tankConfig.upperInvalidCount >= TankConfig::MAX_INVALID_COUNT);
    if (sensorErrorActive && !tankConfig.errorAck) {
      tankConfig.errorAck = true;  // This silences the buzzer in updatePumpLogic
      Serial.println("[USER] Sensor Alarm Silenced by User.");
      xSemaphoreGiveRecursive(systemMutex);
      pendingMqttPublish = true;
      return "Silenced";
    }

    // 2. DRY-RUN ALARM RESET
    // If system is in Dry-Run Alarm or Lock, this clears it and stops the motor.
    if (dryRunConfig.error != 0) {
      dryRunConfig.error = 0;
      dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET;
      digitalWrite(BUZZER_PIN, LOW);
      pumpConfig.motorStatus = 0;
      pumpConfig.manualOverride = true;
      saveMotorStatus();
      Serial.println("[USER] Dry-Run Alarm Reset by User.");
      xSemaphoreGiveRecursive(systemMutex);
      pendingMqttPublish = true;
      return "Success";
    }

    // 3. VENTING LOCK (For Compressor Mode)
    // Block normal Start/Stop commands ONLY if we are currently venting
    if (compConfig.opMode == 1) {
      if (compConfig.isPreVenting) {
        xSemaphoreGiveRecursive(systemMutex);
        return "Blocked:Starting up... Please wait.";
      }
      if (compConfig.isPostVenting) {
        xSemaphoreGiveRecursive(systemMutex);
        return "Blocked:Stopping/Venting... Please wait.";
      }
    }

    // 4. NORMAL TOGGLE (Start or Stop)
    bool wantsToStart = (pumpConfig.motorStatus == 0);
    if (wantsToStart) {
      String blockReason = getStartBlockReason();
      if (blockReason != "") {
        xSemaphoreGiveRecursive(systemMutex);
        return "Blocked:" + blockReason;
      }
      pumpConfig.motorStatus = 1;
      pumpConfig.manualOverride = true;  // VIP Pass to ignore DND
      coolDownConfig.runStartTime = millis();
      dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET;
      Serial.println("[USER] Manual Start Command Accepted.");
    } else {
      pumpConfig.motorStatus = 0;
      pumpConfig.manualOverride = true;
      Serial.println("[USER] Manual Stop Command Accepted.");
    }

    saveMotorStatus();
    xSemaphoreGiveRecursive(systemMutex);
  }
  pendingMqttPublish = true;
  return res;
}

void monitorButton() {
  static int lastReading = HIGH;
  static unsigned long buttonDownTime = 0;
  static bool longPressHandled = false;
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
        // Button just went DOWN
        buttonDownTime = millis();
        longPressHandled = false;
      } else {
        // Button just went UP (Released)
        if (!longPressHandled && buttonDownTime > 0) {
          // It was a short press -> Toggle Pump
          String res = processManualToggle();
          if (res.startsWith("Blocked:")) {
            String reason = res.substring(8);
            setLedColor(255, 0, 0);
            if (mqttClient.connected()) {
              DynamicJsonDocument resp(256);
              resp["alert"] = "System Notice (Device Button)";
              resp["reason"] = reason;
              String respStr;
              serializeJson(resp, respStr);
              mqttClient.publish(statusTopic.c_str(), respStr.c_str());
            }
            webAlertMsg = "Device Button: " + reason;
            webAlertTime = millis();
          }
        }
        buttonDownTime = 0;
      }
    }

    // Check for Long Press (3 seconds) while button is still held DOWN
    if (buttonState == LOW && !longPressHandled && (millis() - buttonDownTime >= 3000)) {
      longPressHandled = true; // Prevent multiple triggers
      Serial.println("[USER] Button Held: Forcing AP Mode!");
      
      // Short Beep to tell the user they can let go!
      digitalWrite(BUZZER_PIN, HIGH);
      delay(200);
      digitalWrite(BUZZER_PIN, LOW);
      
      // Force AP Mode ON
      WiFi.mode(WIFI_AP_STA);
      WiFi.softAP("Auto-Pump-Config", "12345678");
      dnsUdp.begin(DNS_PORT);
      
      // Flash LED to visually indicate Setup Mode is active
      setLedColor(0, 0, 255);
      delay(300);
      setLedColor(255, 255, 255);
    }
  }
  lastReading = reading;
}

void rebootSystem() {
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
  preferences.putBool("wasRunV", pumpConfig.wasRunningBeforeVoltageError);
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
  PumpState tState;
  bool tRunning;
  if (xSemaphoreTakeRecursive(systemMutex, portMAX_DELAY)) {
    tState = currentState;
    tRunning = pumpConfig.isRunning;
    xSemaphoreGiveRecursive(systemMutex);
  }
  if (tState == PumpState::DRY_RUN_ALARM || tState == PumpState::DRY_RUN_LOCKED || tState == PumpState::SENSOR_ERROR || tState == PumpState::VOLTAGE_ERROR) {
    static bool flash = false;
    flash = !flash;
    setLedColor(flash ? 255 : 0, 0, 0);
  } else if (tState == PumpState::COOLING_DOWN) {
    setLedColor(0, 0, 255);
  } else if (tRunning) {
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

void checkExpiry() {
  if (installDate == 0) return;
  time_t now;
  time(&now);
  unsigned long nowEpoch = (unsigned long)now;
  unsigned long expiryDate = installDate + (validDays * 86400UL);
  if (xSemaphoreTakeRecursive(systemMutex, portMAX_DELAY)) {
    isSystemExpired = (nowEpoch > expiryDate);
    xSemaphoreGiveRecursive(systemMutex);
  }
}

bool verifyAndApplyLicense(String tokenBase64, String& outMsg) {
  tokenBase64.trim();
  if (tokenBase64.length() == 0) {
    outMsg = "Empty Token";
    return false;
  }
  unsigned char decoded[512];
  size_t outLen = 0;
  if (mbedtls_base64_decode(decoded, sizeof(decoded), &outLen, (const unsigned char*)tokenBase64.c_str(), tokenBase64.length()) != 0) {
    outMsg = "Invalid Base64";
    return false;
  }
  String raw = "";
  for (size_t i = 0; i < outLen; i++) raw += (char)decoded[i];
  int firstPipe = raw.indexOf('|');
  int lastPipe = raw.lastIndexOf('|');
  if (firstPipe == -1 || lastPipe == -1 || firstPipe == lastPipe) {
    outMsg = "Invalid Token Format";
    return false;
  }
  String tsStr = raw.substring(0, firstPipe);
  String daysStr = raw.substring(firstPipe + 1, lastPipe);
  String signature = raw.substring(lastPipe + 1);
  unsigned long tokenTs = strtoul(tsStr.c_str(), NULL, 10);
  int addDays = daysStr.toInt();
  if (tokenTs <= lastTokenTime) {
    outMsg = "Token Already Used";
    return false;
  }
  time_t now;
  time(&now);
  if ((unsigned long)now > tokenTs + 604800) {
    outMsg = "Token Expired (>7 days old)";
    return false;
  }
  String payload = tsStr + "|" + daysStr + "|ACER123|" + getDeviceID();
  MD5Builder md5;
  md5.begin();
  md5.add(payload);
  md5.calculate();
  if (!md5.toString().equalsIgnoreCase(signature)) {
    outMsg = "Invalid Signature";
    return false;
  }
  if (xSemaphoreTakeRecursive(systemMutex, portMAX_DELAY)) {
    validDays += addDays;
    lastTokenTime = tokenTs;
    xSemaphoreGiveRecursive(systemMutex);
  }
  preferences.begin("pump-control", false);
  preferences.putInt("validDays", validDays);
  preferences.putULong("lastTokenTime", lastTokenTime);
  preferences.end();
  checkExpiry();
  outMsg = "Success! Extended by " + String(addDays) + " days.";
  return true;
}

void processLicenseTokenString(String token) {
  String msg;
  bool success = verifyAndApplyLicense(token, msg);
  if (mqttClient.connected()) {
    DynamicJsonDocument doc(256);
    doc["alert"] = success ? "License Updated" : "License Failed";
    doc["reason"] = msg;
    String alertJson;
    serializeJson(doc, alertJson);
    mqttClient.publish(statusTopic.c_str(), alertJson.c_str());
  }
}

void sendLicenseResponse(int httpCode, String title, String color, String message) {
  String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'><style>body{font-family:sans-serif;text-align:center;padding:50px;background:#121212;color:white;}</style></head><body><h2 style='color:" + color + ";'>" + title + "</h2><p>" + message + "</p><br><a href='/settings' style='color:#03ef;text-decoration:none;font-weight:bold;padding:10px;border:1px solid #03ef;border-radius:8px;'>Back to Settings</a></body></html>";
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
  if (upload.status == UPLOAD_FILE_START) uploadedLicenseToken = "";
  else if (upload.status == UPLOAD_FILE_WRITE)
    for (size_t i = 0; i < upload.currentSize; i++) uploadedLicenseToken += (char)upload.buf[i];
}
void handleApplyLicenseText() {
  if (server.hasArg("key")) {
    String msg;
    if (verifyAndApplyLicense(server.arg("key"), msg)) sendLicenseResponse(200, "Activated!", "#28a745", msg);
    else sendLicenseResponse(403, "Activation Failed", "#dc3545", "Error: " + msg);
  } else {
    sendLicenseResponse(400, "Error", "#dc3545", "No token provided.");
  }
}
void handleAdmin() {
  if (!server.hasArg("secret") || server.arg("secret") != "ACER123") {
    server.send(403, "text/plain", "Forbidden");
    return;
  }
  preferences.begin("pump-control", false);
  if (server.hasArg("extend")) {
    if (xSemaphoreTakeRecursive(systemMutex, portMAX_DELAY)) {
      validDays += server.arg("extend").toInt();
      xSemaphoreGiveRecursive(systemMutex);
    }
    preferences.putInt("validDays", validDays);
    server.send(200, "text/plain", "Success! Total Valid Days: " + String(validDays));
  } else if (server.hasArg("reset")) {
    if (xSemaphoreTakeRecursive(systemMutex, portMAX_DELAY)) {
      installDate = 0;
      xSemaphoreGiveRecursive(systemMutex);
    }
    preferences.putULong("installDate", 0);
    server.send(200, "text/plain", "License Reset.");
  } else {
    server.send(200, "text/plain", "Admin Mode OK\nValid Days: " + String(validDays) + "\nExpired: " + String(isSystemExpired ? "YES" : "NO"));
  }
  preferences.end();
  checkExpiry();
}


void updatePumpLogic() {
  unsigned long currentMillis = millis();
  if (!xSemaphoreTakeRecursive(systemMutex, pdMS_TO_TICKS(50))) return;

  bool triggerPublish = false;
  static PumpState lastReportedState = (PumpState)-1;
  static int lastReportedMinute = -1;
  static int lastFlowStatus = -1;

  // --- 1. SENSORS & FLOW PERSISTENCE (1-Minute Slug Window) ---
  bool physicalFlow = (digitalRead(FLOW_SENSOR_PIN) == LOW);
  if (physicalFlow) pumpConfig.lastFlowTime = currentMillis;
  if (pumpConfig.motorStatus == 0) { pumpConfig.lastFlowTime = 0; }

  bool isInitialGrace = (pumpConfig.motorStatus == 1 && pumpConfig.lastFlowTime == 0 && (currentMillis - coolDownConfig.runStartTime < 60000UL));
  bool isInsideSlugWindow = (pumpConfig.lastFlowTime > 0 && (currentMillis - pumpConfig.lastFlowTime < 60000UL));
  bool effectiveFlow = (physicalFlow || isInitialGrace || isInsideSlugWindow);
  pumpConfig.flowDetected = effectiveFlow;

  bool sensorError = (tankConfig.upperInvalidCount >= TankConfig::MAX_INVALID_COUNT);
  bool voltAbnormal = (voltageConfig.currentVoltage > voltageConfig.HIGH_THRESHOLD || voltageConfig.currentVoltage < voltageConfig.LOW_THRESHOLD);

  // --- 2. CHECK DND STATUS ---
  currentDndActive = false;
  if (scheduleConfig.enabled) {
    struct tm timeinfo;
    // The ", 0" is CRITICAL. It tells the ESP32 to wait 0ms!
    // And we check if year >= 120 (2020) so it doesn't trigger on fake 1970 time.
    if (getLocalTime(&timeinfo, 0) && timeinfo.tm_year >= 120) {
      int hour = timeinfo.tm_hour;
      if (scheduleConfig.dndStart > scheduleConfig.dndEnd) {
        if (hour >= scheduleConfig.dndStart || hour < scheduleConfig.dndEnd) currentDndActive = true;
      } else {
        if (hour >= scheduleConfig.dndStart && hour < scheduleConfig.dndEnd) currentDndActive = true;
      }
    }
  }

  // --- 3. AUTO-FILL TRIGGER & SAFETY STOPS ---
  if (tankConfig.firstReadingDone && tankConfig.displayUpperPercentage <= TankConfig::LOW_THRESHOLD && pumpConfig.motorStatus == 0) {
    // isSystemExpired removed from auto-start conditions
    if (!sensorError && !coolDownConfig.isResting && voltageConfig.status == 1 && dryRunConfig.error == 0 && !currentDndActive) {
      pumpConfig.motorStatus = 1;
      pumpConfig.manualOverride = false;
      saveMotorStatus();
    }
  }
  if (pumpConfig.motorStatus == 1) {
    bool stopNow = false;
    // isSystemExpired removed from auto-stop conditions
    if (tankConfig.displayUpperPercentage >= 100 || sensorError) {
      stopNow = true;
      pumpConfig.manualOverride = false;
    }
    if (currentDndActive && !pumpConfig.manualOverride) stopNow = true;
    if (stopNow) {
      pumpConfig.motorStatus = 0;
      saveMotorStatus();
    }
  }

  // --- 4. TRANSITION & VENTING DETECTION ---
  bool targetRunning = (pumpConfig.motorStatus == 1);
  if (targetRunning && !compConfig.lastTargetStatus) {
    if (compConfig.opMode == 1 && compConfig.valveDelay > 0) {
      compConfig.isPreVenting = true;
      compConfig.isPostVenting = false;
      compConfig.ventStartTime = currentMillis;
    }
  } else if (!targetRunning && compConfig.lastTargetStatus) {
    if (compConfig.opMode == 1 && compConfig.valveDelay > 0) {
      compConfig.isPostVenting = true;
      compConfig.isPreVenting = false;
      compConfig.ventStartTime = currentMillis;
    }
  }
  compConfig.lastTargetStatus = targetRunning;

  if (compConfig.isPreVenting && (currentMillis - compConfig.ventStartTime >= (unsigned long)compConfig.valveDelay * 1000UL)) {
    compConfig.isPreVenting = false;
    triggerPublish = true;
  }
  if (compConfig.isPostVenting && (currentMillis - compConfig.ventStartTime >= (unsigned long)compConfig.valveDelay * 1000UL)) {
    compConfig.isPostVenting = false;
    triggerPublish = true;
  }

  // --- 5. COOL-DOWN & VOLTAGE TIMERS ---
  if (pumpConfig.motorStatus == 1 && !compConfig.isPreVenting && !compConfig.isPostVenting) {
    if (coolDownConfig.runStartTime == 0) coolDownConfig.runStartTime = currentMillis;
    if (coolDownConfig.restMinutes > 0 && (currentMillis - coolDownConfig.runStartTime >= 3600000UL)) {
      coolDownConfig.isResting = true;
      pumpConfig.wasRunningBeforeCoolDown = true;
      coolDownConfig.restStartTime = currentMillis;
      pumpConfig.motorStatus = 0;
      saveMotorStatus();
    }
  } else {
    coolDownConfig.runStartTime = 0;
  }

  if (coolDownConfig.isResting) {
    static unsigned long lastCoolTick = 0;
    if (currentMillis - lastCoolTick >= 1000) {
      lastCoolTick = currentMillis;
      triggerPublish = true;
    }
    if (currentMillis - coolDownConfig.restStartTime >= (unsigned long)coolDownConfig.restMinutes * 60000UL) {
      coolDownConfig.isResting = false;
      triggerPublish = true;
      if (pumpConfig.wasRunningBeforeCoolDown && voltageConfig.status == 1 && (!currentDndActive || pumpConfig.manualOverride)) {
        pumpConfig.motorStatus = 1;
        saveMotorStatus();
      }
      pumpConfig.wasRunningBeforeCoolDown = false;
    }
  }

  if (voltAbnormal) {
    if (voltageConfig.status == 1) {
      pumpConfig.wasRunningBeforeVoltageError = (pumpConfig.motorStatus == 1);
      voltageConfig.status = 0;
      pumpConfig.motorStatus = 0;
      saveMotorStatus();
    }
    voltageConfig.lastCheck = currentMillis;
  } else if (voltageConfig.status == 0) {
    if (currentMillis - voltageConfig.lastCheck >= 1000) {
      voltageConfig.waitSeconds--;
      voltageConfig.lastCheck = currentMillis;
      triggerPublish = true;
      if (voltageConfig.waitSeconds <= 0) {
        voltageConfig.status = 1;
        if (pumpConfig.wasRunningBeforeVoltageError && !sensorError && !coolDownConfig.isResting && (!currentDndActive || pumpConfig.manualOverride)) {
          pumpConfig.motorStatus = 1;
        }
        pumpConfig.wasRunningBeforeVoltageError = false;
        saveMotorStatus();
      }
    }
  }

  // --- 6. STATE DETERMINATION ---
  // PRIORITY 1: Always vent pressure first if stopping, regardless of errors!
  if (compConfig.isPostVenting) currentState = PumpState::POST_STOP_VALVE;

  // PRIORITY 2: Safety & Errors
  else if (voltAbnormal) currentState = PumpState::VOLTAGE_ERROR;
  else if (voltageConfig.status == 0) currentState = PumpState::VOLTAGE_WAIT;
  else if (sensorError) currentState = PumpState::SENSOR_ERROR;
  else if (coolDownConfig.isResting) currentState = PumpState::COOLING_DOWN;
  else if (dryRunConfig.error == 1) currentState = PumpState::DRY_RUN_ALARM;
  else if (dryRunConfig.error == 2) currentState = PumpState::DRY_RUN_LOCKED;

  // PRIORITY 3: Normal Operations
  else if (pumpConfig.motorStatus == 1) {
    if (compConfig.isPreVenting) currentState = PumpState::PRE_START_VALVE;
    else currentState = PumpState::PUMPING;
  } else currentState = PumpState::IDLE;

  // --- 7. HARDWARE EXECUTION ---
  switch (currentState) {
    case PumpState::PUMPING:
      digitalWrite(MOTOR_PIN, HIGH);
      digitalWrite(SOLENOID_PIN, LOW);
      pumpConfig.isRunning = true;
      if (effectiveFlow) {
        if (dryRunConfig.waitSeconds != dryRunConfig.WAIT_SECONDS_SET) {
          dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET;
          triggerPublish = true;
        }
      } else {
        if (currentMillis - dryRunConfig.lastUpdate >= 1000) {
          dryRunConfig.waitSeconds--;
          dryRunConfig.lastUpdate = currentMillis;
          triggerPublish = true;  // Update cloud with countdown
          if (dryRunConfig.waitSeconds <= 0) {
            dryRunConfig.error = 1;
            dryRunConfig.alarmStartTime = currentMillis;
            pumpConfig.motorStatus = 0;
            saveMotorStatus();
          }
        }
      }
      break;

    case PumpState::SENSOR_ERROR:
      digitalWrite(MOTOR_PIN, LOW);
      digitalWrite(SOLENOID_PIN, LOW);
      pumpConfig.isRunning = false;
      // Alarm sound only if not acknowledged
      if (!tankConfig.errorAck) digitalWrite(BUZZER_PIN, (currentMillis / 1000) % 2);
      else digitalWrite(BUZZER_PIN, LOW);
      break;

    case PumpState::DRY_RUN_ALARM:
      digitalWrite(MOTOR_PIN, LOW);
      digitalWrite(SOLENOID_PIN, LOW);
      pumpConfig.isRunning = false;
      digitalWrite(BUZZER_PIN, (currentMillis / 500) % 2);
      if (currentMillis - dryRunConfig.alarmStartTime >= 60000) {
        dryRunConfig.error = 2;
        dryRunConfig.retryCountdown = dryRunConfig.autoRetryMinutes * 60;
        dryRunConfig.lastRetryUpdate = currentMillis;
        digitalWrite(BUZZER_PIN, LOW);
      }
      break;

    case PumpState::DRY_RUN_LOCKED:
      digitalWrite(MOTOR_PIN, LOW);
      digitalWrite(SOLENOID_PIN, LOW);
      pumpConfig.isRunning = false;
      if (dryRunConfig.autoRetryMinutes > 0 && currentMillis - dryRunConfig.lastRetryUpdate >= 1000) {
        dryRunConfig.retryCountdown--;
        dryRunConfig.lastRetryUpdate = currentMillis;
        triggerPublish = true;
        if (dryRunConfig.retryCountdown <= 0) {
          dryRunConfig.error = 0;
          dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET;
          pumpConfig.motorStatus = 1;
          saveMotorStatus();
          triggerPublish = true;
        }
      }
      break;

    case PumpState::PRE_START_VALVE:
    case PumpState::POST_STOP_VALVE:
      digitalWrite(MOTOR_PIN, LOW);
      digitalWrite(SOLENOID_PIN, HIGH);
      pumpConfig.isRunning = false;
      digitalWrite(BUZZER_PIN, LOW);
      break;

    default:
      digitalWrite(MOTOR_PIN, LOW);
      digitalWrite(SOLENOID_PIN, LOW);
      pumpConfig.isRunning = false;
      digitalWrite(BUZZER_PIN, LOW);
      break;
  }

  if (currentState != lastReportedState) {
    lastReportedState = currentState;
    triggerPublish = true;
  }
  // NEW: Monitor for Voltage or Tank changes to trigger live updates
  static int lastRepVolt = -1;
  static int lastRepTank = -1;
  int curVInt = (int)voltageConfig.currentVoltage;
  if (curVInt != lastRepVolt || tankConfig.displayUpperPercentage != lastRepTank) {
    lastRepVolt = curVInt;
    lastRepTank = tankConfig.displayUpperPercentage;
    triggerPublish = true;
  }
  if (triggerPublish) pendingMqttPublish = true;

  xSemaphoreGiveRecursive(systemMutex);
}

// --- LCD Display Helpers ---
void custom0(int col, int r) {
  lcd.setCursor(col, r);
  lcd.write(2);
  lcd.write(8);
  lcd.write(1);
  lcd.setCursor(col, r + 1);
  lcd.write(2);
  lcd.write(6);
  lcd.write(1);
}
void custom1(int col, int r) {
  lcd.setCursor(col, r);
  lcd.write(32);
  lcd.write(32);
  lcd.write(1);
  lcd.setCursor(col, r + 1);
  lcd.write(32);
  lcd.write(32);
  lcd.write(1);
}
void custom2(int col, int r) {
  lcd.setCursor(col, r);
  lcd.write(5);
  lcd.write(3);
  lcd.write(1);
  lcd.setCursor(col, r + 1);
  lcd.write(2);
  lcd.write(6);
  lcd.write(6);
}
void custom3(int col, int r) {
  lcd.setCursor(col, r);
  lcd.write(5);
  lcd.write(3);
  lcd.write(1);
  lcd.setCursor(col, r + 1);
  lcd.write(7);
  lcd.write(6);
  lcd.write(1);
}
void custom4(int col, int r) {
  lcd.setCursor(col, r);
  lcd.write(2);
  lcd.write(6);
  lcd.write(1);
  lcd.setCursor(col, r + 1);
  lcd.write(32);
  lcd.write(32);
  lcd.write(1);
}
void custom5(int col, int r) {
  lcd.setCursor(col, r);
  lcd.write(2);
  lcd.write(3);
  lcd.write(4);
  lcd.setCursor(col, r + 1);
  lcd.write(7);
  lcd.write(6);
  lcd.write(1);
}
void custom6(int col, int r) {
  lcd.setCursor(col, r);
  lcd.write(2);
  lcd.write(3);
  lcd.write(4);
  lcd.setCursor(col, r + 1);
  lcd.write(2);
  lcd.write(6);
  lcd.write(1);
}
void custom7(int col, int r) {
  lcd.setCursor(col, r);
  lcd.write(2);
  lcd.write(8);
  lcd.write(1);
  lcd.setCursor(col, r + 1);
  lcd.write(32);
  lcd.write(32);
  lcd.write(1);
}
void custom8(int col, int r) {
  lcd.setCursor(col, r);
  lcd.write(2);
  lcd.write(3);
  lcd.write(1);
  lcd.setCursor(col, r + 1);
  lcd.write(2);
  lcd.write(6);
  lcd.write(1);
}
void custom9(int col, int r) {
  lcd.setCursor(col, r);
  lcd.write(2);
  lcd.write(3);
  lcd.write(1);
  lcd.setCursor(col, r + 1);
  lcd.write(7);
  lcd.write(6);
  lcd.write(1);
}
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

  int val = 0, vVal = 0;
  bool voltAbnormal = false, isWait = false, exp = false, pRunning = false, cDown = false;
  int errState = 0, autoRetMins = 0, retCd = 0, invCount = 0;
  int restMinsLeft = 0;
  bool physFlow = false, isVenting = false;  // Changed from flowDet to physFlow
  float curVolt = 0;

  if (xSemaphoreTakeRecursive(systemMutex, portMAX_DELAY)) {
    val = tankConfig.displayUpperPercentage;
    voltAbnormal = (voltageConfig.currentVoltage > voltageConfig.HIGH_THRESHOLD || voltageConfig.currentVoltage < voltageConfig.LOW_THRESHOLD);
    isWait = (!voltAbnormal && voltageConfig.status == 0);
    vVal = isWait ? voltageConfig.waitSeconds : (int)voltageConfig.currentVoltage;
    curVolt = voltageConfig.currentVoltage;
    exp = isSystemExpired;
    pRunning = pumpConfig.isRunning;
    errState = dryRunConfig.error;
    autoRetMins = dryRunConfig.autoRetryMinutes;
    retCd = dryRunConfig.retryCountdown;
    invCount = tankConfig.upperInvalidCount;
    cDown = coolDownConfig.isResting;
    isVenting = (currentState == PumpState::PRE_START_VALVE || currentState == PumpState::POST_STOP_VALVE);

    // Check the PHYSICAL state of the pin for the LCD
    physFlow = (digitalRead(FLOW_SENSOR_PIN) == LOW);

    if (cDown) restMinsLeft = (((coolDownConfig.restMinutes * 60000UL) - (millis() - coolDownConfig.restStartTime)) / 60000UL) + 1;
    xSemaphoreGiveRecursive(systemMutex);
  }

  // --- Big Number Drawing ---
  printNumber(val / 100, 0, 0);
  printNumber((val / 10) % 10, 3, 0);
  printNumber(val % 10, 6, 0);
  lcd.setCursor(9, 0);
  lcd.print("%");
  lcd.setCursor(9, 1);
  lcd.print(" ");

  char vUnit = isWait ? 's' : 'V';
  printNumber(vVal / 100, 0, 2);
  printNumber((vVal / 10) % 10, 3, 2);
  printNumber(vVal % 10, 6, 2);
  lcd.setCursor(9, 2);
  lcd.print(vUnit);
  lcd.setCursor(9, 3);
  lcd.print(" ");

  // --- Right Side Text Labels ---
  lcd.setCursor(10, 0);
  if (cDown) lcd.print(" PUMP REST");
  else lcd.print(pRunning ? " PUMP ON  " : " PUMP OFF ");

  String info;
  if (errState == 1) info = " DRY ALRM ";
  else if (errState == 2) {
    if (autoRetMins == 0) info = " LOCKED   ";
    else {
      char buf[11];
      snprintf(buf, sizeof(buf), " WAIT %02dM", (retCd + 59) / 60);
      info = String(buf);
    }
  } else if (invCount >= TankConfig::MAX_INVALID_COUNT) info = " SNR ERR  ";
  else if (isVenting) info = " VENTING  ";
  else if (cDown) {
    char buf[11];
    snprintf(buf, sizeof(buf), " WAIT %02dM", restMinsLeft);
    info = String(buf);
  } else if (pRunning) {
    // NEW: If motor is on, show OK only if water is physically hitting the sensor
    // Otherwise show CHK (priming or air gaps)
    info = physFlow ? " FLOW OK  " : " FLOW CHK ";
  } else info = " STANDBY  ";

  while (info.length() < 10) info += " ";
  lcd.setCursor(10, 1);
  lcd.print(info.substring(0, 10));

  lcd.setCursor(10, 2);
  if (WiFi.status() != WL_CONNECTED) lcd.print(" NO WIFI  ");
  else if (!mqttClient.connected()) lcd.print(" WAITING  ");
  else lcd.print(" ONLINE   ");

  lcd.setCursor(10, 3);
  if (voltAbnormal) lcd.print(curVolt > voltageConfig.HIGH_THRESHOLD ? " OVER     " : " UNDER    ");
  else if (isWait) lcd.print(" DELAY    ");
  else lcd.print(" NORMAL   ");
}

bool reconnectMQTT() {
  if (WiFi.status() != WL_CONNECTED) return false;

  struct tm timeinfo;
  // Non-blocking check: if year is < 120 (before 2020), internet time hasn't synced yet
  if (!getLocalTime(&timeinfo, 10) || timeinfo.tm_year < 120) {
    requestTimeSync();
    return false;  // Exit immediately, don't freeze the system!
  }

  // Time is valid, ensure install date is saved
  checkAndSaveInstallDate();

  espClient.stop();
  // Lower timeout to prevent freezing if router has no internet
  espClient.setHandshakeTimeout(10);
  espClient.setTimeout(10000);

  if (mqttClient.connect(("Pump-" + getDeviceID()).c_str(), mqtt_user, mqtt_pass, onlineTopic.c_str(), 1, true, "0")) {
    mqttClient.publish(onlineTopic.c_str(), "1", true);
    mqttClient.subscribe(subTopic.c_str());
    pendingMqttPublish = true;
    return true;
  }
  return false;
}

void handleScan() {
  if (WiFi.status() != WL_CONNECTED) WiFi.disconnect();
  WiFi.mode(WIFI_AP_STA);
  WiFi.scanNetworks(true);
  server.send(200, "text/plain", "OK");
}
void handleLogo() {
  server.send_P(200, "image/png", (const char*)logo_png, sizeof(logo_png));
}

void handleRoot() {
  if (WiFi.scanComplete() == -2) {
    if (WiFi.status() != WL_CONNECTED) WiFi.disconnect();
    WiFi.scanNetworks(true);
  }
  server.send_P(200, "text/html", index_html);
}

void handleSettings() {
  if (WiFi.scanComplete() == -2) {
    if (WiFi.status() != WL_CONNECTED) WiFi.disconnect();
    WiFi.scanNetworks(true);
  }
  server.send_P(200, "text/html", settings_html);
}

void handleConfig() {
  if (xSemaphoreTakeRecursive(systemMutex, pdMS_TO_TICKS(100))) {
    DynamicJsonDocument doc(2048);
    doc["ssid"] = ssid_saved;
    doc["uH"] = tankConfig.upperHeight;
    doc["vH"] = voltageConfig.HIGH_THRESHOLD;
    doc["vL"] = voltageConfig.LOW_THRESHOLD;
    doc["dD"] = dryRunConfig.WAIT_SECONDS_SET;
    doc["rstM"] = coolDownConfig.restMinutes;
    doc["rM"] = dryRunConfig.autoRetryMinutes;
    doc["opM"] = compConfig.opMode;
    doc["vDly"] = compConfig.valveDelay;
    doc["pin"] = devicePin;
    doc["lang"] = sysLang;
    doc["dndEn"] = scheduleConfig.enabled ? 1 : 0;
    doc["dndS"] = scheduleConfig.dndStart;
    doc["dndE"] = scheduleConfig.dndEnd;
    doc["tzOf"] = scheduleConfig.timezoneOffset;
    doc["ver"] = String(FIRMWARE_VERSION);
    doc["did"] = getDeviceID();

    int n = WiFi.scanComplete();
    doc["ws"] = n;
    if (n > 0) {
      JsonArray nets = doc.createNestedArray("nets");
      for (int i = 0; i < n; ++i) {
        JsonObject n_obj = nets.createNestedObject();
        n_obj["s"] = WiFi.SSID(i);
        n_obj["r"] = WiFi.RSSI(i);
      }
    }

    String json;
    serializeJson(doc, json);
    xSemaphoreGiveRecursive(systemMutex);
    server.send(200, "application/json", json);
  } else {
    server.send(503, "application/json", "{}");
  }
}

void handleSave() {
  preferences.begin("pump-control", false);
  if (xSemaphoreTakeRecursive(systemMutex, portMAX_DELAY)) {
    if (server.hasArg("ssid_sel") && server.arg("ssid_sel") != "__man__") ssid_saved = server.arg("ssid_sel");
    else if (server.hasArg("ssid_man")) ssid_saved = server.arg("ssid_man");
    if (ssid_saved != "") preferences.putString("ssid", ssid_saved);
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
    if (server.hasArg("opM")) {
      compConfig.opMode = server.arg("opM").toInt();
      preferences.putInt("opM", compConfig.opMode);
    }
    if (server.hasArg("vDly")) {
      compConfig.valveDelay = server.arg("vDly").toInt();
      preferences.putInt("vDly", compConfig.valveDelay);
    }
    if (server.hasArg("rstM")) {
      coolDownConfig.restMinutes = server.arg("rstM").toInt();
      preferences.putInt("restM", coolDownConfig.restMinutes);
    }
    if (server.hasArg("rM")) {
      dryRunConfig.autoRetryMinutes = server.arg("rM").toInt();
      preferences.putInt("retryMins", dryRunConfig.autoRetryMinutes);
    }
    if (server.hasArg("pin")) {
      devicePin = server.arg("pin");
      preferences.putString("pin", devicePin);
    }
    if (server.hasArg("sysLang")) {
      sysLang = server.arg("sysLang").toInt();
      preferences.putInt("sysLang", sysLang);
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
    xSemaphoreGiveRecursive(systemMutex);
  }
  preferences.end();
  String html = "<!DOCTYPE html><html><head><meta http-equiv=\"refresh\" content=\"20;url=/\" >"  // Changed to 20s
                "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"><title>Rebooting</title>"
                "<style>body{background:#121212;color:white;font-family:sans-serif;text-align:center;margin-top:50px;}</style></head>"
                "<body><h2>Settings Saved!</h2><p>Rebooting device. Page will refresh in 20 seconds...</p></body></html>";
  server.send(200, "text/html", html);
  rebootSystem();
}

void handleUpdatePage() {
  checkOTA();
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'><title>OTA Update</title><style>body{font-family:sans-serif; background:#121212; color:white; text-align:center; padding:40px 20px; margin:0;} .card{background:#1e1e1e; border-radius:12px; padding:30px 20px; max-width:400px; margin:auto; border:1px solid #333; box-shadow:0 10px 25px rgba(0,0,0,0.5);} h1{font-size:1.8rem; margin:0 0 15px 0; color:#03ef;} p{font-size:1.1rem; color:#ddd; margin-bottom:10px;} .info{color:#888; font-size:0.9rem; margin-top:25px; line-height:1.6; padding:15px; background:#121212; border-radius:8px; border:1px solid #333;} .btn{width:100%; padding:15px; background:#28a745; color:white; border:none; border-radius:8px; margin-top:20px; font-weight:bold; cursor:pointer; font-size:1.1rem;} .btn:active{transform:scale(0.98); opacity:0.9;} .back-link{display:inline-block; margin-top:25px; color:#aaa; text-decoration:none; font-size:1rem; padding:10px 20px; border:1px solid #333; border-radius:8px; background:#121212; transition:0.3s;} .back-link:hover{color:#fff; background:#2a2a2a;}</style></head><body><div class='card'>";
  if (otaConfig.updateAvailable) {
    html += "<h1>Update Available!</h1><p>A newer version (<strong>v" + String(otaConfig.remoteVersion) + "</strong>) is available.</p><p style='font-size:0.95rem; color:#aaa;'>Current version: v" + String(FIRMWARE_VERSION) + "</p><button class='btn' onclick=\"fetch('/start-ota').then(()=>document.body.innerHTML='<h2 style=\\'color:white;text-align:center;margin-top:50px;\\'>Updating...<br><span style=\\'font-size:1rem;color:#888;\\'>Please wait, device will reboot.</span></h2>');\">Start Update Now</button>";
  } else {
    html += "<h1 style='color:#28a745;'>No Updates</h1><p>You are on the latest version (v" + String(FIRMWARE_VERSION) + ").</p>";
  }
  html += "<div class='info'>Remote Version read: " + String(otaConfig.remoteVersion) + "<br>Free Heap: " + String(ESP.getFreeHeap()) + " bytes<br></div><a href='/settings' class='back-link'>← Back to Settings</a></div></body></html>";
  server.send(200, "text/html", html);
}

void handleStatus() {
  if (xSemaphoreTakeRecursive(systemMutex, pdMS_TO_TICKS(100))) {
    String json = generateStatusJson();
    xSemaphoreGiveRecursive(systemMutex);
    server.send(200, "application/json", json);
  } else {
    server.send(503, "application/json", "{}");
  }
}

void handleToggle() {
  String res = processManualToggle();
  if (res == "Silenced") server.send(200, "application/json", "{\"status\":\"success\", \"msg\":\"Silenced\"}");
  else if (res.startsWith("Blocked:")) server.send(200, "application/json", "{\"status\":\"blocked\", \"reason\":\"" + res.substring(8) + "\"}");
  else server.send(200, "application/json", "{\"status\":\"success\"}");
}

void handleReset() {
  if (xSemaphoreTakeRecursive(systemMutex, portMAX_DELAY)) {
    dryRunConfig.error = 0;
    dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET;
    digitalWrite(BUZZER_PIN, LOW);
    saveMotorStatus();
    xSemaphoreGiveRecursive(systemMutex);
  }
  pendingMqttPublish = true;
  server.send(200, "application/json", "{\"status\":\"success\"}");
}

String generateStatusJson() {
  DynamicJsonDocument doc(1024);
  // Data for the Dashboard
  doc["pump"] = pumpConfig.motorStatus;
  doc["tank"] = tankConfig.displayUpperPercentage;
  doc["tStr"] = tankConfig.upperInvalidCount >= TankConfig::MAX_INVALID_COUNT ? "ERR" : (tankConfig.displayUpperPercentage >= 100 ? "FULL" : (tankConfig.displayUpperPercentage <= TankConfig::LOW_THRESHOLD ? "LOW" : String(tankConfig.displayUpperPercentage) + "%"));
  doc["volt"] = (int)voltageConfig.currentVoltage;
  doc["vStat"] = voltageConfig.status == 0 ? "DELAY" : (voltageConfig.currentVoltage > voltageConfig.HIGH_THRESHOLD ? "OVER" : (voltageConfig.currentVoltage < voltageConfig.LOW_THRESHOLD ? "UNDER" : "NORMAL"));
  doc["pStat"] = pumpConfig.isRunning ? "ON" : "OFF";

  // System Info Logic
  String info;
  if (coolDownConfig.isResting) info = "COOLING_DOWN!";
  else if (dryRunConfig.error == 1) info = "DRY_RUN_ALARM!";
  else if (dryRunConfig.error == 2) info = (dryRunConfig.autoRetryMinutes == 0) ? "PUMP_LOCKED!" : "WAITING_RETRY!";
  else if (tankConfig.upperInvalidCount >= TankConfig::MAX_INVALID_COUNT) info = "SENSOR_ERROR!";
  else if (compConfig.isPreVenting || compConfig.isPostVenting) info = "VENTING_VALVE!";
  else if (pumpConfig.isRunning) info = (digitalRead(FLOW_SENSOR_PIN) == LOW) ? "FLOW_DETECTED!" : "FLOW_CHECKING!";
  else info = "SYSTEM_STANDBY!";
  doc["info"] = info;

  // Settings values (To pre-fill the Cloud Settings Page)
  doc["id"] = deviceID;
  doc["ip"] = WiFi.localIP().toString();
  doc["uH"] = tankConfig.upperHeight;
  doc["vH"] = voltageConfig.HIGH_THRESHOLD;
  doc["vL"] = voltageConfig.LOW_THRESHOLD;
  doc["dD"] = dryRunConfig.WAIT_SECONDS_SET;
  doc["opM"] = compConfig.opMode;
  doc["vDly"] = compConfig.valveDelay;       // Fixed: Now sent to cloud
  doc["rstM"] = coolDownConfig.restMinutes;  // Fixed: Now sent to cloud
  doc["rM"] = dryRunConfig.autoRetryMinutes;
  doc["ssid"] = ssid_saved;
  doc["lang"] = sysLang;
  doc["dndEn"] = scheduleConfig.enabled ? 1 : 0;
  doc["dndS"] = scheduleConfig.dndStart;
  doc["dndE"] = scheduleConfig.dndEnd;
  doc["tzOf"] = scheduleConfig.timezoneOffset;
  doc["dndAct"] = currentDndActive ? 1 : 0;
  doc["sErr"] = (tankConfig.upperInvalidCount >= TankConfig::MAX_INVALID_COUNT) ? 1 : 0;
  doc["ack"] = tankConfig.errorAck ? 1 : 0;
  doc["ver"] = FIRMWARE_VERSION;
  doc["nVer"] = otaConfig.remoteVersion;
  doc["ota"] = otaConfig.updateAvailable ? 1 : 0;
  doc["err"] = dryRunConfig.error;
  doc["cd"] = dryRunConfig.waitSeconds;
  doc["rCd"] = dryRunConfig.retryCountdown;
  doc["rstCd"] = coolDownConfig.isResting ? ((coolDownConfig.restMinutes * 60000UL) - (millis() - coolDownConfig.restStartTime)) / 1000 : 0;
  doc["isDR"] = (pumpConfig.isRunning && dryRunConfig.waitSeconds < dryRunConfig.WAIT_SECONDS_SET) ? 1 : 0;

  // License info
  long daysLeft = 0;
  if (installDate > 0) {
    time_t now;
    time(&now);
    unsigned long expiryDate = installDate + (validDays * 86400UL);
    if ((unsigned long)now < expiryDate) daysLeft = (expiryDate - (unsigned long)now + 86399) / 86400;
  }
  doc["expired"] = isSystemExpired ? 1 : 0;
  doc["daysLeft"] = daysLeft;

  String json;
  serializeJson(doc, json);
  return json;
}


void publishState() {
  if (WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
    String json = "";
    if (xSemaphoreTakeRecursive(systemMutex, pdMS_TO_TICKS(100))) {
      json = generateStatusJson();
      xSemaphoreGiveRecursive(systemMutex);
    }
    if (json != "") {
      if (!mqttClient.publish(statusTopic.c_str(), json.c_str(), false)) Serial.println("MQTT Publish Failed!");
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg((char*)payload, length);
  Serial.println("MQTT Received: " + msg);

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, msg);
  if (error) return;

  if (doc.containsKey("lic")) {
    processLicenseTokenString(doc["lic"].as<String>());
    pendingMqttPublish = true;
    return;
  }

  if (doc.containsKey("toggle")) {
    if (doc.containsKey("pin") && String(doc["pin"].as<const char*>()) == devicePin) {

      // --- CLOUD LICENSE LOCK FOR TOGGLE ---
      if (isSystemExpired) {
        DynamicJsonDocument resp(256);
        resp["alert"] = "Cloud Disabled";
        resp["reason"] = "License Expired! Please renew to use cloud control.";
        String respStr;
        serializeJson(resp, respStr);
        mqttClient.publish(statusTopic.c_str(), respStr.c_str());
        return;
      }
      // -------------------------------------

      String result = processManualToggle();  // Capture the result

      // If the command was blocked locally (e.g. Venting), send an alert back to the Cloud dashboard
      if (result.startsWith("Blocked:")) {
        DynamicJsonDocument resp(256);
        resp["alert"] = "System Notice";
        resp["reason"] = result.substring(8);  // Remove "Blocked:" and send the text
        String respStr;
        serializeJson(resp, respStr);
        mqttClient.publish(statusTopic.c_str(), respStr.c_str());
      }
    }
  } else if (doc.containsKey("otaCheck")) {
    if (doc.containsKey("pin") && String(doc["pin"].as<const char*>()) == devicePin) {
      checkOTA();
      DynamicJsonDocument resp(256);
      if (otaConfig.updateAvailable) {
        resp["alert"] = "Update Found!";
        resp["reason"] = "A new version (v" + String(otaConfig.remoteVersion) + ") is available.";
      } else {
        resp["alert"] = "System Up to Date";
        resp["reason"] = "You are already using the latest version: v" + String(FIRMWARE_VERSION);
      }
      String respStr;
      serializeJson(resp, respStr);
      mqttClient.publish(statusTopic.c_str(), respStr.c_str());
    }
  } else if (doc.containsKey("otaStart")) {
    if (doc.containsKey("pin") && String(doc["pin"].as<const char*>()) == devicePin) startOTA();
  } else if (doc.containsKey("reset")) {
    if (doc.containsKey("pin") && String(doc["pin"].as<const char*>()) == devicePin) {
      if (xSemaphoreTakeRecursive(systemMutex, portMAX_DELAY)) {
        dryRunConfig.error = 0;
        dryRunConfig.waitSeconds = dryRunConfig.WAIT_SECONDS_SET;
        digitalWrite(BUZZER_PIN, LOW);
        saveMotorStatus();
        xSemaphoreGiveRecursive(systemMutex);
      }
      pendingMqttPublish = true;
    }
  } else if (doc.containsKey("get")) {
    if (doc.containsKey("pin") && String(doc["pin"].as<const char*>()) == devicePin) pendingMqttPublish = true;
    else {
      DynamicJsonDocument resp(256);
      resp["alert"] = "Access Denied";
      resp["reason"] = "Unauthorized Request - Wrong PIN!";
      String respStr;
      serializeJson(resp, respStr);
      mqttClient.publish(statusTopic.c_str(), respStr.c_str());
    }
  } else if (doc.containsKey("save")) {
    if (doc.containsKey("pin") && String(doc["pin"].as<const char*>()) == devicePin) {

      // --- CLOUD LICENSE LOCK FOR SAVE ---
      if (isSystemExpired) {
        DynamicJsonDocument resp(256);
        resp["alert"] = "Cloud Disabled";
        resp["reason"] = "License Expired! Cannot save settings via Cloud.";
        String respStr;
        serializeJson(resp, respStr);
        mqttClient.publish(statusTopic.c_str(), respStr.c_str());
        return;
      }
      // -----------------------------------

      preferences.begin("pump-control", false);
      if (xSemaphoreTakeRecursive(systemMutex, portMAX_DELAY)) {
        // WiFi Safety: Only update if the string is NOT empty
        if (doc.containsKey("ssid") && doc["ssid"].as<String>() != "") {
          ssid_saved = doc["ssid"].as<String>();
          preferences.putString("ssid", ssid_saved);
        }
        if (doc.containsKey("pass") && doc["pass"].as<String>() != "") {
          pass_saved = doc["pass"].as<String>();
          preferences.putString("pass", pass_saved);
        }

        if (doc.containsKey("uH")) {
          tankConfig.upperHeight = doc["uH"].as<float>();
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
        if (doc.containsKey("opM")) {
          compConfig.opMode = doc["opM"].as<int>();
          preferences.putInt("opM", compConfig.opMode);
        }
        if (doc.containsKey("vDly")) {
          compConfig.valveDelay = doc["vDly"].as<int>();
          preferences.putInt("vDly", compConfig.valveDelay);
        }
        if (doc.containsKey("rstM")) {
          coolDownConfig.restMinutes = doc["rstM"].as<int>();
          preferences.putInt("restM", coolDownConfig.restMinutes);
        }
        if (doc.containsKey("rM")) {
          dryRunConfig.autoRetryMinutes = doc["rM"].as<int>();
          preferences.putInt("retryMins", dryRunConfig.autoRetryMinutes);
        }
        if (doc.containsKey("newPin")) {
          devicePin = doc["newPin"].as<String>();
          preferences.putString("pin", devicePin);
        }
        if (doc.containsKey("sysLang")) {
          sysLang = doc["sysLang"].as<int>();
          preferences.putInt("sysLang", sysLang);
        }

        // DND Sync
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

        xSemaphoreGiveRecursive(systemMutex);
      }
      preferences.end();
      Serial.println("Cloud Settings Saved. Rebooting...");
      delay(500);
      rebootSystem();
    }
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
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  http.setUserAgent("ESP32-Auto-Pump");
  if (http.begin(client, verUrl)) {
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      String cleanVer = "";
      for (int i = 0; i < payload.length(); i++)
        if (isdigit(payload[i])) cleanVer += payload[i];
      if (cleanVer.length() > 0) {
        int remoteVer = cleanVer.toInt();
        if (xSemaphoreTakeRecursive(systemMutex, portMAX_DELAY)) {
          otaConfig.remoteVersion = remoteVer;
          if (remoteVer > FIRMWARE_VERSION) {
            otaConfig.updateAvailable = true;
            otaConfig.newVersion = remoteVer;
          } else {
            otaConfig.updateAvailable = false;
          }
          xSemaphoreGiveRecursive(systemMutex);
        }
      }
      pendingMqttPublish = true;
    }
    http.end();
  }
}

void startOTA() {
  if (WiFi.status() != WL_CONNECTED || !otaConfig.updateAvailable) return;
  if (mqttClient.connected()) mqttClient.disconnect();
  WiFiClientSecure client;
  client.setInsecure();
  client.setTimeout(15000);
  httpUpdate.setLedPin(RGB_LED_PIN, LOW);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("UPDATING SYSTEM");
  lcd.setCursor(0, 1);
  lcd.print("Do not turn off");
  t_httpUpdate_return ret = httpUpdate.update(client, String(FW_URL_BASE) + "firmware.bin");
  if (ret == HTTP_UPDATE_FAILED) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("UPDATE FAILED!");
    delay(5000);
    ESP.restart();
  }
}