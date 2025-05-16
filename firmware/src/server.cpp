// ─────────────────────────────────── server.h ────────────────────────────────
#include "credentials.h"

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>

// ── compile‑time knobs ────────────────────────────────────────────────────────
#define SERVO_COUNT 6                 // change if you really have 6 joints
#define JSON_DOC_SIZE 128             // plenty of room for future keys

// ── globals ──────────────────────────────────────────────────────────────────
extern Servo   servos[SERVO_COUNT];
extern double  MIN_DEG[SERVO_COUNT];
extern double  MAX_DEG[SERVO_COUNT];

AsyncWebServer  server(80);
AsyncWebSocket  ws("/ws");

bool manual_mode = true;              // auto‑runner pauses when true

// ── helpers ──────────────────────────────────────────────────────────────────
static String htmlProcessor(const String& var)
{
    if (var == "SERVO_COUNT") return String(SERVO_COUNT);
    return String();                     
}

static void write_servo(uint8_t idx, float deg)
{
    if (idx >= SERVO_COUNT) return;               // guard array bounds
    deg = constrain(deg, MIN_DEG[idx], MAX_DEG[idx]);
    servos[idx].write(static_cast<int>(deg));
}

// ── WebSocket frame handler ──────────────────────────────────────────────────
static void on_ws_message(void* arg, uint8_t* data, size_t len)
{
    AwsFrameInfo* info = static_cast<AwsFrameInfo*>(arg);
    if (!info->final || info->opcode != WS_TEXT) return;

    StaticJsonDocument<JSON_DOC_SIZE> doc;
    if (deserializeJson(doc, data, len)) return;

    uint8_t id  = doc["id"]  | 0;
    float   deg = doc["deg"] | 90.0;

    manual_mode = true;
    write_servo(id, deg);
}

// ── HTML user‑interface (served from flash) ──────────────────────────────────
static const char CONTROL_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>CTRL ARM DEL</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
 body{font-family:sans-serif;margin:2rem;max-width:480px}
 h1  {font-size:1.4rem;margin-bottom:1rem}
 .slider-block{margin:1rem 0}
 label{display:block;font-weight:bold;margin-bottom:0.3rem}
 input[type=range]{width:100%%;}        
</style>
</head>
<body>
<h1>Manual Control</h1>

<div id="sliders"></div>

<script>
const SERVO_COUNT = %SERVO_COUNT%;      
const socket = new WebSocket(`ws://${location.host}/ws`);

socket.addEventListener('open', () => console.log('WS connected'));

function sendAngle(id, deg){
  if(socket.readyState===1) socket.send(JSON.stringify({id,deg}));
}

function buildUI(){
  const wrap=document.getElementById('sliders');
  for(let i=0;i<SERVO_COUNT;i++){
    const block=document.createElement('div');
    block.className='slider-block';
    block.innerHTML = `
      <label for="s${i}">Servo ${i}</label>
      <input id="s${i}" type="range" min="0" max="180" value="90">
    `;
    block.querySelector('input').addEventListener('input', e=>{
      sendAngle(i, parseInt(e.target.value));
    });
    wrap.appendChild(block);
  }
}
buildUI();
</script>
</body>
</html>
)rawliteral";

// ── Wi‑Fi + server bring‑up ──────────────────────────────────────────────────
void init_server()
{
    Serial.begin(115200);

    // resnet guest: 100.117.32.66

    WiFi.begin(WIFI_SSID_RESNET, WIFI_PASS);      // adjust for your net
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis()-t0 < 15000){
        delay(250);
        Serial.print('.');
    }
    if (WiFi.status() != WL_CONNECTED){
        Serial.println("\n[Wi‑Fi] failed to connect"); return;
    }

    Serial.printf("\n[Wi‑Fi] %s  IP: %s\n",
                  WiFi.SSID().c_str(),
                  WiFi.localIP().toString().c_str());

    // root -> control page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){
    r->send_P(200,
              "text/html",
              CONTROL_PAGE,        // PROGMEM string
              htmlProcessor);      // <- template callback
});

    // add WebSocket and delegate data frames
    ws.onEvent([](  AsyncWebSocket*,AsyncWebSocketClient*,AwsEventType t,
                    void* arg,uint8_t* data,size_t len){
        if(t==WS_EVT_DATA) on_ws_message(arg,data,len);
    });
    server.addHandler(&ws);

    server.begin();
    Serial.println("[HTTP] server started");
}

void server_setup(){ init_server(); }
void server_loop(){ ws.cleanupClients(); }