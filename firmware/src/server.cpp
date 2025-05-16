#include "credentials.h"
#include "server.h"

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>

#define SERVO_COUNT 6                 
#define JSON_DOC_SIZE 128             

extern Servo   servos[SERVO_COUNT];
extern double  MIN_DEG[SERVO_COUNT];
extern double  MAX_DEG[SERVO_COUNT];

constexpr uint8_t CR_SERVO_IDX      = 0;      
constexpr float   CR_DEG_PER_SEC    = 350.0;  
constexpr int     CR_STOP_US        = 1500;   
constexpr int     CR_CW_US          = 1700;  
constexpr int     CR_CCW_US         = 1300;

std::atomic<uint32_t> crStopAt{0};

AsyncWebServer  server(80);
AsyncWebSocket  ws("/ws");

bool manual_mode = true;      

// Helper type shit
static String htmlProcessor(const String& var)
{
    if (var == "SERVO_COUNT") return String(SERVO_COUNT);
    return String();                     
}

// Wrapper to call write() or writeMicroseconds() to move the motors
static void write_servo(uint8_t idx, float deg)
{
    if (idx >= SERVO_COUNT) return;

    if (idx == CR_SERVO_IDX) {
        bool forward  = (deg >= 0);
        float absDeg  = fabsf(deg);

        uint32_t runMs = (uint32_t)((absDeg / CR_DEG_PER_SEC) * 1000.0f);

        if (forward) {
          // rotate CW
          servos[idx].writeMicroseconds(CR_CW_US);
        }
        else {
          // rotate CCW
          servos[idx].writeMicroseconds(CR_CCW_US);
        }

        crStopAt = millis() + runMs;
        return;
    }

    deg = constrain(deg, MIN_DEG[idx], MAX_DEG[idx]);
    servos[idx].write(static_cast<int>(deg));
}

// WebSocket handler
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

// start that shit
void init_server()
{
    // Serial.begin(115200);

    WiFi.begin(WIFI_SSID_RESNET, WIFI_PASS); // nab from credentials.h

    // connect to the network
    Serial.printf("\n[WiFi] Connecting to %s", WIFI_SSID_RESNET);

    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.printf(".");
    }

    Serial.printf("\n[WiFi] %s  IP: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());

    // serve home page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){
      r->send_P(200, "text/html", CONTROL_PAGE, htmlProcessor);   
    });

    // websocket event handler
    ws.onEvent([](AsyncWebSocket*,AsyncWebSocketClient*,AwsEventType t,
                  void* arg,uint8_t* data,size_t len) {
        if(t == WS_EVT_DATA) {
          on_ws_message(arg,data,len);
        }
    });

    // final init
    server.addHandler(&ws);
    server.begin();
    Serial.println("[HTTP] server started");
}

// functions to call from main.cpp
void server_setup(){ init_server(); }
void server_loop(){ ws.cleanupClients(); }