#include "server.h"
#include "credentials.h"

#include <Arduino.h>              
#include <ESP32Servo.h>           
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

bool manual_mode = true;    

static void on_ws_message(void*, uint8_t*, size_t);

extern Servo servos[];
extern double MIN_DEG[];
extern double MAX_DEG[];

/*
* HELPER TYPE SHIT
*/
static void write_servo(uint8_t idx, float deg)
{
    if (idx > 3) return;                       
    deg = constrain(deg, MIN_DEG[idx], MAX_DEG[idx]);
    servos[idx].write(static_cast<int>(deg));
}

/*
* WEBSOCKET TYPE SHIT
*/
static void on_ws_message(void* arg, uint8_t* data, size_t len)
{
    AwsFrameInfo* info = static_cast<AwsFrameInfo*>(arg);
    if (!info->final || info->opcode != WS_TEXT) return;

    StaticJsonDocument<64> doc;
    if (deserializeJson(doc, data, len)) return;

    uint8_t id  = doc["id"]  | 0;
    float   deg = doc["deg"] | 90.0;

    manual_mode = true;
    write_servo(id, deg);
}

/*
* ACTUAL SERVER STUFF
*/
void init_server()
{
    WiFi.begin(WIFI_SSID, NULL);
    while (WiFi.status() != WL_CONNECTED) delay(200);

    Serial.printf("[Wi‑Fi] connected to \"%s\" — IP: %s\n",
                    WiFi.SSID().c_str(),
                    WiFi.localIP().toString().c_str());

    static const char page[] PROGMEM = R"(<!DOCTYPE html> ... )";

    server.on("/", HTTP_GET,
        [](AsyncWebServerRequest* r){ r->send_P(200,"text/html",page); });

    server.on("/set", HTTP_GET,
        [](AsyncWebServerRequest* r){
            if (!r->hasParam("mode")) { r->send(400); return; }

            /*
            String m = r->getParam("mode")->value();
            if      (m == "square") current_path = &square_path;
            else if (m == "spin")   current_path = &spin_path;
            else if (m == "pitch")  current_path = &pitch_sweep_path;
            manual_mode = false;               // resume auto runner
            */
            r->send(200,"text/plain","OK");
        });

    ws.onEvent([](  AsyncWebSocket*,
                    AsyncWebSocketClient*,
                    AwsEventType t,
                    void* arg,
                    uint8_t* data,
                    size_t len)
    {
        if (t == WS_EVT_DATA) on_ws_message(arg, data, len);
    });
    server.addHandler(&ws);

    server.begin();
}

void server_setup() { init_server(); }