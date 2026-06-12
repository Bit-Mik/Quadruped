#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include "servo_manual.h"
#include "globals.h"
#include "safety.h"

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
void onWsEvent(
    AsyncWebSocket *server,
    AsyncWebSocketClient *client,
    AwsEventType type,
    void *arg,
    uint8_t *data,
    size_t len);
void initWebUI()
{
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    if(!LittleFS.begin(true))
    {
        Serial.println("LittleFS mount failed");
        return;
    }

    server.on("/ping", HTTP_GET,
    [](AsyncWebServerRequest *request)
    {
    request->send(200, "text/plain", "pong");
    });

    server.serveStatic("/", LittleFS, "/")
          .setDefaultFile("index.html");
    
    

    server.begin();
}

void onWsEvent(
    AsyncWebSocket *server,
    AsyncWebSocketClient *client,
    AwsEventType type,
    void *arg,
    uint8_t *data,
    size_t len)
{
    if(type != WS_EVT_DATA)
        return;

    String msg;

    for(size_t i = 0; i < len; i++)
        msg += (char)data[i];

    Serial.println(msg);

    if(msg == "forward")
    {
        isGaitRunning = true;
    }

    else if(msg == "backward")
    {
        targetForward = -1.0f;
        targetTurn = 0.0f;
    }

    else if(msg == "left")
    {
        targetForward = 0.0f;
        targetTurn = -1.0f;
    }

    else if(msg == "right")
    {
        targetForward = 0.0f;
        targetTurn = 1.0f;
    }

    else if(msg == "stop")
    {
        isGaitRunning = false;
    }
    if(msg == "stand")
    {
        initializeServos();
    }
}