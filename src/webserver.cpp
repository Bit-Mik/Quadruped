#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include "servo_manual.h"
#include "globals.h"
#include "safety.h"
#include "stabilization.h"
#include <ArduinoJson.h>

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
    {
        msg += (char)data[i];
    }

    if(msg == "forward")
    {
        phaseTime = GAIT_START_PHASE;
        targetForward = 1.0f;
        targetTurn = 0.0f;
        robotMode = MODE_GAIT;
        return;
    }

    else if(msg == "backward")
    {
        phaseTime = GAIT_START_PHASE;
        targetForward = -1.0f;
        targetTurn = 0.0f;
        robotMode = MODE_GAIT;
        return;
    }

    else if(msg == "left")
    {
        phaseTime = GAIT_START_PHASE;
        targetForward = 0.0f;
        targetTurn = -1.0f;
        robotMode = MODE_GAIT;
        return;
    }

    else if(msg == "right")
    {
        phaseTime = GAIT_START_PHASE;
        targetForward = 0.0f;
        targetTurn = 1.0f;
        robotMode = MODE_GAIT;
        return;
    }

    else if(msg == "stop")
    {
        targetForward = 0.0f;
        targetTurn = 0.0f;
        robotMode = MODE_STAND;
        return;
    }
    if(msg == "stand")
    {
        targetForward = 0.0f;
        targetTurn = 0.0f;
        robotMode = MODE_STAND;
        return;
    }

    JsonDocument doc;

    DeserializationError error =
        deserializeJson(doc, msg);

    if(error)
    {
        Serial.println("JSON parse failed");
        return;
    }

    String cmdType = doc["type"];

    if(cmdType == "servo")
    {
        int index = doc["index"];
        float angle = doc["angle"];

        targetForward = 0.0f;
        targetTurn = 0.0f;
        robotMode = MODE_MANUAL;
        setServoManual(index, angle);
        return;
    }

    if(cmdType == "pid")
    {
        JsonObject roll = doc["roll"].as<JsonObject>();
        JsonObject pitch = doc["pitch"].as<JsonObject>();

        float kpRoll = roll["kp"] | KP_ROLL;
        float kiRoll = roll["ki"] | KI_ROLL;
        float kdRoll = roll["kd"] | KD_ROLL;

        float kpPitch = pitch["kp"] | KP_PITCH;
        float kiPitch = pitch["ki"] | KI_PITCH;
        float kdPitch = pitch["kd"] | KD_PITCH;

        float rollDeadband = doc["rollDeadband"] | ROLL_DEADBAND;
        float pitchDeadband = doc["pitchDeadband"] | PITCH_DEADBAND;
        float maxRollCorr = doc["maxRollCorr"] | MAX_ROLL_CORR;
        float maxPitchCorr = doc["maxPitchCorr"] | MAX_PITCH_CORR;

        stabilizationSetConfig(
            kpRoll,
            kiRoll,
            kdRoll,
            kpPitch,
            kiPitch,
            kdPitch,
            rollDeadband,
            pitchDeadband,
            maxRollCorr,
            maxPitchCorr
        );

        client->text("{\"type\":\"pid\",\"status\":\"applied\"}");
        return;
    }

    if(cmdType == "leg")
{
    String leg = doc["leg"];

    float x = doc["x"];
    float y = doc["y"];
    float z = doc["z"];

    targetForward = 0.0f;
    targetTurn = 0.0f;
    robotMode = MODE_MANUAL;

    if(leg == "FR")
        setLegPosition(LEG_FR, x, y, z);

    else if(leg == "FL")
        setLegPosition(LEG_FL, x, y, z);

    else if(leg == "BR")
        setLegPosition(LEG_BR, x, y, z);

    else if(leg == "BL")
        setLegPosition(LEG_BL, x, y, z);
    return;
}
}
