#include "telemetry.h"
#include "imu.h"
#include <ESPAsyncWebServer.h>


extern AsyncWebSocket ws;

void sendTelemetry()
{
    static uint32_t lastSend = 0;

    if(millis() - lastSend < 100)
        return;

    lastSend = millis();
    ImuState imu = imuGetState();
    String json =
        "{\"roll\":" + String(imu.rollDeg, 2) +
        ",\"pitch\":" + String(imu.pitchDeg, 2) +
        ",\"yaw\":0}";

    ws.textAll(json);
}
