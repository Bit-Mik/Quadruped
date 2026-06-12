#include "imu.h"
#include <Arduino.h>
#include "config.h"

HardwareSerial RazorSerial(2);

static ImuState g_imuState;

static String lineBuffer;

bool imuInit()
{
    imuReset();
    RazorSerial.begin(
        57600,
        SERIAL_8N1,
        16,   // RX2
        17    // TX2
    );

    g_imuState.healthy = true;

    return true;
}

static void parseYPR(const String& line)
{
    float yaw;
    float pitch;
    float roll;

    if(sscanf(
        line.c_str(),
        "#YPR=%f,%f,%f",
        &yaw,
        &pitch,
        &roll) == 3)
    {
        g_imuState.yawDeg   = yaw;
        g_imuState.pitchDeg = roll;
        g_imuState.rollDeg  = pitch;

        g_imuState.lastUpdateMs = millis();
        g_imuState.healthy = true;

        static uint32_t lastPrint = 0;

        // if(millis() - lastPrint > 500)
        // {
        //     lastPrint = millis();

        //     DEBUG_PRINT("Roll=");
        //     DEBUG_PRINT(g_imuState.rollDeg);

        //     DEBUG_PRINT(" Pitch=");
        //     DEBUG_PRINT(g_imuState.pitchDeg);

        //     DEBUG_PRINT(" Yaw=");
        //     DEBUG_PRINTLN(g_imuState.yawDeg);
        // }
    }
}

bool imuUpdate(float dt)
{
    while(RazorSerial.available())
    {
        char c = RazorSerial.read();

        if(c == '\n')
        {
            parseYPR(lineBuffer);
            lineBuffer = "";
        }
        else if(c != '\r')
        {
            lineBuffer += c;
            if(lineBuffer.length() > 100)
                lineBuffer = "";
        }
    }

    if(millis() - g_imuState.lastUpdateMs > 1000)
    {
        g_imuState.healthy = false;
    }

    return g_imuState.healthy;
}

const ImuState& imuGetState()
{
    return g_imuState;
}

void imuReset()
{
    g_imuState.rollDeg = 0;
    g_imuState.pitchDeg = 0;
    g_imuState.yawDeg = 0;

    g_imuState.rollRateDps = 0;
    g_imuState.pitchRateDps = 0;
    g_imuState.yawRateDps = 0;

    g_imuState.healthy = false;
}