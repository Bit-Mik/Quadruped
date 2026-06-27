#include "imu.h"
#include <Arduino.h>
#include "config.h"

#ifdef ESP32
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static portMUX_TYPE imuStateMux = portMUX_INITIALIZER_UNLOCKED;
#define IMU_STATE_LOCK() portENTER_CRITICAL(&imuStateMux)
#define IMU_STATE_UNLOCK() portEXIT_CRITICAL(&imuStateMux)
#else
#define IMU_STATE_LOCK()
#define IMU_STATE_UNLOCK()
#endif

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

    unsigned long now = millis();

    IMU_STATE_LOCK();
    g_imuState.healthy = true;
    g_imuState.lastUpdateMs = now;
    IMU_STATE_UNLOCK();

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
        unsigned long now = millis();

        IMU_STATE_LOCK();
        g_imuState.yawDeg   = yaw;
        g_imuState.pitchDeg = roll;
        g_imuState.rollDeg  = pitch;

        g_imuState.lastUpdateMs = now;
        g_imuState.healthy = true;
        IMU_STATE_UNLOCK();

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
    (void)dt;

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

    ImuState snapshot = imuGetState();

    if(millis() - snapshot.lastUpdateMs > 1000)
    {
        IMU_STATE_LOCK();
        g_imuState.healthy = false;
        IMU_STATE_UNLOCK();
        snapshot.healthy = false;
    }

    return snapshot.healthy;
}

ImuState imuGetState()
{
    IMU_STATE_LOCK();
    ImuState snapshot = g_imuState;
    IMU_STATE_UNLOCK();

    return snapshot;
}

void imuReset()
{
    unsigned long now = millis();

    IMU_STATE_LOCK();
    g_imuState.rollDeg = 0;
    g_imuState.pitchDeg = 0;
    g_imuState.yawDeg = 0;

    g_imuState.rollRateDps = 0;
    g_imuState.pitchRateDps = 0;
    g_imuState.yawRateDps = 0;

    g_imuState.lastUpdateMs = now;
    g_imuState.healthy = false;
    IMU_STATE_UNLOCK();
}
