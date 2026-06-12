#include <WiFi.h>
#include <ESPmDNS.h>
#include "secret.h"

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASSWORD;

void connectWiFi()
{
    WiFi.mode(WIFI_STA);

    WiFi.begin(ssid, password);

    Serial.print("Connecting");

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WiFi Connected");

    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    if (MDNS.begin("quadruped"))
    {
        Serial.println("mDNS started");
        Serial.println("Open: http://quadruped.local");
    }
    else
    {
        Serial.println("mDNS failed");
    }
}