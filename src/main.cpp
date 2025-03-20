#include <Arduino.h>
#include "gmp251.h"

constexpr uint8_t RS485_RX = 16; // ESP32 RX pin
constexpr uint8_t RS485_TX = 17; // ESP32 TX pin
constexpr uint8_t RS485_DE = 4;  // Direction Control pin

uint32_t lastReadTime = 0;
uint32_t PRINT_INTERVAL_MS = 1000;

GMP251 gmp251(RS485_RX, RS485_TX, RS485_DE, Serial1);

void setup()
{
    Serial.begin(115200);
    gmp251.begin();
}

void loop()
{
    gmp251.update();

    if (millis() - lastReadTime >= PRINT_INTERVAL_MS)
    {
        lastReadTime = millis();
        Serial.print("Status: ");
        Serial.print(gmp251.getStatus());
        Serial.print("      CO2: ");
        Serial.print(gmp251.getCO2());
        Serial.println(" ppm");
    }
}
