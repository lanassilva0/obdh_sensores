#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>

Adafruit_BMP280 bmp;

void setup()
{

  Serial.begin(115200);
  if (!bmp.begin())
  {
    Serial.println("Cannot connect to BMP280");
    while (1);
  }
}

long currentTime, lastTime;

void loop()
{
  currentTime = millis();

  if (currentTime - lastTime > 2000)
  {

    Serial.print("Temperatura : ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressao : ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print("Altitude : ");
    Serial.print(bmp.readAltitude(1013.25));
    Serial.println(" m");
    Serial.println(" ");

    lastTime = millis();
  }
}