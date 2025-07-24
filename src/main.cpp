#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

Adafruit_BMP280 bmp;

// conferir os pinos corretos
const int MICROSD_PIN_CHIP_SELECT = 21; // Pino serial
const int MICROSD_PIN_MOSI = 19;        // Pino serial
const int MICROSD_PIN_MISO = 18;        // Pino serial
const int MICROSD_PIN_SCK = 22;         // Clock pin
SD_File_Record ObjSD("test.txt", 3);

void setup()
{

  Serial.begin(115200);
  if (!bmp.begin())
  {
    Serial.println("Cannot connect to BMP280");
    while (1)
      ;
  }

  if (!ObjSD.init(MICROSD_PIN_CHIP_SELECT, MICROSD_PIN_MOSI, MICROSD_PIN_MISO, MICROSD_PIN_SCK))
  {
    Serial.println("SD begin fail");
    delay(1000);
    ESP.restart();
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