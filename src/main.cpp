#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("Inicializando MPU6050...");

  if (!mpu.begin())
  {
    Serial.println("Não foi possível encontrar o MPU6050. Verifique a conexão.");
    // while (1) delay(10);
  }

  Serial.println("MPU6050 conectado com sucesso!");

  delay(100);

  if (!bmp.begin())
  {
    Serial.println("Cannot connect to BMP280");
    // while (1);
  }
}

long currentTime, lastTime;

void loop()
{

  currentTime = millis();

  if (currentTime - lastTime > 2000)
  {
    Serial.println("BMP280:");
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

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    Serial.println("MPU6050:");
    Serial.print("Accel X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.println(a.acceleration.z);

    Serial.print("Gyro X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.println(g.gyro.z);

    Serial.print("Temp: ");
    Serial.print(temp.temperature);
    Serial.println(" °C");

    Serial.println();
    delay(1000);

    lastTime = millis();
  }
}