#include <Arduino.h>
#include <ArduinoJson.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <EEPROM.h>
#include <LoRa.h>
#include <SPI.h>
#include <SD.h>

#define EPPROM_SIZE 512
#define EEPROM_ADDR 0

#define SS 18   // GPIO18 CS
#define RST 14  // GPIO14 RESET
#define DI00 26 // GPIO26 IRQ(Interrupt Request)

Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;

// Micro SD Card
//  * MicroSD VCC pin to ESP32 +5V
//  * MicroSD GND pin to ESP32 GND
//  * MicroSD MISO pin to ESP32 GPIO13
//  * MicroSD MOSI pin to ESP32 GPIO12
//  * MicroSD SCK pin to ESP32 GPIO14
//  * MicroSD CS pin to ESP32 GPIO27

const int MICROSD_PIN_CS = 27;   // Pino serial
const int MICROSD_PIN_MOSI = 12; // Pino serial
const int MICROSD_PIN_MISO = 13; // Pino serial
const int MICROSD_PIN_SCK = 14;  // Clock pin

File myFile;

void setup()
{

  EEPROM.begin(EPPROM_SIZE);
  // setupLoRa();
  Serial.begin(115200);
  delay(1000);

  // Serial.println("Inicializando MPU6050...");

  // if (!mpu.begin())
  // {
  //   Serial.println("Não foi possível encontrar o MPU6050. Verifique a conexão.");
  //   // while (1) delay(10);
  // }

  // Serial.println("MPU6050 conectado com sucesso!");

  // delay(100);

  if (!bmp.begin())
  {
    Serial.println("Cannot connect to BMP280");
    // while (1)
  }

  Serial.begin(9600);
  delay(500);

  if (!SD.begin(MICROSD_PIN_CS))
  {
    Serial.println("Erro ao iniciar SD Card");
  }
  else
  {
    Serial.println("Cartão SD inicializado");
  }

  writeFile("/test.txt", "ElectronicWings.com");
  readFile("/test.txt");
}

long currentTime, lastTime;

void saveEEPROM(const StaticJsonDocument<512> &json)
{
  char jsonStr[512];
  serializeJson(json, jsonStr); // converte o JSON para string
  for (int i = 0; i < strlen(jsonStr); i++)
  {
    EEPROM.write(EEPROM_ADDR + i, jsonStr[i]);
  }
  EEPROM.commit();
}

void readFile(const char *path)
{
  myFile = SD.open(path);
  if (myFile)
  {
    Serial.printf("Lendo arquivo de %s\n", path);
    while (myFile.available())
    {
      Serial.write(myFile.read());
    }
    myFile.close();
  }
  else
  {
    Serial.println("Erro ao abrir test.txt");
  }
}

void writeFile(const char *path, const char *message)
{
  myFile = SD.open(path, FILE_WRITE);
  if (myFile)
  {
    Serial.printf("Escrevendo em %s ", path);
    myFile.println(message);
    myFile.close();
    Serial.println("concluído.");
  }
  else
  {
    Serial.println("Erro ao abrir o arquivo");
    Serial.println(path);
  }
}

void loop()
{
  StaticJsonDocument<512> jsonLeitura;

  JsonObject leituraSensores = jsonLeitura.to<JsonObject>();

  currentTime = millis();

  if (currentTime - lastTime > 2000)
  {
    Serial.println("BMP280:");
    Serial.print("Temperatura : ");
    Serial.print(bmp.readTemperature());
    leituraSensores["temperatura"] = bmp.readTemperature();
    Serial.println(" *C");

    Serial.print("Pressao : ");
    Serial.print(bmp.readPressure());
    leituraSensores["pressao"] = bmp.readPressure();
    Serial.println(" Pa");

    Serial.print("Altitude : ");
    Serial.print(bmp.readAltitude(1013.25));
    leituraSensores["altitude"] = bmp.readAltitude(1013.25);
    Serial.println(" m");
    Serial.println(" ");

    // sensors_event_t a, g, temp;
    // mpu.getEvent(&a, &g, &temp);

    // Serial.println("MPU6050:");
    // Serial.print("Accel X: ");
    // leituraSensores["aceleracao"][0] = a.acceleration.x;
    // Serial.print(a.acceleration.x);
    // Serial.print(", Y: ");
    // leituraSensores["aceleracao"][1] = a.acceleration.y;
    // Serial.print(a.acceleration.y);
    // Serial.print(", Z: ");
    // leituraSensores["aceleracao"][2] = a.acceleration.z;
    // Serial.println(a.acceleration.z);

    // Serial.print("Gyro X: ");
    // leituraSensores["giroscopio"][0] = g.gyro.x;
    // Serial.print(g.gyro.x);
    // Serial.print(", Y: ");
    // leituraSensores["giroscopio"][1] = g.gyro.y;
    // Serial.print(g.gyro.y);
    // Serial.print(", Z: ");
    // leituraSensores["giroscopio"][2] = g.gyro.z;
    // Serial.println(g.gyro.z);

    // Serial.print("Temp: ");
    // Serial.print(temp.temperature);
    // Serial.println(" °C");

    String jsonStr;
    serializeJson(jsonLeitura, jsonStr);

    saveEEPROM(leituraSensores);

    Serial.println();
    delay(1000);

    lastTime = millis();
  }
}