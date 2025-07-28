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

#define SCK 5   // GPIO5  SCK
#define MISO 19 // GPIO19 MISO
#define MOSI 27 // GPIO27 MOSI
#define SS 18   // GPIO18 CS
#define RST 14  // GPIO14 RESET
#define DI00 26 // GPIO26 IRQ(Interrupt Request)

#define BAND 433E6

// Intervalo entre os envios
#define INTERVAL 1 // em segundos

Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;

// conferir os pinos corretos
// const int MICROSD_PIN_CHIP_SELECT = 21; // Pino serial
// const int MICROSD_PIN_MOSI = 19;        // Pino serial
// const int MICROSD_PIN_MISO = 18;        // Pino serial
// const int MICROSD_PIN_SCK = 22;         // Clock pin

// SPIClass spiSD(VSPI);

// Compila apenas se MASTER estiver definido no arquivo principal
// #ifdef MASTER

long lastSendTime = 0;

const String GETDATA = "getdata";
const String SETDATA = "setdata=";

// Configurações iniciais do LoRa
// void setupLoRa()
// {
//   SPI.begin(SCK, MISO, MOSI, SS);
//   LoRa.setPins(SS, RST, DI00);

//   if (!LoRa.begin(BAND))
//   {
//     Serial.println("Erro ao iniciar LoRa");
//     while (1)
//       ;
//   }

//   LoRa.enableCrc();
//   LoRa.receive();
// }

void setup()
{

  EEPROM.begin(EPPROM_SIZE);
  // setupLoRa();
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
    // while (1)
  }

  // if (!SD.begin(MICROSD_PIN_CHIP_SELECT, spiSD))
  // {
  //   Serial.println("Erro ao iniciar SD Card");
  // }
  // else
  // {
  //   Serial.println("Cartão SD inicializado");
  // }
}

long currentTime, lastTime;

// void sendLoRaMessage(const StaticJsonDocument<512> message)
// {
//   LoRa.beginPacket();
//   LoRa.print(message);
//   LoRa.endPacket();
//   Serial.print("Enviado: ");
//   Serial.println(message);
// }

void salvaEEPROM(const StaticJsonDocument<512> &json)
{
  char jsonStr[512];
  serializeJson(json, jsonStr); // converte o JSON para string
  for (int i = 0; i < strlen(jsonStr); i++)
  {
    EEPROM.write(EEPROM_ADDR + i, jsonStr[i]);
  }
  EEPROM.commit();
}

StaticJsonDocument<512> geraArquivoJson(float temp, float pressao, float altitude, int type)
{
  StaticJsonDocument<512> doc;
  if (type == 1)
  {
    doc["temperatura"] = temp;
    doc["pressao"] = pressao;
    doc["altitude"] = altitude;
  }
  // else if (type == 2)
  // {
  //   doc["accel_x"] = temp; // Usando temp como exemplo de aceleração X
  //   doc["accel_y"] = pressao; // Usando pressao como exemplo de aceleração Y
  //   doc["accel_z"] = altitude; // Usando altitude como exemplo de aceleração Z
  // }

  serializeJson(doc, Serial);
  Serial.println();

  return doc;
}

void loop()
{
  StaticJsonDocument<512> jsonLeituraBMP;
  StaticJsonDocument<512> jsonLeituraMPUAcceleration;
  StaticJsonDocument<512> jsonLeituraMPUGyro;
  StaticJsonDocument<512> jsonLeituraMPUTemperature;

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

    jsonLeituraBMP = geraArquivoJson(bmp.readTemperature(), bmp.readPressure(), bmp.readAltitude(1013.25), 1);
    salvaEEPROM(jsonLeituraBMP);

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

    // jsonLeituraMPUAcceleration = geraArquivoJson(a.acceleration.x, a.acceleration.y, a.acceleration.z, 2);
    // jsonLeituraMPUGyro = geraArquivoJson(g.gyro.x, g.gyro.y, g.gyro.z, 3);
    // jsonLeituraMPUTemperature = geraArquivoJson(temp.temperature, 0, 0, 4);
    // salvaEEPROM(jsonLeituraMPUAcceleration);

    Serial.println();
    delay(1000);

    lastTime = millis();
  }
}