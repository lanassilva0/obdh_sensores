#include <Arduino.h>
#include <ArduinoJson.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <EEPROM.h>
#include <LoRa.h>
// #include <SPI.h>
// #include <SD.h>

#define EPPROM_SIZE 512

Adafruit_BMP280 bmp;

// conferir os pinos corretos
// const int MICROSD_PIN_CHIP_SELECT = 21; // Pino serial
// const int MICROSD_PIN_MOSI = 19;        // Pino serial
// const int MICROSD_PIN_MISO = 18;        // Pino serial
// const int MICROSD_PIN_SCK = 22;         // Clock pin

// SPIClass spiSD(VSPI);

// Compila apenas se MASTER estiver definido no arquivo principal
// #ifdef MASTER

// Intervalo entre os envios
#define INTERVAL 1 // em segundos

// Tempo do último envio
long lastSendTime = 0;

#define SCK 5   // GPIO5  SCK
#define MISO 19 // GPIO19 MISO
#define MOSI 27 // GPIO27 MOSI
#define SS 18   // GPIO18 CS
#define RST 14  // GPIO14 RESET
#define DI00 26 // GPIO26 IRQ(Interrupt Request)

#define BAND 433E6

const String GETDATA = "getdata";
const String SETDATA = "setdata=";

// Configurações iniciais do LoRa
void setupLoRa()
{
  SPI.begin(SCK, MISO, MOSI, SS);
  LoRa.setPins(SS, RST, DI00);

  if (!LoRa.begin(BAND))
  {
    Serial.println("Erro ao iniciar LoRa");
    while (1)
      ;
  }

  LoRa.enableCrc();
  LoRa.receive();
}

void setup()
{

  EEPROM.begin(EPPROM_SIZE);
  setupLoRa();

  Serial.begin(115200);
  if (!bmp.begin())
  {
    Serial.println("Cannot connect to BMP280");
    while (1)
      ;
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

StaticJsonDocument<512> geraArquivoJson(float temp, float pressao, float altitude)
{
  StaticJsonDocument<512> doc;
  doc["temperatura"] = temp;
  doc["pressao"] = pressao;
  doc["altitude"] = altitude;

  serializeJson(doc, Serial);
  Serial.println();

  return doc;
}

void loop()
{
  StaticJsonDocument<512> jsonLeitura;
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

    jsonLeitura = geraArquivoJson(bmp.readTemperature(), bmp.readPressure(), bmp.readAltitude(1013.25));

    lastTime = millis();
  }

  // EEPROM.write(0, jsonLeitura);
  EEPROM.commit();
}