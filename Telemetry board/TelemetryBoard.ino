#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "board_def.h"

// Definições do MPU9250
#define MPU 0x68
#define accX_H 0x3B
#define accY_H 0x3D
#define accZ_H 0x3F
#define gyroX_H 0x43
#define gyroY_H 0x45
#define gyroZ_H 0x47
#define TEMP_H 0x41

// Definições dos LEDs
#define LED_BMP 23
#define LED_GPS 19

Adafruit_BMP280 bmp;
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

bool bmpAvailable = false;
bool mpuAvailable = false;

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float tempC;
float bmpTemp, altitude;

int GPS_RX = 34;
int GPS_TX = 12;

// Funções I2C
void i2cWrite(uint8_t address, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

int16_t i2cRead(uint8_t address, int8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return 0;
  }
  Wire.requestFrom(address, 2, true);
  if (Wire.available() >= 2) {
    return (Wire.read() << 8) | Wire.read();
  }
  return 0;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Configura LEDs
  pinMode(LED_BMP, OUTPUT);
  pinMode(LED_GPS, OUTPUT);
  digitalWrite(LED_BMP, LOW);
  digitalWrite(LED_GPS, LOW);

  // Inicializa BMP280
  bmpAvailable = bmp.begin(0x76);
  if (bmpAvailable) {
    Serial.println("BMP280 inicializado");
    digitalWrite(LED_BMP, HIGH);  // Liga LED do BMP se ok
  } else {
    Serial.println("BMP280 nao detectado");
    digitalWrite(LED_BMP, LOW);
  }

  // Inicializa GPS UART
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS iniciado");

  // Verifica comunicação GPS (mesmo sem fix)
  delay(1000);
  if (gpsSerial.available()) {
    Serial.println("GPS comunicando");
    digitalWrite(LED_GPS, HIGH);  // Liga LED se houver comunicação
  } else {
    Serial.println("GPS sem comunicação");
    digitalWrite(LED_GPS, LOW);
  }

  // Verifica MPU9250
  Wire.beginTransmission(MPU);
  if (Wire.endTransmission() == 0) {
    mpuAvailable = true;
    Serial.println("MPU9250 detectado");
    i2cWrite(MPU, 0x6B, 0); // Liga o sensor
  } else {
    Serial.println("MPU9250 nao detectado");
  }

  // Inicializa LoRa
  SPI.begin(CONFIG_CLK, CONFIG_MISO, CONFIG_MOSI, CONFIG_NSS);
  LoRa.setPins(CONFIG_NSS, CONFIG_RST, CONFIG_DIO0);
  if (!LoRa.begin(BAND)) {
    Serial.println("Falha na inicializacao do LoRa");
    while (1);
  }
  Serial.println("LoRa iniciado");
}

void loop() {
  // Lê GPS
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
    digitalWrite(LED_GPS, HIGH);  // Mantém LED aceso se há dados chegando
  }

  // Lê sensores MPU9250
  if (mpuAvailable) {
    int16_t rawTemp = i2cRead(MPU, TEMP_H);
    tempC = (rawTemp / 340.0) + 36.53;

    accX = i2cRead(MPU, accX_H) / 16384.0f;
    accY = i2cRead(MPU, accY_H) / 16384.0f;
    accZ = i2cRead(MPU, accZ_H) / 16384.0f;

    gyroX = i2cRead(MPU, gyroX_H) / 131.0f;
    gyroY = i2cRead(MPU, gyroY_H) / 131.0f;
    gyroZ = i2cRead(MPU, gyroZ_H) / 131.0f;
  } else {
    tempC = accX = accY = accZ = gyroX = gyroY = gyroZ = 0;
  }

  // Lê BMP280
  if (bmpAvailable) {
    bmpTemp = bmp.readTemperature();
    altitude = bmp.readAltitude(1013.25);
    digitalWrite(LED_BMP, HIGH);
  } else {
    bmpTemp = 0;
    altitude = 0;
    digitalWrite(LED_BMP, LOW);
  }

  // Exibe no Monitor Serial
  Serial.printf("Temperatura MPU: %.2f °C, Temperatura BMP: %.2f °C, Altitude: %.2f m\n", tempC, bmpTemp, altitude);
  Serial.printf("Accel (g): X=%.3f Y=%.3f Z=%.3f\n", accX, accY, accZ);
  Serial.printf("Gyro (deg/s): X=%.3f Y=%.3f Z=%.3f\n", gyroX, gyroY, gyroZ);

  // Prepara dados GPS
  double lat = 0.0, lon = 0.0;
  int dia = 0, mes = 0, ano = 0, hora = 0, minuto = 0, segundo = 0;
  bool gpsValido = gps.location.isValid();

  if (gpsValido) {
    lat = gps.location.lat();
    lon = gps.location.lng();
    dia = gps.date.day();
    mes = gps.date.month();
    ano = gps.date.year();
    hora = gps.time.hour();
    minuto = gps.time.minute();
    segundo = gps.time.second();

    Serial.printf("GPS Lat: %.6f, Lon: %.6f, Data: %02d/%02d/%04d, Hora: %02d:%02d:%02d\n",
                  lat, lon, dia, mes, ano, hora, minuto, segundo);
  } else {
    Serial.println("GPS inválido — enviando dados sem coordenadas.");
  }

  // Monta mensagem LoRa
  String mensagem = "TempMPU:" + String(tempC, 2) +
                    ",TempBMP:" + String(bmpTemp, 2) +
                    ",Alt:" + String(altitude, 2) +
                    ",AccX:" + String(accX, 3) +
                    ",AccY:" + String(accY, 3) +
                    ",AccZ:" + String(accZ, 3) +
                    ",GyroX:" + String(gyroX, 3) +
                    ",GyroY:" + String(gyroY, 3) +
                    ",GyroZ:" + String(gyroZ, 3);

  if (gpsValido) {
    mensagem += ",Lat:" + String(lat, 6) +
                ",Lon:" + String(lon, 6) +
                ",Date:" + String(dia) + "/" + String(mes) + "/" + String(ano) +
                ",Time:" + String(hora) + ":" + String(minuto) + ":" + String(segundo);
  } else {
    mensagem += ",Lat:0.000000,Lon:0.000000,Date:N/A,Time:N/A";
  }

  // Envia via LoRa
  LoRa.beginPacket();
  LoRa.print(mensagem);
  LoRa.endPacket();

  Serial.println("Dados enviados via LoRa:");
  Serial.println(mensagem);
  Serial.println("------------------------------");

  delay(500); // menor delay de envio
}
