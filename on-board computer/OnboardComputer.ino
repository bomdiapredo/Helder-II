#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// ======= DEFINIÇÕES DE PINOS =======
#define SKIB 26           // Renomeado de LED_PIN
#define BUZZER_PIN 12
#define SD_CS_PIN 5
#define MPU_ADDR 0x68

// ======= LEDS DE STATUS =======
#define LED_VERDE 15      // Ativa quando SD e BMP estão OK
#define LED_AMARELO 4     // Ativa quando SD estiver funcionando
#define LED_VERMELHO 2    // Ativa quando SD falhar

// ======= REGISTRADORES DO MPU9250 =======
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47
#define TEMP_OUT_H   0x41

#define GYRO_CONFIG_MODE 0
#define ACC_CONFIG_MODE  1

// ======= OBJETOS =======
Adafruit_BMP280 bmp;

// ======= VARIÁVEIS =======
float accX, accY, accZ, gyroX, gyroY, gyroZ, tempC;
float referencePressure = 0;
float referenceAltitude = 0;
bool firstReading = true;
float altitudeVariation;

// ======= ESTADO DOS SENSORES =======
bool sdOk = false;
bool bmpOk = false;

// ======= PROTÓTIPOS =======
void initMPU9250();
void i2cWrite(uint8_t address, uint8_t reg, uint8_t val);
int16_t i2cRead(uint8_t address, uint8_t reg);
void readAccel();
void readGyro();
void readTemperature();
void readBMP280();
void writeFile(fs::FS &fs, const char * path, const char * message);

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(SKIB, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_VERDE, OUTPUT);
  pinMode(LED_AMARELO, OUTPUT);
  pinMode(LED_VERMELHO, OUTPUT);

  Serial.println("Inicializando sensores...");

  // ======= BMP280 =======
  if (!bmp.begin(0x76)) {
    Serial.println("❌ Erro ao inicializar o BMP280!");
    bmpOk = false;
    digitalWrite(LED_VERDE, LOW);
  } else {
    bmpOk = true;
    Serial.println("✅ BMP280 inicializado!");
  }

  // ======= SD CARD =======
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("❌ Falha ao montar o cartão SD!");
    sdOk = false;
    digitalWrite(LED_AMARELO, LOW);
    digitalWrite(LED_VERMELHO, HIGH); // Erro → vermelho aceso
  } else {
    sdOk = true;
    digitalWrite(LED_AMARELO, HIGH);  // SD OK → amarelo aceso
    digitalWrite(LED_VERMELHO, LOW);
    Serial.println("✅ Cartão SD montado com sucesso.");
  }

  // ======= LED VERDE (BMP + SD OK) =======
  if (sdOk && bmpOk) {
    digitalWrite(LED_VERMELHO, HIGH);
  } else {
    digitalWrite(LED_VERMELHO, LOW);
  }

  // ======= MPU9250 =======
  initMPU9250();

  delay(2000);
  if (sdOk) {
    writeFile(SD, "/log.txt", "Iniciando a leitura dos sensores...\n");
  }
}

void loop() {
  readAccel();
  readGyro();
  readTemperature();
  readBMP280();

  // ======= DETECÇÃO DE QUEDA =======
  if (altitudeVariation >= 2.0) {
    digitalWrite(SKIB, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
    Serial.println("⚠️ QUEDA DETECTADA!");
  } else {
    digitalWrite(SKIB, LOW);
    digitalWrite(BUZZER_PIN, LOW);
  }

  // ======= EXIBIÇÃO NO SERIAL =======
  Serial.print("Acelerômetro X: "); Serial.print(accX);
  Serial.print(" | Y: "); Serial.print(accY);
  Serial.print(" | Z: "); Serial.println(accZ);

  Serial.print("Giroscópio X: "); Serial.print(gyroX);
  Serial.print(" | Y: "); Serial.print(gyroY);
  Serial.print(" | Z: "); Serial.println(gyroZ);

  Serial.print("Temperatura MPU: "); Serial.print(tempC); Serial.println(" °C");

  // ======= GRAVAÇÃO NO CARTÃO SD =======
  if (sdOk) {
    String data = "Acelerômetro X: " + String(accX) + " Y: " + String(accY) + " Z: " + String(accZ) + "\n";
    data += "Giroscópio X: " + String(gyroX) + " Y: " + String(gyroY) + " Z: " + String(gyroZ) + "\n";
    data += "Temperatura MPU: " + String(tempC) + " °C\n";
    data += "Altitude relativa: " + String(altitudeVariation) + " m\n\n";
    writeFile(SD, "/log.txt", data.c_str());
  }

  // ======= STATUS DOS LEDS =======
  if (sdOk && bmpOk) {
    digitalWrite(LED_VERDE, HIGH);
  } else {
    digitalWrite(LED_VERDE, LOW);
  }

  if (sdOk) {
    digitalWrite(LED_AMARELO, HIGH);
    digitalWrite(LED_VERMELHO, LOW);
  } else {
    digitalWrite(LED_AMARELO, LOW);
    digitalWrite(LED_VERMELHO, HIGH);
  }

  delay(1000);
}

// ======================================================
// ======= FUNÇÕES AUXILIARES ===========================
// ======================================================

void initMPU9250() {
  i2cWrite(MPU_ADDR, 0x6B, 0x00);
  delay(100);
  i2cWrite(MPU_ADDR, 0x1B, GYRO_CONFIG_MODE);
  i2cWrite(MPU_ADDR, 0x1C, ACC_CONFIG_MODE);
}

void i2cWrite(uint8_t address, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

int16_t i2cRead(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t)2, true);
  return (Wire.read() << 8 | Wire.read());
}

void readAccel() {
  accX = i2cRead(MPU_ADDR, ACCEL_XOUT_H) / 16384.0;
  accY = i2cRead(MPU_ADDR, ACCEL_YOUT_H) / 16384.0;
  accZ = i2cRead(MPU_ADDR, ACCEL_ZOUT_H) / 16384.0;
}

void readGyro() {
  gyroX = i2cRead(MPU_ADDR, GYRO_XOUT_H) / 131.0;
  gyroY = i2cRead(MPU_ADDR, GYRO_YOUT_H) / 131.0;
  gyroZ = i2cRead(MPU_ADDR, GYRO_ZOUT_H) / 131.0;
}

void readTemperature() {
  int16_t rawTemp = i2cRead(MPU_ADDR, TEMP_OUT_H);
  tempC = (rawTemp / 340.0) + 16.53;
}

void readBMP280() {
  float currentPressure = bmp.readPressure() / 100.0F;
  float currentTemp = bmp.readTemperature();

  if (firstReading) {
    referencePressure = currentPressure;
    referenceAltitude = bmp.readAltitude(1013.25);
    firstReading = false;
    Serial.println("Ponto de referência definido!");
  }

  altitudeVariation = 44330.0 * (1.0 - pow(currentPressure / referencePressure, 0.1903));

  Serial.print("Pressão atual: ");
  Serial.print(currentPressure);
  Serial.print(" hPa | Altitude relativa: ");
  Serial.print(altitudeVariation);
  Serial.print(" m | Temp BMP: ");
  Serial.print(currentTemp);
  Serial.println(" °C");
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Falha ao abrir arquivo para escrita!");
    return;
  }
  if (file.print(message)) {
    Serial.println("Dados salvos no SD.");
  } else {
    Serial.println("Falha ao salvar dados.");
  }
  file.close();
}
