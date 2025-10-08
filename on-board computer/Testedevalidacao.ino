#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

#define SD_CS_PIN 5
#define LED_PIN 13
#define BUZZER_PIN 12

Adafruit_BMP280 bmp;

float referencePressure = 0;
float lastAltitude = 0;
bool firstReading = true;

unsigned long lastCheckTime = 0;
const unsigned long checkInterval = 500; // 500 ms
const float quedaThreshold = 1.0; // Queda maior que 1 metro

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  if (!bmp.begin(0x76)) {
    Serial.println("Erro ao inicializar o BMP280!");
    while (1);
  }

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Falha ao montar o cartão SD!");
    return;
  }

  Serial.println("Cartão SD montado.");
  writeFile(SD, "/log.txt", "Iniciando monitoramento de queda com barômetro...\n");

  delay(2000); // Tempo para estabilização do sensor

  // Define ponto de referência
  referencePressure = bmp.readPressure() / 100.0F; // em hPa
  lastAltitude = bmp.readAltitude(referencePressure);
  firstReading = false;
  Serial.print("Altitude inicial: ");
  Serial.print(lastAltitude);
  Serial.println(" m");
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastCheckTime >= checkInterval) {
    float currentPressure = bmp.readPressure() / 100.0F; // em hPa
    float currentAltitude = bmp.readAltitude(referencePressure);
    float deltaAltitude = lastAltitude - currentAltitude; // diferença positiva = queda

    Serial.print("Altitude atual: ");
    Serial.print(currentAltitude);
    Serial.print(" m | Diferença: ");
    Serial.print(deltaAltitude);
    Serial.println(" m");

    if (deltaAltitude > quedaThreshold) {
      digitalWrite(LED_PIN, HIGH);
      digitalWrite(BUZZER_PIN, HIGH);
      Serial.println("QUEDA DETECTADA!");

      writeFile(SD, "/log.txt", "QUEDA DETECTADA!\n");
    } else {
      digitalWrite(LED_PIN, LOW);
      ;
    }

    lastAltitude = currentAltitude;
    lastCheckTime = currentTime;
  }

  delay(50); // Pequeno atraso para evitar uso excessivo da CPU
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Escrevendo arquivo: %s\n", path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Falha ao abrir o arquivo para escrita");
    return;
  }
  if (file.print(message)) {
    Serial.println("Arquivo escrito");
  } else {
    Serial.println("Falha ao escrever no arquivo");
  }
  file.close();
}
