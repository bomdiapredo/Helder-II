#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define LED_PIN 26
#define BUZZER_PIN 12

Adafruit_BMP280 bmp;

float referencePressure = 0;
float referenceAltitude = 0;
bool firstReading = true;
float altitudeVariation;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  if (!bmp.begin(0x76)) {
    Serial.println("Erro ao inicializar o BMP280!");
    while (1);
  }

  Serial.println("Sensor BMP280 inicializado.");
  delay(2000);
}

void loop() {
  readBMP280();

  // Se a variação de altitude for maior que 2m em relação ao ponto inicial → queda detectada
  if (altitudeVariation >= 2.0) {
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
    Serial.println("⚠️ Queda detectada!");
  } else {
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
  }

  delay(1000);
}

void readBMP280() {
  float currentPressure = bmp.readPressure() / 100.0F; // Pressão em hPa
  float currentTemp = bmp.readTemperature();

  if(firstReading) {
    referencePressure = currentPressure;
    referenceAltitude = bmp.readAltitude(1013.25);
    firstReading = false;
    Serial.println("Ponto de referência definido!");
  }

  // Altitude relativa ao ponto inicial
  altitudeVariation = 44330.0 * (1.0 - pow(currentPressure / referencePressure, 0.1903));

  Serial.print("Pressão atual: ");
  Serial.print(currentPressure);
  Serial.print(" hPa, Altitude relativa: ");
  Serial.print(altitudeVariation);
  Serial.print(" m, Temperatura: ");
  Serial.print(currentTemp);
  Serial.println(" °C");
}
