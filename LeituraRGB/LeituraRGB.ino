#include <Wire.h>
#include "Adafruit_TCS34725.h"

#define TCAADDR 0x70  // Endereço do multiplexador TCA9548A

// Definição dos canais dos sensores RGB
#define RGB1_CHANNEL 0  // Sensor RGB 1
#define RGB2_CHANNEL 1  // Sensor RGB 2

// Instanciação dos sensores
Adafruit_TCS34725 sensor1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 sensor2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X);

// Função para selecionar canal do TCA9548A
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(1);
}

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // Inicializa sensor 1
  tcaSelect(RGB1_CHANNEL);
  if (!sensor1.begin()) {
    Serial.println("Sensor RGB 1 não encontrado!");
  } else {
    Serial.println("Sensor RGB 1 iniciado!");
  }

  // Inicializa sensor 2
  tcaSelect(RGB2_CHANNEL);
  if (!sensor2.begin()) {
    Serial.println("Sensor RGB 2 não encontrado!");
  } else {
    Serial.println("Sensor RGB 2 iniciado!");
  }
}

void loop() {
  uint16_t r1, g1, b1, c1;
  uint16_t r2, g2, b2, c2;

  // Leitura do sensor 1
  tcaSelect(RGB1_CHANNEL);
  sensor1.getRawData(&r1, &g1, &b1, &c1);
  Serial.print("Sensor 1 -> R: "); Serial.print(r1);
  Serial.print(" G: "); Serial.print(g1);
  Serial.print(" B: "); Serial.print(b1);
  Serial.print(" C: "); Serial.println(c1);

  // Leitura do sensor 2
  tcaSelect(RGB2_CHANNEL);
  sensor2.getRawData(&r2, &g2, &b2, &c2);
  Serial.print("Sensor 2 -> R: "); Serial.print(r2);
  Serial.print(" G: "); Serial.print(g2);
  Serial.print(" B: "); Serial.print(b2);
  Serial.print(" C: "); Serial.println(c2);

  Serial.println("-----------------------------------");
  delay(500);
}
