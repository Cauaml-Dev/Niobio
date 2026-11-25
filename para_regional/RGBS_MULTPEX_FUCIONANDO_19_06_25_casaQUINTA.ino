//#include <Wire.h>
//#include "Adafruit_TCS34725.h"

//#define TCAADDR 0x70  // Endereço do multiplexador TCA9548A

// Definição dos canais dos sensores RGB:
//#define RGB1_CHANNEL 0  // Sensor RGB 1 (lado esquerdo)
//#define RGB2_CHANNEL 1  // Sensor RGB 2 (lado direito)

// Instancia os sensores com parâmetros de integração e ganho desejados
//Adafruit_TCS34725 sensor1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X);
//Adafruit_TCS34725 sensor2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X);

//bool RGB1_verde = false;
//bool RGB1_vermelho = false;
//bool RGB1_branco = false;

//bool RGB2_verde = false;
//bool RGB2_vermelho = false;
//bool RGB2_branco = false;


//uint16_t r1, g1, b1, c1;
//uint16_t r2, g2, b2, c2;


// Função para selecionar o canal do multiplexador TCA9548A
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);  // Ativa o canal desejado (bit correspondente)
  if (Wire.endTransmission() != 0) {
    Serial.print("Erro ao selecionar canal: ");
    Serial.println(channel);
  }
  delay(1);  // Delay para estabilizar a troca de canal
}

void defineRGBDi(uint16_t r1, uint16_t g1, uint16_t b1, uint16_t c1) {

  if (c1 > 10000) {
    RGB1_branco = true;
  } else {
    RGB1_branco = false;
  }


  if ((g1 > b1) && (g1 > r1) && (c1 < 10000)) {
    RGB1_verde = true;
  } else {
    RGB1_verde = false;
  }


  if ((r1 > b1) && (r1 > g1) && (c1 < 10000)) {
    RGB1_vermelho = true;
  } else {
    RGB1_vermelho = false;
  }
}

void defineRGBEs(uint16_t r2, uint16_t g2, uint16_t b2, uint16_t c2) {

  if (c2 > 20000) {
    RGB2_branco = true;
  } else {
    RGB2_branco = false;
  }


  if ((g2 > b2) && (g2 > r2) && (c2 < 20000)) {
    RGB2_verde = true;
  } else {
    RGB2_verde = false;
  }


  if ((r2 > b2) && (r2 > g2) && (c2 < 20000)) {
    RGB2_vermelho = true;
  } else {
    RGB2_vermelho = false;
  }
}

String RGB_DIREITO() {
  if (RGB1_branco == true) {
    return "branco";
  }
  if (RGB1_verde == true) {
    return "verde";
  }
  if (RGB1_vermelho == true) {
    return "vermelho";
  } else {
    return "indefinido";
  }
}

String RGB_ESQUERDO() {
  if (RGB2_branco == true) {
    return "branco";
  }
  if (RGB2_verde == true) {
    return "verde";
  }
  if (RGB2_vermelho == true) {
    return "vermelho";
  } else {
    return "indefinido";
  }
}

void RGBsetup() {
  Wire.begin();
  Serial.begin(9600);

  // Inicializa o sensor 1 (lado esquerdo)
  tcaSelect(RGB1_CHANNEL);
  if (!sensor1.begin()) {
    Serial.println("Sensor RGB 1 não encontrado!");
  } else {
    Serial.println("Sensor RGB 1 iniciado!");
  }

  // Inicializa o sensor 2 (lado direito)
  tcaSelect(RGB2_CHANNEL);
  if (!sensor2.begin()) {
    Serial.println("Sensor RGB 2 não encontrado!");
  } else {
    Serial.println("Sensor RGB 2 iniciado!");
  }
}

void RGBloop() {

  // Leitura do sensor RGB 1 (lado direito)
  tcaSelect(RGB1_CHANNEL);
  sensor1.getRawData(&r1, &g1, &b1, &c1);
  defineRGBDi(r1, g1, b1, c1);


  // Leitura do sensor RGB 2 (lado esquerdo)
  tcaSelect(RGB2_CHANNEL);
  sensor2.getRawData(&r2, &g2, &b2, &c2);
  defineRGBEs(r2, g2, b2, c2);
/*
  //String corRGB1 = RGB_DIREITO();
  //String corRGB2 = RGB_ESQUERDO();

  //Serial.print("Cor detectada no sensor D: ");
  Serial.println(corRGB1);
  Serial.print("Cor detectada no sensor E: ");
  Serial.println(corRGB2);
  Serial.println("/////////////////////////");


  // Impressão dos valores brutos dos dois sensores
  //Serial.print("Sensor 1 -> R: "); Serial.print(r1);
  //Serial.print(" | G: "); Serial.print(g1);
  //Serial.print(" | B: "); Serial.print(b1);
  //Serial.print(" | C: "); Serial.println(c1);
  //Serial.print("Sensor 2 -> R: "); Serial.print(r2);
  //Serial.print(" | G: "); Serial.print(g2);
  //Serial.print(" | B: "); Serial.print(b2);
  //Serial.print(" | C: "); Serial.println(c2);
  //Serial.println("/////////////////////////////////////////////");

 // delay(10);  // Tempo de leitura ajustável
 */
 return;
}
