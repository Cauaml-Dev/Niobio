//#include <Wire.h>
//#include "Adafruit_TCS34725.h"

// Endereço do multiplexador TCA9548A
//#define TCAADDR 0x70  

// Definição dos canais dos sensores RGB
//#define RGB1_CHANNEL 0  // Sensor RGB 1 (lado direito)
//#define RGB2_CHANNEL 1  // Sensor RGB 2 (lado esquerdo)

// Instanciação dos sensores com tempo de integração e ganho
//Adafruit_TCS34725 sensor1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X);
//Adafruit_TCS34725 sensor2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X);

// Limites de claridade (thresholds) para cada sensor
const uint16_t LIMIAR_CLARIDADE_RGB1 = 10000;  // Sensor direito
const uint16_t LIMIAR_CLARIDADE_RGB2 = 20000;  // Sensor esquerdo

// Flags de cor detectada pelo sensor direito (RGB1)
bool RGB1_verde = false;
bool RGB1_vermelho = false;
bool RGB1_branco = false;

// Flags de cor detectada pelo sensor esquerdo (RGB2)
bool RGB2_verde = false;
bool RGB2_vermelho = false;
bool RGB2_branco = false;

// Variáveis para armazenar valores brutos dos sensores
uint16_t r1, g1, b1, c1;  // Sensor direito
uint16_t r2, g2, b2, c2;  // Sensor esquerdo

//---------------------------------------------
// Seleciona o canal ativo no multiplexador TCA9548A
//---------------------------------------------
void tcaSelect(uint8_t channel) {
  if (channel > 7) return; // Proteção contra canal inválido
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);  // Ativa o bit correspondente ao canal
  if (Wire.endTransmission() != 0) {
    Serial.print("Erro ao selecionar canal: ");
    Serial.println(channel);
  }
  delay(1);  // Pequeno delay para estabilizar a troca de canal
}

//---------------------------------------------
// Interpretação das cores no sensor direito (RGB1)
//---------------------------------------------
void defineRGBDi(uint16_t r1, uint16_t g1, uint16_t b1, uint16_t c1) {
  // Branco: alta intensidade de luz
  RGB1_branco = (c1 > LIMIAR_CLARIDADE_RGB1);

  // Verde: valor de G maior que R e B, com luminosidade moderada
  RGB1_verde = ((g1 > b1) && (g1 > r1) && (c1 < LIMIAR_CLARIDADE_RGB1));

  // Vermelho: valor de R maior que G e B, com luminosidade moderada
  RGB1_vermelho = ((r1 > b1) && (r1 > g1) && (c1 < LIMIAR_CLARIDADE_RGB1));
}

//---------------------------------------------
// Interpretação das cores no sensor esquerdo (RGB2)
//---------------------------------------------
void defineRGBEs(uint16_t r2, uint16_t g2, uint16_t b2, uint16_t c2) {
  // Branco: alta intensidade de luz
  RGB2_branco = (c2 > LIMIAR_CLARIDADE_RGB2);

  // Verde: valor de G maior que R e B, com luminosidade moderada
  RGB2_verde = ((g2 > b2) && (g2 > r2) && (c2 < LIMIAR_CLARIDADE_RGB2));

  // Vermelho: valor de R maior que G e B, com luminosidade moderada
  RGB2_vermelho = ((r2 > b2) && (r2 > g2) && (c2 < LIMIAR_CLARIDADE_RGB2));
}

//---------------------------------------------
// Retorna a cor interpretada pelo sensor direito
//---------------------------------------------
String RGB_DIREITO() {
  if (RGB1_branco) return "branco";
  if (RGB1_verde) return "verde";
  if (RGB1_vermelho) return "vermelho";
  return "indefinido";
}

//---------------------------------------------
// Retorna a cor interpretada pelo sensor esquerdo
//---------------------------------------------
String RGB_ESQUERDO() {
  if (RGB2_branco) return "branco";
  if (RGB2_verde) return "verde";
  if (RGB2_vermelho) return "vermelho";
  return "indefinido";
}

//---------------------------------------------
// Setup inicial dos sensores RGB
//---------------------------------------------
void RGBsetup() {
  Wire.begin();
  Serial.begin(9600);

  // Inicializa o sensor direito (canal 0)
  tcaSelect(RGB1_CHANNEL);
  if (!sensor1.begin()) {
    Serial.println("Sensor RGB 1 não encontrado!");
  } else {
    Serial.println("Sensor RGB 1 iniciado!");
  }

  // Inicializa o sensor esquerdo (canal 1)
  tcaSelect(RGB2_CHANNEL);
  if (!sensor2.begin()) {
    Serial.println("Sensor RGB 2 não encontrado!");
  } else {
    Serial.println("Sensor RGB 2 iniciado!");
  }
}

//---------------------------------------------
// Loop de leitura dos sensores RGB
//---------------------------------------------
void RGBloop() {
  // Leitura do sensor direito
  tcaSelect(RGB1_CHANNEL);
  sensor1.getRawData(&r1, &g1, &b1, &c1);
  defineRGBDi(r1, g1, b1, c1);

  // Leitura do sensor esquerdo
  tcaSelect(RGB2_CHANNEL);
  sensor2.getRawData(&r2, &g2, &b2, &c2);
  defineRGBEs(r2, g2, b2, c2);

  // Debug opcional (comentado para não poluir Serial)
  /*
  String corRGB1 = RGB_DIREITO();
  String corRGB2 = RGB_ESQUERDO();

  Serial.print("Cor detectada no sensor D: ");
  Serial.println(corRGB1);
  Serial.print("Cor detectada no sensor E: ");
  Serial.println(corRGB2);
  Serial.println("/////////////////////////");

  Serial.print("Sensor 1 -> R: "); Serial.print(r1);
  Serial.print(" | G: "); Serial.print(g1);
  Serial.print(" | B: "); Serial.print(b1);
  Serial.print(" | C: "); Serial.println(c1);

  Serial.print("Sensor 2 -> R: "); Serial.print(r2);
  Serial.print(" | G: "); Serial.print(g2);
  Serial.print(" | B: "); Serial.print(b2);
  Serial.print(" | C: "); Serial.println(c2);
  Serial.println("/////////////////////////////////////////////");
  */
  
  return;
}
