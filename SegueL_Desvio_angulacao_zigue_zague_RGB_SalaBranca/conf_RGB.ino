#include <Wire.h>
#include "Adafruit_TCS34725.h"

#define TCAADDR 0x70

#define RGB1_CHANNEL 0
#define RGB2_CHANNEL 1

Adafruit_TCS34725* sensor1 = nullptr;
Adafruit_TCS34725* sensor2 = nullptr;

// =============================================
// CALIBRAÇÃO - coloque aqui os valores lidos no DEBUG
// em cima de cada cor
// =============================================

// Limiares de branco ajustados 
const uint16_t LIMIAR_BRANCO_S1 = 23000;
const uint16_t LIMIAR_BRANCO_S2 = 34000;

// Limiares de prata (claridade é MAIOR que a do branco)
// >>> PREENCHER com o valor de "C" lido no DEBUG em cima da fita prata
uint16_t LIMIAR_PRATA_S1 = 40000;
uint16_t LIMIAR_PRATA_S2 = 50000;

// Sensor 1 - verde
const uint16_t S1_VERDE_R = 540;
const uint16_t S1_VERDE_G = 1300;
const uint16_t S1_VERDE_B = 900;

// Sensor 2 - verde  
const uint16_t S2_VERDE_R = 710;
const uint16_t S2_VERDE_G = 1700;
const uint16_t S2_VERDE_B = 1060;

// Sensor 2 - cor vermelha
const uint16_t S2_VERM_R = 3900 ;
const uint16_t S2_VERM_G = 1000 ;
const uint16_t S2_VERM_B = 800 ;
// Claridade no vermelho 5560

// Sensor 1 - cor vermelha
const uint16_t S1_VERM_R = 2900;
const uint16_t S1_VERM_G = 770 ;
const uint16_t S1_VERM_B = 650 ; 
// Claridade no vermelho 3790

// Tolerâncias
const uint16_t TOLERANCIA_S1 = 300;
const uint16_t TOLERANCIA_S2 = 400;

// =============================================

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  if (Wire.endTransmission() != 0) {
    Serial.print("Erro ao selecionar canal: ");
    Serial.println(channel);
  }
  delay(1);
}

bool dentroDaFaixa(uint16_t valor, uint16_t referencia, uint16_t tolerancia) {
  return valor >= (referencia > tolerancia ? referencia - tolerancia : 0) &&
         valor <= referencia + tolerancia;
}

void defineRGBDi(uint16_t r1, uint16_t g1, uint16_t b1, uint16_t c1) {
  RGB1_prata    = (c1 > LIMIAR_PRATA_S1);
  RGB1_branco   = !RGB1_prata && (c1 > LIMIAR_BRANCO_S1);
  RGB1_verde    = !RGB1_prata && !RGB1_branco &&
                  dentroDaFaixa(r1, S1_VERDE_R, TOLERANCIA_S1) &&
                  dentroDaFaixa(g1, S1_VERDE_G, TOLERANCIA_S1) &&
                  dentroDaFaixa(b1, S1_VERDE_B, TOLERANCIA_S1);
  RGB1_vermelho = !RGB1_prata && !RGB1_branco && !RGB1_verde &&
                  dentroDaFaixa(r1, S1_VERM_R, TOLERANCIA_S1) &&
                  dentroDaFaixa(g1, S1_VERM_G, TOLERANCIA_S1) &&
                  dentroDaFaixa(b1, S1_VERM_B, TOLERANCIA_S1);
}

void defineRGBEs(uint16_t r2, uint16_t g2, uint16_t b2, uint16_t c2) {
  RGB2_prata    = (c2 > LIMIAR_PRATA_S2);
  RGB2_branco   = !RGB2_prata && (c2 > LIMIAR_BRANCO_S2);
  RGB2_verde    = !RGB2_prata && !RGB2_branco &&
                  dentroDaFaixa(r2, S2_VERDE_R, TOLERANCIA_S2) &&
                  dentroDaFaixa(g2, S2_VERDE_G, TOLERANCIA_S2) &&
                  dentroDaFaixa(b2, S2_VERDE_B, TOLERANCIA_S2);
  RGB2_vermelho = !RGB2_prata && !RGB2_branco && !RGB2_verde &&
                  dentroDaFaixa(r2, S2_VERM_R, TOLERANCIA_S2) &&
                  dentroDaFaixa(g2, S2_VERM_G, TOLERANCIA_S2) &&
                  dentroDaFaixa(b2, S2_VERM_B, TOLERANCIA_S2);
}

String RGB_DIREITO() {
  if (RGB1_prata) return "prata";
  if (RGB1_branco) return "branco";
  if (RGB1_verde) return "verde";
  if (RGB1_vermelho) return "vermelho";
  return "indefinido";
}

String RGB_ESQUERDO() {
  if (RGB2_prata) return "prata";
  if (RGB2_branco) return "branco";
  if (RGB2_verde) return "verde";
  if (RGB2_vermelho) return "vermelho";
  return "indefinido";
}


// TCS34725_INTEGRATIONTIME_600MS — isso significa que o sensor leva 600ms para fazer UMA leitura.
void RGBsetup() {
  tcaSelect(RGB1_CHANNEL);
  sensor1 = new Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X);
  if (!sensor1->begin()) {
    Serial.println("Sensor RGB 1 não encontrado!");
  } else {
    Serial.println("Sensor RGB 1 iniciado!");
  }

  tcaSelect(RGB2_CHANNEL);
  sensor2 = new Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X);
  if (!sensor2->begin()) {
    Serial.println("Sensor RGB 2 não encontrado!");
  } else {
    Serial.println("Sensor RGB 2 iniciado!");
  }
}

void RGBloop() {
  tcaSelect(RGB1_CHANNEL);
  sensor1->getRawData(&r1, &g1, &b1, &c1);
  defineRGBDi(r1, g1, b1, c1);

  tcaSelect(RGB2_CHANNEL);
  sensor2->getRawData(&r2, &g2, &b2, &c2);
  defineRGBEs(r2, g2, b2, c2);

  // DEBUG - descomente para calibrar (use para achar LIMIAR_PRATA_S1/S2)
  // Serial.print("S1 R:"); Serial.print(r1);
  // Serial.print(" G:"); Serial.print(g1);
  // Serial.print(" B:"); Serial.print(b1);
  // Serial.print(" C:"); Serial.print(c1);
  // Serial.print(" | branco:"); Serial.print(RGB1_branco);
  // Serial.print(" verde:"); Serial.print(RGB1_verde);
  // Serial.print(" verm:"); Serial.print(RGB1_vermelho);
  // Serial.print(" prata:"); Serial.println(RGB1_prata);

  // Serial.print("S2 R:"); Serial.print(r2);
  // Serial.print(" G:"); Serial.print(g2);
  // Serial.print(" B:"); Serial.print(b2);
  // Serial.print(" C:"); Serial.print(c2);
  // Serial.print(" | branco:"); Serial.print(RGB2_branco);
  // Serial.print(" verde:"); Serial.print(RGB2_verde);
  // Serial.print(" verm:"); Serial.print(RGB2_vermelho);
  // Serial.print(" prata:"); Serial.println(RGB2_prata);
}


// Avança levemente e faz múltiplas leituras para detectar verde com mais confiança
void verificarVerde(bool &verdeDir, bool &verdeEsq) {
  verdeDir = false;
  verdeEsq = false;

  // Avança ~1 cm para garantir que os sensores RGB estejam sobre a marca
  andarFrente(80, 80);
  delay(280);  // ajuste fino: ~1 cm a velocidade 80
  pararMotores();
  delay(100);

  // Faz 4 leituras com votação por maioria
  int contDir = 0, contEsq = 0;
  for (int i = 0; i < 4; i++) {
    RGBloop();
    if (RGB1_verde) contDir++;
    if (RGB2_verde) contEsq++;
    delay(650);  // tempo de integração do TCS34725 em 600MS
  }

  // Confirma verde se apareceu em pelo menos 2 das 4 leituras
  verdeDir = (contDir >= 2);
  verdeEsq = (contEsq >= 2);
}

// Leitura RÁPIDA (1 amostra só) - usada dentro do loop principal, que roda
// o tempo todo enquanto o robô segue linha. NÃO usar a versão com votação
// aqui, pois ela leva ~2,6s bloqueado e travaria o seguidor de linha.
bool detectouFitaPrata() {
  RGBloop();
  return (RGB1_prata || RGB2_prata);
}

// Confirmação com votação por maioria (4 leituras), no mesmo padrão do
// verificarVerde(). Só chamar DEPOIS de já ter parado o robô, quando a
// leitura rápida acima já deu positivo uma vez.
bool confirmarFitaPrata() {
  int contPrata = 0;
  for (int i = 0; i < 4; i++) {
    RGBloop();
    if (RGB1_prata || RGB2_prata) contPrata++;
    delay(650);
  }
  return (contPrata >= 2);
}