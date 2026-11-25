////////////////////
/// BIBLIOTECAS  ///
////////////////////
#include <AFMotor.h>
#include <Wire.h>
#include <Ultrasonic.h>
#include "Adafruit_TCS34725.h"

bool RGB1_verde = false;
bool RGB1_vermelho = false;
bool RGB1_branco = false;

bool RGB2_verde = false;
bool RGB2_vermelho = false;
bool RGB2_branco = false;

bool RGB1_prata = false;
bool RGB2_prata = false;

uint16_t r1, g1, b1, c1;
uint16_t r2, g2, b2, c2;

////////////////////
/// SENSOR : QTR ///
////////////////////
int sensores[] = { A15, A14, A13, A12, A11, A10, A9, A8 };
int estadoSensores[8];
int pesos[] = { -3500, -2500, -1500, -500, 500, 1500, 2500, 3500 };

int limiarmin = 250;
int limiarmax = 550;

float Kp = 1.35;
float Kd = 6.13;
float erro = 0;
float erroAnterior = 0;
float correcao = 0;
int offset = 0;

bool TudoPreto = false;
bool TudoBranco = false;
bool Na_90ESQUERDA = false;
bool Na_90DIREITA = false;
bool curva90E_executada = false;
bool curva90D_executada = false;


// Flags individuais para cada par de sensores
bool NO_D1eD2 = false;
bool NO_D2eD3 = false;
bool NO_D3eD4 = false;
bool NO_CENTRO = false;  // normalmente D4 e D5
bool NO_D5eD6 = false;
bool NO_D6eD7 = false;
bool NO_D7eD8 = false;

////////////
/// MOTORES
////////////
AF_DCMotor motorEsq(1);
AF_DCMotor motorDir(2);
int velocidadeBase = 150;

// Ultrassom
Ultrasonic ultrassom(A0, A1);
long distancia;

////////////////////
/// ESTADOS
////////////////////
enum Estado {
  SEGUIR_LINHA,
  CURVA_ESQUERDA,
  CURVA_DIREITA,
  PARAR_TUDO_PRETO,
  PERDIDO_TUDO_BRANCO,
  VERIFICAR_OBSTACULOS,
  SEGUIR_OBJETO,
  RECUAR,
  PARAR,
  GIRAR_DIREITA,
  AVANCAR_3S,
  GIRAR_ESQUERDA_1,
  AVANCAR_ATE_PASSAR_OBJETO,
  GIRAR_ESQUERDA_2,
  AVANCAR_FINAL,
  FINALIZAR
};

Estado estadoAtual = SEGUIR_LINHA;

////////////////////
/// SETUP INICIAL ///
////////////////////
void setup() {
  Serial.begin(9600);
  delay(3000);
  Serial.println("=== INICIO ===");
  Wire.begin();

  // // SCAN geral
  // Serial.println("Escaneando barramento I2C...");
  // int encontrados = 0;
  // for (byte addr = 1; addr < 127; addr++) {
  //   Wire.beginTransmission(addr);
  //   byte erro = Wire.endTransmission();
  //   if (erro == 0) {
  //     Serial.print("Dispositivo encontrado no endereço 0x");
  //     if (addr < 16) Serial.print("0");
  //     Serial.println(addr, HEX);
  //     encontrados++;
  //   }
  // }
  // if (encontrados == 0) Serial.println("Nenhum dispositivo I2C encontrado!");
  // Serial.println("Scan concluído.");

  // // SCAN por canal do TCA
  // Serial.println("Scan completo por canal...");
  // for (uint8_t canal = 0; canal < 4; canal++) {
  //   tcaSelect(canal);
  //   delay(10);
  //   Serial.print("Canal "); Serial.print(canal); Serial.println(":");
  //   for (byte addr = 1; addr < 127; addr++) {
  //     Wire.beginTransmission(addr);
  //     if (Wire.endTransmission() == 0) {
  //       Serial.print("  Dispositivo em 0x");
  //       Serial.println(addr, HEX);
  //     }
  //   }
  // }

  // Restante do setup
  mpu_begin();
  delay(200);

  RGBsetup();
  calibrarSensores();

  motorEsq.setSpeed(150);
  motorDir.setSpeed(180);
  motorEsq.run(BACKWARD);
  motorDir.run(BACKWARD);
  delay(500);
  Serial.println("Sistema inicializado!");

  for (int i = 0; i < 100; i++) {
    mpu_loop();
    delay(5);
  }

  unsigned long inicio = millis();
  while (millis() - inicio < 500) {
    andarFrente(150, 150);
  }
  freiaMotoresN();
}  // Única chave de fechamento do setup()

////////////////////
/// LOOP PRINCIPAL ///
////////////////////
void loop() {
  leituraQTR();
  distancia = ultrassom.Ranging(CM);
  mpu_loop();

  // --- Entrada na sala branca (fita prata) ---
  if (detectouFitaPrata()) {
    varreduraSalaBranca();
    return;  // volta pro início do loop já seguindo linha normalmente
  }

  // -------- Lógica de desvio --------
  if (estadoAtual != SEGUIR_LINHA && estadoAtual != CURVA_ESQUERDA && estadoAtual != CURVA_DIREITA && estadoAtual != PARAR_TUDO_PRETO && estadoAtual != PERDIDO_TUDO_BRANCO) {
    if (estadoAtual == VERIFICAR_OBSTACULOS) {
      Serial.println("OBSTÁCULO DETECTADO!");
      estadoAtual = SEGUIR_OBJETO;
    } else if (estadoAtual == SEGUIR_OBJETO) {
      seguirAteObjeto(4);
      estadoAtual = RECUAR;
    } else if (estadoAtual == RECUAR) {
      recuarAteDistancia(8);
      estadoAtual = PARAR;
    } else if (estadoAtual == PARAR) {
      pararMotores();
      delay(1000);
      estadoAtual = GIRAR_DIREITA;
    } else if (estadoAtual == GIRAR_DIREITA) {
      girarDireita90();
      estadoAtual = AVANCAR_3S;
    } else if (estadoAtual == AVANCAR_3S) {
      seguirReto(1600);
      estadoAtual = GIRAR_ESQUERDA_1;
    } else if (estadoAtual == GIRAR_ESQUERDA_1) {
      girarEsquerda90();
      estadoAtual = AVANCAR_ATE_PASSAR_OBJETO;
    } else if (estadoAtual == AVANCAR_ATE_PASSAR_OBJETO) {
      seguirReto(3800);
      estadoAtual = GIRAR_ESQUERDA_2;
    } else if (estadoAtual == GIRAR_ESQUERDA_2) {
      girarEsquerda90();
      estadoAtual = AVANCAR_FINAL;
    } else if (estadoAtual == AVANCAR_FINAL) {
      seguirReto(1650);
      estadoAtual = FINALIZAR;
    } else if (estadoAtual == FINALIZAR) {
      girarDireita90();
      pararMotores();
      Serial.println("Sequência de desvio finalizada!");
      estadoAtual = SEGUIR_LINHA;
    }
  }

  // -------- Entrada no desvio --------
  else if (distancia < 20 && estadoAtual == SEGUIR_LINHA) {
    estadoAtual = VERIFICAR_OBSTACULOS;
  }

  // -------- Lógica de seguir linha --------
  else {
    float anguloLadeira = getAngleY();
    float anguloLateral = getAngleX();

    // DEBUG
    // Serial.print("Y: ");
    // Serial.print(anguloLadeira);
    // Serial.print(" | X: ");
    // Serial.print(anguloLateral);
    // Serial.print(" | TudoBranco: ");
    // Serial.print(TudoBranco);
    // Serial.print(" | TudoPreto: ");
    // Serial.print(TudoPreto);
    // Serial.print(" | N90E: ");
    // Serial.print(Na_90ESQUERDA);
    // Serial.print(" | N90D: ");
    // Serial.println(Na_90DIREITA);

    // Lombada: NÃO usa IMU, só reduz velocidade pelo QTR normalmente (TudoPreto)
    // Gangorra: USA IMU, inclinação real de ~20°

    if (abs(anguloLadeira) > 15 || abs(anguloLateral) > 15) {
      Serial.println("Gangorra detectada");
      velocidadeBase = 80;
      if (TudoBranco) {
        andarFrente(velocidadeBase, velocidadeBase);
        return;
      }
      // com linha: segue normal com velocidade reduzida
    } else {
      velocidadeBase = 150;  // nivelado: velocidade normal
    }

    // --- A partir daqui QTR comanda normalmente ---
    // (tanto nivelado quanto inclinado COM linha detectada)

    // --- Verde / Encruzilhada / TudoPreto ---
    if (TudoPreto) {
      pararMotores();
      delay(200);
      leituraQTR();

      bool verdeDir = false, verdeEsq = false;
      verificarVerde(verdeDir, verdeEsq);
      // DEBUG
      // Serial.print("verdeDir:");
      // Serial.print(verdeDir);
      // Serial.print(" verdeEsq:");
      // Serial.println(verdeEsq);
      // Serial.print("S1 R:");
      // Serial.print(r1);
      // Serial.print(" G:");
      // Serial.print(g1);
      // Serial.print(" B:");
      // Serial.print(b1);
      // Serial.print(" C:");
      // Serial.println(c1);
      // Serial.print("S2 R:");
      // Serial.print(r2);
      // Serial.print(" G:");
      // Serial.print(g2);
      // Serial.print(" B:");
      // Serial.print(b2);
      // Serial.print(" C:");
      // Serial.println(c2);

      if (verdeDir && verdeEsq) {
        Serial.println("Meia volta verde detectada!");
        girar180();
      } else if (verdeEsq) {
        Serial.println("Curva verde à esquerda!");
        girarEsquerda90();
      } else if (verdeDir) {
        Serial.println("Curva verde à direita!");
        girarDireita90();
      } else {
        Serial.println("Sem verde - verificando encruzilhada ou lombada");
        andarFrente(100, 100);
        delay(800);
        pararMotores();
        leituraQTR();

        if (linhaFrenteDetectada() || !TudoBranco) {
          Serial.println("Encruzilhada confirmada - seguindo em frente");
          velocidadeBase = 150;
        } else {
          unsigned long inicio = millis();
          while ((TudoPreto || TudoBranco) && (millis() - inicio < 2000)) {
            andarFrente(100, 100);
            leituraQTR();
          }
          freiaMotoresS();
          Serial.println("Lombada concluída");
        }
      }
    }

    // --- Curva na Esquerda ---
    else if (Na_90ESQUERDA && !curva90E_executada) {
      pararMotores();
      delay(200);
      leituraQTR();

      bool verdeDir = false, verdeEsq = false;
      verificarVerde(verdeDir, verdeEsq);

      if (verdeDir && verdeEsq) {
        Serial.println("Meia volta verde detectada!");
        girar180();
      } else if (verdeEsq) {
        Serial.println("Curva verde à esquerda!");
        girarEsquerda90();
      } else {
        Serial.println("Sem verde - verificando encruzilhada");
        andarFrente(100, 100);
        delay(1000);
        pararMotores();
        leituraQTR();

        if (linhaFrenteDetectada() || !TudoBranco) {
          Serial.println("Encruzilhada confirmada - seguindo em frente");
          velocidadeBase = 150;
        } else {
          Serial.println("Não era encruzilhada - curva esquerda");
          while (TudoBranco) {
            andarTras(100, 100);
            leituraQTR();
          }
          freiaMotoresS();
          delay(300);
          girarEsquerda90();
        }
      }
      curva90E_executada = true;
    }

    // --- Curva na Direita ---
    else if (Na_90DIREITA && !curva90D_executada) {
      pararMotores();
      delay(200);
      leituraQTR();

      bool verdeDir = false, verdeEsq = false;
      verificarVerde(verdeDir, verdeEsq);

      if (verdeDir && verdeEsq) {
        Serial.println("Meia volta verde detectada!");
        girar180();
      } else if (verdeDir) {
        Serial.println("Curva verde à direita!");
        girarDireita90();
      } else {
        Serial.println("Sem verde - verificando encruzilhada");
        andarFrente(100, 100);
        delay(1000);
        pararMotores();
        leituraQTR();

        if (linhaFrenteDetectada() || !TudoBranco) {
          Serial.println("Encruzilhada confirmada - seguindo em frente");
          velocidadeBase = 150;
        } else {
          Serial.println("Não era encruzilhada - curva direita");
          while (TudoBranco) {
            andarTras(100, 100);
            leituraQTR();
          }
          freiaMotoresS();
          delay(300);
          girarDireita90();
        }
      }
      curva90D_executada = true;
    }


    // --- GAP / TudoBranco ---
    else if (TudoBranco) {
      // Avança até o sensor ficar sobre a possível faixa vermelha
      andarFrente(80, 80);
      delay(1300);
      pararMotores();
      delay(100);

      // Lê com votação igual ao verde
      int contVerm = 0;
      for (int i = 0; i < 4; i++) {
        RGBloop();
        if (RGB1_vermelho || RGB2_vermelho) contVerm++;
        delay(650);
      }

      if (contVerm >= 2) {
        pararMotores();
        Serial.println("Faixa vermelha detectada - Fim da execução!");
        while (true) {}
      }

      // Gap normal
      unsigned long inicio = millis();
      while (TudoBranco && (millis() - inicio < 5000)) {
        andarFrente(100, 100);
        leituraQTR();
      }
      freiaMotoresS();

      if (!TudoBranco) {
        segueL();
      } else {
        while (TudoBranco) {
          andarTras(100, 100);
          leituraQTR();
        }
        freiaMotoresS();
        segueL();
      }
    }


    // --- Linha normal ---
    else {
      segueL();
    }


  }  // Fecha o else da lógica de seguir linha
}  // Fecha o loop