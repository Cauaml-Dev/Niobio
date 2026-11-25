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
float Kd = 7.51 ; // 6.13
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
bool emLombada = false;   



// Flags individuais para cada par de sensores
bool NO_D1eD2 = false;
bool NO_D2eD3 = false;
bool NO_D3eD4 = false;
bool NO_CENTRO = false;  // normalmente D4 e D5
bool NO_D5eD6 = false;
bool NO_D6eD7 = false;
bool NO_D7eD8 = false;

int contadorObstaculo = 0;
const int CONFIRMACOES_OBSTACULO = 3; // precisa ler <20cm 3 vezes seguidas

int contadorGangorra = 0;
const int CONFIRMACOES_GANGORRA = 8; // era 5 - exige mais consistência antes de confirmar rampa


int contadorVermelho = 0;
const int CONFIRMACOES_VERMELHO = 3; // precisa ler vermelho 3 vezes seguidas

int contadorBrancoConsecutivo = 0;
const int CONFIRMACOES_BRANCO = 3; // precisa ler TudoBranco 3 vezes seguidas pra confirmar GAP

int contadorCurvaEsq = 0;
int contadorCurvaDir = 0;
const int CONFIRMACOES_CURVA = 3; // precisa ler o padrão de curva 3 vezes seguidas antes de tratar como curva real

// ---- VELOCIDADES ----
const int VELOCIDADE_NORMAL = 130;   // era 110 - velocidade normal ao seguir linha
const int VELOCIDADE_GANGORRA = 100; // velocidade reduzida ao subir/descer rampa

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
  distancia = lerDistanciaFiltrada();
  mpu_loop();


  // -------- Lógica de desvio --------
  if (estadoAtual != SEGUIR_LINHA && estadoAtual != CURVA_ESQUERDA && estadoAtual != CURVA_DIREITA && estadoAtual != PARAR_TUDO_PRETO && estadoAtual != PERDIDO_TUDO_BRANCO) {
    if (estadoAtual == VERIFICAR_OBSTACULOS) {
      Serial.println("OBSTÁCULO DETECTADO!");
      estadoAtual = SEGUIR_OBJETO;
    } else if (estadoAtual == SEGUIR_OBJETO) {
      bool obstaculoReal = seguirAteObjeto(4, 1000);
      if (obstaculoReal) {
        estadoAtual = RECUAR;
      } else {
        Serial.println("Falso alarme - voltando a seguir linha.");
        estadoAtual = SEGUIR_LINHA;
        contadorObstaculo = 0;
      }
    } else if (estadoAtual == RECUAR) {
      recuarAteDistancia(8, 3000);
      estadoAtual = PARAR;
    } else if (estadoAtual == PARAR) {
      pararMotores();
      delay(1000);
      estadoAtual = GIRAR_DIREITA;
    } else if (estadoAtual == GIRAR_DIREITA) {
      girarDireita90();
      estadoAtual = AVANCAR_3S;
    } else if (estadoAtual == AVANCAR_3S) {
      seguirReto(1700);
      estadoAtual = GIRAR_ESQUERDA_1;
    } else if (estadoAtual == GIRAR_ESQUERDA_1) {
      girarEsquerda90();
      estadoAtual = AVANCAR_ATE_PASSAR_OBJETO;
    } else if (estadoAtual == AVANCAR_ATE_PASSAR_OBJETO) {
      seguirReto(390);
      estadoAtual = GIRAR_ESQUERDA_2;
    } else if (estadoAtual == GIRAR_ESQUERDA_2) {
      girarEsquerda90();
      estadoAtual = AVANCAR_FINAL;
    } else if (estadoAtual == AVANCAR_FINAL) {
      seguirReto(2200);
      estadoAtual = FINALIZAR;
    } else if (estadoAtual == FINALIZAR) {
      girarDireita90();
      pararMotores();
      Serial.println("Sequência de desvio finalizada!");
      estadoAtual = SEGUIR_LINHA;
    }
  }

  // -------- Entrada no desvio / Lógica de seguir linha --------
  else if (estadoAtual == SEGUIR_LINHA) {
    if (distancia > 2 && distancia < 20) {
      contadorObstaculo++;
    } else {
      contadorObstaculo = 0;
    }

    if (contadorObstaculo >= CONFIRMACOES_OBSTACULO) {
      contadorObstaculo = 0;
      estadoAtual = VERIFICAR_OBSTACULOS;
    } else {
      // -------- Lógica de seguir linha --------
      float anguloLadeira = getAngleY();
      float anguloLateral = getAngleX();

      if (abs(anguloLadeira) > 15 || abs(anguloLateral) > 15) {
        contadorGangorra++;
      } else {
        contadorGangorra = 0;
      }

      if (contadorGangorra >= CONFIRMACOES_GANGORRA) {
        Serial.println("Gangorra detectada");
        velocidadeBase = VELOCIDADE_GANGORRA;
        if (TudoBranco) {
          andarFrente(velocidadeBase, velocidadeBase);
          return;
        }
      } else {
        velocidadeBase = VELOCIDADE_NORMAL;
      }

      // Conclui a lombada assim que o QTR deixar de ler TudoPreto
      if (emLombada && !TudoPreto) {
        freiaMotoresS();
        emLombada = false;
        Serial.println("Lombada concluída");
      }

      // --- Verde / Encruzilhada / TudoPreto / Lombada ---
      if (TudoPreto) {
        contadorCurvaEsq = 0;
        contadorCurvaDir = 0;

        if (emLombada) {
          andarFrente(215, 200);
        }
        else {
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
              velocidadeBase = VELOCIDADE_NORMAL;
            } else {
              emLombada = true;
              Serial.println("Lombada detectada - atravessando rápido...");
            }
          }
        }
      }

      // --- Curva na Esquerda ---
      else if (Na_90ESQUERDA && !curva90E_executada) {
        contadorCurvaEsq++;
        if (contadorCurvaEsq < CONFIRMACOES_CURVA) {
          segueL();
        } else {
          contadorCurvaEsq = 0;
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
              velocidadeBase = VELOCIDADE_NORMAL;
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
      }

      // --- Curva na Direita ---
      else if (Na_90DIREITA && !curva90D_executada) {
        contadorCurvaDir++;
        if (contadorCurvaDir < CONFIRMACOES_CURVA) {
          segueL();
        } else {
          contadorCurvaDir = 0;
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
              velocidadeBase = VELOCIDADE_NORMAL;
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
      }


      

// --- GAP / TudoBranco ---
      else if (TudoBranco) {
        contadorCurvaEsq = 0;
        contadorCurvaDir = 0;
        contadorBrancoConsecutivo++;
        if (contadorBrancoConsecutivo < CONFIRMACOES_BRANCO) {
          segueL();
        } else {
          // GAP confirmado - avança em pequenos incrementos, checando o
          // BLOCO CENTRAL (sensores 2,3,4,5) E o vermelho a cada passo,
          // tudo enquanto anda pra frente (sem parar antes)
          bool linhaJaAchada = false;
          int contVerm = 0;
          unsigned long inicioAproximacao = millis();
          while (millis() - inicioAproximacao < 500) {
            // checa o vermelho ANTES de mover, pra não perder iteração com o motor
            RGBloop();
            if (RGB1_vermelho || RGB2_vermelho) contVerm++;
            if (contVerm >= 1) break;  // já confirmou vermelho - sai do avanço e para
 
            andarFrente(80, 80);
            leituraQTR();
            if (estadoSensores[2] == 1 || estadoSensores[3] == 1 || estadoSensores[4] == 1 || estadoSensores[5] == 1) {
              linhaJaAchada = true;
              break;
            }
          }
          pararMotores();
          if (contVerm >= 1) {
            Serial.println("Faixa vermelha detectada - Fim da execução!");
            while (true) {}
          }
          if (linhaJaAchada) {
            contadorBrancoConsecutivo = 0;
            segueL();
          } else {
            unsigned long inicio = millis();
            bool linhaReencontrada = false;
            while (millis() - inicio < 4500) {
              andarFrente(100, 100);
              leituraQTR();
              if (estadoSensores[1] == 1 || estadoSensores[2] == 1 || estadoSensores[3] == 1 || estadoSensores[4] == 1 || estadoSensores[5] == 1 || estadoSensores[6] == 1) {
                linhaReencontrada = true;
                break;
              }
            }
            if (linhaReencontrada) {
              contadorBrancoConsecutivo = 0;
              segueL();
            } else {
              freiaMotoresS();
              unsigned long inicioRecuo = millis();
              while (!(estadoSensores[2] == 1 || estadoSensores[3] == 1 || estadoSensores[4] == 1 || estadoSensores[5] == 1) && (millis() - inicioRecuo < 6000)) {
                andarTras(100, 100);
                leituraQTR();
              }
              freiaMotoresS();
              contadorBrancoConsecutivo = 0;
              segueL();
            }
          }
        }
      }




      // --- Linha normal ---
      else {
        contadorBrancoConsecutivo = 0;
        contadorCurvaEsq = 0;
        contadorCurvaDir = 0;
        segueL();
      }

    } // fecha o else do contadorObstaculo
  } // fecha o else if (estadoAtual == SEGUIR_LINHA)

} // fecha o loop