////////////////////
/// BIBLIOTECAS  ///
////////////////////
#include <AFMotor.h>
#include <Wire.h>
#include <Ultrasonic.h>

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

////////////
/// MOTORES
////////////
AF_DCMotor motorEsq(1);
AF_DCMotor motorDir(2);
int velocidadeBase = 150;

// Ultrassom
Ultrasonic ultrassom(A0, A1);
// Variável global para distância 
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
  Wire.begin();
  calibrarSensores();   // calibra os sensores QTR

  // Configuração inicial dos motores
  motorEsq.setSpeed(150);
  motorDir.setSpeed(180);
  motorEsq.run(BACKWARD);
  motorDir.run(BACKWARD);
  delay(300);
  Serial.println("Sistema inicializado!");

  // --- Largada: anda reto por 1,5 segundos ---
  unsigned long inicio = millis();
  while (millis() - inicio < 1500) {
    andarFrente(150, 150);   // anda para frente com velocidade moderada
  }
  freiaMotoresN();           // freia suavemente ao terminar a largada
}

////////////////////
/// LOOP PRINCIPAL ///
////////////////////
void loop() {
  leituraQTR();
  distancia = ultrassom.Ranging(CM);

  // -------- Lógica de desvio --------
  if (estadoAtual != SEGUIR_LINHA && estadoAtual != CURVA_ESQUERDA && estadoAtual != CURVA_DIREITA &&
      estadoAtual != PARAR_TUDO_PRETO && estadoAtual != PERDIDO_TUDO_BRANCO) {
    if (estadoAtual == VERIFICAR_OBSTACULOS) {
      Serial.println("OBSTÁCULO DETECTADO!");
      estadoAtual = SEGUIR_OBJETO;
    }
    else if (estadoAtual == SEGUIR_OBJETO) {
      seguirAteObjeto(4);
      estadoAtual = RECUAR;
    }
    else if (estadoAtual == RECUAR) {
      recuarAteDistancia(12);
      estadoAtual = PARAR;
    }
    else if (estadoAtual == PARAR) {
      pararMotores();
      delay(1000);
      estadoAtual = GIRAR_DIREITA;
    }
    else if (estadoAtual == GIRAR_DIREITA) {
      girarDireita90();
      estadoAtual = AVANCAR_3S;
    }
    else if (estadoAtual == AVANCAR_3S) {
      seguirReto(2300);
      estadoAtual = GIRAR_ESQUERDA_1;
    }
    else if (estadoAtual == GIRAR_ESQUERDA_1) {
      girarEsquerda90();
      estadoAtual = AVANCAR_ATE_PASSAR_OBJETO;
    }
    else if (estadoAtual == AVANCAR_ATE_PASSAR_OBJETO) {
      seguirReto(5500);
      estadoAtual = GIRAR_ESQUERDA_2;
    }
    else if (estadoAtual == GIRAR_ESQUERDA_2) {
      girarEsquerda90();
      estadoAtual = AVANCAR_FINAL;
    }
    else if (estadoAtual == AVANCAR_FINAL) {
      seguirReto(2300);
      estadoAtual = FINALIZAR;
    }
    else if (estadoAtual == FINALIZAR) {
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
    // Atualiza giroscópio
    mpu_loop();
    float anguloLadeira = getAngleY();   // inclinação frente/trás
    float anguloLateral = getAngleX();   // inclinação lateral

    if (abs(anguloLadeira) > 12 || abs(anguloLateral) > 12) {
      // Inclinação detectada -> ignora QTR e anda reto devagar
      Serial.println("Inclinação detectada - andando reto devagar");
      andarFrente(100, 100);
      delay(300);   // tempo de estabilização
    }
    else if (TudoPreto) {
      // anda devagar até sair da área preta
      andarFrente(80, 80);
      leituraQTR();
    } 
    else if (Na_90ESQUERDA && !curva90E_executada) {
      freiaMotoresN();
      pararMotores();
      delay(100);

      andarFrente(80, 80);
      delay(200);
      leituraQTR();

      if (!TudoBranco) {
        segueL();
      } else {
        while (!Na_90ESQUERDA) {
          andarTras(80, 80);
          leituraQTR();
        }
        freiaMotoresS();
        delay(100);

        GIrosetup();
        mpu_reset();
        while (getAngleZ() > -85) {
          mpu_loop();
          Motores90E();
        }
        pararMotores();
        curva90E_executada = true;
      }
    } 
    else if (Na_90DIREITA && !curva90D_executada) {
      freiaMotoresN();
      pararMotores();
      delay(100);

      andarFrente(80, 80);
      delay(200);
      leituraQTR();

      if (!TudoBranco) {
        segueL();
      } else {
        while (!Na_90DIREITA) {
          andarTras(80, 80);
          leituraQTR();
        }
        freiaMotoresS();
        delay(100);

        GIrosetup();
        mpu_reset();
        while (getAngleZ() < 85) {
          mpu_loop();
          Motores90D();
        }
        pararMotores();
        curva90D_executada = true;
      }
    } 
    else if (TudoBranco && !Na_90ESQUERDA && !Na_90DIREITA) {
      // GAP: anda devagar para frente tentando achar linha
      andarFrente(80, 80);
      delay(200);
      leituraQTR();

      if (TudoBranco) {
        // se ainda não achou, volta lentamente até reencontrar
        while (TudoBranco) {
          andarTras(80, 80);
          leituraQTR();
        }
        freiaMotoresS();
      }
    } 
    else {
      // linha detectada normalmente
      segueL();
    }
  }
}
