////////////////////
/// BIBLIOTECAS  ///
////////////////////
#include <AFMotor.h>
#include <Wire.h>

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

////////////////////
/// ESTADOS
////////////////////
enum Estado {
  SEGUIR_LINHA,
  CURVA_ESQUERDA,
  CURVA_DIREITA,
  PARAR_TUDO_PRETO,
  PERDIDO_TUDO_BRANCO
};

Estado estadoAtual = SEGUIR_LINHA;

////////////////////
/// SETUP INICIAL ///
////////////////////
void setup() {
  Serial.begin(9600);
  Wire.begin();
  calibrarSensores();

  motorEsq.setSpeed(150);
  motorDir.setSpeed(180);
  motorEsq.run(BACKWARD);
  motorDir.run(BACKWARD);
  delay(300);
  Serial.println("Sistema inicializado!");

  // --- Largada: anda reto por ~0,5s ---
  unsigned long inicio = millis();
  while (millis() - inicio < 500) {
    andarFrente(150, 150);
  }
  freiaMotoresN();
}

////////////////////
/// LOOP PRINCIPAL ///
////////////////////
void loop() {
  leituraQTR();

  // -------- Lógica de seguir linha --------
  mpu_loop();
  float anguloLadeira = getAngleY();
  float anguloLateral = getAngleX();

  if (abs(anguloLadeira) > 12 || abs(anguloLateral) > 12) {
    Serial.println("Inclinação detectada - andando reto devagar");
    andarFrente(100, 100);
    delay(300);
  }
  else if (TudoPreto) {
    pararMotores();
    delay(200);

    andarFrente(100, 100);
    delay(1500);
    leituraQTR();

    if (!TudoBranco) {
      segueL();
    } else {
      pararMotores();
    }
  }
  else if (Na_90ESQUERDA && !curva90E_executada) {
    pararMotores();
    delay(200);

    andarFrente(100, 100);
    delay(1500);
    leituraQTR();

    if (TudoBranco) {
      while (TudoBranco) {
        andarTras(100, 100);
        leituraQTR();
      }
      freiaMotoresS();

      delay(300);
      girarEsquerda90();
      curva90E_executada = true;
    } else {
      segueL();
    }
  }
  else if (Na_90DIREITA && !curva90D_executada) {
    pararMotores();
    delay(200);

    andarFrente(100, 100);
    delay(1500);
    leituraQTR();

    if (TudoBranco) {
      while (TudoBranco) {
        andarTras(100, 100);
        leituraQTR();
      }
      freiaMotoresS();

      delay(300);
      girarDireita90();
      curva90D_executada = true;
    } else {
      segueL();
    }
  }
  else if (TudoBranco) {
    // GAP: tenta avançar até reencontrar a linha
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
  else {
    // Linha normal: segue com PID
    segueL();
  }
}
