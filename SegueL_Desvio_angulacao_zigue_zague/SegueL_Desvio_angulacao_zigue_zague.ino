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
  calibrarSensores();

  motorEsq.setSpeed(150);
  motorDir.setSpeed(180);
  motorEsq.run(BACKWARD);
  motorDir.run(BACKWARD);
  delay(300);
  Serial.println("Sistema inicializado!");

  // --- Largada: anda reto por ~0,8s ---
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
  mpu_loop();
  float anguloLadeira = getAngleY();
  float anguloLateral = getAngleX();

  // --- Detecta inclinação (gangorra ou lombada) ---
  if (abs(anguloLadeira) > 15 || abs(anguloLateral) > 15) {
    Serial.println("Gangorra detectada - reduzindo velocidade");
    velocidadeBase = 80;
    segueL();
  }
  else if (abs(anguloLadeira) > 8 || abs(anguloLateral) > 8) {
    Serial.println("Lombada detectada - passando devagar");
    velocidadeBase = 100;
    segueL();
  }

  // --- Caso tudo preto (verifica verde) ---
  else if (TudoPreto) {
    pararMotores();
    delay(200);

    bool verde_esquerda = false;
    bool verde_direita = false;
    bool dar_meiavolta = false;

    RGBsetup();

    while (true) {
      RGBloop();
      leituraQTR();

      motorEsq.setSpeed(82);
      motorDir.setSpeed(82);
      motorEsq.run(BACKWARD);
      motorDir.run(BACKWARD);

      if (RGB2_verde && RGB1_branco) { verde_esquerda = true; break; }
      if (RGB1_verde && RGB2_branco) { verde_direita = true; break; }
      if (RGB1_verde && RGB2_verde)   { dar_meiavolta = true; break; }

      if (!TudoPreto) break;
    }

    freiaMotoresN();

    if (dar_meiavolta) {
      Serial.println("Meia-volta verde detectada!");
      GIrosetup();
      while (getAngleZ() < 175) { mpu_loop(); Motores90D(); }
      pararMotores();
      mpu_reset();
      segueL();
    }
    else if (verde_esquerda) {
      Serial.println("Curva verde à esquerda!");
      GIrosetup();
      while (getAngleZ() > -85) { mpu_loop(); Motores90E(); }
      pararMotores();
      mpu_reset();
    }
    else if (verde_direita) {
      Serial.println("Curva verde à direita!");
      GIrosetup();
      while (getAngleZ() < 85) { mpu_loop(); Motores90D(); }
      pararMotores();
      mpu_reset();
    }
    else {
      // Se não detectou verde, comportamento padrão
      andarFrente(100, 100);
      delay(1500);
      leituraQTR();
      if (!TudoBranco) segueL();
      else pararMotores();
    }
  }

  // --- Curva à esquerda ---
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
    } 
    else if (linhaEmZigueZague()) {
      Serial.println("Zigue-zague detectado - reduzindo velocidade");
      velocidadeBase = 100;
      segueL();
    } 
    else {
      segueL(); // encruzilhada
    }
  }

  // --- Curva à direita ---
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
    } 
    else if (linhaEmZigueZague()) {
      Serial.println("Zigue-zague detectado - reduzindo velocidade");
      velocidadeBase = 100;
      segueL();
    } 
    else {
      segueL(); // encruzilhada
    }
  }

  // --- GAP (tudo branco) ---
  else if (TudoBranco) {
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
    velocidadeBase = 150;
    segueL();
  }
}

}
