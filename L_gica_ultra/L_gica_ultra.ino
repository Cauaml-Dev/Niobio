#include <Ultrasonic.h>
#include <AFMotor.h>
#include <Wire.h>

// Motores
AF_DCMotor motorEsq(1); // motor esquerdo
AF_DCMotor motorDir(2); // motor direito

// Sensor ultrassônico frontal
Ultrasonic ultrassom(A0, A1);

// Distância
long distancia;

// Estados
enum Estado {
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

Estado estadoAtual = VERIFICAR_OBSTACULOS;

//////////////////////
// Setup e Loop
//////////////////////

void setup() {
  Serial.begin(9600);
  Serial.println("MPU6050 inicializado!");
}

void loop() {
  distancia = ultrassom.Ranging(CM);

  if (estadoAtual == VERIFICAR_OBSTACULOS) {
    estadoAtual = SEGUIR_OBJETO;

  } else if (estadoAtual == SEGUIR_OBJETO) {
    seguirAteObjeto(4);
    estadoAtual = RECUAR;

  } else if (estadoAtual == RECUAR) {
    recuarAteDistancia(15);
    estadoAtual = PARAR;

  } else if (estadoAtual == PARAR) {
    pararMotores();
    delay(1000);
    estadoAtual = GIRAR_DIREITA;

  } else if (estadoAtual == GIRAR_DIREITA) {
    girarDireita90();
    estadoAtual = AVANCAR_3S;

  } else if (estadoAtual == AVANCAR_3S) {
    seguirReto(1500);
    estadoAtual = GIRAR_ESQUERDA_1;

  } else if (estadoAtual == GIRAR_ESQUERDA_1) {
    girarEsquerda90();
    estadoAtual = AVANCAR_ATE_PASSAR_OBJETO;

  } else if (estadoAtual == AVANCAR_ATE_PASSAR_OBJETO) {
    seguirReto(2000);
    estadoAtual = GIRAR_ESQUERDA_2;

  } else if (estadoAtual == GIRAR_ESQUERDA_2) {
    girarEsquerda90();
    estadoAtual = AVANCAR_FINAL;

  } else if (estadoAtual == AVANCAR_FINAL) {
    seguirReto(1500);
    estadoAtual = FINALIZAR;

  } else if (estadoAtual == FINALIZAR) {
    girarDireita90();
    pararMotores();
    Serial.println("Sequência finalizada!");
    while (true); // trava o loop
  }

  delay(100);
}
