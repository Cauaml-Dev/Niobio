
////////////////////
/////BIBLIOTECAS////
////////////////////

#include <AFMotor.h>
#include <Wire.h>

//////////////////
///SENSOR : QTR///
//////////////////

// Pinos dos sensores (D1 a D8)
int sensores[] = { A15, A14, A13, A12, A11, A10, A9, A8 };
int estadoSensores[8];

// Pesos dos sensores
int pesos[] = {-3500, -2500, -1500, -500, 500, 1500, 2500, 3500};

// Limiares calibrados automaticamente
int limiarmin = 250;
int limiarmax = 550;

// PID (sem Ki)
float Kp = 1.35;  //1.0 1.8 2.3 2.5 2.9 3.3 3.5 3.8 1.1 1.2 1.3 1.35
float Kd = 6.13;  //6.9  6.8 6.7 6.6 6.5 6.4 6.3 6.2 6.15 6.14 6.13

float erro = 0;
float erroAnterior = 0;
float derivada = 0;
float correcao = 0;
int offset = 0;  // ajuste fino se sensor estiver fisicamente torto

bool TudoPreto = false;
bool TudoBranco = false;
bool Na_90ESQUERDA = false;
bool Na_90DIREITA = false;
bool NO_D1eD2 = false;
bool NO_D2eD3 = false;
bool NO_D3eD4 = false;
bool NO_CENTRO = false;
bool NO_D5eD6 = false;
bool NO_D6eD7 = false;
bool NO_D7eD8 = false;

////////////
// Motores//
////////////
// Motores
AF_DCMotor motorEsq(1);
AF_DCMotor motorDir(2);
int velocidadeBase = 150;  // aumenta base para compensar engrenagens


////////////////////////////
/////RGB + MULTIPEXADOR/////
////////////////////////////

#include <Wire.h>
#include "Adafruit_TCS34725.h"

#define TCAADDR 0x70  // Endereço do multiplexador TCA9548A

// Definição dos canais dos sensores RGB:
#define RGB1_CHANNEL 2 // Sensor RGB 1 (lado esquerdo)
#define RGB2_CHANNEL 3  // Sensor RGB 2 (lado direito)

// Instancia os sensores com parâmetros de integração e ganho desejados
Adafruit_TCS34725 sensor1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 sensor2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X);

bool RGB1_verde = false;
bool RGB1_vermelho = false;
bool RGB1_branco = false;

bool RGB2_verde = false;
bool RGB2_vermelho = false;
bool RGB2_branco = false;
bool curva90E_executada = false;


uint16_t r1, g1, b1, c1;
uint16_t r2, g2, b2, c2;


void setup() {
  Serial.begin(9600);
  calibrarSensores();
 motorEsq.setSpeed(150);
  motorDir.setSpeed(180);
  motorEsq.run(BACKWARD);
  motorDir.run(BACKWARD);
  delay(300);
}

// curva para direita(-); curva para esquerda (+)

// ========== LOOP PRINCIPAL REORGANIZADO ==========
void loop() {


  leituraQTR();  // Atualiza os estados dos sensores

  // 1. PRIMEIRA DECISÃO: Se tudo preto, pare
 if (TudoPreto) {
    // Freia e para completamente
   // freiaMotoresN();
    pararMotores();
    delay(10);  // Pequena pausa para estabilização

    // Variável para controlar se detectou tudo branco durante o avanço
    bool verde_esquerda = false;  //RGB2_verde && RGB1_branco
    bool verde_direita = false;   //RGB1_verde && RGB2_branco
    bool dar_meiavolta = false;   //RGB1_verde && RGB2_verde

    // Inicializa os sensores RGB
    RGBsetup();

    // Avança devagar até encontrar RGB2 verde e RGB1 branco
    while (true) {
      // Atualiza leituras
      RGBloop();
      leituraQTR();

      // Movimento para frente em velocidade reduzida
      motorEsq.setSpeed(82);  // Velocidade reduzida
      motorDir.setSpeed(82);
      motorEsq.run(BACKWARD);
      motorDir.run(BACKWARD);

      // Verifica condição RGB desejada
      if (RGB2_verde && RGB1_branco) {  //curva 90 graus para esquerda
        verde_esquerda = true;
        break;
      }

      if (RGB1_verde && RGB2_branco) {
        verde_direita = true;
        break;
      }

      if (RGB1_verde && RGB2_verde) {
        dar_meiavolta = true;
        break;
      }

      // Se não for mais 90 esquerda e não for tudo branco, sai do loop
      if (!TudoPreto) {
        break;
      }

      //delay(10); // Pequeno delay para evitar sobrecarga
    }

    // Freia após sair do loop de movimento
    freiaMotoresN();

    if (dar_meiavolta) {
     // Serial.println("CONDIÇÃO RGB ATINGIDA");

      // Gira enquanto ângulo < 70
      while (TudoPreto) {
        leituraQTR();
        // Movimento para frente em velocidade reduzida
      motorEsq.setSpeed(200);  // Velocidade reduzida
      motorDir.setSpeed(200);
      motorEsq.run(FORWARD);
      motorDir.run(FORWARD);
     // Serial.println("INDO PARA TRAZ");
      }
      pararMotores();
      //delay(100);

      //Serial.println("CONDIÇÃO RGB ATINGIDA - INICIANDO CURVA");

      // Configura giroscópio
      GIrosetup();
      //delay(100);

      // Gira enquanto ângulo < 70
      while (getAngleZ() < 175) {
        mpu_loop();
        Motores90D();
       // Serial.print("Ângulo: ");
        //Serial.println(getAngleZ());
      }

      // Quando atingir 70°
      pararMotores();
      //Serial.println("ÂNGULO 180º ALCANÇADO");
      mpu_reset();
      dar_meiavolta = false;
      segueL();
      //delay(200);
    }

    // Lógica para cada caso
    else if (verde_esquerda) {
      //Serial.println("CONDIÇÃO RGB ATINGIDA - INICIANDO CURVA");

      // Configura giroscópio
      GIrosetup();
      //delay(200);

      // Gira enquanto ângulo < 70
      while (getAngleZ() > -85) {
        mpu_loop();
        Motores90E();
       // Serial.print("Ângulo: ");
        //Serial.println(getAngleZ());
      }

      // Quando atingir 70°
      pararMotores();
     // Serial.println("ÂNGULO 70º ALCANÇADO");
      mpu_reset();
      verde_esquerda = false;
      //delay(200);
    }

    else if (verde_direita) {
      //Serial.println("CONDIÇÃO RGB ATINGIDA - INICIANDO CURVA");

      // Configura giroscópio
      GIrosetup();
      //delay(200);

      // Gira enquanto ângulo < 70
      while (getAngleZ() < 85) {
        mpu_loop();
        Motores90D();
        //Serial.print("Ângulo: ");
        //Serial.println(getAngleZ());
      }

      // Quando atingir 70°
      pararMotores();
      //Serial.println("ÂNGULO 70º ALCANÇADO");
      mpu_reset();
      verde_direita = false;
      //delay(200);
    }


  }




  // 2. SEGUNDA DECISÃO: Se curva à esquerda detectada
  else if (Na_90ESQUERDA) {
    // Freia e para completamente
    freiaMotoresN();
    pararMotores();
    delay(10);  // Pequena pausa para estabilização

    // Variável para controlar se detectou tudo branco durante o avanço
    bool detectouBranco1 = false;
    bool condicaoRGB1 = false;

    // Inicializa os sensores RGB
    RGBsetup();

    // Avança devagar até encontrar RGB2 verde e RGB1 branco
    while (true) {
      // Atualiza leituras
      RGBloop();
      leituraQTR();

      // Movimento para frente em velocidade reduzida
      motorEsq.setSpeed(82);  // Velocidade reduzida
      motorDir.setSpeed(82);
      motorEsq.run(BACKWARD);
      motorDir.run(BACKWARD);

      // Verifica condição RGB desejada
      if (RGB2_verde && RGB1_branco) {
        condicaoRGB1 = true;
        break;
      }

      // Verifica se saiu da linha (tudo branco)
      if (TudoBranco) {
        detectouBranco1 = true;
        break;
      }

      // Se não for mais 90 esquerda e não for tudo branco, sai do loop
      if (!Na_90ESQUERDA && !TudoBranco) {
        break;
      }

      //delay(10); // Pequeno delay para evitar sobrecarga
    }

    // Freia após sair do loop de movimento
    freiaMotoresN();

    // Lógica para cada caso
    if (condicaoRGB1) {
      //Serial.println("CONDIÇÃO RGB ATINGIDA - INICIANDO CURVA");

      // Configura giroscópio
      GIrosetup();
      //delay(200);

      // Gira enquanto ângulo < 70
      while (getAngleZ() > -85) {
        mpu_loop();
        Motores90E();
       // Serial.print("Ângulo: ");
        //Serial.println(getAngleZ());
      }

      // Quando atingir 70°
      pararMotores();
     // Serial.println("ÂNGULO 70º ALCANÇADO");
      mpu_reset();
      condicaoRGB1 = false;
      //delay(200);
    } else if (detectouBranco1) {
      //Serial.println("TUDO BRANCO DETECTADO - RECUANDO");

      // Recua devagar até encontrar 90 esquerda novamente
      while (!Na_90ESQUERDA) {
        leituraQTR();
        motorEsq.setSpeed(82);
        motorDir.setSpeed(82);
        motorEsq.run(FORWARD);
        motorDir.run(FORWARD);
        // delay(10);
      }

      freiaMotoresS();
      //Serial.println("90 ESQUERDA REENCONTRADA - INICIANDO CURVA");

      // Configura giroscópio
      GIrosetup();
      mpu_reset();
      //delay(200);

      // Gira enquanto ângulo < 70
      while (getAngleZ() > -85) {
        mpu_loop();
        Motores90E();
        //Serial.print("Ângulo: ");
        //Serial.println(getAngleZ());
      }

      // Quando atingir 70°
      pararMotores();
      //Serial.println("ÂNGULO 70º ALCANÇADO");
      mpu_reset();
      detectouBranco1 = false;
      //delay(200);
    }
    // Se não for nenhum dos casos acima, simplesmente continua
  }


  // 3. TERCEIRA DECISÃO: Se curva à direita detectada
  else if (Na_90DIREITA) {
    // Freia e para completamente
    freiaMotoresN();
    pararMotores();
    delay(10);  // Pequena pausa para estabilização

    // Variável para controlar se detectou tudo branco durante o avanço
    bool detectouBranco2 = false;
    bool condicaoRGB2 = false;

    // Inicializa os sensores RGB
    RGBsetup();

    // Avança devagar até encontrar RGB2 verde e RGB1 branco
    while (true) {
      // Atualiza leituras
      RGBloop();
      leituraQTR();

      // Movimento para frente em velocidade reduzida
      motorEsq.setSpeed(82);  // Velocidade reduzida
      motorDir.setSpeed(82);
      motorEsq.run(BACKWARD);
      motorDir.run(BACKWARD);

      // Verifica condição RGB desejada
      if (RGB1_verde && RGB2_branco) {
        condicaoRGB2 = true;
        break;
      }

      // Verifica se saiu da linha (tudo branco)
      if (TudoBranco) {
        detectouBranco2 = true;
        break;
      }

      // Se não for mais 90 esquerda e não for tudo branco, sai do loop
      if (!Na_90DIREITA && !TudoBranco) {
        break;
      }

      //delay(10); // Pequeno delay para evitar sobrecarga
    }

    // Freia após sair do loop de movimento
    freiaMotoresN();

    // Lógica para cada caso
    if (condicaoRGB2) {
      //Serial.println("CONDIÇÃO RGB ATINGIDA - INICIANDO CURVA");

      // Configura giroscópio
      GIrosetup();
      mpu_reset();
      //delay(200);

      // Gira enquanto ângulo < 70
      while (getAngleZ() < 85) {
        mpu_loop();
        Motores90D();
        //Serial.print("Ângulo: ");
        //Serial.println(getAngleZ());
      }

      // Quando atingir 70°
      pararMotores();
      //Serial.println("ÂNGULO 70º ALCANÇADO");
      mpu_reset();
      condicaoRGB2 = false;
      //delay(200);
    } else if (detectouBranco2) {
      //Serial.println("TUDO BRANCO DETECTADO - RECUANDO");

      // Recua devagar até encontrar 90 esquerda novamente
      while (!Na_90DIREITA) {
        leituraQTR();
        motorEsq.setSpeed(82);
        motorDir.setSpeed(82);
        motorEsq.run(FORWARD);
        motorDir.run(FORWARD);
        // delay(10);
      }

      freiaMotoresS();
      //Serial.println("90 ESQUERDA REENCONTRADA - INICIANDO CURVA");

      // Configura giroscópio
      GIrosetup();
      mpu_reset();
      //delay(200);

      // Gira enquanto ângulo < 70
      while (getAngleZ() < 85) {
        mpu_loop();
        Motores90D();
        //Serial.print("Ângulo: ");
        //Serial.println(getAngleZ());
      }

      // Quando atingir 70°
      pararMotores();
      //Serial.println("ÂNGULO 70º ALCANÇADO");
      mpu_reset();
      detectouBranco2 = false;
      //delay(200);
    }
    // Se não for nenhum dos casos acima, simplesmente continua
  }

  // 4. COMPORTAMENTO PADRÃO: Seguir linha
  else {
    segueL();
  }
}



  