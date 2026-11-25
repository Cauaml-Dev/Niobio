
/////////////
///MOTORES///
////todos////
/////////////
void moverMotores(int velEsq, int velDir) {

  if (velEsq > 223) velEsq = 223;
  if (velDir > 205) velDir = 205;
  if (velEsq < 0) velEsq = 0;
  if (velDir < 0) velDir = 0;

  motorEsq.setSpeed(velEsq);
  motorDir.setSpeed(velDir);

  motorEsq.run(BACKWARD);
  motorDir.run(BACKWARD);
}

//parar
void pararMotores() {

  motorEsq.setSpeed(0);
  motorDir.setSpeed(0);
  motorEsq.run(RELEASE);
  motorDir.run(RELEASE);
}

//giro pra esquerda
void Motores90E() {
  motorEsq.setSpeed(150);
  motorDir.setSpeed(180);
  motorEsq.run(FORWARD);
  motorDir.run(BACKWARD);
}


//giro para direita
void Motores90D() {
  motorEsq.setSpeed(150);
  motorDir.setSpeed(150);
  motorEsq.run(BACKWARD);
  motorDir.run(FORWARD);
}

/////////////////
//FREIA motores//
/////////////////

//se estiver undo para traz
void freiaMotoresS() {
  // Freio mais suave e eficiente
  motorEsq.setSpeed(150);
  motorDir.setSpeed(125);
  motorEsq.run(BACKWARD);  // Sentido oposto para frear
  motorDir.run(BACKWARD);
  delay(50);  // Tempo suficiente para frear sem travar
  motorEsq.setSpeed(0);
  motorDir.setSpeed(0);
  motorEsq.run(RELEASE);
  motorDir.run(RELEASE);
}

//se estiver indo para frente
void freiaMotoresN() {
  // Freio mais suave e eficiente
  motorEsq.setSpeed(150);
  motorDir.setSpeed(125);
  motorEsq.run(FORWARD);  // Sentido oposto para frear
  motorDir.run(FORWARD);
  delay(50);  // Tempo suficiente para frear sem travar
  motorEsq.setSpeed(0);
  motorDir.setSpeed(0);
  motorEsq.run(RELEASE);
  motorDir.run(RELEASE);
}



/////////////////////
///VOID SEGUELINHA///
////////////////////
void segueL() {
  
  leituraQTR();
  int posicao = calcularPosicaoLinha();
  erro = posicao - offset;

  correcao = Kp * erro + Kd * (erro - erroAnterior);

  int velocidadeEsq = velocidadeBase - correcao;
  int velocidadeDir = velocidadeBase + correcao;

  moverMotores(velocidadeEsq, velocidadeDir + 40);
  erroAnterior = erro;

  // DEBUG opcional:
 // Serial.print("Sensores: ");
  //for (int i = 0; i < 8; i++) {
   // Serial.print(estadoSensores[i]);
    //Serial.print(" ");
  //}
  //Serial.println();

  // Debug
 // Serial.print("Erro: ");
  //Serial.print(erro);
  //Serial.print(" | Integral: ");
  //Serial.print(erroIntegral);
  //Serial.print(" | Derivada: ");
  //Serial.print(derivada);
  //Serial.print(" | Correção: ");
  //Serial.println(correcao);
}


/////////////////////////////
///posicionamento da linha///
////////para o PID///////////
/////////////////////////////

int calcularPosicaoLinha() {
  long somaPesos = 0;
  long somaLeituras = 0;

  for (int i = 0; i < 8; i++) {
    int leitura = analogRead(sensores[i]);

    // Normaliza a leitura entre 0 e 1000 baseado na calibração
    int valor = map(constrain(leitura, limiarmin, limiarmax), limiarmin, limiarmax, 0, 1000);

    somaPesos += (long)valor * pesos[i];
    somaLeituras += valor;
  }

  if (somaLeituras == 0) return 0;

  return somaPesos / somaLeituras;
}

/////////////////////
///LEITURA do QTR//// 
///////D1-D8////////

//para toda as cituações da pista
void leituraQTR() {
  for (int i = 0; i < 8; i++) {
    int leitura = analogRead(sensores[i]);

    if (leitura < limiarmin) {
      estadoSensores[i] = 0;  // branco
    }
    if ((leitura > limiarmin) && (leitura < limiarmax)) {
      estadoSensores[i] = 1;  // preto
    }
  }

  // Tudo preto? D1 a D8 pretos (todos 1)
  TudoPreto = true;
  for (int i = 0; i < 8; i++) {
    if (estadoSensores[i] == 0) {
      TudoPreto = false;
      break;
    }
  }

  // Tudo branco? D1 a D8 brancos (todos 0)
  TudoBranco = true;
  for (int i = 0; i < 8; i++) {
    if (estadoSensores[i] == 1) {
      TudoBranco = false;
      break;
    }
  }

  // Detecta curva 90º esquerda: D1 a D3 brancos e D4 a D8 pretos
  bool d1a3_brancos = true;
  for (int i = 0; i <= 2; i++) {
    if (estadoSensores[i] == 1) {
      d1a3_brancos = false;
      break;
    }
  }
  bool d4a8_pretos = true;
  for (int i = 3; i <= 7; i++) {
    if (estadoSensores[i] == 0) {
      d4a8_pretos = false;
      break;
    }
  }
  Na_90ESQUERDA = (d1a3_brancos && d4a8_pretos);

  // Detecta curva 90º direita: D1 a D5 pretos e D6 a D8 brancos
  bool d1a5_pretos = true;
  for (int i = 0; i <= 4; i++) {
    if (estadoSensores[i] == 0) {
      d1a5_pretos = false;
      break;
    }
  }
  bool d6a8_brancos = true;
  for (int i = 5; i <= 7; i++) {
    if (estadoSensores[i] == 1) {
      d6a8_brancos = false;
      break;
    }
  }
  Na_90DIREITA = (d1a5_pretos && d6a8_brancos);

  // Libera a flag curva90E_executada apenas quando sair da curva anterior
  static bool jaSaiuDa90E = true;

  if (!Na_90ESQUERDA && !TudoBranco) {
    jaSaiuDa90E = true;
  }

  if (Na_90ESQUERDA && jaSaiuDa90E) {
    curva90E_executada = false;
    jaSaiuDa90E = false;
  }
}

/////////////////////////////////
////CALIBRAÇÃO para VER PRETO////
/////////////////////////////////
void calibrarSensores() {
  int leitura;
  int minimo = 1023;
  int maximo = 0;

  for (int i = 0; i < 100; i++) {
    for (int j = 3; j < 4; j++) {
      leitura = analogRead(sensores[j]);
      if (leitura < minimo) minimo = leitura;
      if (leitura > maximo) maximo = leitura;
    }
    delay(5);
  }
  // usado para limitar o preto e conciderar o verde, vermelho, como "BRANCO" ou "PRETO"

  limiarmin = minimo - 10;//define o minimo para preto
  limiarmax = maximo + 10;// maximo valor de preto possivel na pista

  Serial.print("Calibrado. Mín: ");
  Serial.print(limiarmin);
  Serial.print(" | Máx: ");
  Serial.println(limiarmax);
}
//////////////////////////////
/////////GIROSCOPIO/////////// 
////tudo preciso até agora////
//////////////////////////////

void GIrosetup() {//envestigar oq faz o carrinho dar ré quando vai sunir uma ladeira ou lombada
  Serial.begin(9600);

  mpu_begin();

  Serial.println("Calibrando, deixa parado!");
  delay(1000);
  mpu_calibrate(200);
  Serial.println("===== Calibrado! =====\n");

  mpu_reset();
}

void GIroloop() {
 mpu_loop();

  //Serial.print("roll(x):");Serial.println(getAngleX());
  //Serial.print("pitch(y):");Serial.println(getAngleY());
  //Serial.print("yaw(z):");
  Serial.print("GIRO: ");
  Serial.print(getAngleZ());
  Serial.print("||");
  Serial.print("inclinação LATERAL: ");
   Serial.print(getAngleX());
   Serial.print("||");
   Serial.print("inclinação LADEIRA: ");
   Serial.print(getAngleY());
   Serial.println("");

}
 

