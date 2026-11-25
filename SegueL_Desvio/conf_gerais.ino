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

// Anda devagar para frente (usa BACKWARD)
void andarFrente(int velEsq, int velDir) {
  if (velEsq > 223) velEsq = 223;
  if (velDir > 205) velDir = 205;
  if (velEsq < 0) velEsq = 0;
  if (velDir < 0) velDir = 0;

  motorEsq.setSpeed(velEsq);
  motorDir.setSpeed(velDir);
  motorEsq.run(BACKWARD);  // BACKWARD = frente
  motorDir.run(BACKWARD);
}

// Anda devagar para trás (usa FORWARD)
void andarTras(int velEsq, int velDir) {
  if (velEsq > 223) velEsq = 223;
  if (velDir > 205) velDir = 205;
  if (velEsq < 0) velEsq = 0;
  if (velDir < 0) velDir = 0;

  motorEsq.setSpeed(velEsq);
  motorDir.setSpeed(velDir);
  motorEsq.run(FORWARD);   // FORWARD = trás
  motorDir.run(FORWARD);
}


void pararMotores() {
  motorEsq.setSpeed(0);
  motorDir.setSpeed(0);
  motorEsq.run(RELEASE);
  motorDir.run(RELEASE);
}

void Motores90E() {
  motorEsq.setSpeed(150);
  motorDir.setSpeed(180);
  motorEsq.run(FORWARD);
  motorDir.run(BACKWARD);
}

void Motores90D() {
  motorEsq.setSpeed(150);
  motorDir.setSpeed(150);
  motorEsq.run(BACKWARD);
  motorDir.run(FORWARD);
}


// Giro à esquerda
void girarEsquerda90() {
  Serial.println("Virando 90° à esquerda...");
  GIrosetup();       // inicializa e calibra
  mpu_reset();       // zera o ângulo
  while (getAngleZ() > -82) {   // condição para esquerda
    mpu_loop();                 // atualiza leitura do giroscópio
    Motores90E();               // função que aciona motores para girar esquerda
    Serial.print("Ângulo atual: ");
    Serial.println(getAngleZ());
  }
  pararMotores();
  Serial.println("Giro à esquerda concluído!");
}

// Giro à direita
void girarDireita90() {
  Serial.println("Virando 90° à direita...");
  GIrosetup();       // inicializa e calibra
  mpu_reset();       // zera o ângulo
  while (getAngleZ() < 82) {    // condição para direita
    mpu_loop();                 // atualiza leitura do giroscópio
    Motores90D();               // função que aciona motores para girar direita
    Serial.print("Ângulo atual: ");
    Serial.println(getAngleZ());
  }
  pararMotores();
  Serial.println("Giro à direita concluído!");
}


/////////////////
//FREIA motores//
/////////////////

//se estiver indo para trás
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
  leituraQTR(); // atualiza estado dos sensores

  int posicao = calcularPosicaoLinha();
  erro = posicao - offset;

  correcao = Kp * erro + Kd * (erro - erroAnterior);

  int velocidadeEsq = velocidadeBase - correcao;
  int velocidadeDir = velocidadeBase + correcao;

  moverMotores(velocidadeEsq, velocidadeDir + 40);
  erroAnterior = erro;

  // DEBUG opcional:
  // Serial.print("Erro: ");
  // Serial.println(erro);
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
    int valor = map(constrain(leitura, limiarmin, limiarmax), limiarmin, limiarmax, 0, 1000);

    somaPesos += (long)valor * pesos[i];
    somaLeituras += valor;
  }

  if (somaLeituras == 0) return 0;
  return somaPesos / somaLeituras;
}

/////////////////////
///LEITURA do QTR////
/////////////////////
void leituraQTR() {
  for (int i = 0; i < 8; i++) {
    int leitura = analogRead(sensores[i]);
    if (leitura < limiarmin) estadoSensores[i] = 0; // branco
    else if (leitura < limiarmax) estadoSensores[i] = 1; // preto
  }

  // Tudo preto
  TudoPreto = true;
  for (int i = 0; i < 8; i++) {
    if (estadoSensores[i] == 0) { TudoPreto = false; break; }
  }

  // Tudo branco
  TudoBranco = true;
  for (int i = 0; i < 8; i++) {
    if (estadoSensores[i] == 1) { TudoBranco = false; break; }
  }

  // Curva 90º esquerda: D1-D3 brancos e D4-D8 pretos
  bool d1a3_brancos = true;
  for (int i = 0; i <= 2; i++) if (estadoSensores[i] == 1) { d1a3_brancos = false; break; }
  bool d4a8_pretos = true;
  for (int i = 3; i <= 7; i++) if (estadoSensores[i] == 0) { d4a8_pretos = false; break; }
  Na_90ESQUERDA = (d1a3_brancos && d4a8_pretos);

  // Curva 90º direita: D1-D5 pretos e D6-D8 brancos
  bool d1a5_pretos = true;
  for (int i = 0; i <= 4; i++) if (estadoSensores[i] == 0) { d1a5_pretos = false; break; }
  bool d6a8_brancos = true;
  for (int i = 5; i <= 7; i++) if (estadoSensores[i] == 1) { d6a8_brancos = false; break; }
  Na_90DIREITA = (d1a5_pretos && d6a8_brancos);

  // Controle da curva esquerda
  static bool jaSaiuDa90E = true;
  if (!Na_90ESQUERDA && !TudoBranco) jaSaiuDa90E = true;
  if (Na_90ESQUERDA && jaSaiuDa90E) {
    curva90E_executada = false;
    jaSaiuDa90E = false;
  }

  // Controle da curva direita
  static bool jaSaiuDa90D = true;
  if (!Na_90DIREITA && !TudoBranco) jaSaiuDa90D = true;
  if (Na_90DIREITA && jaSaiuDa90D) {
    curva90D_executada = false;
    jaSaiuDa90D = false;
  }
}

/////////////////////////////////
///CALIBRAÇÃO dos sensores
/////////////////////////////////
void calibrarSensores() {
  int leitura;
  int minimo = 1023;
  int maximo = 0;

  for (int i = 0; i < 100; i++) {
    for (int j = 0; j < 8; j++) { // calibrar todos os sensores
      leitura = analogRead(sensores[j]);
      if (leitura < minimo) minimo = leitura;
      if (leitura > maximo) maximo = leitura;
    }
    delay(5);
  }

  limiarmin = minimo - 10;
  limiarmax = maximo + 10;

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
  mpu_calibrate(292);
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


//////////////////////
// Funções auxiliares
//////////////////////

// Seguir reto por tempo
void seguirReto(int tempo) {
  moverMotores(180, 180);  // usa função genérica
  delay(tempo);
  freiaMotoresN();         // freio suave ao parar
}

// Recuar até distância segura
void recuarAteDistancia(int limiteCM) {
  Serial.print("Recuando até ");
  Serial.print(limiteCM);
  Serial.println(" cm...");
  motorEsq.setSpeed(180);
  motorDir.setSpeed(180);
  motorEsq.run(FORWARD);   // FORWARD = trás
  motorDir.run(FORWARD);
  while (ultrassom.Ranging(CM) < limiteCM) {
    delay(50);
  }
  freiaMotoresS();  // freio suave ao parar
}

// Seguir até objeto
void seguirAteObjeto(int limiteCM) {
  moverMotores(180, 180);  // avanço
  while (ultrassom.Ranging(CM) > limiteCM) {
    delay(50);
  }
  freiaMotoresN();  // freio suave ao parar
}

