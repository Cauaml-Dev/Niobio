
/////////////
///MOTORES///
/////////////

// Movimento genérico com limite de velocidade
void moverMotores(int velEsq, int velDir) {
  if (velEsq > 223) velEsq = 223;
  if (velDir > 205) velDir = 205;
  if (velEsq < 0) velEsq = 0;
  if (velDir < 0) velDir = 0;

  motorEsq.setSpeed(velEsq);
  motorDir.setSpeed(velDir);

  motorEsq.run(FORWARD);
  motorDir.run(FORWARD);
}

// Parar motores
void pararMotores() {
  motorEsq.setSpeed(0);
  motorDir.setSpeed(0);
  motorEsq.run(RELEASE);
  motorDir.run(RELEASE);
}

// Freio suave indo para trás
void freiaMotoresS() {
  motorEsq.setSpeed(150);
  motorDir.setSpeed(125);
  motorEsq.run(BACKWARD);
  motorDir.run(BACKWARD);
  delay(50);
  pararMotores();
}

// Freio suave indo para frente
void freiaMotoresN() {
  motorEsq.setSpeed(150);
  motorDir.setSpeed(125);
  motorEsq.run(FORWARD);
  motorDir.run(FORWARD);
  delay(50);
  pararMotores();
}

//////////////////////
// Funções auxiliares
//////////////////////

// Seguir reto por tempo
void seguirReto(int tempo) {
  moverMotores(180, 180);  // usa função genérica
  delay(tempo);
  freiaMotoresN();  // freio suave ao parar
}

// Giro à esquerda
void girarEsquerda90() {
  Serial.println("Virando 90° à esquerda...");
  GIrosetup();
  GIroloop();
  while (getAngleZ() < 85) {
    GIroloop();
    motorEsq.setSpeed(150);
    motorDir.setSpeed(180);
    motorEsq.run(FORWARD);
    motorDir.run(BACKWARD);
    Serial.print("Ângulo atual: ");
    Serial.println(getAngleZ());
  }
  pararMotores();
  Serial.println("Giro à esquerda concluído!");
}

// Giro à direita
void girarDireita90() {
  Serial.println("Virando 90° à direita...");
  GIrosetup();
  GIroloop();
  while (getAngleZ() > -85) {
    GIroloop();
    motorEsq.setSpeed(180);
    motorDir.setSpeed(150);
    motorEsq.run(BACKWARD);
    motorDir.run(FORWARD);
    Serial.print("Ângulo atual: ");
    Serial.println(getAngleZ());
  }
  pararMotores();
  Serial.println("Giro à direita concluído!");
}
// Recuar até distância
void recuarAteDistancia(int limiteCM) {
  Serial.print("Recuando até ");
  Serial.print(limiteCM);
  Serial.println(" cm...");
  motorEsq.setSpeed(180);
  motorDir.setSpeed(180);
  // Ajuste solicitado: usar BACKWARD no motorEsq e FORWARD no motorDir
  motorEsq.run(BACKWARD);
  motorDir.run(BACKWARD);
  while (ultrassom.Ranging(CM) < limiteCM) {
    delay(50);
  }
  freiaMotoresS();  // freio suave ao parar
}


// Seguir até objeto
void seguirAteObjeto(int limiteCM) {
  moverMotores(180, 180);  // avanço
  while (ultrassom.Ranging(CM) > limiteCM) delay(50);
  freiaMotoresN();  // freio suave ao parar
}

//////////////////////////////
/////////GIROSCOPIO///////////
////tudo preciso até agora////
//////////////////////////////
void GIrosetup() {  //envestigar oq faz o carrinho dar ré quando vai sunir uma ladeira ou lombada
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