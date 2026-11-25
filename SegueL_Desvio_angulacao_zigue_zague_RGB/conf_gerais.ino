/////////////
  ///MOTORES///
  ////todos////
  /////////////
  void moverMotores(int velEsq, int velDir) {

    if (velEsq > 215) velEsq = 215;
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
    if (velEsq > 215) velEsq = 215;
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
    motorDir.setSpeed(165);
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

  // Giro de 180° (meia-volta)
  void girar180() {
    Serial.println("Virando 180°...");
    GIrosetup();       // inicializa e calibra o giroscópio
    mpu_reset();       // zera o ângulo

    // gira até atingir aproximadamente 180°
    while (getAngleZ() < 175) {   // condição para meia-volta à direita
      mpu_loop();                 // atualiza leitura do giroscópio
      Motores90D();               // função que aciona motores para girar à direita
      Serial.print("Ângulo atual: ");
      Serial.println(getAngleZ());
    }

    pararMotores();
    Serial.println("Giro de 180° concluído!");
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

  // Limita a correção máxima - evita o "pulso" violento no instante
  // em que a linha está sumindo (poucos sensores ativos = erro extremo),
  // que estava torcendo o robô antes de entrar no gap
  if (correcao > 150) correcao = 150;
  if (correcao < -150) correcao = -150;

  // Reduz velocidade proporcionalmente ao erro - só entra em curvas fechadas
  // (S, arredondadas, ou só 1 sensor na borda), não afeta a reta. Dá mais
  // ciclos de loop pro PID corrigir antes do robô sair da linha.
  int velocidadeAjustada = velocidadeBase;
  int erroAbs = abs(erro);
  if (erroAbs > 500) {
    velocidadeAjustada = velocidadeBase - (erroAbs - 500) / 10;
    if (velocidadeAjustada < 70) velocidadeAjustada = 70; // não deixa cair demais
  }

  int velocidadeEsq = velocidadeAjustada - correcao;
  int velocidadeDir = velocidadeAjustada + correcao;

  moverMotores(velocidadeEsq, velocidadeDir + 50);
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
  // Leitura dos sensores QTR (D1–D8)
  void leituraQTR() {
    for (int i = 0; i < 8; i++) {
      int leitura = analogRead(sensores[i]);

      if (leitura < limiarmin) {
        estadoSensores[i] = 0;  // branco
      }
      else if (leitura >= limiarmin && leitura <= limiarmax) {
        estadoSensores[i] = 1;  // preto
      }
      else {
        estadoSensores[i] = 0;  // fora do range, considera branco
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

    // Detecta curva 90º esquerda: D1–D3 brancos (tolera 1 errado) e D4–D8 pretos (tolera 1 errado)
    int errosEsq1 = 0;
    for (int i = 0; i <= 2; i++) {
      if (estadoSensores[i] == 1) errosEsq1++;
    }
    int errosEsq2 = 0;
    for (int i = 3; i <= 7; i++) {
      if (estadoSensores[i] == 0) errosEsq2++;
    }
    Na_90ESQUERDA = (errosEsq1 <= 1 && errosEsq2 <= 1);

    // Detecta curva 90º direita: D1–D5 pretos (tolera 1 errado) e D6–D8 brancos (tolera 1 errado)
    int errosDir1 = 0;
    for (int i = 0; i <= 4; i++) {
      if (estadoSensores[i] == 0) errosDir1++;
    }
    int errosDir2 = 0;
    for (int i = 5; i <= 7; i++) {
      if (estadoSensores[i] == 1) errosDir2++;
    }
    Na_90DIREITA = (errosDir1 <= 1 && errosDir2 <= 1);

    // Libera a flag curva90E_executada apenas quando sair da curva anterior
    static bool jaSaiuDa90E = true;
    if (!Na_90ESQUERDA && !TudoBranco) jaSaiuDa90E = true;
    if (Na_90ESQUERDA && jaSaiuDa90E) {
      curva90E_executada = false;
      jaSaiuDa90E = false;
    }

    // Libera a flag curva90D_executada apenas quando sair da curva anterior
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

    // Calibra usando SOMENTE os sensores centrais (índices 3 e 4),
    // que por definição sempre estarão sobre a linha preta no início,
    // independente de quantos sensores estão cobrindo a linha.
    int sensoresCalibragem[] = { 3, 4 };

    for (int i = 0; i < 100; i++) {
      for (int k = 0; k < 2; k++) {
        int j = sensoresCalibragem[k];
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
    moverMotores(180, 185);  // 180 * (205/187) ≈ 197 compensa o motor direito
    delay(tempo);
    freiaMotoresN();
  }

  // Recuar até distância segura (com timeout de segurança pra não travar pra sempre)
  bool recuarAteDistancia(int limiteCM, unsigned long timeoutMs) {
    Serial.print("Recuando até ");
    Serial.print(limiteCM);
    Serial.println(" cm...");
    motorEsq.setSpeed(180);
    motorDir.setSpeed(197);  // mesmo ajuste proporcional
    motorEsq.run(FORWARD);
    motorDir.run(FORWARD);

    unsigned long inicio = millis();
    while (ultrassom.Ranging(CM) < limiteCM) {
      if (millis() - inicio > timeoutMs) {
        freiaMotoresS();
        Serial.println("Timeout no recuo - seguindo sequência mesmo assim.");
        return false;
      }
      delay(50);
    }
    freiaMotoresS();
    return true;
  }

  // Seguir até objeto (com timeout de segurança - retorna false se foi falso alarme)
  bool seguirAteObjeto(int limiteCM, unsigned long timeoutMs) {
    moverMotores(180, 197);  // mesmo ajuste

    unsigned long inicio = millis();
    while (ultrassom.Ranging(CM) > limiteCM) {
      if (millis() - inicio > timeoutMs) {
        freiaMotoresN();
        Serial.println("Timeout - obstáculo não confirmado, falso alarme!");
        return false;
      }
      delay(50);
    }
    freiaMotoresN();
    return true;
  }

  // ================= FILTRO DE MEDIANA - ULTRASSOM =================
  // Descarta ruído de vibração / eco duplo fazendo 3 leituras rápidas
  // e usando a mediana (ignora o valor extremo em vez de diluir ele numa média).
  long lerDistanciaFiltrada() {
    long a = ultrassom.Ranging(CM);
    delayMicroseconds(200); // pequena pausa pra não pegar eco residual do pulso anterior
    long b = ultrassom.Ranging(CM);
    delayMicroseconds(200);
    long c = ultrassom.Ranging(CM);

    // leitura 0 = erro de eco do sensor; trata como "sem detecção" (valor alto)
    // pra não ser confundida com obstáculo perto
    if (a == 0) a = 999;
    if (b == 0) b = 999;
    if (c == 0) c = 999;

    long maior = max(a, max(b, c));
    long menor = min(a, min(b, c));
    long mediana = a + b + c - maior - menor;

    return mediana;
  }

  bool linhaEmZigueZague() {
    // Sensores laterais ativos → indica linha em diagonal
    return (estadoSensores[0] > limiarmin || estadoSensores[7] > limiarmin) &&
          !(TudoPreto) && !(TudoBranco);
  }

  bool encruzilhadaDetectada() {
    // Exemplo de lógica: linha presente nos sensores centrais e também nos laterais
    // Ajuste conforme sua leitura QTR e disposição dos sensores
    bool linhaCentro = (estadoSensores[3] == 1 || estadoSensores[4] == 1); // sensores centrais
    bool linhaEsquerda = (estadoSensores[0] == 1 || estadoSensores[1] == 1); // sensores à esquerda
    bool linhaDireita = (estadoSensores[6] == 1 || estadoSensores[7] == 1); // sensores à direita

    // Encruzilhada = linha no centro + pelo menos um lado ativo
    if (linhaCentro && (linhaEsquerda || linhaDireita)) {
      return true;
    }
    return false;
  }


  bool curvaEmVDetectada() {
    // Curva em V = sensores centrais ativos + laterais também ativos
    bool centro = (estadoSensores[3] == 1 || estadoSensores[4] == 1);
    bool esquerda = (estadoSensores[0] == 1 || estadoSensores[1] == 1);
    bool direita  = (estadoSensores[6] == 1 || estadoSensores[7] == 1);

    if (centro && (esquerda || direita)) {
      return true;
    }
    return false;
  }


  void tratarCurvaEmV() {
    Serial.println("Curva em V detectada - desacelerando para PID reagir");

    velocidadeBase = 60; // reduz bastante a velocidade

    unsigned long inicio = millis();
    while (millis() - inicio < 1500) {   // mantém por ~1,5s
      leituraQTR();   // atualiza sensores
      segueL();       // PID corrige
      delay(25);      // dá tempo para leitura e reação
    }

    velocidadeBase = 150; // restaura velocidade normal
  }


  // Função para recuar até reencontrar curva à esquerda
  void recuarAteCurvaEsquerda() {
    Serial.println("Recuando até reencontrar curva à esquerda...");
    while (!Na_90ESQUERDA) {
      leituraQTR();
      motorEsq.setSpeed(80);
      motorDir.setSpeed(80);
      motorEsq.run(FORWARD);   // recua
      motorDir.run(FORWARD);
    }
    freiaMotoresS();
    Serial.println("Curva à esquerda reencontrada!");
  }

  // Função para recuar até reencontrar curva à direita
  void recuarAteCurvaDireita() {
    Serial.println("Recuando até reencontrar curva à direita...");
    while (!Na_90DIREITA) {
      leituraQTR();
      motorEsq.setSpeed(80);
      motorDir.setSpeed(80);
      motorEsq.run(FORWARD);   // recua
      motorDir.run(FORWARD);
    }
    freiaMotoresS();
    Serial.println("Curva à direita reencontrada!");
  }


  bool linhaFrenteDetectada() {
    bool centro = (estadoSensores[3] == 1 || estadoSensores[4] == 1); // um ou os dois centrais
    bool esquerda = (estadoSensores[0] == 1 || estadoSensores[1] == 1);
    bool direita  = (estadoSensores[6] == 1 || estadoSensores[7] == 1);

    return (centro && (esquerda || direita));
  }


// void verificarCurva90() {
//   pararMotores();
//   delay(200);
//   leituraQTR();

//   bool verdeDir = false, verdeEsq = false;
//   verificarVerde(verdeDir, verdeEsq);

//   if (verdeDir && verdeEsq) {
//     Serial.println("Meia volta verde detectada!");
//     girar180();
//   } else if (verdeEsq && !verdeDir) {
//     Serial.println("Curva verde à esquerda!");
//     girarEsquerda90();
//   } else if (verdeDir && !verdeEsq) {
//     Serial.println("Curva verde à direita!");
//     girarDireita90();
//   } else {
//     Serial.println("Sem verde - avançando pra reavaliar");
//     andarFrente(100, 100);
//     delay(1000);
//     pararMotores();
//     leituraQTR();

//     if (!TudoBranco) {
//       Serial.println("Linha à frente confirmada - seguindo");
//       velocidadeBase = VELOCIDADE_NORMAL;
//     } else {
//       while (TudoBranco) {
//         andarTras(100, 100);
//         leituraQTR();
//       }
//       freiaMotoresS();
//       delay(300);
//       leituraQTR();

//       // Usa a posição ponderada da linha (mesma lógica do PID) pra decidir
//       // o lado real - evita empates que causavam viés pro lado errado
//       int posicao = calcularPosicaoLinha();

//       if (posicao < 0) {
//         Serial.println("Preto à esquerda - virando esquerda");
//         girarEsquerda90();
//       } else {
//         Serial.println("Preto à direita - virando direita");
//         girarDireita90();
//       }
//     }
//   }
// }