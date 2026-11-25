// ===================================================================
// SALA BRANCA (ÁREA DE RESGATE) - ENTRADA (fita prata) / SAÍDA (fita preta)
// ===================================================================
// Estratégia: varredura fixa em zigue-zague, sem sensor, cobrindo a sala.
// Ao ver a fita PRATA (entrada) o robô interrompe o seguidor de linha,
// faz a varredura, e ao achar a fita PRETA (saída) volta a seguir linha
// normalmente (o loop principal não precisa saber que isso aconteceu).
//
// Usa só os voids de movimento que já existem no projeto:
// andarFrente(), girarEsquerda90(), girarDireita90(), freiaMotoresN(),
// pararMotores(). Nenhuma lib nova, nenhum pino novo.

// -----------------------------------------------------------------
// DETECÇÃO DA FITA PRATA (entrada) e DA FITA PRETA (saída)
// -----------------------------------------------------------------
// A fita PRATA é detectada pelos sensores RGB (TCS34725), no mesmo
// padrão do verde/vermelho. detectouFitaPrata() e confirmarFitaPrata()
// estão na aba RGB.ino — preencha lá as constantes S1_PRATA_R/G/B e
// S2_PRATA_R/G/B com os valores lidos no DEBUG em cima da fita prata.
//
// A fita PRETA de saída continua usando o QTR (TudoPreto), igual já
// era usado no seguidor de linha.

bool detectouFitaPretaSaida() {
  // reaproveita o TudoPreto que o leituraQTR() já calcula todo ciclo
  return TudoPreto;
}

// -----------------------------------------------------------------
// VARREDURA EM ZIGUE-ZAGUE
// -----------------------------------------------------------------
// >>> AJUSTAR conforme o tamanho real da sala:
const int NUM_FAIXAS = 4;             // quantas "fileiras" o robô varre
const int TEMPO_PERNA_LONGA = 1800;   // ms andando ao longo de cada fileira
const int TEMPO_PASSO_LATERAL = 350;  // ms do passo curto entre fileiras
const int VEL_VARREDURA = 150;

void varreduraSalaBranca() {
  pararMotores();
  delay(100);

  // confirma a fita prata com votação antes de comprometer a varredura inteira
  if (!confirmarFitaPrata()) {
    Serial.println("Falso positivo de prata - voltando a seguir linha");
    return;
  }

  Serial.println("=== Entrada confirmada (fita prata) - iniciando varredura ===");

  bool proximaViradaDireita = true;  // alterna o lado do zigue-zague

  for (int faixa = 0; faixa < NUM_FAIXAS; faixa++) {

    // ---- perna longa da fileira ----
    andarFrente(VEL_VARREDURA, VEL_VARREDURA);
    unsigned long inicio = millis();
    while (millis() - inicio < TEMPO_PERNA_LONGA) {
      leituraQTR();
      if (detectouFitaPretaSaida()) {
        freiaMotoresN();
        Serial.println("Fita preta encontrada durante a varredura - saindo!");
        return;
      }
    }
    freiaMotoresN();
    delay(100);

    if (faixa == NUM_FAIXAS - 1) break;  // última fileira não precisa virar

    // ---- troca de fileira: vira, passo curto, vira de novo ----
    if (proximaViradaDireita) {
      girarDireita90();
      andarFrente(VEL_VARREDURA, VEL_VARREDURA);
      delay(TEMPO_PASSO_LATERAL);
      freiaMotoresN();
      girarDireita90();
    } else {
      girarEsquerda90();
      andarFrente(VEL_VARREDURA, VEL_VARREDURA);
      delay(TEMPO_PASSO_LATERAL);
      freiaMotoresN();
      girarEsquerda90();
    }
    proximaViradaDireita = !proximaViradaDireita;
  }

  // Não achou a saída durante o zigue-zague: gira 180 e volta procurando
  Serial.println("Varredura concluída sem achar a saída - voltando para procurar...");
  girar180();
  unsigned long inicioBusca = millis();
  while (!detectouFitaPretaSaida() && (millis() - inicioBusca < 8000)) {
    leituraQTR();
    andarFrente(VEL_VARREDURA, VEL_VARREDURA);
  }
  freiaMotoresN();
  Serial.println("=== Saída encontrada (fita preta) - retomando seguidor de linha ===");
}
