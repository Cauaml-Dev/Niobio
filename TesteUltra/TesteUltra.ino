#include <Ultrasonic.h>   // Biblioteca para o sensor ultrassônico

// Cria o objeto do sensor ultrassônico usando A0 (TRIG) e A1 (ECHO)
Ultrasonic ultrassom(A0, A1);

long distancia;           // Variável para armazenar a distância

void setup() {
  Serial.begin(9600);     // Inicializa a comunicação serial
  Serial.println("Iniciando leitura do sensor...");
}

void loop() {
  // Lê a distância em centímetros
  distancia = ultrassom.Ranging(CM);

  // Exibe apenas a distância no monitor serial
  Serial.print("Distância = ");
  Serial.print(distancia);
  Serial.println(" cm");

  delay(1000); // Aguarda 1 segundo antes da próxima leitura
}
