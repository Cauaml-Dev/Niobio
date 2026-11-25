
// Pinos dos sensores QTR
int sensores[] = { A15, A14, A13, A12, A11, A10, A9, A8 };

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.print("Leituras QTR: ");
  for (int i = 0; i < 8; i++) {
    int leitura = analogRead(sensores[i]);
    Serial.print(leitura);
    Serial.print(" ");
  }
  Serial.println();

  delay(200); // pequena pausa para não inundar o Serial
}
