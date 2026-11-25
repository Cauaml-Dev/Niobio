
// ===== Setup e Loop =====
void setup() {
GIrosetup();
}

void loop() {
  GIroloop();
  Serial.print("Roll (X): ");
  Serial.print(getAngleX());
  Serial.print(" || Pitch (Y): ");
  Serial.print(getAngleY());
  Serial.print(" || Yaw (Z): ");
  Serial.println(getAngleZ());
  delay(200); // só para facilitar leitura no Serial Monitor
}
