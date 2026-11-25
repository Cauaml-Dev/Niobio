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