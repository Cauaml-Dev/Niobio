#include <Wire.h>

const int MPU = 0x68;

// ================== FILTRO ==================
#define FILTRO_N 5

float accX_buffer[FILTRO_N];
float accY_buffer[FILTRO_N];
float accZ_buffer[FILTRO_N];
int filtroIndex = 0;

// ============================================

float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;

float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;

float roll, pitch, yaw;
float elapsedTime, currentTime, previousTime;

// ================= SETUP ===================
void mpu_begin() {
  // Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x1C);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x1B);
  Wire.write(0);
  Wire.endTransmission(true);

  delay(20);
}

// ================= ACELERÔMETRO FILTRADO =================
void readAcel() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  float rawX = (Wire.read() << 8 | Wire.read()) / 16384.0f;
  float rawY = (Wire.read() << 8 | Wire.read()) / 16384.0f;
  float rawZ = (Wire.read() << 8 | Wire.read()) / 16384.0f;

  accX_buffer[filtroIndex] = rawX;
  accY_buffer[filtroIndex] = rawY;
  accZ_buffer[filtroIndex] = rawZ;

  filtroIndex = (filtroIndex + 1) % FILTRO_N;

  AccX = 0;
  AccY = 0;
  AccZ = 0;

  for (int i = 0; i < FILTRO_N; i++) {
    AccX += accX_buffer[i];
    AccY += accY_buffer[i];
    AccZ += accZ_buffer[i];
  }

  AccX /= FILTRO_N;
  AccY /= FILTRO_N;
  AccZ /= FILTRO_N;
}

// ================= GIRO =================
void readGiro() {
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);

  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0f;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0f;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0f;

  // DEADZONE (anti vibração)
  if (abs(GyroX) < 0.5) GyroX = 0;
  if (abs(GyroY) < 0.5) GyroY = 0;
  if (abs(GyroZ) < 0.5) GyroZ = 0;
}

// ================= CALIBRAÇÃO =================
void mpu_calibrate(int leituras) {

  AccErrorX = 0;
  AccErrorY = 0;
  GyroErrorX = 0;
  GyroErrorY = 0;
  GyroErrorZ = 0;

  for (int nR = 0; nR < leituras; nR++) {
    readAcel();
    AccErrorX += (atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI);
    AccErrorY += (atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI);
  }

  AccErrorX /= leituras;
  AccErrorY /= leituras;

  for (int nR = 0; nR < leituras; nR++) {
    readGiro();
    GyroErrorX += GyroX;
    GyroErrorY += GyroY;
    GyroErrorZ += GyroZ;
  }

  GyroErrorX /= leituras;
  GyroErrorY /= leituras;
  GyroErrorZ /= leituras;
}

// ================= LOOP =================
void mpu_loop() {

  readAcel();

  // Acelerômetro já com erro subtraído
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX;
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;

  previousTime = currentTime;
  currentTime = micros();
  elapsedTime = (currentTime - previousTime) / 1000000.0f;

  if (elapsedTime > 0.05) elapsedTime = 0.05;

  readGiro();

  GyroX -= GyroErrorX;
  GyroY -= GyroErrorY;
  GyroZ -= GyroErrorZ;

  gyroAngleX += GyroX * elapsedTime;
  gyroAngleY += GyroY * elapsedTime;
  gyroAngleZ += GyroZ * elapsedTime;

  // MUDANÇA 1: mais peso no acelerômetro (0.05) para corrigir drift
  float newRoll  = 0.95 * gyroAngleX + 0.05 * accAngleX;
  float newPitch = 0.95 * gyroAngleY + 0.05 * accAngleY;

  roll  = 0.9 * roll  + 0.1 * newRoll;
  pitch = 0.9 * pitch + 0.1 * newPitch;

  yaw = gyroAngleZ;
}

// ================= RESET =================
void mpu_reset() {
  gyroAngleX = 0;
  gyroAngleY = 0;
  gyroAngleZ = 0;

  currentTime = micros();
  mpu_loop();
}

// ================= GETTERS =================
// MUDANÇA 2: retorna accAngle diretamente — sem drift, leitura instantânea real
float getAngleX() { return accAngleX; }
float getAngleY() { return accAngleY; }
float getAngleZ() { return yaw; }        // yaw ainda usa giroscópio (acelerômetro não mede yaw)

int getDeltaZ(int target) {
  if (target < 0) return (round(getAngleZ() - target));
  return (round(target - getAngleZ()));
}