 #include <Wire.h>
#include "Adafruit_TCS34725_DUE.h"

/* Inicializa com os valores padrões(int time = 2.4ms, gain = 1x) */
// Adafruit_TCS34725_W0 sensor1 = Adafruit_TCS34725_W0();
// Adafruit_TCS34725_W1 sensor2 = Adafruit_TCS34725_W1();

//------------
bool S1;//SERVE PARA SABER SE O SENSOR FOI DETECTADO OU NÃO// S = 0 --> SENSOR NAO ENCONTRADO S = 1 --> SENSOR ENCONTRADO
bool S2;//
//------------

/* Inicializa com um 'int time' e 'gain values' especificos */
Adafruit_TCS34725_W0 sensor1 = Adafruit_TCS34725_W0(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_4X);  
Adafruit_TCS34725_W1 sensor2 = Adafruit_TCS34725_W1(TCS34725_INTEGRATIONTIME_700MS2, TCS34725_GAIN_4X2);

//TABELA DE VELOCIDADES e GANHOS:

//SENSOR1:                               

//  TCS34725_INTEGRATIONTIME_2_4MS       TCS34725_GAIN_1X
//  TCS34725_INTEGRATIONTIME_24MS        TCS34725_GAIN_4X
//  TCS34725_INTEGRATIONTIME_50MS        TCS34725_GAIN_16X
//  TCS34725_INTEGRATIONTIME_101MS       TCS34725_GAIN_60X
//  TCS34725_INTEGRATIONTIME_154MS   
//  TCS34725_INTEGRATIONTIME_700MS

//SENSOR2:

//  TCS34725_INTEGRATIONTIME_2_4MS2      TCS34725_GAIN_1X2
//  TCS34725_INTEGRATIONTIME_24MS2       TCS34725_GAIN_4X2
//  TCS34725_INTEGRATIONTIME_50MS2       TCS34725_GAIN_16X2
//  TCS34725_INTEGRATIONTIME_101MS2      TCS34725_GAIN_60X2
//  TCS34725_INTEGRATIONTIME_154MS2   
//  TCS34725_INTEGRATIONTIME_700MS2



void setup(void) {
  Serial.begin(9600);
  delay(1000);
  
}

void loop(void) {


   if (sensor1.begin()) { S1=1; }
 else { S1=0; }
    
   if (sensor2.begin()) { S2=1; }
 else { S2=0; }

 
  
  if (S1 == 1)
  {
  uint16_t r, g, b, c;
  
  sensor1.getRawData(&r, &g, &b, &c);
  
  Serial.print("R1: "); Serial.print(r, DEC); Serial.print("\t");
  Serial.print("G1: "); Serial.print(g, DEC); Serial.print("\t");
  Serial.print("B1: "); Serial.print(b, DEC); Serial.print("\t");
  Serial.println("    ");
  }else{
  Serial.print("R1: 0\t"); Serial.print("\t");
  Serial.print("G1: 0\t"); Serial.print("\t");
  Serial.print("B1: 0\t"); Serial.print("\t");
  Serial.println("    ");  
       }

    if (S2 == 1){
  uint16_t r2, g2, b2, c2;
  
  sensor2.getRawData2(&r2, &g2, &b2, &c2);
  
  Serial.print("R2: "); Serial.print(r2, DEC); Serial.print(" \t");
  Serial.print("G2: "); Serial.print(g2, DEC); Serial.print(" \t");
  Serial.print("B2: "); Serial.print(b2, DEC); Serial.print(" \t");
  Serial.println("    ");
  Serial.println("/-----------------------------------------/  ");
  }else{
  Serial.print("R2: 0\t"); Serial.print("\t");
  Serial.print("G2: 0\t"); Serial.print("\t");
  Serial.print("B2: 0\t"); Serial.print("\t");
  Serial.println("    ");
  Serial.println("/-----------------------------------------/  ");
       }
}

