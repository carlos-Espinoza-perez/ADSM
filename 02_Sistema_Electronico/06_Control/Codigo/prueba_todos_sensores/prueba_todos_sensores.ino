#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <AS5600.h>
#include "HX711.h"

Adafruit_MPU6050 mpu;
AS5600 as5600;

const int DT_TALON = 16;
const int SCK_TALON = 17;
const int DT_PUNTA = 4;
const int SCK_PUNTA = 13;

HX711 hxTalon;
HX711 hxPunta;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);

  if (!mpu.begin()) {
    Serial.println("MPU6050 no responde");
  } else {
    Serial.println("MPU6050 OK");
  }

  as5600.begin();
  if (!as5600.isConnected()) {
    Serial.println("AS5600 no responde");
  } else {
    Serial.println("AS5600 OK");
  }

  hxTalon.begin(DT_TALON, SCK_TALON);
  hxPunta.begin(DT_PUNTA, SCK_PUNTA);

  Serial.println("Iniciando lectura conjunta...");
  delay(1000);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.println("---------------------------");
  Serial.print("MPU6050  accX: "); Serial.print(a.acceleration.x, 2);
  Serial.print("  accY: "); Serial.print(a.acceleration.y, 2);
  Serial.print("  accZ: "); Serial.println(a.acceleration.z, 2);

  int angulo = as5600.readAngle();
  Serial.print("AS5600   angulo crudo: "); Serial.println(angulo);



  delay(500);
}
