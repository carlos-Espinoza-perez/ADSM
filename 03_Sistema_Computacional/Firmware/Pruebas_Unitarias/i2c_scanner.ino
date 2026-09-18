#include <Wire.h>

void setup() {
  Wire.begin(21, 22); // SDA = GPIO21, SCL = GPIO22
  Serial.begin(115200);
  delay(1000);
  Serial.println("Escaner I2C iniciado");
}

void loop() {
  byte error, address;
  int encontrados = 0;

  Serial.println("Buscando dispositivos I2C...");

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Dispositivo encontrado en 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);

      if (address == 0x68) Serial.print("  <- MPU6050");
      if (address == 0x36) Serial.print("  <- AS5600");

      Serial.println();
      encontrados++;
    }
  }

  if (encontrados == 0) {
    Serial.println("No se encontro ningun dispositivo. Revisar VCC/GND/SDA/SCL.");
  } else {
    Serial.print(encontrados);
    Serial.println(" dispositivo(s) encontrado(s).");
  }

  Serial.println();
  delay(3000);
}
