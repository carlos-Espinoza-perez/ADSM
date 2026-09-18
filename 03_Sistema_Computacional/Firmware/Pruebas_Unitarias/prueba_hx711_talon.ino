#include "HX711.h"

const int DT = 16;
const int SCK_PIN = 17;

HX711 balanza;

void setup() {
  Serial.begin(115200);
  balanza.begin(DT, SCK_PIN);
  Serial.println("Iniciando... no toques la celda");
  delay(2000);
}

void loop() {
  if (balanza.is_ready()) {
    long valor = balanza.read();
    Serial.print("Valor crudo: ");
    Serial.println(valor);
  } else {
    Serial.println("HX711 no responde, revisar DT/SCK/VCC/GND");
  }
  delay(300);
}
