// Prueba básica del GA25-370 con el TB6612FNG
// STBY está puenteado directo a 3.3V por ahora (sin fines de carrera aún)

const int PWM = 25;
const int IN1 = 26;
const int IN2 = 27;

void setup() {
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  Serial.begin(115200);
  delay(500);
  Serial.println("Prueba de motor iniciada");
}

void girarAdelante(int velocidad) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(PWM, velocidad); // 0-255
}

void girarAtras(int velocidad) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(PWM, velocidad);
}

void detener() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(PWM, 0);
}

void loop() {
  Serial.println("Girando adelante...");
  girarAdelante(150); // velocidad baja para la primera prueba
  delay(2000);

  Serial.println("Deteniendo...");
  detener();
  delay(1000);

  Serial.println("Girando atras...");
  girarAtras(150);
  delay(2000);

  Serial.println("Deteniendo...");
  detener();
  delay(2000);
}
