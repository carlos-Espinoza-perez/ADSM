// Barrido de PWM para encontrar velocidad minima (arranque) y maxima del GA25-370
// Usa el encoder (canal A con interrupcion, canal B para referencia) para contar pulsos

const int PWM = 25;
const int IN1 = 26;
const int IN2 = 27;
const int ENC_A = 34;
const int ENC_B = 35;

volatile long pulsos = 0;

void IRAM_ATTR contarPulso() {
  pulsos++;
}

void girarAdelante(int velocidad) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(PWM, velocidad);
}

void detener() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(PWM, 0);
}

void setup() {
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A), contarPulso, RISING);

  Serial.begin(115200);
  delay(1000);
  Serial.println("Barrido de PWM iniciado");
  Serial.println("PWM , pulsos_en_1.5s , pulsos_por_segundo");
}

void probarNivel(int pwmVal) {
  pulsos = 0;
  girarAdelante(pwmVal);
  delay(1500); // tiempo de estabilizacion + ventana de conteo
  long conteo = pulsos;
  detener();
  delay(800); // pausa antes del siguiente nivel

  float pulsosPorSeg = conteo / 1.5;
  Serial.print(pwmVal);
  Serial.print(" , ");
  Serial.print(conteo);
  Serial.print(" , ");
  Serial.println(pulsosPorSeg);
}

void loop() {
  // Barrido de arranque: valores bajos para encontrar el PWM minimo que mueve el eje
  Serial.println("--- Buscando PWM minimo de arranque ---");
  for (int v = 20; v <= 120; v += 10) {
    probarNivel(v);
  }

  // Barrido completo hasta velocidad maxima
  Serial.println("--- Barrido completo hasta velocidad maxima ---");
  for (int v = 120; v <= 255; v += 15) {
    probarNivel(v);
  }

  Serial.println("=== Barrido terminado, repitiendo en 5s ===");
  delay(5000);
}
