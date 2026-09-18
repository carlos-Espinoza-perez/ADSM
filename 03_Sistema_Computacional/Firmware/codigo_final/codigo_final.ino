/*
  Tobillo protesico activo - firmware v6

  ESP32, GA25-370 con encoder integrado, AS5600, MPU6050, 2 microswitches,
  TB6612FNG con canales A y B en paralelo.

  Posicion angular: contador de pulsos del encoder. Tabla de 5 puntos
  (-30, -15, 0, +15, +30) con interpolacion lineal por tramo.
  AS5600: verificacion cruzada por magnitud de campo, no realimentacion.
  Configuracion y tabla en NVS, se cambian por comando sin reflashear.

  Requisitos de hardware:
    - Acople eje-husillo con prisionero o pasador. Si patina, el encoder
      cuenta y la articulacion no se mueve, y el error sobrevive al apagado.
    - Pull-ups de 10k en GPIO34 y GPIO35: son entrada pura, sin pull-up interna.

  Tres modos de operacion:
    - Sensores: el terreno sale de la inclinacion tibial menos el angulo de
      articulacion, promediado durante la fase de apoyo plano.
    - Terreno inyectado: el valor llega por MQTT o serial y el lazo hace una
      correccion punto a punto. Clasificadores identicos al modo anterior.
    - Paso simulado: un secuenciador recorre las ocho subfases del ciclo de
      marcha sobre una base de tiempo estirada y el tobillo sigue la
      trayectoria angular completa, apoyo incluido. Solo para banco, sin
      carga: no difiere el movimiento por contacto.

  Convencion de signo: dorsiflexion positiva, plantiflexion negativa.

  Comandos por serial o por MQTT. '?' para la lista.
*/

#include <WiFiManager.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Preferences.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <math.h>
#include <string.h>

// --- Pines ---

#define SDA_PIN 21
#define SCL_PIN 22
#define AS5600_ADDR 0x36
#define MPU6050_ADDR 0x68

#define REG_RAW_ANGLE_H 0x0C
#define REG_MAGNITUDE_H 0x1B
#define REG_AGC 0x1A

#define BOTON_RESET 0
#define TIEMPO_RESET_MS 3000

#define PIN_TALON 32
#define PIN_PUNTA 33
#define DEBOUNCE_MS 35

#define PIN_ENC_A 34
#define PIN_ENC_B 35

#define PIN_AIN1 25
#define PIN_AIN2 26
#define PIN_PWM 27
#define PIN_STBY 14

#define PWM_FREQ 20000
#define PWM_RES 8

// Invertir si el mecanismo se mueve al reves de lo pedido.
#define SIGNO_MOTOR 1

// 0 = flancos de subida de A. 1 = ambos flancos de A, duplica los pulsos
// por grado y obliga a recalibrar la tabla.
#define DECODIF_2X 0

// --- Red ---

#define PREFS_NAMESPACE "tobillo"
#define PREFS_KEY_BROKER "brokerIp"
#define PREFS_KEY_TABLA "tablaPul"
#define PREFS_KEY_ANGULO "angUlt"
#define PREFS_KEY_PULSOS "pulsUlt"
#define PREFS_KEY_LIMPIO "cierreOk"
#define PREFS_KEY_PWMMIN "pwmMin"
#define PREFS_KEY_KP "kp"
#define PREFS_KEY_SALIDA "extSalida"
#define PREFS_KEY_TERRENO "sgTerreno"
#define PREFS_KEY_GANANCIA "ganObj"
#define PREFS_KEY_CICLO "msCiclo"
#define PREFS_KEY_AMPLITUD "amplPaso"

#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "tobilloESP32"
#define MQTT_RETRY_MS 5000
#define MQTT_BUFFER 2560

#define TOPIC_ESTADO "tobillo/estado"
#define TOPIC_COMANDO "tobillo/comando"
#define TOPIC_TERRENO "tobillo/terreno"
#define TOPIC_TELEMETRIA "tobillo/telemetria"
#define TOPIC_CICLO "tobillo/ciclo"
#define TOPIC_PASO "tobillo/paso"
#define TOPIC_LOG "tobillo/log"

// --- Control ---

#define PWM_KICK 255
#define KICK_MAX_MS 150
#define MOV_CONFIRMA_PULSOS 8         // ~0.3 deg a 26 pulsos/grado
#define PWM_MAX 200
#define MS_FRENO 40
#define CONTROL_MS 20

#define ZONA_MUERTA_CFG 2.0           // inercia tras el freno: ~0.8 deg
#define PULSOS_ZONA_MUERTA 3
#define TICKS_ASENTADO 3
#define MAX_INVERSIONES 2

#define TIMEOUT_AJUSTE_MS 1200
#define TIMEOUT_TOTAL_MS 8000
#define STALL_MS 200
#define STALL_PULSOS 10               // < 2 deg/s en ventana de 200 ms
#define ENFRIAMIENTO_MS 2000
#define MAX_BLOQUEOS 3

// Al alcanzar MAX_BLOQUEOS el sistema se libera solo tras esta espera, mas
// larga que el enfriamiento normal. Si hace falta varias veces seguidas sin
// un movimiento exitoso en medio, deja de liberarse: eso ya no es un falso
// positivo, es algo mecanico que hay que revisar a mano.
#define ENFRIAMIENTO_LIMITE_MS 5000
#define MAX_AUTOLIBERAR 3

#define LIMITE_ANGULO 30.0
#define MARGEN_PULSOS 25              // ~1 deg mas alla del limite
#define JOG_MAX_GRADOS 5.0
#define PULSOS_POR_GRADO_NOMINAL 26.0 // medido con inclinometro: ~25.7
#define JOG_MS 1500
#define TOPE_MAX_INTENTOS 40

// --- Verificacion cruzada ---

#define DISCREPANCIA_MAX 8.0
#define VERIFICACION_MS 500
#define MAGNITUD_MINIMA 10
#define DERIVA_AGC 40
#define N_PROM_MAG 6
#define RANGO_VERIFICACION 15.0       // la curva de magnitud llega hasta 15 deg
#define SENS_MIN_MAG 0.8              // unidades de magnitud por grado
#define ZONA_CIEGA_VERIF 5.0          // cerca del cero la curva es plana

// --- Terreno ---

#define EJE_SAGITAL 2
#define SIGNO_TIBIA 1.0
#define OFFSET_TIBIA 0.0
#define RETARDO_ESTABILIZACION_MS 30
#define MIN_MUESTRAS_PLANO 2
#define TOLERANCIA_GRAVEDAD 0.35      // relajado para manipulacion manual

// --- Simulacion de terreno ---

#define LIMITE_TERRENO_SIM 30.0
#define SERIE_MAX 12
#define SERIE_PAUSA_MS 1500

// --- Paso simulado ---

// El ciclo humano dura ~1.1 s. El prebalanceo pide del orden de 270 deg/s
// y el mecanismo da unos 35, asi que la base de tiempo va estirada. Esto
// reproduce la forma de la trayectoria y la secuencia de subfases, no la
// dinamica de la marcha real.
#define MS_CICLO_MIN 1000
#define MS_CICLO_MAX 10000
#define MS_CICLO_DEFECTO 4000

#define AMPLITUD_MIN 0.2
#define AMPLITUD_MAX 1.2

#define ZONA_SEGUIMIENTO 1.0          // banda sin actuacion en seguimiento
#define PAUSA_PASOS_MS 800
#define PASO_RESUMEN_MS 120

// Reintentos mientras el sistema se libera solo, para que un bloqueo
// transitorio no mate una caminata entera.
#define REINTENTO_CAMINATA_MS 1000
#define MAX_REINTENTOS_CAMINATA 12

// --- Clasificadores ---

#define N_CLASES 5
#define HISTERESIS 1.5
#define FUZZY_ANCHO 7.5

#define TELEMETRIA_MS 100
#define VEL_VENTANA_MS 100

// --- Tabla de posicion ---

// La interpolacion lineal entre puntos deja del orden de 0.5 a 1 deg de
// error en el medio de cada tramo, por la curvatura del eslabonamiento.
#define N_REF 5
#define IDX_CERO 2
const float refAngulos[N_REF] = {-30, -15, 0, 15, 30};

long refPulsos[N_REF] = {0, 0, 0, 0, 0};
bool tablaValida = false;

// Magnitud de campo del AS5600 contra valor absoluto del angulo. Monotona
// decreciente en cada rama. Solo para verificacion cruzada.
#define N_MAG 7
const float magAbsAngulo[N_MAG] = {0, 3, 5, 8, 10, 12, 15};
const float magRamaPos[N_MAG]   = {41.6, 39.4, 36.6, 32.7, 28.3, 24.6, 21.2};
const float magRamaNeg[N_MAG]   = {41.6, 41.5, 39.7, 37.3, 33.9, 30.8, 26.4};

// --- Trayectoria del ciclo de marcha ---

// Subfases segun la terminologia de Rancho Los Amigos, como fraccion del
// ciclo medido de contacto de talon a contacto del mismo talon.
// Apoyo 0 a 60 por ciento, balanceo 60 a 100.
#define PCT_RESP_CARGA 0.10
#define PCT_MEDIO_APOYO 0.30
#define PCT_APOYO_TERM 0.50
#define PCT_DESPEGUE 0.60
#define PCT_BAL_INICIAL 0.73
#define PCT_MEDIO_BAL 0.87

// Puntos de la trayectoria angular del tobillo. Dorsiflexion positiva.
// Neutro al contacto, plantiflexion en la respuesta de carga, dorsiflexion
// creciente pasado el medio apoyo con maximo antes del contacto opuesto,
// plantiflexion rapida en el despegue y vuelta a neutro en el balanceo.
// Interpolacion lineal entre puntos: aproxima la forma, no la curva exacta.
#define N_TRAY 8
const float trayPct[N_TRAY] = {0.00, 0.08, 0.30, 0.48, 0.60, 0.75, 0.87, 1.00};
const float trayAng[N_TRAY] = {0.0, -5.0,  2.0,  9.0, -14.0, -4.0,  0.0,  0.0};

// --- Configuracion persistente ---

int pwmMin = 110;              // duty minimo de sostenimiento
float kpControl = 1.2;         // duty por grado de error
int extremoSalida = 1;         // lado donde el husillo sale de la tuerca
float signoTerreno = 1.0;      // convencion de compensacion
float gananciaObjetivo = 1.0;  // multiplica la salida del clasificador
unsigned long msCicloPaso = MS_CICLO_DEFECTO;
float amplitudPaso = 0.80;     // escala la trayectoria de marcha
int dutyJog = 140;             // velocidad de los jogs, no se guarda

// --- Estado de red ---

Preferences prefs;
char brokerIp[16] = "192.168.1.1";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
bool wifiHabilitado = false;
bool telemetriaContinua = true;

// --- Consola con espejo en MQTT ---

// Las lineas impresas se encolan y se publican desde loop(), nunca desde
// dentro de un print, para no meter latencia de red en el lazo de control.

#define LOG_LINEAS 12
#define LOG_LARGO 160

class ConsolaMqtt : public Stream {
  HardwareSerial* hw;
  char cola[LOG_LINEAS][LOG_LARGO];
  int entrada = 0;
  int salida = 0;
  int largoActual = 0;

public:
  ConsolaMqtt(HardwareSerial* p) : hw(p) { cola[0][0] = '\0'; }

  void begin(unsigned long baud) { hw->begin(baud); }

  int available() override { return hw->available(); }
  int read() override { return hw->read(); }
  int peek() override { return hw->peek(); }
  void flush() override { hw->flush(); }

  size_t write(uint8_t c) override {
    hw->write(c);
    if (c == '\r') return 1;
    if (c == '\n') { cerrarLinea(); return 1; }
    if (largoActual < LOG_LARGO - 1) {
      cola[entrada][largoActual++] = (char)c;
    } else {
      cerrarLinea();
      cola[entrada][largoActual++] = (char)c;
    }
    return 1;
  }

  size_t write(const uint8_t* b, size_t n) override {
    for (size_t i = 0; i < n; i++) write(b[i]);
    return n;
  }

  void cerrarLinea() {
    if (largoActual == 0) return;
    cola[entrada][largoActual] = '\0';
    largoActual = 0;
    int siguienteIdx = (entrada + 1) % LOG_LINEAS;
    if (siguienteIdx == salida) salida = (salida + 1) % LOG_LINEAS;
    entrada = siguienteIdx;
  }

  bool pendiente() { return entrada != salida; }

  const char* sacarLinea() {
    if (entrada == salida) return NULL;
    const char* p = cola[salida];
    salida = (salida + 1) % LOG_LINEAS;
    return p;
  }

  void descartar() { salida = entrada; }
};

ConsolaMqtt Consola(&Serial);
#define Serial Consola

unsigned long ultimoIntentoMqtt = 0;
unsigned long ultimaTelemetria = 0;
unsigned long ultimoControl = 0;
unsigned long ultimaVerificacion = 0;

// --- Encoder ---

volatile long pulsosEncoder = 0;

void IRAM_ATTR isrEncoder() {
#if DECODIF_2X
  if (digitalRead(PIN_ENC_A) == digitalRead(PIN_ENC_B)) pulsosEncoder--;
  else pulsosEncoder++;
#else
  if (digitalRead(PIN_ENC_B)) pulsosEncoder++;
  else pulsosEncoder--;
#endif
}

// --- Posicion ---

float anguloArticulacion = 0.0;
bool posicionConfiable = false;
bool fueraDeTabla = false;

float velAngular = 0.0;        // deg/s de la articulacion, con signo
float velPulsosSeg = 0.0;      // pulsos/s del encoder, con signo
long pulsosVentana = 0;
float anguloVentana = 0.0;
unsigned long tVentanaVel = 0;

// --- AS5600 ---

float bufMagnitud[N_PROM_MAG];
int idxMag = 0;
bool magLleno = false;
int magnitudActual = -1;
int agcActual = -1;
uint8_t agcReferencia = 0;
float anguloVerificacion = 0.0;
bool verificacionValida = false;
float discrepanciaActual = 0.0;
bool divergenciaDetectada = false;
bool imanDesalineado = false;

// --- MPU6050 ---

float anguloTibia = 0.0;
bool tibiaValida = false;
float moduloAcc = 1.0;

// --- Contactos y fases ---

bool contactoTalon = false;
bool contactoPunta = false;

#define BALANCEO 0
#define CONTACTO_TALON 1
#define APOYO_PLANO 2
#define DESPEGUE 3

int fase = BALANCEO;
int fasePrevia = BALANCEO;
unsigned long tEntradaFase = 0;
unsigned long duracionFasePrevia = 0;

// --- Terreno ---

float terrenoEstimado = 0.0;
bool terrenoValido = false;
float acumTerreno = 0.0;
int muestrasPlano = 0;
int muestrasRechazadas = 0;

bool simTerreno = false;
float terrenoInyectado = 0.0;
bool origenSimulado = false;

float serieValores[SERIE_MAX];
int serieN = 0;
int serieIdx = 0;
bool serieActiva = false;
int serieFallos = 0;
unsigned long serieEsperaHasta = 0;

// --- Paso simulado ---

bool pasoActivo = false;
unsigned long tInicioPaso = 0;
unsigned long numeroPaso = 0;
float terrenoPaso = 0.0;
float offsetTerreno = 0.0;
float offsetPrevio = 0.0;
float offsetNuevo = 0.0;
float offsetInicialPaso = 0.0;
bool offsetAplicado = false;
float setpointPaso = 0.0;
float pctPaso = 0.0;
const char* subfasePaso = "inactivo";

// Estadisticas de seguimiento del paso en curso.
long muestrasPaso = 0;
float acumErr2Paso = 0.0;
float errMaxPaso = 0.0;
float pctErrMaxPaso = 0.0;
float velPicoPaso = 0.0;
int pwmPicoPaso = 0;

float caminataValores[SERIE_MAX];
int caminataN = 0;
int caminataIdx = 0;
int caminataReintentos = 0;
int caminataFallos = 0;
bool caminataActiva = false;
unsigned long caminataEsperaHasta = 0;

// --- Clasificadores ---

enum ModoClasificador { CLASIF_HISTERESIS, CLASIF_DIFUSO };
ModoClasificador modoActual = CLASIF_HISTERESIS;
String metodoActivo = "histeresis";
bool modoManual = false;

const float centroClase[N_CLASES] = {-10, -5, 0, 5, 10};
const char* nombreClase[N_CLASES] = {"bajada10", "bajada5", "plano", "subida5", "subida10"};
int claseActual = 2;
int claseEmitida = 2;

// --- Lazo de ajuste ---

float anguloObjetivo = 0.0;
float objetivoPendiente = 0.0;

int pwmAplicado = 0;
int sentidoAplicado = 0;
bool motorActivo = false;
const char* razonCorte = "arranque";
unsigned long frenoHasta = 0;

// Pico retenido entre publicaciones de telemetria. El PWM es un pulso y las
// correcciones duran menos que el periodo de publicacion, asi que el valor
// instantaneo se pierde entre muestras.
int pwmPicoVentana = 0;
int sentidoPicoVentana = 0;

bool bloqueoDetectado = false;
int bloqueosConsecutivos = 0;
unsigned long tBloqueo = 0;

bool autoLiberar = true;
int autoLiberaciones = 0;
bool latchPermanente = false;

bool ajustePendiente = false;
bool ajusteEnCurso = false;
unsigned long tInicioAjuste = 0;
unsigned long tFaseMov = 0;
unsigned long msDiferidos = 0;
unsigned long tDiferidoDesde = 0;

int sentidoActual = 0;
int inversiones = 0;
bool arrancado = false;
long pulsosAlArrancar = 0;
int ticksEnZona = 0;
long pulsosRef = 0;
unsigned long tPulsosRef = 0;

// Metricas del ciclo en curso, para las graficas de Node-RED.
unsigned long latenciaArranqueMs = 0;
bool latenciaTomada = false;
float velPicoCiclo = 0.0;
float sobrepasoCiclo = 0.0;
float errorInicialCiclo = 0.0;
int signoErrorInicial = 0;
int pwmPicoCiclo = 0;

unsigned long numeroCiclo = 0;
unsigned long ciclosDescartados = 0;

bool esperandoAsentamiento = false;
unsigned long tAsentamiento = 0;
bool exitoPendiente = false;
const char* cierrePendiente = "";
unsigned long duracionPendiente = 0;
int inversionesPendiente = 0;

char lineaSerial[64];
int idxLinea = 0;

void cerrarAjuste(bool exito, const char* cierre, unsigned long duracion);
void guardarPosicion();
void inyectarTerreno(float t);
void terminarPaso(const char* cierre);

// --- Tabla de posicion ---

bool tablaCompleta() {
  if (refPulsos[IDX_CERO] != 0) return false;
  for (int i = 0; i < N_REF; i++) {
    if (i != IDX_CERO && refPulsos[i] == 0) return false;
  }
  return true;
}

bool tablaEsMonotona() {
  int signoRef = 0;
  for (int i = 0; i < N_REF - 1; i++) {
    long d = refPulsos[i + 1] - refPulsos[i];
    if (d == 0) return false;
    int sg = (d > 0) ? 1 : -1;
    if (signoRef == 0) signoRef = sg;
    else if (sg != signoRef) return false;
  }
  return true;
}

int puntosFaltantes() {
  int n = 0;
  for (int i = 0; i < N_REF; i++) {
    if (i != IDX_CERO && refPulsos[i] == 0) n++;
  }
  return n;
}

void revalidarTabla() {
  tablaValida = tablaCompleta() && tablaEsMonotona();
}

float anguloDesdePulsos(long p) {
  fueraDeTabla = false;
  if (!tablaValida) return 0.0;

  bool creciente = refPulsos[N_REF - 1] > refPulsos[0];

  if ((creciente && p <= refPulsos[0]) || (!creciente && p >= refPulsos[0])) {
    if (p != refPulsos[0]) fueraDeTabla = true;
    float m = (refAngulos[1] - refAngulos[0]) / (float)(refPulsos[1] - refPulsos[0]);
    return refAngulos[0] + m * (p - refPulsos[0]);
  }

  int u = N_REF - 1;
  if ((creciente && p >= refPulsos[u]) || (!creciente && p <= refPulsos[u])) {
    if (p != refPulsos[u]) fueraDeTabla = true;
    float m = (refAngulos[u] - refAngulos[u - 1]) / (float)(refPulsos[u] - refPulsos[u - 1]);
    return refAngulos[u] + m * (p - refPulsos[u]);
  }

  for (int i = 0; i < N_REF - 1; i++) {
    long a = refPulsos[i], b = refPulsos[i + 1];
    bool dentro = creciente ? (p >= a && p <= b) : (p <= a && p >= b);
    if (dentro) {
      float t = (float)(p - a) / (float)(b - a);
      return refAngulos[i] + t * (refAngulos[i + 1] - refAngulos[i]);
    }
  }
  return 0.0;
}

float pulsosPorGradoLocal() {
  if (!tablaValida) return 1.0;
  long p = pulsosEncoder;
  bool creciente = refPulsos[N_REF - 1] > refPulsos[0];

  for (int i = 0; i < N_REF - 1; i++) {
    long a = refPulsos[i], b = refPulsos[i + 1];
    bool dentro = creciente ? (p >= a && p <= b) : (p <= a && p >= b);
    if (dentro) {
      float dg = refAngulos[i + 1] - refAngulos[i];
      return fabs((b - a) / dg);
    }
  }
  float dg = refAngulos[1] - refAngulos[0];
  return fabs((refPulsos[1] - refPulsos[0]) / dg);
}

float zonaMuertaEfectiva() {
  float porResolucion = PULSOS_ZONA_MUERTA / pulsosPorGradoLocal();
  return (porResolucion > ZONA_MUERTA_CFG) ? porResolucion : ZONA_MUERTA_CFG;
}

long pulsosEnAngulo(float ang) {
  if (!tablaValida) return 0;
  if (ang <= refAngulos[0]) return refPulsos[0];
  if (ang >= refAngulos[N_REF - 1]) return refPulsos[N_REF - 1];

  for (int i = 0; i < N_REF - 1; i++) {
    if (ang >= refAngulos[i] && ang <= refAngulos[i + 1]) {
      float t = (ang - refAngulos[i]) / (refAngulos[i + 1] - refAngulos[i]);
      return refPulsos[i] + (long)(t * (refPulsos[i + 1] - refPulsos[i]));
    }
  }
  return 0;
}

// El lado de salida no se detecta por calado: el mecanismo se libera en vez
// de trabarse. El presupuesto de pulsos es la unica proteccion de ese lado.
long topePulsosSalida() {
  long extremo = pulsosEnAngulo(extremoSalida > 0 ? LIMITE_ANGULO : -LIMITE_ANGULO);
  bool creciente = refPulsos[N_REF - 1] > refPulsos[0];
  bool salidaEsAlza = (extremoSalida > 0) == creciente;
  return salidaEsAlza ? (extremo + MARGEN_PULSOS) : (extremo - MARGEN_PULSOS);
}

bool excedeTopeSalida(long p) {
  if (!tablaValida) return false;
  long tope = topePulsosSalida();
  bool creciente = refPulsos[N_REF - 1] > refPulsos[0];
  bool salidaEsAlza = (extremoSalida > 0) == creciente;
  return salidaEsAlza ? (p >= tope) : (p <= tope);
}

bool sentidoVaHaciaSalida(int sentido) {
  return (extremoSalida > 0) ? (sentido > 0) : (sentido < 0);
}

// --- Persistencia ---

void cargarBrokerIp() {
  prefs.begin(PREFS_NAMESPACE, true);
  String g = prefs.getString(PREFS_KEY_BROKER, brokerIp);
  g.toCharArray(brokerIp, sizeof(brokerIp));
  prefs.end();
}

void guardarBrokerIp(const char* ip) {
  prefs.begin(PREFS_NAMESPACE, false);
  prefs.putString(PREFS_KEY_BROKER, ip);
  prefs.end();
}

void cargarConfiguracion() {
  prefs.begin(PREFS_NAMESPACE, true);
  pwmMin = prefs.getInt(PREFS_KEY_PWMMIN, pwmMin);
  kpControl = prefs.getFloat(PREFS_KEY_KP, kpControl);
  extremoSalida = prefs.getInt(PREFS_KEY_SALIDA, extremoSalida);
  signoTerreno = prefs.getFloat(PREFS_KEY_TERRENO, signoTerreno);
  gananciaObjetivo = prefs.getFloat(PREFS_KEY_GANANCIA, gananciaObjetivo);
  msCicloPaso = prefs.getULong(PREFS_KEY_CICLO, msCicloPaso);
  amplitudPaso = prefs.getFloat(PREFS_KEY_AMPLITUD, amplitudPaso);
  prefs.end();
  dutyJog = pwmMin + 30;
}

void guardarConfiguracion() {
  prefs.begin(PREFS_NAMESPACE, false);
  prefs.putInt(PREFS_KEY_PWMMIN, pwmMin);
  prefs.putFloat(PREFS_KEY_KP, kpControl);
  prefs.putInt(PREFS_KEY_SALIDA, extremoSalida);
  prefs.putFloat(PREFS_KEY_TERRENO, signoTerreno);
  prefs.putFloat(PREFS_KEY_GANANCIA, gananciaObjetivo);
  prefs.putULong(PREFS_KEY_CICLO, msCicloPaso);
  prefs.putFloat(PREFS_KEY_AMPLITUD, amplitudPaso);
  prefs.end();
}

void cargarTabla() {
  prefs.begin(PREFS_NAMESPACE, true);
  size_t leido = prefs.getBytes(PREFS_KEY_TABLA, refPulsos, sizeof(refPulsos));
  prefs.end();
  if (leido != sizeof(refPulsos)) {
    for (int i = 0; i < N_REF; i++) refPulsos[i] = 0;
  }
  revalidarTabla();
}

void guardarTabla() {
  prefs.begin(PREFS_NAMESPACE, false);
  prefs.putBytes(PREFS_KEY_TABLA, refPulsos, sizeof(refPulsos));
  prefs.end();
  revalidarTabla();
}

void marcarMovimiento() {
  prefs.begin(PREFS_NAMESPACE, false);
  prefs.putBool(PREFS_KEY_LIMPIO, false);
  prefs.end();
}

void guardarPosicion() {
  prefs.begin(PREFS_NAMESPACE, false);
  prefs.putFloat(PREFS_KEY_ANGULO, anguloArticulacion);
  prefs.putLong(PREFS_KEY_PULSOS, pulsosEncoder);
  prefs.putBool(PREFS_KEY_LIMPIO, true);
  prefs.end();
}

// --- Puente y freno ---

void liberarFreno() {
  digitalWrite(PIN_AIN1, LOW);
  digitalWrite(PIN_AIN2, LOW);
  ledcWrite(PIN_PWM, 0);
  digitalWrite(PIN_STBY, LOW);
  frenoHasta = 0;
}

bool enFreno() { return frenoHasta != 0; }

void frenarMotor(const char* razon) {
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, HIGH);
  ledcWrite(PIN_PWM, 255);
  digitalWrite(PIN_STBY, HIGH);

  pwmAplicado = 0;
  sentidoActual = 0;
  sentidoAplicado = 0;
  motorActivo = false;
  razonCorte = razon;
  frenoHasta = millis() + MS_FRENO;
}

void atenderFreno() {
  if (enFreno() && millis() >= frenoHasta) liberarFreno();
}

void moverMotor(int sentido, int duty) {
  if (enFreno()) return;

  if (motorActivo && sentidoAplicado != 0 && sentido != sentidoAplicado) {
    frenarMotor("inversion sin freno previo");
    return;
  }

  int s = sentido * SIGNO_MOTOR;
  digitalWrite(PIN_STBY, HIGH);

  if (s > 0) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
  }

  ledcWrite(PIN_PWM, duty);
  pwmAplicado = sentido * duty;
  sentidoAplicado = sentido;
  motorActivo = true;
  razonCorte = "ninguna";

  if (duty > pwmPicoVentana) {
    pwmPicoVentana = duty;
    sentidoPicoVentana = sentido;
  }
}

// --- Velocidad angular ---

void actualizarVelocidad() {
  unsigned long ahora = millis();

  if (tVentanaVel == 0) {
    tVentanaVel = ahora;
    pulsosVentana = pulsosEncoder;
    anguloVentana = anguloArticulacion;
    return;
  }

  unsigned long dt = ahora - tVentanaVel;
  if (dt < VEL_VENTANA_MS) return;

  velAngular = (anguloArticulacion - anguloVentana) * 1000.0 / (float)dt;
  velPulsosSeg = (float)(pulsosEncoder - pulsosVentana) * 1000.0 / (float)dt;

  tVentanaVel = ahora;
  pulsosVentana = pulsosEncoder;
  anguloVentana = anguloArticulacion;

  if (ajusteEnCurso && fabs(velAngular) > fabs(velPicoCiclo)) {
    velPicoCiclo = velAngular;
  }
}

// --- AS5600 ---

int leerRegistro16(uint8_t regAlto) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(regAlto);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(AS5600_ADDR, 2) != 2) return -1;
  if (Wire.available() < 2) return -1;
  int raw = (Wire.read() << 8) | Wire.read();
  return raw & 0x0FFF;
}

int leerAgc() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(REG_AGC);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(AS5600_ADDR, 1) != 1) return -1;
  if (Wire.available() < 1) return -1;
  return Wire.read();
}

void muestrearMagnitud() {
  int mag = leerRegistro16(REG_MAGNITUDE_H);
  if (mag < 0) return;
  magnitudActual = mag;
  bufMagnitud[idxMag] = (float)mag;
  idxMag++;
  if (idxMag >= N_PROM_MAG) { idxMag = 0; magLleno = true; }
}

// Fuera del rango caracterizado o en tramos planos no se afirma nada.
bool anguloDesdeMagnitud(float mag, int signo, float &absAng) {
  const float* curva = (signo >= 0) ? magRamaPos : magRamaNeg;

  if (mag >= curva[0]) return false;
  if (mag <= curva[N_MAG - 1]) return false;

  for (int i = 0; i < N_MAG - 1; i++) {
    if (mag <= curva[i] && mag >= curva[i + 1]) {
      float t = (curva[i] - mag) / (curva[i] - curva[i + 1]);
      float dMag = fabs(curva[i] - curva[i + 1]);
      float dAng = magAbsAngulo[i + 1] - magAbsAngulo[i];
      if (dAng <= 0 || (dMag / dAng) < SENS_MIN_MAG) return false;

      absAng = magAbsAngulo[i] + t * (magAbsAngulo[i + 1] - magAbsAngulo[i]);
      return true;
    }
  }
  return false;
}

void verificarPosicion() {
  if (millis() - ultimaVerificacion < VERIFICACION_MS) return;
  ultimaVerificacion = millis();

  int agc = leerAgc();
  if (agc >= 0) agcActual = agc;

  int n = magLleno ? N_PROM_MAG : idxMag;
  if (n < 3) { verificacionValida = false; return; }

  float suma = 0;
  for (int i = 0; i < n; i++) suma += bufMagnitud[i];
  float mag = suma / n;

  bool campoDebil = (magnitudActual >= 0) && (magnitudActual < MAGNITUD_MINIMA);
  bool derivaAgc = (agc >= 0) && (abs(agc - (int)agcReferencia) > DERIVA_AGC);

  if (campoDebil || derivaAgc) {
    if (!imanDesalineado) {
      imanDesalineado = true;
      Serial.println();
      Serial.println("[aviso] AS5600 fuera de servicio como verificacion.");
      Serial.print("  AGC "); Serial.print(agc);
      Serial.print(" contra referencia "); Serial.print(agcReferencia);
      Serial.print(" | magnitud "); Serial.println(magnitudActual);
      Serial.println("  El control sigue con el encoder, sin verificacion cruzada.");
    }
    verificacionValida = false;
    return;
  }
  imanDesalineado = false;

  if (fabs(anguloArticulacion) < ZONA_CIEGA_VERIF) {
    verificacionValida = false;
    return;
  }
  if (fabs(anguloArticulacion) > RANGO_VERIFICACION) {
    verificacionValida = false;
    return;
  }

  int signo = (anguloArticulacion >= 0) ? 1 : -1;
  float absAng;
  if (!anguloDesdeMagnitud(mag, signo, absAng)) { verificacionValida = false; return; }

  anguloVerificacion = signo * absAng;
  verificacionValida = true;
  discrepanciaActual = fabs(anguloArticulacion - anguloVerificacion);

  if (posicionConfiable && discrepanciaActual > DISCREPANCIA_MAX) {
    if (!divergenciaDetectada) {
      divergenciaDetectada = true;
      Serial.println();
      Serial.println("[divergencia] Encoder y AS5600 no concuerdan.");
      Serial.print("  Encoder "); Serial.print(anguloArticulacion, 2);
      Serial.print(" deg | verificacion "); Serial.print(anguloVerificacion, 2);
      Serial.print(" deg | desacuerdo "); Serial.print(discrepanciaActual, 2);
      Serial.println(" deg");
      Serial.println("  Revisar el acople eje-husillo. Enviar 'cero' en neutro.");
    }
    posicionConfiable = false;
    if (motorActivo) frenarMotor("divergencia de sensores");
  }
}

// --- MPU6050 ---

void inicializarMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

bool leerMPU6050(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU6050_ADDR, 14) != 14) return false;
  if (Wire.available() < 14) return false;

  int16_t rax = (Wire.read() << 8) | Wire.read();
  int16_t ray = (Wire.read() << 8) | Wire.read();
  int16_t raz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  int16_t rgx = (Wire.read() << 8) | Wire.read();
  int16_t rgy = (Wire.read() << 8) | Wire.read();
  int16_t rgz = (Wire.read() << 8) | Wire.read();

  ax = rax / 16384.0; ay = ray / 16384.0; az = raz / 16384.0;
  gx = rgx / 131.0;   gy = rgy / 131.0;   gz = rgz / 131.0;
  return true;
}

float anguloTibiaGrados(float ax, float ay, float az) {
  float ejePlano = (EJE_SAGITAL == 1) ? ay : az;
  return SIGNO_TIBIA * (atan2(ejePlano, ax) * 180.0 / PI) - OFFSET_TIBIA;
}

// --- Contactos y fases ---

bool leerMicroswitch(uint8_t pin, bool &estable, bool &previa, unsigned long &tCambio) {
  bool lectura = (digitalRead(pin) == LOW);
  if (lectura != previa) { previa = lectura; tCambio = millis(); }
  else if (millis() - tCambio > DEBOUNCE_MS) estable = lectura;
  return estable;
}

void actualizarContactos() {
  static bool eT = false, pT = false, eP = false, pP = false;
  static unsigned long tT = 0, tP = 0;
  contactoTalon = leerMicroswitch(PIN_TALON, eT, pT, tT);
  contactoPunta = leerMicroswitch(PIN_PUNTA, eP, pP, tP);
}

int faseDesdeContactos() {
  if (contactoTalon && contactoPunta) return APOYO_PLANO;
  if (contactoTalon) return CONTACTO_TALON;
  if (contactoPunta) return DESPEGUE;
  return BALANCEO;
}

const char* nombreFase(int f) {
  switch (f) {
    case BALANCEO:       return "balanceo";
    case CONTACTO_TALON: return "contacto_talon";
    case APOYO_PLANO:    return "apoyo_plano";
    default:             return "despegue";
  }
}

void acumularTerreno() {
  if (simTerreno) return;
  if (fase != APOYO_PLANO) return;
  if (millis() - tEntradaFase < RETARDO_ESTABILIZACION_MS) return;
  if (!tibiaValida) return;

  if (fabs(moduloAcc - 1.0) > TOLERANCIA_GRAVEDAD) { muestrasRechazadas++; return; }
  acumTerreno += (anguloTibia - anguloArticulacion);
  muestrasPlano++;
}

// --- Clasificadores ---

int claseNominal(float t) {
  if (t < -7.5) return 0;
  if (t < -2.5) return 1;
  if (t <  2.5) return 2;
  if (t <  7.5) return 3;
  return 4;
}

float calcularObjetivoHisteresis(float terreno, int &claseOut) {
  float centro = centroClase[claseActual];
  float mitad = 2.5 + HISTERESIS;
  bool dentro = (terreno >= centro - mitad) && (terreno <= centro + mitad);

  if (claseActual == 0 && terreno <= centro + mitad) dentro = true;
  if (claseActual == 4 && terreno >= centro - mitad) dentro = true;
  if (!dentro) claseActual = claseNominal(terreno);

  claseOut = claseActual;
  return centroClase[claseOut];
}

float calcularObjetivoDifuso(float terreno, int &claseOut) {
  float sumaMu = 0, sumaPond = 0, mejorMu = -1;
  claseOut = 2;

  for (int i = 0; i < N_CLASES; i++) {
    float d = fabs(terreno - centroClase[i]);
    float mu = (d < FUZZY_ANCHO) ? (1.0 - d / FUZZY_ANCHO) : 0.0;
    sumaMu += mu;
    sumaPond += mu * centroClase[i];
    if (mu > mejorMu) { mejorMu = mu; claseOut = i; }
  }

  if (sumaMu < 0.001) {
    claseOut = (terreno < 0) ? 0 : 4;
    return centroClase[claseOut];
  }
  return sumaPond / sumaMu;
}

// Los dos clasificadores saturan en +-10 deg por construccion de los
// centros. La ganancia escala esa salida sin tocar la clasificacion.
float calcularObjetivo(float terreno) {
  float base = (modoActual == CLASIF_HISTERESIS)
             ? calcularObjetivoHisteresis(terreno, claseEmitida)
             : calcularObjetivoDifuso(terreno, claseEmitida);

  base *= gananciaObjetivo;
  if (base >  LIMITE_ANGULO) base =  LIMITE_ANGULO;
  if (base < -LIMITE_ANGULO) base = -LIMITE_ANGULO;
  return base;
}

// Evalua sin alterar el estado de la histeresis. Para diagnostico.
float objetivoTentativo(float terreno, int &claseOut) {
  int guardado = claseActual;
  float r = calcularObjetivo(terreno);
  claseOut = claseEmitida;
  claseActual = guardado;
  return r;
}

// --- Trayectoria de marcha ---

float anguloTrayectoria(float pct) {
  if (pct <= trayPct[0]) return trayAng[0];
  if (pct >= trayPct[N_TRAY - 1]) return trayAng[N_TRAY - 1];

  for (int i = 0; i < N_TRAY - 1; i++) {
    if (pct >= trayPct[i] && pct <= trayPct[i + 1]) {
      float t = (pct - trayPct[i]) / (trayPct[i + 1] - trayPct[i]);
      return trayAng[i] + t * (trayAng[i + 1] - trayAng[i]);
    }
  }
  return 0.0;
}

const char* subfaseDePct(float p) {
  if (p < 0.02)            return "contacto_inicial";
  if (p < PCT_RESP_CARGA)  return "respuesta_carga";
  if (p < PCT_MEDIO_APOYO) return "medio_apoyo";
  if (p < PCT_APOYO_TERM)  return "apoyo_terminal";
  if (p < PCT_DESPEGUE)    return "prebalanceo";
  if (p < PCT_BAL_INICIAL) return "balanceo_inicial";
  if (p < PCT_MEDIO_BAL)   return "medio_balanceo";
  return "balanceo_terminal";
}

// Excursion angular que pide la trayectoria con la amplitud actual, para
// avisar si no cabe en el recorrido mecanico.
float excursionTrayectoria() {
  float mn = trayAng[0], mx = trayAng[0];
  for (int i = 1; i < N_TRAY; i++) {
    if (trayAng[i] < mn) mn = trayAng[i];
    if (trayAng[i] > mx) mx = trayAng[i];
  }
  return (mx - mn) * amplitudPaso;
}

// --- Telemetria ---

bool publicarJson(const char* topic, JsonDocument &doc, size_t cap) {
  if (!wifiHabilitado || !mqttClient.connected()) return false;

  static char buffer[2000];
  if (cap > sizeof(buffer)) cap = sizeof(buffer);

  size_t largo = serializeJson(doc, buffer, cap);
  if (largo == 0 || largo >= cap) {
    Serial.print("[mqtt] Payload truncado, sin publicar en ");
    Serial.println(topic);
    return false;
  }
  return mqttClient.publish(topic, buffer, largo);
}

void publicarCiclo(float alcanzado, unsigned long duracion, bool exito,
                   const char* cierre, int inv) {
  static StaticJsonDocument<1024> doc;
  doc.clear();

  doc["ciclo"] = numeroCiclo;
  doc["simulado"] = origenSimulado;
  doc["metodoActivo"] = metodoActivo;
  doc["modoManual"] = modoManual;
  doc["clase"] = nombreClase[claseEmitida];
  doc["claseIndice"] = claseEmitida;
  doc["terreno"] = terrenoEstimado;

  doc["anguloObjetivo"] = anguloObjetivo;
  doc["anguloAlcanzado"] = alcanzado;
  doc["errorInicial"] = errorInicialCiclo;
  doc["errorFinal"] = alcanzado - anguloObjetivo;
  doc["sobrepaso"] = sobrepasoCiclo;
  doc["zonaMuerta"] = zonaMuertaEfectiva();

  doc["duracionMs"] = duracion;
  doc["latenciaMs"] = latenciaTomada ? (long)latenciaArranqueMs : -1;
  doc["msDiferidos"] = msDiferidos;

  doc["velPico"] = velPicoCiclo;
  doc["pwmPico"] = pwmPicoCiclo;
  doc["inversiones"] = inv;

  doc["exito"] = exito;
  doc["cierre"] = cierre;
  doc["pulsos"] = pulsosEncoder;
  doc["discrepancia"] = verificacionValida ? discrepanciaActual : -1.0;
  doc["ciclosDescartados"] = ciclosDescartados;
  doc["ganancia"] = gananciaObjetivo;
  doc["kp"] = kpControl;
  doc["pwmMin"] = pwmMin;

  publicarJson(TOPIC_CICLO, doc, 1024);
}

void publicarTelemetria() {
  static StaticJsonDocument<2048> doc;
  doc.clear();

  doc["angulo"] = anguloArticulacion;
  doc["pulsos"] = pulsosEncoder;
  doc["velAngular"] = velAngular;
  doc["velPulsos"] = velPulsosSeg;
  doc["posicionConfiable"] = posicionConfiable;
  doc["fueraDeTabla"] = fueraDeTabla;

  doc["anguloVerificacion"] = verificacionValida ? anguloVerificacion : 0.0;
  doc["discrepancia"] = verificacionValida ? discrepanciaActual : -1.0;
  doc["divergencia"] = divergenciaDetectada;
  doc["imanDesalineado"] = imanDesalineado;
  doc["agc"] = agcActual;
  doc["magnitud"] = magnitudActual;

  doc["anguloTibia"] = anguloTibia;
  doc["tibiaValida"] = tibiaValida;
  doc["moduloAcc"] = moduloAcc;

  doc["contactoTalon"] = contactoTalon;
  doc["contactoPunta"] = contactoPunta;
  doc["fase"] = nombreFase(fase);
  doc["faseIndice"] = fase;
  doc["faseAnterior"] = nombreFase(fasePrevia);
  doc["duracionFasePrevia"] = duracionFasePrevia;

  doc["terreno"] = terrenoEstimado;
  doc["terrenoValido"] = terrenoValido;
  doc["simTerreno"] = simTerreno;
  doc["terrenoInyectado"] = terrenoInyectado;
  doc["serieActiva"] = serieActiva;
  doc["serieIdx"] = serieIdx;
  doc["serieN"] = serieN;

  doc["pasoActivo"] = pasoActivo;
  doc["paso"] = numeroPaso;
  doc["pctCiclo"] = pasoActivo ? pctPaso * 100.0 : -1.0;
  doc["subfase"] = pasoActivo ? subfasePaso : "inactivo";
  doc["setpoint"] = pasoActivo ? setpointPaso : anguloObjetivo;
  doc["errSeguimiento"] = pasoActivo ? (setpointPaso - anguloArticulacion) : 0.0;
  doc["offsetTerreno"] = offsetTerreno;
  doc["msCiclo"] = msCicloPaso;
  doc["amplitud"] = amplitudPaso;
  doc["caminataActiva"] = caminataActiva;

  doc["clase"] = nombreClase[claseEmitida];
  doc["anguloObjetivo"] = anguloObjetivo;
  doc["error"] = anguloObjetivo - anguloArticulacion;
  doc["zonaMuerta"] = zonaMuertaEfectiva();

  doc["pwm"] = pwmAplicado;
  doc["pwmMag"] = abs(pwmAplicado);
  doc["sentidoMotor"] = sentidoAplicado;
  doc["pwmPicoVentana"] = pwmPicoVentana;
  doc["sentidoPico"] = sentidoPicoVentana;
  doc["motorActivo"] = motorActivo;
  doc["ajusteEnCurso"] = ajusteEnCurso;
  doc["inversiones"] = inversiones;
  doc["bloqueo"] = bloqueoDetectado;
  doc["bloqueosConsecutivos"] = bloqueosConsecutivos;
  doc["autoLiberar"] = autoLiberar;
  doc["autoLiberaciones"] = autoLiberaciones;
  doc["latchPermanente"] = latchPermanente;
  doc["razonCorte"] = razonCorte;

  doc["ciclo"] = numeroCiclo;
  doc["ciclosDescartados"] = ciclosDescartados;
  doc["metodoActivo"] = metodoActivo;
  doc["modoManual"] = modoManual;
  doc["tablaValida"] = tablaValida;
  doc["pwmMin"] = pwmMin;
  doc["kp"] = kpControl;
  doc["ganancia"] = gananciaObjetivo;

  publicarJson(TOPIC_TELEMETRIA, doc, 2000);

  // El pico se reinicia recien despues de publicarlo: cada muestra reporta
  // el maximo del intervalo, no el instante en que se tomo.
  pwmPicoVentana = 0;
  sentidoPicoVentana = 0;
}

void publicarPaso(const char* cierre, unsigned long duracion) {
  static StaticJsonDocument<1024> doc;
  doc.clear();

  float rms = (muestrasPaso > 0) ? sqrt(acumErr2Paso / (float)muestrasPaso) : -1.0;

  doc["paso"] = numeroPaso;
  doc["terreno"] = terrenoPaso;
  doc["metodoActivo"] = metodoActivo;
  doc["clase"] = nombreClase[claseEmitida];
  doc["claseIndice"] = claseEmitida;
  doc["offsetInicial"] = offsetInicialPaso;
  doc["offsetTerreno"] = offsetTerreno;

  doc["msCiclo"] = msCicloPaso;
  doc["duracionMs"] = duracion;
  doc["amplitud"] = amplitudPaso;
  doc["excursion"] = excursionTrayectoria();

  doc["errRms"] = rms;
  doc["errMax"] = errMaxPaso;
  doc["pctErrMax"] = pctErrMaxPaso;
  doc["muestras"] = muestrasPaso;

  doc["velPico"] = velPicoPaso;
  doc["pwmPico"] = pwmPicoPaso;
  doc["anguloFinal"] = anguloArticulacion;

  doc["cierre"] = cierre;
  doc["ganancia"] = gananciaObjetivo;
  doc["kp"] = kpControl;
  doc["pwmMin"] = pwmMin;

  publicarJson(TOPIC_PASO, doc, 1024);
}

void atenderLog() {
  if (!wifiHabilitado || !mqttClient.connected()) {
    Consola.descartar();
    return;
  }
  for (int i = 0; i < 3 && Consola.pendiente(); i++) {
    const char* l = Consola.sacarLinea();
    if (l && l[0]) mqttClient.publish(TOPIC_LOG, l);
  }
}

// --- Bloqueos ---

void activarBloqueo(const char* razon) {
  bloqueoDetectado = true;
  bloqueosConsecutivos++;
  tBloqueo = millis();

  Serial.print("[bloqueo] "); Serial.print(razon);
  Serial.print(" | consecutivos "); Serial.print(bloqueosConsecutivos);
  Serial.print(" de "); Serial.println(MAX_BLOQUEOS);

  if (bloqueosConsecutivos >= MAX_BLOQUEOS) {
    if (autoLiberar && !latchPermanente) {
      Serial.print("  Limite alcanzado. Se libera solo en ");
      Serial.print(ENFRIAMIENTO_LIMITE_MS / 1000);
      Serial.println(" s.");
    } else {
      Serial.println("  Limite alcanzado. Revisar el mecanismo y enviar 'liberar'.");
    }
  }
}

// Marca que hubo un movimiento que cerro bien: los bloqueos ya no son
// consecutivos y el contador de autoliberaciones vuelve a cero.
void registrarExito() {
  bloqueosConsecutivos = 0;
  autoLiberaciones = 0;
  latchPermanente = false;
}

void atenderEnfriamiento() {
  if (!bloqueoDetectado) return;

  if (bloqueosConsecutivos >= MAX_BLOQUEOS) {
    if (!autoLiberar || latchPermanente) return;
    if (millis() - tBloqueo < ENFRIAMIENTO_LIMITE_MS) return;

    autoLiberaciones++;

    if (autoLiberaciones > MAX_AUTOLIBERAR) {
      latchPermanente = true;
      Serial.println("[bloqueo] Se libero solo demasiadas veces sin un movimiento");
      Serial.println("  que cierre bien en medio. Queda trabado a proposito:");
      Serial.println("  revisar el mecanismo y enviar 'liberar'.");
      return;
    }

    bloqueoDetectado = false;
    bloqueosConsecutivos = 0;
    Serial.print("[bloqueo] Liberado automaticamente | autoliberacion ");
    Serial.print(autoLiberaciones); Serial.print(" de ");
    Serial.println(MAX_AUTOLIBERAR);
    return;
  }

  if (millis() - tBloqueo < ENFRIAMIENTO_MS) return;
  bloqueoDetectado = false;
  Serial.println("[bloqueo] Liberado tras enfriamiento.");
}

// --- Ciclo de ajuste ---

bool puedeMover(bool avisar) {
  if (!tablaValida) {
    if (avisar) Serial.println("Rechazado: tabla sin calibrar. Ver 'tabla'.");
    return false;
  }
  if (!posicionConfiable) {
    if (avisar) Serial.println("Rechazado: posicion no confiable. Alinear en neutro y enviar 'cero'.");
    return false;
  }
  if (latchPermanente) {
    if (avisar) Serial.println("Rechazado: trabado tras varias autoliberaciones. Enviar 'liberar'.");
    return false;
  }
  if (bloqueosConsecutivos >= MAX_BLOQUEOS) {
    if (avisar) Serial.println("Rechazado: limite de bloqueos, esperando autoliberacion.");
    return false;
  }
  return true;
}

void iniciarAjuste(float objetivo) {
  if (objetivo >  LIMITE_ANGULO) objetivo =  LIMITE_ANGULO;
  if (objetivo < -LIMITE_ANGULO) objetivo = -LIMITE_ANGULO;

  anguloObjetivo = objetivo;
  sentidoActual = 0;
  inversiones = 0;
  arrancado = false;
  pulsosAlArrancar = pulsosEncoder;
  ticksEnZona = 0;
  msDiferidos = 0;
  tDiferidoDesde = 0;
  pulsosRef = pulsosEncoder;
  tPulsosRef = millis();
  tInicioAjuste = millis();
  tFaseMov = millis();
  ajustePendiente = true;
  ajusteEnCurso = true;

  errorInicialCiclo = anguloObjetivo - anguloArticulacion;
  signoErrorInicial = (errorInicialCiclo > 0) ? 1 : -1;
  latenciaArranqueMs = 0;
  latenciaTomada = false;
  velPicoCiclo = 0.0;
  sobrepasoCiclo = 0.0;
  pwmPicoCiclo = 0;

  marcarMovimiento();

  Serial.print("[ajuste] objetivo "); Serial.print(anguloObjetivo, 1);
  Serial.print(" deg | actual "); Serial.print(anguloArticulacion, 2);
  Serial.print(" deg | error "); Serial.print(errorInicialCiclo, 2);
  Serial.print(" deg | zona muerta "); Serial.print(zonaMuertaEfectiva(), 2);
  Serial.println(" deg");
}

void cerrarAjuste(bool exito, const char* cierre, unsigned long duracion) {
  ajusteEnCurso = false;
  ajustePendiente = false;
  esperandoAsentamiento = true;
  tAsentamiento = millis();
  exitoPendiente = exito;
  cierrePendiente = cierre;
  duracionPendiente = duracion;
  inversionesPendiente = inversiones;
  if (exito) registrarExito();
  else if (serieActiva) serieFallos++;
}

void atenderAsentamiento() {
  if (!esperandoAsentamiento) return;
  if (millis() - tAsentamiento < MS_FRENO + 110) return;

  esperandoAsentamiento = false;
  guardarPosicion();

  Serial.print("[ajuste] cierre: "); Serial.print(cierrePendiente);
  Serial.print(" | asentado "); Serial.print(anguloArticulacion, 2);
  Serial.print(" deg | error "); Serial.print(anguloArticulacion - anguloObjetivo, 2);
  Serial.print(" deg | "); Serial.print(duracionPendiente);
  Serial.print(" ms | latencia ");
  if (latenciaTomada) { Serial.print(latenciaArranqueMs); Serial.print(" ms"); }
  else Serial.print("sin arranque");
  Serial.print(" | vel pico "); Serial.print(velPicoCiclo, 1);
  Serial.print(" deg/s | inversiones "); Serial.println(inversionesPendiente);

  publicarCiclo(anguloArticulacion, duracionPendiente, exitoPendiente,
                cierrePendiente, inversionesPendiente);
}

void ejecutarAjuste() {
  if (!ajusteEnCurso) return;

  if (!posicionConfiable) {
    if (motorActivo) frenarMotor("posicion no confiable");
    cerrarAjuste(false, "posicion no confiable", millis() - tInicioAjuste);
    return;
  }

  if (bloqueoDetectado) {
    if (motorActivo) frenarMotor("bloqueo activo");
    cerrarAjuste(false, "bloqueo activo", millis() - tInicioAjuste);
    return;
  }

  float error = anguloObjetivo - anguloArticulacion;
  float absError = fabs(error);
  float zm = zonaMuertaEfectiva();
  unsigned long transcurrido = millis() - tInicioAjuste - msDiferidos;

  if (signoErrorInicial != 0 && error * signoErrorInicial < 0 && absError > sobrepasoCiclo) {
    sobrepasoCiclo = absError;
  }

  if (absError <= zm) {
    if (motorActivo) frenarMotor("objetivo alcanzado");
    ticksEnZona++;
    if (ticksEnZona >= TICKS_ASENTADO) cerrarAjuste(true, "objetivo alcanzado", transcurrido);
    return;
  }
  ticksEnZona = 0;

  if (contactoTalon || contactoPunta) {
    if (tDiferidoDesde == 0) {
      tDiferidoDesde = millis();
      if (motorActivo) frenarMotor("contacto en el pie");
      Serial.println("[ajuste] contacto detectado, movimiento diferido.");
    }
    if (millis() - tInicioAjuste > TIMEOUT_TOTAL_MS) {
      frenarMotor("espera excedida");
      cerrarAjuste(false, "sin ventana de balanceo", millis() - tInicioAjuste);
    }
    return;
  }

  if (tDiferidoDesde != 0) {
    msDiferidos += millis() - tDiferidoDesde;
    tDiferidoDesde = 0;
    arrancado = false;
    pulsosAlArrancar = pulsosEncoder;
    tFaseMov = millis();
    Serial.println("[ajuste] pie libre, se reanuda.");
  }

  if (enFreno()) return;

  // Igual que en el seguimiento: si el motor estuvo parado (freno, diferido
  // por contacto), la referencia de atasco se vuelve a tomar al reengancharse.
  if (!motorActivo) {
    pulsosRef = pulsosEncoder;
    tPulsosRef = millis();
  }

  if (transcurrido > TIMEOUT_AJUSTE_MS) {
    frenarMotor("timeout de carrera");
    activarBloqueo("timeout de carrera");
    cerrarAjuste(false, "timeout", transcurrido);
    return;
  }

  if (motorActivo && millis() - tPulsosRef > STALL_MS) {
    if (labs(pulsosEncoder - pulsosRef) < STALL_PULSOS) {
      frenarMotor("atasco mecanico");
      activarBloqueo("duty aplicado sin pulsos de encoder");
      cerrarAjuste(false, "atasco mecanico", transcurrido);
      return;
    }
    pulsosRef = pulsosEncoder;
    tPulsosRef = millis();
  }

  int sentido = (error > 0) ? 1 : -1;

  if (excedeTopeSalida(pulsosEncoder) && sentidoVaHaciaSalida(sentido)) {
    frenarMotor("tope de salida");
    cerrarAjuste(false, "tope de salida en pulsos", transcurrido);
    Serial.println("[ajuste] detenido en el tope del lado de salida.");
    return;
  }

  if (absError > 30.0) {
    frenarMotor("error anomalo");
    cerrarAjuste(false, "error anomalo", transcurrido);
    return;
  }

  if (sentidoActual != 0 && sentido != sentidoActual) {
    inversiones++;
    frenarMotor("inversion de sentido");
    arrancado = true;

    if (inversiones > MAX_INVERSIONES) {
      cerrarAjuste(false, "oscilacion excesiva", transcurrido);
      Serial.print("[ajuste] abortado tras "); Serial.print(inversiones);
      Serial.println(" inversiones. Bajar kp.");
      return;
    }
    sentidoActual = sentido;
    tFaseMov = millis();
    return;
  }
  sentidoActual = sentido;

  if (!arrancado && labs(pulsosEncoder - pulsosAlArrancar) > MOV_CONFIRMA_PULSOS) {
    arrancado = true;
    if (!latenciaTomada) {
      latenciaArranqueMs = millis() - tInicioAjuste - msDiferidos;
      latenciaTomada = true;
    }
  }

  int duty;
  if (!arrancado && (millis() - tFaseMov) < KICK_MAX_MS) {
    duty = PWM_KICK;
  } else {
    duty = (int)(pwmMin + kpControl * absError);
    if (duty > PWM_MAX) duty = PWM_MAX;
    if (duty < pwmMin) duty = pwmMin;
  }
  if (duty > pwmPicoCiclo) pwmPicoCiclo = duty;

  static unsigned long tPrint = 0;
  if (millis() - tPrint > 180) {
    tPrint = millis();
    Serial.print("  actual "); Serial.print(anguloArticulacion, 2);
    Serial.print(" | obj "); Serial.print(anguloObjetivo, 1);
    Serial.print(" | err "); Serial.print(error, 2);
    Serial.print(" | vel "); Serial.print(velAngular, 1);
    Serial.print(" | pul "); Serial.print(pulsosEncoder);
    Serial.print(" | pwm "); Serial.print(duty);
    Serial.println(arrancado ? " | proporcional" : " | arranque");
  }

  moverMotor(sentido, duty);
}

// --- Paso simulado ---

void iniciarPaso(float terreno) {
  if (pasoActivo) {
    Serial.println("Ya hay un paso en curso. Esperar o enviar 'paso:parar'.");
    return;
  }
  if (!puedeMover(true)) return;

  // Este modo sigue la trayectoria de apoyo y no difiere por contacto, asi
  // que no hay proteccion contra mover el sinfin bajo carga. Se exige que
  // el pie este libre al arrancar. Para dejarlo sin guarda, borrar el bloque.
  if (contactoTalon || contactoPunta) {
    Serial.println("Rechazado: hay contacto en el pie.");
    Serial.println("  El paso simulado mueve el tobillo durante el apoyo y no");
    Serial.println("  difiere por carga. El pie tiene que estar libre.");
    return;
  }

  if (fabs(terreno) > LIMITE_TERRENO_SIM) {
    terreno = (terreno > 0) ? LIMITE_TERRENO_SIM : -LIMITE_TERRENO_SIM;
    Serial.print("Terreno saturado a "); Serial.print(LIMITE_TERRENO_SIM, 0);
    Serial.println(" deg.");
  }

  if (ajusteEnCurso) { frenarMotor("inicio de paso simulado"); }
  ajusteEnCurso = false;
  ajustePendiente = false;
  esperandoAsentamiento = false;
  modoManual = false;
  serieActiva = false;

  terrenoPaso = terreno;
  terrenoEstimado = terreno;
  terrenoValido = true;
  origenSimulado = true;
  offsetAplicado = false;
  offsetInicialPaso = 0.0;
  offsetPrevio = 0.0;
  offsetNuevo = 0.0;
  offsetTerreno = 0.0;

  muestrasPaso = 0;
  acumErr2Paso = 0.0;
  errMaxPaso = 0.0;
  pctErrMaxPaso = 0.0;
  velPicoPaso = 0.0;
  pwmPicoPaso = 0;

  pulsosRef = pulsosEncoder;
  tPulsosRef = millis();
  tInicioPaso = millis();
  pctPaso = 0.0;
  pasoActivo = true;
  numeroPaso++;

  marcarMovimiento();

  Serial.println();
  Serial.print("[paso "); Serial.print(numeroPaso);
  Serial.print("] terreno "); Serial.print(terrenoPaso, 1);
  Serial.print(" deg | offset heredado "); Serial.print(offsetInicialPaso, 1);
  Serial.print(" deg | ciclo "); Serial.print(msCicloPaso);
  Serial.print(" ms | amplitud "); Serial.print(amplitudPaso, 2);
  Serial.print(" | excursion "); Serial.print(excursionTrayectoria(), 1);
  Serial.print(" deg | "); Serial.println(metodoActivo);

  if (excursionTrayectoria() > 2.0 * LIMITE_ANGULO) {
    Serial.println("  Aviso: la excursion excede el recorrido calibrado, habra recorte.");
  }
}

void terminarPaso(const char* cierre) {
  if (!pasoActivo) return;

  unsigned long duracion = millis() - tInicioPaso;
  pasoActivo = false;
  pctPaso = 0.0;
  subfasePaso = "inactivo";
  offsetTerreno = 0.0;
  offsetPrevio = 0.0;
  offsetNuevo = 0.0;
  offsetInicialPaso = 0.0;

  float rms = (muestrasPaso > 0) ? sqrt(acumErr2Paso / (float)muestrasPaso) : -1.0;

  if (strcmp(cierre, "completado") == 0) registrarExito();
  else if (caminataActiva) caminataFallos++;

  guardarPosicion();

  Serial.print("[paso "); Serial.print(numeroPaso);
  Serial.print("] cierre: "); Serial.print(cierre);
  Serial.print(" | "); Serial.print(duracion); Serial.print(" ms");
  Serial.print(" | err rms "); Serial.print(rms, 2);
  Serial.print(" deg | err max "); Serial.print(errMaxPaso, 2);
  Serial.print(" deg al "); Serial.print(pctErrMaxPaso * 100.0, 0);
  Serial.print(" % | vel pico "); Serial.print(velPicoPaso, 1);
  Serial.print(" deg/s | offset "); Serial.print(offsetTerreno, 1);
  Serial.println(" deg");

  publicarPaso(cierre, duracion);

  // Retorno automatico a neutro (0 deg) al concluir el paso simulado
  if (fabs(anguloArticulacion) > zonaMuertaEfectiva()) {
    // Si hubo un bloqueo transitorio por inversion, se limpia para permitir recuperar neutro
    bloqueoDetectado = false;
    iniciarAjuste(0.0);
  }
}

// Seguimiento continuo de la trayectoria. A diferencia del lazo punto a
// punto, aca el objetivo se mueve en cada tick: no hay cierre por zona
// muerta, ni conteo de inversiones, ni timeout de carrera. El paso termina
// cuando se cumple el ciclo.
void ejecutarTrayectoria() {
  if (!pasoActivo) return;

  if (!posicionConfiable) {
    frenarMotor("posicion no confiable");
    terminarPaso("posicion no confiable");
    return;
  }
  if (bloqueoDetectado) {
    frenarMotor("bloqueo activo");
    terminarPaso("bloqueo activo");
    return;
  }

  unsigned long t = millis() - tInicioPaso;
  pctPaso = (float)t / (float)msCicloPaso;

  if (pctPaso >= 1.0) {
    frenarMotor("fin de ciclo");
    terminarPaso("completado");
    return;
  }

  subfasePaso = subfaseDePct(pctPaso);

  // La adaptacion se reparte a lo largo del balanceo, desde el despegue
  // hasta el medio balanceo, para que quede puesta antes del contacto
  // siguiente. Aplicarla de golpe metia un escalon en el setpoint del
  // tamano del cambio de offset, con saturacion de duty detras.
  if (!offsetAplicado && pctPaso >= PCT_DESPEGUE) {
    offsetAplicado = true;
    offsetPrevio = offsetTerreno;
    offsetNuevo = calcularObjetivo(terrenoPaso);
    Serial.print("  despegue al "); Serial.print(PCT_DESPEGUE * 100.0, 0);
    Serial.print(" % | clase "); Serial.print(nombreClase[claseEmitida]);
    Serial.print(" | offset "); Serial.print(offsetPrevio, 1);
    Serial.print(" -> "); Serial.print(offsetNuevo, 1);
    Serial.print(" deg en rampa hasta el "); Serial.print(PCT_MEDIO_BAL * 100.0, 0);
    Serial.println(" %");
  }

  if (offsetAplicado) {
    if (pctPaso <= PCT_MEDIO_BAL) {
      float u = (pctPaso - PCT_DESPEGUE) / (PCT_MEDIO_BAL - PCT_DESPEGUE);
      if (u < 0.0) u = 0.0;
      if (u > 1.0) u = 1.0;
      offsetTerreno = u * offsetNuevo;
    } else {
      // Retorno suave hacia 0 deg antes de finalizar el ciclo
      float u = (pctPaso - PCT_MEDIO_BAL) / (1.0 - PCT_MEDIO_BAL);
      if (u < 0.0) u = 0.0;
      if (u > 1.0) u = 1.0;
      offsetTerreno = offsetNuevo * (1.0 - u);
    }
  }

  float sp = anguloTrayectoria(pctPaso) * amplitudPaso + offsetTerreno;
  if (sp >  LIMITE_ANGULO) sp =  LIMITE_ANGULO;
  if (sp < -LIMITE_ANGULO) sp = -LIMITE_ANGULO;
  setpointPaso = sp;

  float error = sp - anguloArticulacion;
  float absError = fabs(error);

  muestrasPaso++;
  acumErr2Paso += error * error;
  if (absError > errMaxPaso) { errMaxPaso = absError; pctErrMaxPaso = pctPaso; }
  if (fabs(velAngular) > fabs(velPicoPaso)) velPicoPaso = velAngular;

  if (enFreno()) return;

  // La ventana de atasco solo mide tramos de actuacion continua. En
  // seguimiento el motor entra en rueda libre cada vez que el error cae
  // dentro de la banda, y si la referencia no se refresca ahi, al volver a
  // engancharse el chequeo dispara contra una referencia vieja.
  int sentido = (error > 0) ? 1 : -1;
  float zm = (pctPaso >= PCT_MEDIO_BAL) ? zonaMuertaEfectiva() : ZONA_SEGUIMIENTO;
  bool dentroBanda = (absError <= zm);

  // Al arrancar o invertir sentido de giro se refresca la referencia de atasco para
  // permitir que el mecanismo frene la inercia e invierta sin disparar falsos bloqueos.
  if (!motorActivo || (sentidoActual != 0 && sentido != sentidoActual)) {
    pulsosRef = pulsosEncoder;
    tPulsosRef = millis();
    sentidoActual = sentido;
  }

  if (motorActivo && millis() - tPulsosRef > STALL_MS) {
    if (labs(pulsosEncoder - pulsosRef) < STALL_PULSOS) {
      frenarMotor("atasco mecanico");
      activarBloqueo("duty aplicado sin pulsos durante el paso");
      terminarPaso("atasco mecanico");
      return;
    }
    pulsosRef = pulsosEncoder;
    tPulsosRef = millis();
  }

  int duty = 0;
  if (!dentroBanda) {
    float boost = 0.0f;
    if (absError > 2.0f) {
      boost = (absError - 2.0f) * 14.0f;
    }
    duty = (int)(pwmMin + kpControl * absError + boost);
    if (duty > PWM_MAX) duty = PWM_MAX;
    if (duty < pwmMin) duty = pwmMin;
    if (duty > pwmPicoPaso) pwmPicoPaso = duty;
  }

  // El print va antes de la banda para que el muestreo sea uniforme. Si
  // quedaba despues, los tramos en rueda libre no aparecian en el log.
  static unsigned long tPrintPaso = 0;
  if (millis() - tPrintPaso > PASO_RESUMEN_MS) {
    tPrintPaso = millis();
    Serial.print("  "); Serial.print(pctPaso * 100.0, 0);
    Serial.print(" % "); Serial.print(subfasePaso);
    Serial.print(" | sp "); Serial.print(sp, 2);
    Serial.print(" | act "); Serial.print(anguloArticulacion, 2);
    Serial.print(" | err "); Serial.print(error, 2);
    Serial.print(" | off "); Serial.print(offsetTerreno, 1);
    Serial.print(" | vel "); Serial.print(velAngular, 1);
    Serial.print(" | pwm "); Serial.println(duty);
  }

  // Dentro de la banda se corta el duty sin frenar. Frenar en cada tick
  // arruinaria el seguimiento y calentaria el puente sin necesidad.
  if (dentroBanda) {
    if (motorActivo) {
      ledcWrite(PIN_PWM, 0);
      digitalWrite(PIN_STBY, LOW);
      pwmAplicado = 0;
      sentidoAplicado = 0;
      sentidoActual = 0;
      motorActivo = false;
      razonCorte = "dentro de la banda de seguimiento";
    }
    return;
  }

  if (excedeTopeSalida(pulsosEncoder) && sentidoVaHaciaSalida(sentido)) {
    frenarMotor("tope de salida");
    terminarPaso("tope de salida en pulsos");
    return;
  }

  moverMotor(sentido, duty);
}

void atenderCaminata() {
  if (!caminataActiva) return;
  if (pasoActivo) return;
  if (ajusteEnCurso || esperandoAsentamiento) return;
  if (millis() < caminataEsperaHasta) return;

  if (caminataIdx >= caminataN) {
    caminataActiva = false;
    Serial.println();
    Serial.print("[caminata] terminada: "); Serial.print(caminataN);
    Serial.print(" pasos | "); Serial.print(caminataFallos);
    Serial.println(" sin completar");
    Serial.println();
    return;
  }

  // Un bloqueo transitorio no aborta la caminata: se espera a que la
  // autoliberacion actue y se reintenta el mismo punto.
  if (!puedeMover(false)) {
    caminataReintentos++;
    if (caminataReintentos > MAX_REINTENTOS_CAMINATA) {
      caminataActiva = false;
      Serial.println("[caminata] abortada: el sistema no se libero.");
      puedeMover(true);
      return;
    }
    caminataEsperaHasta = millis() + REINTENTO_CAMINATA_MS;
    return;
  }

  if (contactoTalon || contactoPunta) {
    caminataReintentos++;
    if (caminataReintentos > MAX_REINTENTOS_CAMINATA) {
      caminataActiva = false;
      Serial.println("[caminata] abortada: sigue habiendo contacto en el pie.");
      return;
    }
    caminataEsperaHasta = millis() + REINTENTO_CAMINATA_MS;
    return;
  }

  float t = caminataValores[caminataIdx];
  iniciarPaso(t);

  if (!pasoActivo) {
    caminataActiva = false;
    Serial.println("[caminata] abortada: el paso no arranco.");
    return;
  }

  caminataIdx++;
  caminataReintentos = 0;
  caminataEsperaHasta = millis() + PAUSA_PASOS_MS;

  Serial.print("[caminata] paso "); Serial.print(caminataIdx);
  Serial.print(" de "); Serial.println(caminataN);
}

void arrancarCaminata(String lista) {
  caminataN = 0;
  caminataIdx = 0;
  caminataReintentos = 0;
  caminataFallos = 0;

  int desde = 0;
  while (desde < (int)lista.length() && caminataN < SERIE_MAX) {
    int coma = lista.indexOf(',', desde);
    String tok = (coma < 0) ? lista.substring(desde) : lista.substring(desde, coma);
    tok.trim();
    if (tok.length() > 0) caminataValores[caminataN++] = tok.toFloat();
    if (coma < 0) break;
    desde = coma + 1;
  }

  if (caminataN == 0) {
    Serial.println("Usar caminar:-10,-5,0,5,10");
    return;
  }

  caminataActiva = true;
  caminataEsperaHasta = 0;

  Serial.print("[caminata] "); Serial.print(caminataN);
  Serial.print(" pasos de "); Serial.print(msCicloPaso);
  Serial.print(" ms con "); Serial.print(metodoActivo);
  Serial.print(", pausa "); Serial.print(PAUSA_PASOS_MS);
  Serial.println(" ms entre pasos.");
}

// --- Fases y disparo automatico ---

void actualizarFase() {
  // Durante un paso simulado los microswitches reales quedan fuera: la
  // secuencia de fases la gobierna el secuenciador.
  if (pasoActivo) return;

  int detectada = faseDesdeContactos();
  if (detectada == fase) return;

  fasePrevia = fase;
  fase = detectada;
  duracionFasePrevia = millis() - tEntradaFase;
  tEntradaFase = millis();

  if (fase == APOYO_PLANO) {
    acumTerreno = 0.0;
    muestrasPlano = 0;
    muestrasRechazadas = 0;
  }

  if (modoManual) return;

  // Con terreno simulado el valor no se deduce de los sensores, pero las
  // fases siguen gobernando cuando es seguro mover.
  if (fasePrevia == APOYO_PLANO && !simTerreno) {
    if (muestrasPlano >= MIN_MUESTRAS_PLANO) {
      terrenoEstimado = signoTerreno * (acumTerreno / muestrasPlano);
      terrenoValido = true;
      origenSimulado = false;
      numeroCiclo++;
      objetivoPendiente = calcularObjetivo(terrenoEstimado);
      ajustePendiente = true;

      Serial.println();
      Serial.print("[ciclo "); Serial.print(numeroCiclo);
      Serial.print("] terreno "); Serial.print(terrenoEstimado, 1);
      Serial.print(" deg sobre "); Serial.print(muestrasPlano);
      Serial.print(" muestras | "); Serial.print(metodoActivo);
      Serial.print(" | clase "); Serial.print(nombreClase[claseEmitida]);
      Serial.print(" | objetivo "); Serial.print(objetivoPendiente, 1);
      Serial.println(" deg");
      if (muestrasRechazadas > 0) {
        Serial.print("  "); Serial.print(muestrasRechazadas);
        Serial.println(" muestras descartadas por aceleracion de manipulacion.");
      }
    } else {
      ciclosDescartados++;
      Serial.print("[descartado] apoyo plano corto: ");
      Serial.print(muestrasPlano); Serial.print(" de ");
      Serial.print(MIN_MUESTRAS_PLANO);
      Serial.print(" | total "); Serial.println(ciclosDescartados);
    }
  }

  if (fase == BALANCEO && ajustePendiente && !ajusteEnCurso && puedeMover(false)) {
    iniciarAjuste(objetivoPendiente);
  }
}

// --- Terreno simulado ---

void inyectarTerreno(float t) {
  if (!simTerreno) {
    Serial.println("Simulacion apagada. Enviar 'sim:on' primero.");
    return;
  }
  if (fabs(t) > LIMITE_TERRENO_SIM) {
    Serial.print("Terreno saturado a "); Serial.print(LIMITE_TERRENO_SIM, 0);
    Serial.println(" deg.");
    t = (t > 0) ? LIMITE_TERRENO_SIM : -LIMITE_TERRENO_SIM;
  }

  terrenoInyectado = t;
  terrenoEstimado = t;
  terrenoValido = true;
  origenSimulado = true;
  modoManual = false;
  numeroCiclo++;
  objetivoPendiente = calcularObjetivo(terrenoEstimado);
  ajustePendiente = true;

  Serial.println();
  Serial.print("[sim ciclo "); Serial.print(numeroCiclo);
  Serial.print("] terreno "); Serial.print(terrenoEstimado, 1);
  Serial.print(" deg | "); Serial.print(metodoActivo);
  Serial.print(" | clase "); Serial.print(nombreClase[claseEmitida]);
  Serial.print(" | objetivo "); Serial.print(objetivoPendiente, 1);
  Serial.println(" deg");

  if (ajusteEnCurso || esperandoAsentamiento) {
    Serial.println("  Ajuste anterior en curso, este queda encolado.");
    return;
  }
  if (!puedeMover(true)) return;
  iniciarAjuste(objetivoPendiente);
}

// Arranca un objetivo simulado que quedo encolado.
void atenderPendienteSimulado() {
  if (!simTerreno) return;
  if (!ajustePendiente || ajusteEnCurso || esperandoAsentamiento) return;
  if (!puedeMover(false)) return;
  iniciarAjuste(objetivoPendiente);
}

void atenderSerie() {
  if (!serieActiva) return;
  if (ajusteEnCurso || esperandoAsentamiento || ajustePendiente) return;
  if (millis() < serieEsperaHasta) return;

  if (serieIdx >= serieN) {
    serieActiva = false;
    Serial.println();
    Serial.print("[serie] terminada: "); Serial.print(serieN);
    Serial.print(" puntos | "); Serial.print(serieFallos);
    Serial.println(" cierres sin exito");
    Serial.println();
    return;
  }

  float t = serieValores[serieIdx];
  serieIdx++;
  serieEsperaHasta = millis() + SERIE_PAUSA_MS;

  Serial.print("[serie] punto "); Serial.print(serieIdx);
  Serial.print(" de "); Serial.println(serieN);
  inyectarTerreno(t);
}

void arrancarSerie(String lista) {
  serieN = 0;
  serieIdx = 0;
  serieFallos = 0;

  int desde = 0;
  while (desde < (int)lista.length() && serieN < SERIE_MAX) {
    int coma = lista.indexOf(',', desde);
    String tok = (coma < 0) ? lista.substring(desde) : lista.substring(desde, coma);
    tok.trim();
    if (tok.length() > 0) serieValores[serieN++] = tok.toFloat();
    if (coma < 0) break;
    desde = coma + 1;
  }

  if (serieN == 0) {
    Serial.println("Usar serie:-10,-5,0,5,10");
    return;
  }

  if (!simTerreno) {
    simTerreno = true;
    Serial.println("Simulacion activada para la serie.");
  }

  serieActiva = true;
  serieEsperaHasta = 0;

  Serial.print("[serie] "); Serial.print(serieN);
  Serial.print(" puntos con "); Serial.print(metodoActivo);
  Serial.print(", pausa "); Serial.print(SERIE_PAUSA_MS);
  Serial.println(" ms entre puntos.");
}

// --- Movimiento manual ---

// Devuelve los pulsos recorridos. motivo describe por que se detuvo.
long moverPorPulsos(int sentido, long incremento, int duty,
                    unsigned long msMax, const char* &motivo) {
  bool haciaSalida = sentidoVaHaciaSalida(sentido);

  if (haciaSalida && tablaValida && excedeTopeSalida(pulsosEncoder)) {
    motivo = "ya estas en el tope del lado de salida";
    return 0;
  }

  marcarMovimiento();

  long pulInicio = pulsosEncoder;
  unsigned long tInicio = millis();
  long pulRef = pulsosEncoder;
  unsigned long tRef = millis();
  bool enKick = true;
  motivo = "incremento cumplido";

  moverMotor(sentido, PWM_KICK);

  while (millis() - tInicio < msMax) {
    unsigned long t = millis() - tInicio;

    if (enKick && t >= KICK_MAX_MS) {
      enKick = false;
      moverMotor(sentido, duty);
      pulRef = pulsosEncoder;
      tRef = millis();
    }

    if (labs(pulsosEncoder - pulInicio) >= incremento) break;

    if (haciaSalida && tablaValida && excedeTopeSalida(pulsosEncoder)) {
      motivo = "tope de salida";
      break;
    }

    if (!enKick && millis() - tRef > STALL_MS) {
      if (labs(pulsosEncoder - pulRef) < STALL_PULSOS) {
        motivo = haciaSalida ? "sin pulsos del lado de salida, revisar acople"
                             : "tope de la tuerca";
        break;
      }
      pulRef = pulsosEncoder;
      tRef = millis();
    }

    // Atiende comandos y red durante el movimiento, para no perder un
    // 'parar' que llegue a mitad del recorrido.
    if (wifiHabilitado && mqttClient.connected()) mqttClient.loop();
    delay(10);
  }

  frenarMotor("fin de movimiento manual");
  delay(MS_FRENO + 120);
  liberarFreno();

  if (tablaValida) anguloArticulacion = anguloDesdePulsos(pulsosEncoder);
  guardarPosicion();

  return pulsosEncoder - pulInicio;
}

float escalaPulsosGrado() {
  return tablaValida ? pulsosPorGradoLocal() : PULSOS_POR_GRADO_NOMINAL;
}

long pulsosDeGrados(float grados) {
  if (grados <= 0) grados = 1.0;
  if (grados > JOG_MAX_GRADOS) grados = JOG_MAX_GRADOS;
  long n = (long)(grados * escalaPulsosGrado());
  return (n < 1) ? 1 : n;
}

void jog(int sentido, long incremento) {
  long maxInc = (long)(JOG_MAX_GRADOS * escalaPulsosGrado());
  if (incremento <= 0 || incremento > maxInc) incremento = maxInc;

  const char* motivo;
  long mov = moverPorPulsos(sentido, incremento, dutyJog, JOG_MS, motivo);

  Serial.print("  jog: "); Serial.print(mov);
  Serial.print(" pulsos | total "); Serial.print(pulsosEncoder);
  if (tablaValida) {
    Serial.print(" | angulo "); Serial.print(anguloArticulacion, 2); Serial.print(" deg");
  }
  Serial.print(" | "); Serial.println(motivo);
}

// Busca el extremo de la tuerca, el unico lado detectable por calado.
void buscarTope() {
  int sentido = -extremoSalida;

  Serial.println();
  Serial.println("Buscando el tope de la tuerca. Mano en el interruptor de 12 V.");

  int calados = 0;
  for (int i = 0; i < TOPE_MAX_INTENTOS; i++) {
    const char* motivo;
    long mov = moverPorPulsos(sentido, pulsosDeGrados(2.0), dutyJog, JOG_MS, motivo);

    Serial.print("  "); Serial.print(i + 1);
    Serial.print(": "); Serial.print(mov);
    Serial.print(" pulsos | total "); Serial.print(pulsosEncoder);
    Serial.print(" | "); Serial.println(motivo);

    if (labs(mov) < STALL_PULSOS * 2) {
      calados++;
      if (calados >= 2) {
        Serial.println();
        Serial.println("Tope alcanzado.");
        Serial.print("  Pulsos en este punto: "); Serial.println(pulsosEncoder);
        Serial.println("  Medir el angulo real con el inclinometro y enviar");
        Serial.println("  cal:-30 o cal:30 segun el signo. Despues el punto de");
        Serial.println("  15 deg de ese mismo lado, antes de pasar al otro extremo.");
        Serial.println();
        return;
      }
    } else {
      calados = 0;
    }
  }

  Serial.println();
  Serial.println("  No se detecto el tope. Revisar 'salida' y que el encoder cuente.");
  Serial.println();
}

// --- Calibracion ---

void mostrarTabla() {
  Serial.println();
  Serial.println("Tabla de posicion");
  for (int i = 0; i < N_REF; i++) {
    Serial.print("  ");
    if (refAngulos[i] >= 0) Serial.print(" ");
    Serial.print(refAngulos[i], 0);
    Serial.print(" deg  ->  ");
    if (i != IDX_CERO && refPulsos[i] == 0) Serial.println("sin calibrar");
    else { Serial.print(refPulsos[i]); Serial.println(" pulsos"); }
  }

  Serial.print("  Valida: ");
  if (tablaValida) Serial.println("si");
  else if (!tablaCompleta()) Serial.println("NO, falta calibrar algun extremo");
  else Serial.println("NO, la tabla no es monotona, algun punto quedo mal");

  if (tablaValida) {
    Serial.print("  Pulsos por grado: "); Serial.println(pulsosPorGradoLocal(), 2);
    Serial.print("  Zona muerta:      "); Serial.print(zonaMuertaEfectiva(), 2);
    Serial.println(" deg");
    Serial.print("  Tope de salida:   "); Serial.print(topePulsosSalida());
    Serial.println(" pulsos");
  }
  Serial.println();
}

void calibrarPunto(float ang) {
  int slot = -1;
  for (int i = 0; i < N_REF; i++) {
    if (fabs(refAngulos[i] - ang) < 3.0) { slot = i; break; }
  }

  if (slot < 0) {
    Serial.println("Puntos validos: -30, -15, 0, 15 y 30 grados.");
    Serial.println("El cero se fija con 'cero'; los demas con cal:<angulo>.");
    return;
  }
  if (slot == IDX_CERO) {
    Serial.println("El cero se fija con 'cero', no con 'cal'.");
    return;
  }

  refPulsos[slot] = pulsosEncoder;
  guardarTabla();

  Serial.print("Registrado: "); Serial.print(refAngulos[slot], 0);
  Serial.print(" deg -> "); Serial.print(refPulsos[slot]); Serial.println(" pulsos");

  if (tablaValida) {
    Serial.println("Tabla completa y valida. Control habilitado.");
    Serial.print("  Pulsos por grado: "); Serial.println(pulsosPorGradoLocal(), 2);
    Serial.print("  Zona muerta:      "); Serial.print(zonaMuertaEfectiva(), 2);
    Serial.println(" deg");
  } else if (tablaCompleta()) {
    Serial.println("Aviso: la tabla no es monotona. Algun punto quedo mal registrado.");
  } else {
    Serial.print("Faltan "); Serial.print(puntosFaltantes());
    Serial.println(" punto(s) por calibrar. Ver 'tabla'.");
  }
}

void resetContador() {
  if (motorActivo) frenarMotor("reset de contador");
  ajusteEnCurso = false;
  ajustePendiente = false;
  serieActiva = false;

  long antes = pulsosEncoder;
  pulsosEncoder = 0;
  anguloArticulacion = anguloDesdePulsos(0);

  Serial.print("Contador en cero (venia de "); Serial.print(antes);
  Serial.println(" pulsos). La tabla no se toco.");

  if (tablaValida) {
    posicionConfiable = false;
    Serial.println("  La referencia de la tabla se perdio.");
    Serial.println("  Alinear en neutro y enviar 'cero' antes de mover en lazo cerrado.");
  }
}

// Fija el cero en la posicion actual. Si la tabla ya estaba calibrada, los
// otros cuatro puntos quedan referidos a un origen distinto, asi que se
// invalidan en vez de dejar una tabla internamente incoherente.
void ponerCero() {
  bool habiaTabla = tablaValida;
  bool desplazado = (pulsosEncoder != 0);

  pulsosEncoder = 0;
  refPulsos[IDX_CERO] = 0;
  anguloArticulacion = 0.0;
  posicionConfiable = true;
  divergenciaDetectada = false;

  if (habiaTabla && desplazado) {
    for (int i = 0; i < N_REF; i++) {
      if (i != IDX_CERO) refPulsos[i] = 0;
    }
    Serial.println("Cero fijado. El contador venia desplazado, asi que los");
    Serial.println("puntos de -30, -15, 15 y 30 quedaron invalidados.");
    Serial.println("Recalibrar con 'tope' y 'cal:<angulo>' antes de mover en lazo cerrado.");
  } else {
    Serial.println("Cero fijado en la posicion actual y guardado.");
  }

  guardarTabla();
  guardarPosicion();

  if (!tablaValida) {
    Serial.print("Faltan "); Serial.print(puntosFaltantes());
    Serial.println(" punto(s) por calibrar.");
  }
}

// --- Diagnosticos ---

void ensayoRuido() {
  Serial.println();
  Serial.println("Ruido en reposo, 20 s. Motor apagado, no tocar el mecanismo.");

  long pulMin = pulsosEncoder, pulMax = pulsosEncoder;
  unsigned long t0 = millis();

  while (millis() - t0 < 20000) {
    muestrearMagnitud();
    if (pulsosEncoder < pulMin) pulMin = pulsosEncoder;
    if (pulsosEncoder > pulMax) pulMax = pulsosEncoder;
    verificarPosicion();
    if (wifiHabilitado && mqttClient.connected()) mqttClient.loop();
    delay(CONTROL_MS);
  }

  Serial.print("  Deriva de pulsos: "); Serial.print(pulMax - pulMin);
  Serial.println(" pulsos");
  if (pulMax - pulMin == 0) {
    Serial.println("  Cero deriva, es lo esperado.");
    if (tablaValida) {
      Serial.print("  La zona muerta la fija la resolucion: ");
      Serial.print(zonaMuertaEfectiva(), 2); Serial.println(" deg");
    }
  } else {
    Serial.println("  Hay conteo con el motor detenido: ruido en las lineas del encoder.");
    Serial.println("  Revisar pull-ups, apantallado y separacion del cableado de potencia.");
  }
  Serial.println();
}

void diagnosticoSensor() {
  int raw = leerRegistro16(REG_RAW_ANGLE_H);
  int mag = leerRegistro16(REG_MAGNITUDE_H);
  int agc = leerAgc();

  Serial.println();
  Serial.println("AS5600, verificacion cruzada");
  if (raw < 0) {
    Serial.println("  Sin respuesta I2C. El control sigue con el encoder.");
    Serial.println();
    return;
  }
  Serial.print("  Magnitud: "); Serial.println(mag);
  Serial.print("  AGC:      "); Serial.print(agc);
  Serial.print(" (referencia "); Serial.print(agcReferencia); Serial.println(")");
  Serial.print("  Encoder:  "); Serial.print(anguloArticulacion, 2); Serial.println(" deg");
  if (verificacionValida) {
    Serial.print("  Verificacion: "); Serial.print(anguloVerificacion, 2); Serial.println(" deg");
    Serial.print("  Desacuerdo:   "); Serial.print(discrepanciaActual, 2);
    Serial.print(" deg, tolerancia "); Serial.print(DISCREPANCIA_MAX, 1);
    Serial.println(" deg");
  } else {
    Serial.println("  Verificacion no disponible.");
  }
  Serial.println();
}

void diagnosticoTerreno() {
  Serial.println();
  Serial.println("Terreno");
  Serial.print("  Fuente: ");
  if (simTerreno) {
    Serial.print("simulada, ultimo valor inyectado ");
    Serial.print(terrenoInyectado, 2); Serial.println(" deg");
  } else {
    Serial.println("sensores");
  }

  Serial.print("  Inclinacion tibia: ");
  if (tibiaValida) { Serial.print(anguloTibia, 2); Serial.println(" deg"); }
  else Serial.println("MPU6050 sin lectura valida");
  Serial.print("  Modulo aceleracion: "); Serial.print(moduloAcc, 3); Serial.println(" g");
  Serial.print("  Articulacion: "); Serial.print(anguloArticulacion, 2); Serial.println(" deg");
  Serial.print("  Velocidad:    "); Serial.print(velAngular, 2); Serial.println(" deg/s");

  float inst = signoTerreno * (anguloTibia - anguloArticulacion);
  Serial.print("  Terreno instantaneo: "); Serial.print(inst, 2); Serial.println(" deg");
  Serial.print("  Ultimo ciclo: ");
  if (terrenoValido) { Serial.print(terrenoEstimado, 2); Serial.println(" deg"); }
  else Serial.println("sin medir");

  int cl;
  float obj = objetivoTentativo(inst, cl);
  Serial.print("  Objetivo que daria: "); Serial.print(obj, 2);
  Serial.print(" deg (clase "); Serial.print(nombreClase[cl]); Serial.println(")");
  Serial.println("  En pendiente conocida el terreno debe leer cerca del valor real");
  Serial.println("  y con su mismo signo. Si sale invertido: signoterreno:-");
  Serial.println();
}

void mostrarConfiguracion() {
  Serial.println();
  Serial.println("Configuracion guardada");
  Serial.print("  pwmmin:        "); Serial.println(pwmMin);
  Serial.print("  kp:            "); Serial.println(kpControl, 2);
  Serial.print("  ganancia:      "); Serial.print(gananciaObjetivo, 2);
  Serial.println(gananciaObjetivo == 1.0 ? " (clasificador sin escalar)"
                                         : " (objetivo escalado, no comparable con ganancia 1)");
  Serial.print("  salida:        "); Serial.print(extremoSalida > 0 ? "+" : "-");
  Serial.println(extremoSalida > 0 ? " (sale hacia angulos positivos)"
                                   : " (sale hacia angulos negativos)");
  Serial.print("  signoterreno:  "); Serial.println(signoTerreno > 0 ? "+" : "-");
  Serial.print("  duty de jog:   "); Serial.print(dutyJog);
  Serial.println(" (no se guarda, vuelve a pwmmin+30 al reiniciar)");
  Serial.println();
}

void estadoGeneral() {
  Serial.println();
  Serial.println("Estado");
  Serial.print("  Angulo:      "); Serial.print(anguloArticulacion, 2);
  Serial.print(" deg desde "); Serial.print(pulsosEncoder); Serial.println(" pulsos");
  Serial.print("  Velocidad:   "); Serial.print(velAngular, 2); Serial.println(" deg/s");
  Serial.print("  Posicion:    ");
  Serial.println(posicionConfiable ? "confiable" : "NO confiable, enviar 'cero' en neutro");
  if (fueraDeTabla) Serial.println("  Aviso:       fuera del rango calibrado");
  Serial.print("  Tabla:       "); Serial.println(tablaValida ? "valida" : "INVALIDA, ver 'tabla'");
  Serial.print("  Metodo:      "); Serial.println(metodoActivo);
  Serial.print("  Modo:        "); Serial.println(modoManual ? "manual" : "marcha automatica");
  Serial.print("  Terreno:     ");
  if (simTerreno) {
    Serial.print("SIMULADO | ultimo "); Serial.print(terrenoInyectado, 1); Serial.println(" deg");
  } else {
    Serial.println("desde sensores");
  }
  if (serieActiva) {
    Serial.print("  Serie:       punto "); Serial.print(serieIdx);
    Serial.print(" de "); Serial.println(serieN);
  }
  if (pasoActivo) {
    Serial.print("  Paso:        "); Serial.print(numeroPaso);
    Serial.print(" | "); Serial.print(pctPaso * 100.0, 0);
    Serial.print(" % "); Serial.print(subfasePaso);
    Serial.print(" | setpoint "); Serial.print(setpointPaso, 2); Serial.println(" deg");
  } else {
    Serial.print("  Paso:        inactivo | ciclo "); Serial.print(msCicloPaso);
    Serial.print(" ms | amplitud "); Serial.println(amplitudPaso, 2);
  }
  if (caminataActiva) {
    Serial.print("  Caminata:    paso "); Serial.print(caminataIdx);
    Serial.print(" de "); Serial.println(caminataN);
  }
  Serial.print("  Objetivo:    "); Serial.print(anguloObjetivo, 2); Serial.println(" deg");
  Serial.print("  Zona muerta: "); Serial.print(zonaMuertaEfectiva(), 2); Serial.println(" deg");
  Serial.print("  Fase:        "); Serial.print(nombreFase(fase));
  Serial.print(" (previa "); Serial.print(nombreFase(fasePrevia));
  Serial.print(", "); Serial.print(duracionFasePrevia); Serial.println(" ms)");
  Serial.print("  Contactos:   talon "); Serial.print(contactoTalon ? "si" : "no");
  Serial.print(" | punta "); Serial.println(contactoPunta ? "si" : "no");
  Serial.print("  Motor:       "); Serial.print(motorActivo ? "activo" : "detenido");
  Serial.print(" | pwm "); Serial.print(pwmAplicado);
  Serial.print(" | corte por "); Serial.println(razonCorte);
  Serial.print("  Bloqueo:     "); Serial.print(bloqueoDetectado ? "si" : "no");
  Serial.print(" | consecutivos "); Serial.println(bloqueosConsecutivos);
  Serial.print("  Autoliberar: ");
  if (latchPermanente) {
    Serial.println("TRABADO tras varias autoliberaciones, enviar 'liberar'");
  } else if (autoLiberar) {
    Serial.print("si | usadas "); Serial.print(autoLiberaciones);
    Serial.print(" de "); Serial.println(MAX_AUTOLIBERAR);
  } else {
    Serial.println("no, liberacion manual");
  }
  Serial.print("  Ciclos:      "); Serial.print(numeroCiclo);
  Serial.print(" validos | "); Serial.print(ciclosDescartados); Serial.println(" descartados");
  Serial.print("  Red:         ");
  if (!wifiHabilitado) Serial.println("sin WiFi, operacion local");
  else Serial.println(mqttClient.connected() ? "MQTT conectado" : "MQTT reconectando");
  Serial.print("  Telemetria:  ");
  Serial.println(telemetriaContinua ? "continua a 100 ms" : "solo por ciclo");
  Serial.println();
}

void ayuda() {
  Serial.println();
  Serial.println("Comandos (terminar con Enter)");
  Serial.println("  Ver");
  Serial.println("    estado          angulo, velocidad, fase, motor, bloqueos");
  Serial.println("    tabla           tabla de calibracion y validez");
  Serial.println("    config          parametros guardados");
  Serial.println("  Calibrar");
  Serial.println("    cero            fija 0 deg aqui, invalida el resto si estaba desplazado");
  Serial.println("    tope            va solo al extremo de la tuerca");
  Serial.println("    cal:<ang>       registra la posicion actual (-30 -15 15 30)");
  Serial.println("    reset           pone el contador en cero, no toca la tabla");
  Serial.println("    tabla:borrar    borra la calibracion");
  Serial.println("  Terreno simulado");
  Serial.println("    sim:on          el terreno se inyecta, no se lee de sensores");
  Serial.println("    sim:off         vuelve a leerlo de MPU6050 y microswitches");
  Serial.println("    sim:-10         inyecta -10 deg y dispara el ciclo completo");
  Serial.println("    serie:-10,-5,0,5,10   recorre la lista con pausa entre puntos");
  Serial.println("    serie:parar     aborta la serie en curso");
  Serial.println("  Paso simulado (banco, sin carga)");
  Serial.println("    paso            un ciclo de marcha en terreno plano");
  Serial.println("    paso:+10        un ciclo completo con terreno +10 deg");
  Serial.println("    caminar:0,5,10  un paso por cada terreno de la lista");
  Serial.println("    paso:parar      aborta el paso y la caminata");
  Serial.println("    offset:reset    pone en cero la adaptacion heredada");
  Serial.println("    trayectoria     muestra los puntos y la excursion");
  Serial.println("    ciclo:4000      duracion del ciclo en ms, se guarda");
  Serial.println("    amplitud:1      escala la trayectoria, se guarda");
  Serial.println("  Mover");
  Serial.println("    f | b           un grado adelante o atras");
  Serial.println("    p               pulsos y angulo actuales");
  Serial.println("    jog:+2          avanza 2 grados en positivo (tope 5)");
  Serial.println("    mover:5         objetivo 5 deg en lazo cerrado");
  Serial.println("    parar           frena y bloquea");
  Serial.println("    liberar         desbloquea");
  Serial.println("    autoliberar:on  se libera solo tras el limite de bloqueos");
  Serial.println("    autoliberar:off liberacion solo a mano");
  Serial.println("    auto            vuelve a marcha automatica");
  Serial.println("  Ajustar");
  Serial.println("    duty:150        velocidad de los jogs");
  Serial.println("    pwmmin:110      duty minimo del control, se guarda");
  Serial.println("    kp:6            ganancia del control, se guarda");
  Serial.println("    ganancia:1      escala el objetivo del clasificador, se guarda");
  Serial.println("    salida:+        lado por donde sale de la tuerca, se guarda");
  Serial.println("    signoterreno:-  convencion de compensacion, se guarda");
  Serial.println("  Metodo");
  Serial.println("    histeresis | difuso");
  Serial.println("  Diagnostico");
  Serial.println("    diag | terreno | ruido");
  Serial.println("    telemetria:on | telemetria:off");
  Serial.println();
}

// --- Consola de comandos ---

void ejecutarComando(const char* cmd) {
  String s = String(cmd);
  s.trim();
  s.toLowerCase();
  if (s.length() == 0) return;

  if (s == "?" || s == "ayuda") { ayuda(); return; }
  if (s == "estado") { estadoGeneral(); return; }
  if (s == "tabla") { mostrarTabla(); return; }
  if (s == "config") { mostrarConfiguracion(); return; }
  if (s == "diag") { diagnosticoSensor(); return; }
  if (s == "terreno") { diagnosticoTerreno(); return; }
  if (s == "ruido") { ensayoRuido(); return; }
  if (s == "cero") { ponerCero(); return; }
  if (s == "tope") { buscarTope(); return; }
  if (s == "reset") { resetContador(); return; }

  if (s == "f") { jog(1, pulsosDeGrados(1.0)); return; }
  if (s == "b") { jog(-1, pulsosDeGrados(1.0)); return; }

  if (s == "p") {
    Serial.print("pulsos: "); Serial.print(pulsosEncoder);
    if (tablaValida) {
      Serial.print(" | angulo "); Serial.print(anguloArticulacion, 2);
      Serial.print(" deg | "); Serial.print(pulsosPorGradoLocal(), 2);
      Serial.print(" pulsos/grado");
      if (!posicionConfiable) Serial.print(" | NO CONFIABLE");
    } else {
      Serial.print(" | sin tabla, escala nominal ");
      Serial.print(PULSOS_POR_GRADO_NOMINAL, 1); Serial.print(" pulsos/grado");
    }
    Serial.println();
    return;
  }

  if (s == "tabla:borrar") {
    for (int i = 0; i < N_REF; i++) refPulsos[i] = 0;
    guardarTabla();
    posicionConfiable = false;
    Serial.println("Tabla borrada. Control inhabilitado hasta recalibrar.");
    return;
  }

  if (s.startsWith("cal:")) { calibrarPunto(s.substring(4).toFloat()); return; }

  // Terreno simulado. sim:on / sim:off conmutan la fuente; sim:<numero>
  // inyecta un valor y dispara clasificacion mas ajuste.
  if (s == "sim:on") {
    simTerreno = true;
    modoManual = false;
    Serial.println("Terreno simulado activo. Inyectar con sim:<grados> o por");
    Serial.println("el topico tobillo/terreno. Los sensores dejan de estimarlo.");
    return;
  }

  if (s == "sim:off") {
    simTerreno = false;
    serieActiva = false;
    Serial.println("Terreno desde sensores. La simulacion queda apagada.");
    return;
  }

  if (s.startsWith("sim:")) {
    String arg = s.substring(4);
    arg.trim();
    if (arg.length() == 0) { Serial.println("Usar sim:on, sim:off o sim:<grados>"); return; }
    inyectarTerreno(arg.toFloat());
    return;
  }

  if (s == "serie:parar") {
    serieActiva = false;
    Serial.println("Serie abortada.");
    return;
  }

  if (s.startsWith("serie:")) { arrancarSerie(s.substring(6)); return; }

  // Paso simulado: recorre el ciclo de marcha completo sobre la base de
  // tiempo estirada, con el tobillo siguiendo la trayectoria angular.
  if (s == "paso:parar" || s == "caminar:parar") {
    caminataActiva = false;
    if (pasoActivo) {
      frenarMotor("paso abortado");
      terminarPaso("abortado por comando");
    } else {
      Serial.println("No hay paso en curso.");
    }
    return;
  }

  if (s == "offset:reset") {
    if (pasoActivo) { Serial.println("No durante un paso. Enviar 'paso:parar' primero."); return; }
    offsetTerreno = 0.0;
    offsetPrevio = 0.0;
    offsetNuevo = 0.0;
    Serial.println("Offset de terreno en cero. El proximo paso arranca en plano.");
    return;
  }

  if (s == "paso") { iniciarPaso(0.0); return; }

  if (s.startsWith("paso:")) {
    String arg = s.substring(5);
    arg.trim();
    iniciarPaso(arg.length() ? arg.toFloat() : 0.0);
    return;
  }

  if (s.startsWith("caminar:")) { arrancarCaminata(s.substring(8)); return; }

  if (s == "trayectoria") {
    Serial.println();
    Serial.println("Trayectoria del ciclo de marcha (dorsiflexion positiva)");
    for (int i = 0; i < N_TRAY; i++) {
      Serial.print("  "); Serial.print(trayPct[i] * 100.0, 0);
      Serial.print(" %  "); Serial.print(subfaseDePct(trayPct[i]));
      Serial.print("  ->  "); Serial.print(trayAng[i] * amplitudPaso, 1);
      Serial.println(" deg");
    }
    Serial.print("  Excursion total: "); Serial.print(excursionTrayectoria(), 1);
    Serial.println(" deg");
    Serial.print("  Ciclo: "); Serial.print(msCicloPaso);
    Serial.print(" ms | balanceo desde el "); Serial.print(PCT_DESPEGUE * 100.0, 0);
    Serial.println(" %");
    Serial.println();
    return;
  }

  if (s.startsWith("ciclo:")) {
    long v = s.substring(6).toInt();
    if (v < MS_CICLO_MIN || v > MS_CICLO_MAX) {
      Serial.print("ciclo debe estar entre "); Serial.print(MS_CICLO_MIN);
      Serial.print(" y "); Serial.print(MS_CICLO_MAX); Serial.println(" ms.");
      return;
    }
    msCicloPaso = (unsigned long)v;
    guardarConfiguracion();
    Serial.print("ciclo = "); Serial.print(msCicloPaso);
    Serial.println(" ms guardado.");
    if (msCicloPaso < 2000) {
      Serial.println("  Por debajo de 2 s el mecanismo no alcanza el prebalanceo.");
    }
    return;
  }

  if (s.startsWith("amplitud:")) {
    float v = s.substring(9).toFloat();
    if (v < AMPLITUD_MIN || v > AMPLITUD_MAX) {
      Serial.print("amplitud debe estar entre "); Serial.print(AMPLITUD_MIN, 1);
      Serial.print(" y "); Serial.println(AMPLITUD_MAX, 1);
      return;
    }
    amplitudPaso = v;
    guardarConfiguracion();
    Serial.print("amplitud = "); Serial.print(amplitudPaso, 2);
    Serial.print(" | excursion "); Serial.print(excursionTrayectoria(), 1);
    Serial.println(" deg");
    return;
  }

  if (s == "telemetria:on")  { telemetriaContinua = true;  Serial.println("Telemetria continua activa."); return; }
  if (s == "telemetria:off") { telemetriaContinua = false; Serial.println("Telemetria continua apagada, solo ciclos."); return; }

  // El argumento son grados, no pulsos.
  if (s.startsWith("jog:")) {
    String arg = s.substring(4);
    int sentido = arg.startsWith("-") ? -1 : 1;
    float grados = fabs(arg.toFloat());
    if (grados <= 0) grados = 1.0;
    if (grados > JOG_MAX_GRADOS) {
      Serial.print("jog saturado a "); Serial.print(JOG_MAX_GRADOS, 1);
      Serial.println(" deg.");
      grados = JOG_MAX_GRADOS;
    }
    long inc = pulsosDeGrados(grados);
    Serial.print("  jog de "); Serial.print(grados, 1);
    Serial.print(" deg = "); Serial.print(inc); Serial.println(" pulsos");
    jog(sentido, inc);
    return;
  }

  if (s.startsWith("mover:")) {
    if (!puedeMover(true)) return;
    caminataActiva = false;
    if (pasoActivo) { frenarMotor("mover manual"); terminarPaso("interrumpido por mover"); }
    modoManual = true;
    serieActiva = false;
    bloqueoDetectado = false;
    origenSimulado = false;
    float v = s.substring(6).toFloat();
    if (fabs(v) > LIMITE_ANGULO) {
      Serial.print("Objetivo saturado a "); Serial.print(LIMITE_ANGULO, 1);
      Serial.println(" deg.");
    }
    iniciarAjuste(v);
    return;
  }

  if (s.startsWith("duty:")) {
    int v = s.substring(5).toInt();
    if (v < 60 || v > 255) { Serial.println("duty debe estar entre 60 y 255."); return; }
    dutyJog = v;
    Serial.print("duty de jog = "); Serial.println(dutyJog);
    return;
  }

  if (s.startsWith("pwmmin:")) {
    int v = s.substring(7).toInt();
    if (v < 60 || v > PWM_MAX) {
      Serial.print("pwmmin debe estar entre 60 y "); Serial.println(PWM_MAX);
      return;
    }
    pwmMin = v;
    dutyJog = pwmMin + 30;
    if (dutyJog > 255) dutyJog = 255;
    guardarConfiguracion();
    Serial.print("pwmmin = "); Serial.print(pwmMin);
    Serial.print(" guardado. kp sugerido: ");
    Serial.println((PWM_MAX - pwmMin) / (LIMITE_ANGULO * 0.5), 1);
    return;
  }

  if (s.startsWith("kp:")) {
    float v = s.substring(3).toFloat();
    if (v <= 0 || v > 30) { Serial.println("kp debe estar entre 0 y 30."); return; }
    kpControl = v;
    guardarConfiguracion();
    Serial.print("kp = "); Serial.print(kpControl, 2); Serial.println(" guardado.");
    return;
  }

  if (s.startsWith("ganancia:")) {
    float v = s.substring(9).toFloat();
    if (v < 0.2 || v > 3.0) { Serial.println("ganancia debe estar entre 0.2 y 3.0."); return; }
    gananciaObjetivo = v;
    guardarConfiguracion();
    Serial.print("ganancia = "); Serial.print(gananciaObjetivo, 2);
    Serial.println(" guardada.");
    if (v != 1.0) {
      Serial.println("  Los ciclos con ganancia distinta de 1 no son comparables");
      Serial.println("  con los de ganancia 1. Queda registrada en la telemetria.");
    }
    return;
  }

  if (s.startsWith("salida:")) {
    String a = s.substring(7);
    if (a != "+" && a != "-") { Serial.println("Usar salida:+ o salida:-"); return; }
    extremoSalida = (a == "+") ? 1 : -1;
    guardarConfiguracion();
    Serial.print("salida = "); Serial.print(a);
    Serial.println(" guardado. El tope por pulsos protege ese lado.");
    return;
  }

  if (s.startsWith("signoterreno:")) {
    String a = s.substring(13);
    if (a != "+" && a != "-") { Serial.println("Usar signoterreno:+ o signoterreno:-"); return; }
    signoTerreno = (a == "+") ? 1.0 : -1.0;
    guardarConfiguracion();
    Serial.print("signoterreno = "); Serial.print(a); Serial.println(" guardado.");
    return;
  }

  if (s == "parar") {
    frenarMotor("parada solicitada");
    ajusteEnCurso = false;
    ajustePendiente = false;
    serieActiva = false;
    caminataActiva = false;
    if (pasoActivo) terminarPaso("parada solicitada");
    activarBloqueo("parada solicitada");
    return;
  }

  if (s == "auto") {
    modoManual = false;
    caminataActiva = false;
    if (pasoActivo) { frenarMotor("cambio a automatico"); terminarPaso("cambio a automatico"); }
    if (ajusteEnCurso) frenarMotor("cambio a automatico");
    ajusteEnCurso = false;
    ajustePendiente = false;
    Serial.print("Marcha automatica reanudada, terreno ");
    Serial.println(simTerreno ? "simulado." : "desde sensores.");
    return;
  }

  if (s == "liberar") {
    bloqueoDetectado = false;
    bloqueosConsecutivos = 0;
    autoLiberaciones = 0;
    latchPermanente = false;
    divergenciaDetectada = false;
    imanDesalineado = false;
    Serial.println("Bloqueos y avisos liberados.");
    if (!posicionConfiable) Serial.println("La posicion sigue no confiable: enviar 'cero' en neutro.");
    return;
  }

  if (s == "autoliberar:on") {
    autoLiberar = true;
    latchPermanente = false;
    autoLiberaciones = 0;
    Serial.print("Autoliberacion activa: tras el limite de bloqueos se libera en ");
    Serial.print(ENFRIAMIENTO_LIMITE_MS / 1000); Serial.println(" s.");
    return;
  }

  if (s == "autoliberar:off") {
    autoLiberar = false;
    Serial.println("Autoliberacion apagada. Hay que enviar 'liberar' a mano.");
    return;
  }

  if (s == "histeresis" || s == "difuso") {
    modoActual = (s == "histeresis") ? CLASIF_HISTERESIS : CLASIF_DIFUSO;
    metodoActivo = s;
    modoManual = false;
    claseActual = 2;
    Serial.print("Metodo activo: "); Serial.print(metodoActivo);
    Serial.println(" | estado de histeresis reiniciado en plano");
    return;
  }

  Serial.print("Comando '"); Serial.print(s);
  Serial.println("' no reconocido. Enviar '?' para la lista.");
}

void atenderSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (idxLinea > 0) {
        lineaSerial[idxLinea] = '\0';
        ejecutarComando(lineaSerial);
        idxLinea = 0;
      }
    } else if (idxLinea < (int)sizeof(lineaSerial) - 1) {
      lineaSerial[idxLinea++] = c;
    }
  }
}

// --- MQTT ---

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  String t = String(topic);

  String m;
  for (unsigned int i = 0; i < length; i++) m += (char)payload[i];
  m.trim();
  if (m.length() == 0) return;

  // Canal dedicado para el terreno: acepta un numero suelto, que es lo que
  // un slider o un inject de Node-RED manda sin formatear nada.
  if (t == TOPIC_TERRENO) {
    Serial.print("[mqtt terreno] "); Serial.println(m);
    if (!simTerreno) {
      simTerreno = true;
      modoManual = false;
      Serial.println("  Simulacion activada por el topico de terreno.");
    }
    inyectarTerreno(m.toFloat());
    return;
  }

  if (t != TOPIC_COMANDO) return;

  Serial.print("[mqtt] "); Serial.println(m);
  if (isdigit(m[0]) || m[0] == '-' || m[0] == '+') m = "mover:" + m;
  ejecutarComando(m.c_str());
}

void reconectarMqtt() {
  if (!wifiHabilitado) return;
  if (millis() - ultimoIntentoMqtt < MQTT_RETRY_MS) return;
  ultimoIntentoMqtt = millis();

  if (mqttClient.connect(MQTT_CLIENT_ID, TOPIC_ESTADO, 0, true, "desconectado")) {
    Serial.println("[mqtt] conectado.");
    mqttClient.publish(TOPIC_ESTADO, "conectado", true);
    mqttClient.subscribe(TOPIC_COMANDO);
    mqttClient.subscribe(TOPIC_TERRENO);
  }
}

// --- Arranque de red ---

bool resetSolicitadoPorBoton() {
  if (digitalRead(BOTON_RESET) != LOW) return false;

  Serial.print("Boton presionado, mantener ");
  Serial.print(TIEMPO_RESET_MS / 1000);
  Serial.println(" s para borrar la configuracion WiFi...");

  unsigned long inicio = millis();
  while (digitalRead(BOTON_RESET) == LOW) {
    if (millis() - inicio >= TIEMPO_RESET_MS) {
      Serial.println("Configuracion WiFi borrada.");
      return true;
    }
    delay(20);
  }
  Serial.println("Boton liberado antes del tiempo.");
  return false;
}

void configurarWifi(bool forzar) {
  WiFiManager wm;
  wm.setConfigPortalTimeout(12);
  if (forzar) wm.resetSettings();

  WiFiManagerParameter pb("broker", "IP Broker MQTT", brokerIp, 16);
  wm.addParameter(&pb);

  if (!wm.autoConnect("TobilloConfig", "tobillo123")) {
    Serial.println("Sin WiFi. Operacion local.");
    wifiHabilitado = false;
    return;
  }

  strcpy(brokerIp, pb.getValue());
  guardarBrokerIp(brokerIp);
  Serial.print("WiFi conectado, IP "); Serial.println(WiFi.localIP());
  wifiHabilitado = true;
}

// --- Restauracion de posicion ---

void restaurarPosicion() {
  prefs.begin(PREFS_NAMESPACE, true);
  bool limpio = prefs.getBool(PREFS_KEY_LIMPIO, false);
  float ang = prefs.getFloat(PREFS_KEY_ANGULO, 0.0);
  long pul = prefs.getLong(PREFS_KEY_PULSOS, 0);
  prefs.end();

  if (!tablaValida) {
    Serial.println("  Tabla sin calibrar. Control inhabilitado.");
    posicionConfiable = false;
    return;
  }

  if (!limpio) {
    Serial.println("  La sesion anterior no cerro con el motor detenido.");
    Serial.println("  Posicion NO restaurada. Alinear en neutro y enviar 'cero'.");
    posicionConfiable = false;
    return;
  }

  for (int i = 0; i < N_PROM_MAG; i++) { muestrearMagnitud(); delay(20); }
  int n = magLleno ? N_PROM_MAG : idxMag;
  float mag = 0;
  for (int i = 0; i < n; i++) mag += bufMagnitud[i];
  if (n > 0) mag /= n;

  float absAng;
  int signo = (ang >= 0) ? 1 : -1;
  bool verificable = (n >= 3) && anguloDesdeMagnitud(mag, signo, absAng);

  if (verificable && fabs(ang - signo * absAng) > DISCREPANCIA_MAX) {
    Serial.print("  Guardado "); Serial.print(ang, 2);
    Serial.print(" deg contra verificacion "); Serial.print(signo * absAng, 2);
    Serial.println(" deg.");
    Serial.println("  Desacuerdo fuera de tolerancia. Revisar el acople eje-husillo.");
    Serial.println("  Posicion NO restaurada. Alinear en neutro y enviar 'cero'.");
    posicionConfiable = false;
    return;
  }

  pulsosEncoder = pul;
  anguloArticulacion = anguloDesdePulsos(pul);
  posicionConfiable = true;

  Serial.print("  Posicion restaurada: "); Serial.print(anguloArticulacion, 2);
  Serial.print(" deg desde "); Serial.print(pul); Serial.println(" pulsos");
  if (!verificable) Serial.println("  Sin verificacion del AS5600 en el arranque.");
}

// --- setup ---

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(BOTON_RESET, INPUT_PULLUP);
  pinMode(PIN_TALON, INPUT_PULLUP);
  pinMode(PIN_PUNTA, INPUT_PULLUP);
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);

#if DECODIF_2X
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncoder, CHANGE);
#else
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncoder, RISING);
#endif

  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  ledcAttach(PIN_PWM, PWM_FREQ, PWM_RES);
  liberarFreno();
  razonCorte = "arranque";

  Wire.begin(SDA_PIN, SCL_PIN);
  inicializarMPU6050();

  Serial.println();
  Serial.println("Tobillo protesico activo - v6");

  int agc = leerAgc();
  if (agc >= 0) {
    agcReferencia = (uint8_t)agc;
    agcActual = agc;
    Serial.print("  AS5600 responde, AGC "); Serial.println(agcReferencia);
  } else {
    Serial.println("  AS5600 sin respuesta. El control no lo necesita.");
  }

  float ax, ay, az, gx, gy, gz;
  if (leerMPU6050(ax, ay, az, gx, gy, gz)) {
    Serial.print("  MPU6050 responde, inclinacion ");
    Serial.print(anguloTibiaGrados(ax, ay, az), 2); Serial.println(" deg");
  } else {
    Serial.println("  MPU6050 sin respuesta. Sin estimacion de terreno por sensores.");
  }

  cargarConfiguracion();
  cargarTabla();
  Serial.print("  Tabla: "); Serial.println(tablaValida ? "valida" : "sin calibrar");
  if (gananciaObjetivo != 1.0) {
    Serial.print("  Ganancia de objetivo en "); Serial.println(gananciaObjetivo, 2);
  }
  Serial.print("  Paso simulado: ciclo "); Serial.print(msCicloPaso);
  Serial.print(" ms | amplitud "); Serial.print(amplitudPaso, 2);
  Serial.print(" | excursion "); Serial.print(excursionTrayectoria(), 1);
  Serial.println(" deg");

  restaurarPosicion();

  bool forzar = resetSolicitadoPorBoton();
  cargarBrokerIp();
  configurarWifi(forzar);

  if (wifiHabilitado) {
    mqttClient.setServer(brokerIp, MQTT_PORT);
    mqttClient.setCallback(onMqttMessage);
    mqttClient.setBufferSize(MQTT_BUFFER);
  }

  tEntradaFase = millis();
  tVentanaVel = 0;
  Serial.println("Sistema listo. Enviar '?' para los comandos.");
  Serial.println();
}

// --- loop ---

void loop() {
  actualizarContactos();
  atenderFreno();
  atenderEnfriamiento();

  if (millis() - ultimoControl > CONTROL_MS) {
    ultimoControl = millis();

    if (tablaValida) anguloArticulacion = anguloDesdePulsos(pulsosEncoder);
    actualizarVelocidad();
    muestrearMagnitud();

    float ax, ay, az, gx, gy, gz;
    if (leerMPU6050(ax, ay, az, gx, gy, gz)) {
      moduloAcc = sqrt(ax * ax + ay * ay + az * az);
      anguloTibia = anguloTibiaGrados(ax, ay, az);
      tibiaValida = true;
    } else {
      tibiaValida = false;
    }

    actualizarFase();
    acumularTerreno();

    // El paso simulado y el lazo punto a punto son excluyentes.
    if (pasoActivo) {
      ejecutarTrayectoria();
    } else {
      ejecutarAjuste();
      atenderAsentamiento();
      atenderPendienteSimulado();
      atenderSerie();
    }
    atenderCaminata();
  }

  verificarPosicion();

  if (wifiHabilitado) {
    if (!mqttClient.connected()) {
      reconectarMqtt();
    } else {
      mqttClient.loop();
      atenderLog();
      if (telemetriaContinua && millis() - ultimaTelemetria > TELEMETRIA_MS) {
        ultimaTelemetria = millis();
        publicarTelemetria();
      }
    }
  }

  atenderSerial();
}
