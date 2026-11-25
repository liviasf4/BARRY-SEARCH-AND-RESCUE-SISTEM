// =====================================================================
//  SOCORRISTA - TTGO T-Beam T22 V0.7  (VERSÃO SEM GPS / PONTOS FIXOS)
//
//  Fluxo:
//   1. Vá até o ponto fixo N (coordenada conhecida, tabela PONTOS abaixo).
//   2. Aperte o botão -> o ESP32 coleta CAPTURE_SAMPLES pacotes do beacon
//      e grava a média do RSSI como a "âncora" do ponto N.
//   3. Vá ao ponto seguinte e repita.
//   4. Com MIN_ANCHORS pontos coletados, a multilateração estima o beacon
//      (a cada novo ponto a estimativa é refeita).
//
//  Comandos serial (Enter): status | reset | ponto <n> | n <v> | a <dBm> | estimarA 0/1
//   "ponto 3" -> volta para o ponto 3 (recoletar / pular).
// =====================================================================
#include <Arduino.h>
#include <Preferences.h>
#include <TinyGPS++.h>      // só para distanceBetween / courseTo (funções estáticas)
#include "lora_common.h"
#include "multilateration.h"

// ---- UART para a Pico (matriz de LEDs) ----
#ifndef PICO_TX_PIN
#define PICO_TX_PIN 13
#endif
#ifndef PICO_RX_PIN
#define PICO_RX_PIN 2
#endif
#define PICO_BAUD 115200

// ---- botão ----
// T-Beam V0.7: botão do usuário em GPIO39
#ifndef BUTTON_PIN
#define BUTTON_PIN 39
#endif

#ifndef MODEL_A_DBM
#define MODEL_A_DBM -45.0
#endif
#ifndef MODEL_N
#define MODEL_N 2.5
#endif

// ---- PONTOS FIXOS CONHECIDOS ----
struct Ponto { double lat, lon; };
static const Ponto PONTOS[] = {
  { -8.0564630, -34.9507170 },  // ponto 1
  { -8.0568564, -34.9506161 },  // ponto 2
  { -8.0568544, -34.9504444 },  // ponto 3
  { -8.0569616, -34.9503978 },  // ponto 4
  { -8.0568591, -34.9501497 },  // ponto 5
};
static const int N_PONTOS = sizeof(PONTOS) / sizeof(PONTOS[0]);

// ---- parâmetros ---- n

static const float    SIGMA_SHADOW_DB  = 4.0f;
static const float    SIGMA_FADING_DB  = 4.0f;
static const int      MIN_ANCHORS      = 4;      // 3 incógnitas (x, y, A) -> mínimo 4 pontos
static const int      MAX_BEACONS      = 4;
static const int      CAPTURE_SAMPLES  = 10;     // pacotes por ponto
static const uint32_t CAPTURE_TIMEOUT_MS = 30000;
static const uint32_t DEBOUNCE_MS      = 50;
static const uint32_t STATUS_PERIOD_MS = 15000;

// ---- estado ----
static Preferences prefs;
static loc::PathLossModel model = { (float)MODEL_A_DBM, (float)MODEL_N };
static loc::SolverOptions solverOpt;

// Plano local: origem = ponto 1. px/py = posição de cada ponto em metros.
static double lat0, lon0, cosLat0;
static double px[N_PONTOS], py[N_PONTOS];
static const double EARTH_R = 6371000.0;

struct VirtualAnchor {
  double   x, y;
  float    meanRssi;
  float    m2;
  uint16_t count;       // 0 = ponto ainda não coletado para este beacon
  uint32_t lastMs;
};

struct BeaconTrack {
  bool     used;
  uint32_t id;
  uint32_t lastSeq;
  uint32_t received;
  uint32_t lost;
  uint32_t lastSeenMs;
  uint8_t  lastFlags;
  uint16_t batteryMv;
  bool     hasTruth;
  double   truthLat, truthLon;
  VirtualAnchor anchors[N_PONTOS];   // uma âncora por ponto fixo (índice = ponto)
  loc::Estimate est;
};
static BeaconTrack tracks[MAX_BEACONS];

// ---- estado da coleta ----
static int      curPoint     = 0;     // próximo ponto a coletar
static bool     capturing    = false;
static int      capRemaining = 0;
static uint32_t capStartMs   = 0;
static int      lastDonePoint = -1;   // último ponto concluído (referência p/ rumo)

// ---------------------------------------------------------------------
static void toLocal(double lat, double lon, double& x, double& y) {
  x = (lon - lon0) * DEG_TO_RAD * EARTH_R * cosLat0;
  y = (lat - lat0) * DEG_TO_RAD * EARTH_R;
}
static void toGeo(double x, double y, double& lat, double& lon) {
  lat = lat0 + (y / EARTH_R) * RAD_TO_DEG;
  lon = lon0 + (x / (EARTH_R * cosLat0)) * RAD_TO_DEG;
}

static const char* compass(double deg) {
  static const char* p[] = {"N", "NE", "L", "SE", "S", "SO", "O", "NO"};
  return p[(int)((deg + 22.5) / 45.0) & 7];
}

static BeaconTrack* getTrack(uint32_t id) {
  BeaconTrack* freeSlot = nullptr;
  for (auto& t : tracks) {
    if (t.used && t.id == id) return &t;
    if (!t.used && !freeSlot) freeSlot = &t;
  }
  if (!freeSlot) {
    freeSlot = &tracks[0];
    for (auto& t : tracks) if (t.lastSeenMs < freeSlot->lastSeenMs) freeSlot = &t;
  }
  *freeSlot = BeaconTrack{};
  freeSlot->used = true;
  freeSlot->id = id;
  return freeSlot;
}

// Soma uma medição na âncora do ponto fixo 'idx' (média de Welford).
static void addToPoint(BeaconTrack& t, int idx, float rssi) {
  VirtualAnchor& a = t.anchors[idx];
  a.x = px[idx];
  a.y = py[idx];
  a.count++;
  float delta = rssi - a.meanRssi;
  a.meanRssi += delta / a.count;
  a.m2 += delta * (rssi - a.meanRssi);
  a.lastMs = millis();
}

static int countAnchors(const BeaconTrack& t) {
  int n = 0;
  for (int i = 0; i < N_PONTOS; i++) if (t.anchors[i].count > 0) n++;
  return n;
}

// ---------------------------------------------------------------------
static void runEstimate(BeaconTrack& t) {
  loc::Anchor buf[N_PONTOS];
  int n = 0;
  for (int i = 0; i < N_PONTOS; i++) {
    const VirtualAnchor& v = t.anchors[i];
    if (v.count == 0) continue;
    float var = SIGMA_SHADOW_DB * SIGMA_SHADOW_DB +
                SIGMA_FADING_DB * SIGMA_FADING_DB / v.count;
    buf[n++] = { (float)v.x, (float)v.y, v.meanRssi, 1.0f / var };
  }
  if (n < MIN_ANCHORS) {
    Serial.printf("%d de %d pontos\n", n, MIN_ANCHORS);
    return;
  }

  t.est = loc::multilaterate(buf, n, model, solverOpt);
  if (!t.est.valid) return;

  double elat, elon;
  toGeo(t.est.x, t.est.y, elat, elon);

  // Sem GPS: distância/rumo a partir do último ponto fixo em que você esteve.
  double dist = -1, course = -1;
  if (lastDonePoint >= 0) {
    dist   = TinyGPSPlus::distanceBetween(PONTOS[lastDonePoint].lat, PONTOS[lastDonePoint].lon, elat, elon);
    course = TinyGPSPlus::courseTo(PONTOS[lastDonePoint].lat, PONTOS[lastDonePoint].lon, elat, elon);
  }
  double trueErr = -1;
  if (t.hasTruth)
    trueErr = TinyGPSPlus::distanceBetween(t.truthLat, t.truthLon, elat, elon);

  Serial.printf("ALVO %.0f m\n", dist);
  Serial.printf("lat %.6f\n", elat);
  Serial.printf("lon %.6f\n",elon);

  //if (dist >= 0)
  //  Serial.printf("  do ponto %d: %.0f m, rumo %.0f graus %s\n",
  //                lastDonePoint + 1, dist, course, compass(course));
 //Serial.printf("  pontos=%d A=%.1f residuo=%.1f dB geom=%.2f\n",n, t.est.A, t.est.rmsResidualDb, t.est.geometry);
  //if (trueErr >= 0) Serial.printf("  erro real (GPS do beacon) = %.1f m\n", trueErr);
  //if (t.est.geometry < 0.3f) Serial.println("  geometria ruim: pontos quase alinhados");

  if (dist >= 0 && course >= 0) Serial2.printf("%s\n", compass(course));
}

// ---------------------------------------------------------------------
static volatile bool rxFlag = false;
static void IRAM_ATTR onRxDone() { rxFlag = true; }

static void finishCapture(bool ok) {
  capturing = false;
  if (!ok) {
    //Serial.printf("ponto %d: nenhum pacote recebido (timeout). Aperte o botao de novo.\n", curPoint + 1);
    return;
  }
  Serial.printf("p%d coletado\n", curPoint + 1);
  lastDonePoint = curPoint;
  for (auto& t : tracks) if (t.used && t.anchors[curPoint].count > 0) runEstimate(t);
  if (curPoint < N_PONTOS - 1) {
    curPoint++;
    Serial.printf("va ate ponto %d\n", curPoint + 1);
  } else {
    //Serial.println("# todos os pontos coletados (use 'ponto <n>' para recoletar)");
  }
}

static void startCapture() {
  if (capturing) return;
  // recoleta: zera a âncora desse ponto em todos os beacons
  for (auto& t : tracks) if (t.used) memset(&t.anchors[curPoint], 0, sizeof(VirtualAnchor));
  capturing    = true;
  capRemaining = CAPTURE_SAMPLES;
  capStartMs   = millis();
  //Serial.printf("# coletando no ponto %d (%.6f, %.6f)... fique parado\n", curPoint + 1, PONTOS[curPoint].lat, PONTOS[curPoint].lon);
}

static void handlePacket() {
  uint8_t buf[256];
  size_t size = radio.getPacketLength();
  int16_t st = radio.readData(buf, size);
  if (st != RADIOLIB_ERR_NONE || size != sizeof(BeaconPacket)) return;
  BeaconPacket p;
  memcpy(&p, buf, sizeof(p));
  if (p.magic != PKT_MAGIC || p.version != PKT_VERSION) return;

  float rssi    = radio.getRSSI();
  float snr     = radio.getSNR();
  float rssiEff = snr < 0 ? rssi + snr : rssi;
  float rssiNorm = rssiEff - (p.txPowerDbm - REF_TX_POWER_DBM);

  BeaconTrack& t = *getTrack(p.beaconId);
  if (t.received > 0 && p.seq > t.lastSeq + 1) t.lost += p.seq - t.lastSeq - 1;
  t.lastSeq = p.seq;
  t.received++;
  t.lastSeenMs = millis();
  t.lastFlags = p.flags;
  t.batteryMv = p.batteryMv;
  if (p.flags & FLAG_GPS_VALID) {
    t.hasTruth = true;
    t.truthLat = p.latE7 / 1e7;
    t.truthLon = p.lonE7 / 1e7;
  }

  if (!capturing) {
    //Serial.printf("# [%08X] rssi=%.0f snr=%.1f (aperte o botao no ponto %d p/ coletar)\n",
    //              p.beaconId, rssi, snr, curPoint + 1);
    return;
  }

  digitalWrite(LED_PIN, HIGH);
  addToPoint(t, curPoint, rssiNorm);
  //Serial.printf("# ponto %d  [%08X]  amostra %d/%d  rssi=%.0f\n", curPoint + 1, p.beaconId,
  //              CAPTURE_SAMPLES - capRemaining + 1, CAPTURE_SAMPLES, rssiNorm);
  if (--capRemaining <= 0) finishCapture(true);
  digitalWrite(LED_PIN, LOW);
}

// ---------------------------------------------------------------------
static void handleButton() {
  static bool lastRaw = HIGH, stable = HIGH;
  static uint32_t changedMs = 0;
  bool raw = digitalRead(BUTTON_PIN);
  if (raw != lastRaw) { lastRaw = raw; changedMs = millis(); }
  if (millis() - changedMs > DEBOUNCE_MS && raw != stable) {
    stable = raw;
    if (stable == LOW) startCapture();    // borda de descida = botão apertado
  }
}

static void printStatus() {
  Serial.printf("SOCORRISTA %.1f MHz\n", LORA_FREQ_HZ / 1e6);
  for (auto& t : tracks) {
    if (!t.used) continue;
    //Serial.printf("# beacon %08X: rx=%lu pontos=%d bat=%umV visto ha %lus%s\n",
    //              t.id, (unsigned long)t.received, countAnchors(t), t.batteryMv,
    //              (unsigned long)((millis() - t.lastSeenMs) / 1000),
    //              (t.lastFlags & FLAG_SOS) ? " SOS" : "");
  }
}

// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial2.begin(PICO_BAUD, SERIAL_8N1, PICO_RX_PIN, PICO_TX_PIN);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, BUTTON_PIN >= 34 ? INPUT : INPUT_PULLUP);
  delay(200);

  prefs.begin("socorrista", false);
  model.A = prefs.getFloat("A", MODEL_A_DBM);
  model.n = prefs.getFloat("n", MODEL_N);
  solverOpt.estimateA = prefs.getBool("estA", true);

  // Plano local com origem no ponto 1; converte todos os pontos para metros
  lat0 = PONTOS[0].lat; lon0 = PONTOS[0].lon; cosLat0 = cos(lat0 * DEG_TO_RAD);
  for (int i = 0; i < N_PONTOS; i++) toLocal(PONTOS[i].lat, PONTOS[i].lon, px[i], py[i]);

  int16_t st = loraInit();
  if (st == RADIOLIB_ERR_NONE) {
    radio.setPacketReceivedAction(onRxDone);
    st = radio.startReceive();
  }
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("# ERRO: falha ao iniciar o SX1276 (RadioLib %d)\n", st);
    while (true) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); }
  }

  printStatus();
  Serial.printf("va ao ponto 1\n");
}

void loop() {
  handleButton();

  if (rxFlag) {
    rxFlag = false;
    handlePacket();
  }

  if (capturing && millis() - capStartMs > CAPTURE_TIMEOUT_MS) {
    // aceita se pelo menos 1 amostra chegou
    bool any = false;
    for (auto& t : tracks) if (t.used && t.anchors[curPoint].count > 0) any = true;
    finishCapture(any);
  }

  static uint32_t lastStatus = 0;
  if (millis() - lastStatus > STATUS_PERIOD_MS) {
    lastStatus = millis();
    //printStatus();
  }
}
