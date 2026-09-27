// =====================================================================
//  SOCORRISTA (unidade móvel) - TTGO T-Beam T22 V0.7
//  - Recebe os pacotes dos beacons e mede RSSI / SNR / erro de frequência.
//  - Marca cada recepção com a própria posição GPS -> "âncora virtual".
//  - Agrupa amostras próximas (ANCHOR_RADIUS_M) em uma mesma âncora,
//    fazendo a média do RSSI (reduz o desvanecimento rápido).
//  - Estima a posição do beacon por multilateração (grade + LM)
//    e informa distância e rumo do socorrista até o alvo.
//
//  Saída serial (115200) em CSV + linhas humanas iniciadas por '#':
//    PKT,ms,id,seq,rssi,snr,rssi_eff,freq_err,lat,lon,hdop,sats,ancora,flags,b_lat,b_lon
//    EST,ms,id,lat,lon,raio1s_m,n_ancoras,A,rms_db,geom,dist_m,rumo_deg,erro_real_m
//
//  Comandos (terminar com Enter):
//    status | reset | n <valor> | a <dBm> | raio <m> | estimarA 0/1
//    cal <dist_m>  -> as próximas 10 recepções entram na calibração com essa distância
//    autocal 0/1   -> usa o GPS do beacon (se enviado) como distância real
//    ajustar       -> ajusta A e n por mínimos quadrados com os pontos de calibração
//    calreset      -> apaga os pontos de calibração
// =====================================================================
#include <Arduino.h>
#include <Preferences.h>
#include <TinyGPS++.h>
#include "lora_common.h"
#include "multilateration.h"

#ifndef MODEL_A_DBM
#define MODEL_A_DBM -45.0
#endif
#ifndef MODEL_N
#define MODEL_N 2.7
#endif
#ifndef ANCHOR_RADIUS_M
#define ANCHOR_RADIUS_M 8.0
#endif

// ---- parâmetros de qualidade ----
static const float    MAX_HDOP          = 5.0f;   // GPS do socorrista precisa estar bom
static const uint32_t MAX_FIX_AGE_MS    = 2000;
static const int      MIN_SATS          = 4;
static const float    SIGMA_SHADOW_DB   = 4.0f;   // sombreamento (não cai com a média)
static const float    SIGMA_FADING_DB   = 4.0f;   // desvanecimento rápido (cai com 1/N)
static const int      MIN_ANCHORS       = 3;
static const int      MAX_ANCHORS       = 40;
static const int      MAX_BEACONS       = 4;
static const int      CAL_SAMPLES       = 10;
static const uint32_t STATUS_PERIOD_MS  = 15000;

// ---- estado ----
static TinyGPSPlus gps;
static Preferences prefs;
static loc::PathLossModel model = { (float)MODEL_A_DBM, (float)MODEL_N };
static loc::SolverOptions solverOpt;
static float anchorRadius = ANCHOR_RADIUS_M;

// Origem do plano local (primeira posição GPS válida do socorrista)
static bool   originSet = false;
static double lat0 = 0, lon0 = 0, cosLat0 = 1;
static const double EARTH_R = 6371000.0;

struct VirtualAnchor {
  double  x, y;          // posição média (m)
  float   meanRssi;      // média de Welford
  float   m2;
  uint16_t count;
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
  VirtualAnchor anchors[MAX_ANCHORS];
  int      nAnchors;
  loc::Estimate est;
};
static BeaconTrack tracks[MAX_BEACONS];

// Calibração: regressão linear  rssi = A + n * x,  x = -10 log10(d)
struct CalSums { double n, sx, sy, sxx, sxy; } cal = {0, 0, 0, 0, 0};
static int   calRemaining = 0;
static float calDistance  = 0;
static bool  autoCal      = false;

// ---------------------------------------------------------------------
static void toLocal(double lat, double lon, double& x, double& y) {
  x = (lon - lon0) * DEG_TO_RAD * EARTH_R * cosLat0;
  y = (lat - lat0) * DEG_TO_RAD * EARTH_R;
}
static void toGeo(double x, double y, double& lat, double& lon) {
  lat = lat0 + (y / EARTH_R) * RAD_TO_DEG;
  lon = lon0 + (x / (EARTH_R * cosLat0)) * RAD_TO_DEG;
}

static bool gpsGood() {
  return gps.location.isValid() && gps.location.age() < MAX_FIX_AGE_MS &&
         gps.hdop.isValid() && gps.hdop.hdop() <= MAX_HDOP &&
         (int)gps.satellites.value() >= MIN_SATS;
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
  if (!freeSlot) {                         // substitui o mais antigo
    freeSlot = &tracks[0];
    for (auto& t : tracks) if (t.lastSeenMs < freeSlot->lastSeenMs) freeSlot = &t;
  }
  *freeSlot = BeaconTrack{};
  freeSlot->used = true;
  freeSlot->id = id;
  Serial.printf("# novo beacon %08X\n", id);
  return freeSlot;
}

// Adiciona uma medição à âncora virtual mais próxima (ou cria uma nova).
static int addToAnchor(BeaconTrack& t, double x, double y, float rssi) {
  int best = -1; double bestD = 1e18;
  for (int i = 0; i < t.nAnchors; i++) {
    double dx = t.anchors[i].x - x, dy = t.anchors[i].y - y;
    double d = sqrt(dx * dx + dy * dy);
    if (d < bestD) { bestD = d; best = i; }
  }
  VirtualAnchor* a;
  if (best >= 0 && bestD <= anchorRadius) {
    a = &t.anchors[best];
  } else {
    if (t.nAnchors < MAX_ANCHORS) {
      best = t.nAnchors++;
    } else {                               // cheio: recicla a âncora mais antiga
      best = 0;
      for (int i = 1; i < t.nAnchors; i++)
        if (t.anchors[i].lastMs < t.anchors[best].lastMs) best = i;
    }
    a = &t.anchors[best];
    memset(a, 0, sizeof(*a));
  }
  a->count++;
  float delta = rssi - a->meanRssi;
  a->meanRssi += delta / a->count;
  a->m2 += delta * (rssi - a->meanRssi);
  a->x += (x - a->x) / a->count;
  a->y += (y - a->y) / a->count;
  a->lastMs = millis();
  return best;
}

static void runEstimate(BeaconTrack& t) {
  loc::Anchor buf[MAX_ANCHORS];
  int n = 0;
  for (int i = 0; i < t.nAnchors; i++) {
    const VirtualAnchor& v = t.anchors[i];
    float var = SIGMA_SHADOW_DB * SIGMA_SHADOW_DB +
                SIGMA_FADING_DB * SIGMA_FADING_DB / v.count;
    buf[n++] = { (float)v.x, (float)v.y, v.meanRssi, 1.0f / var };
  }
  if (n < MIN_ANCHORS) {
    Serial.printf("# [%08X] %d/%d ancoras - continue se deslocando\n", t.id, n, MIN_ANCHORS);
    return;
  }

  uint32_t t0 = micros();
  t.est = loc::multilaterate(buf, n, model, solverOpt);
  uint32_t dt = micros() - t0;
  if (!t.est.valid) return;

  double elat, elon;
  toGeo(t.est.x, t.est.y, elat, elon);

  double dist = -1, course = -1;
  if (gps.location.isValid()) {
    dist   = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), elat, elon);
    course = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), elat, elon);
  }
  double trueErr = -1;
  if (t.hasTruth)
    trueErr = TinyGPSPlus::distanceBetween(t.truthLat, t.truthLon, elat, elon);

  Serial.printf("EST,%lu,%08X,%.7f,%.7f,%.1f,%d,%.1f,%.2f,%.2f,%.1f,%.0f,%.1f\n",
                (unsigned long)millis(), t.id, elat, elon, t.est.radius1s, n,
                t.est.A, t.est.rmsResidualDb, t.est.geometry, dist, course, trueErr);
  Serial.printf("# [%08X] ALVO ~ %.6f, %.6f  (+/- %.0f m)  |  %.0f m rumo %.0f%s %s\n",
                t.id, elat, elon, t.est.radius1s, dist, course, "\xC2\xB0", compass(course));
  Serial.printf("#            ancoras=%d  A=%.1f%s  residuo=%.1f dB  geom=%.2f  (%lu ms)\n",
                n, t.est.A, t.est.estimatedA ? "(est.)" : "", t.est.rmsResidualDb,
                t.est.geometry, (unsigned long)(dt / 1000));
  if (trueErr >= 0) Serial.printf("#            erro real (GPS do beacon) = %.1f m\n", trueErr);
  if (t.est.geometry < 0.3f)
    Serial.println("#            geometria ruim: desloque-se PERPENDICULAR ao trajeto atual");
}

static void addCalPoint(float rssi, double d) {
  if (d < 2.0) return;
  double x = -10.0 * log10(d);
  cal.n += 1; cal.sx += x; cal.sy += rssi; cal.sxx += x * x; cal.sxy += x * rssi;
}

static void fitCalibration() {
  double den = cal.n * cal.sxx - cal.sx * cal.sx;
  if (cal.n < 5) { Serial.println("# calibracao: pontos insuficientes (min 5)"); return; }
  if (den / (cal.n * cal.n) < 1.0) {
    // pouca variação de distância: ajusta só A mantendo n
    model.A = (cal.sy - model.n * cal.sx) / cal.n;
    Serial.printf("# calibracao (so A, distancias pouco variadas): A=%.1f n=%.2f\n", model.A, model.n);
  } else {
    double nn = (cal.n * cal.sxy - cal.sx * cal.sy) / den;
    double A  = (cal.sy - nn * cal.sx) / cal.n;
    if (nn < 1.5 || nn > 6.0) {
      Serial.printf("# calibracao rejeitada: n=%.2f fora de [1.5, 6]\n", nn);
      return;
    }
    model.A = A; model.n = nn;
    Serial.printf("# calibracao: A=%.1f dBm  n=%.2f  (%d pontos)\n", model.A, model.n, (int)cal.n);
  }
  prefs.putFloat("A", model.A);
  prefs.putFloat("n", model.n);
}

// ---------------------------------------------------------------------
// DIO0 (RxDone) só levanta a flag; o pacote é lido fora da interrupção.
static volatile bool rxFlag = false;
static void IRAM_ATTR onRxDone() { rxFlag = true; }

static void handlePacket() {
  uint8_t buf[256];
  size_t size = radio.getPacketLength();
  int16_t st = radio.readData(buf, size);   // o rádio continua em RX contínuo
  if (st != RADIOLIB_ERR_NONE || size != sizeof(BeaconPacket)) return;  // CRC/cabeçalho ruim ou outro formato
  BeaconPacket p;
  memcpy(&p, buf, sizeof(p));
  if (p.magic != PKT_MAGIC || p.version != PKT_VERSION) return;

  float rssi  = radio.getRSSI();
  float snr   = radio.getSNR();
  long  ferr  = lroundf(radio.getFrequencyError());
  // Abaixo do piso de ruído (SNR<0) o RSSI do pacote superestima o sinal:
  // potência do sinal ≈ RSSI + SNR  (datasheet SX1276, sec. 5.5.5)
  float rssiEff = snr < 0 ? rssi + snr : rssi;
  // Normaliza para a potência de referência usada na calibração
  float rssiNorm = rssiEff - (p.txPowerDbm - REF_TX_POWER_DBM);

  digitalWrite(LED_PIN, HIGH);

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

  bool good = gpsGood();
  int anchorIdx = -1;
  double myLat = gps.location.lat(), myLon = gps.location.lng();
  if (good) {
    if (!originSet) {
      originSet = true; lat0 = myLat; lon0 = myLon; cosLat0 = cos(lat0 * DEG_TO_RAD);
    }
    double x, y;
    toLocal(myLat, myLon, x, y);
    anchorIdx = addToAnchor(t, x, y, rssiNorm);
  }

  Serial.printf("PKT,%lu,%08X,%lu,%.0f,%.2f,%.1f,%ld,%.7f,%.7f,%.1f,%lu,%d,%u,%.7f,%.7f\n",
                (unsigned long)millis(), p.beaconId, (unsigned long)p.seq, rssi, snr, rssiEff,
                ferr, myLat, myLon, gps.hdop.hdop(), (unsigned long)gps.satellites.value(),
                anchorIdx, p.flags, p.latE7 / 1e7, p.lonE7 / 1e7);
  if (p.flags & FLAG_SOS) Serial.printf("# !!! SOS ativo no beacon %08X !!!\n", p.beaconId);
  if (!good) Serial.println("# GPS do socorrista sem qualidade - amostra nao usada como ancora");

  // ---- calibração ----
  if (calRemaining > 0) {
    addCalPoint(rssiNorm, calDistance);
    if (--calRemaining == 0)
      Serial.printf("# calibracao: coleta a %.1f m concluida (%d pontos no total)\n",
                    calDistance, (int)cal.n);
  }
  if (autoCal && good && (p.flags & FLAG_GPS_VALID)) {
    addCalPoint(rssiNorm, TinyGPSPlus::distanceBetween(myLat, myLon, t.truthLat, t.truthLon));
  }

  if (good) runEstimate(t);
  digitalWrite(LED_PIN, LOW);
}

static void printStatus() {
  Serial.printf("# SOCORRISTA  f=%.1f MHz SF%d BW%ld kHz\n", LORA_FREQ_HZ / 1e6, LORA_SF,
                LORA_BW_HZ / 1000);
  Serial.printf("# modelo: A=%.1f dBm  n=%.2f  raio_ancora=%.1f m  estimarA=%d  autocal=%d\n",
                model.A, model.n, anchorRadius, solverOpt.estimateA, autoCal);
  Serial.printf("# GPS: fix=%d sats=%lu hdop=%.1f  bom=%d\n", gps.location.isValid(),
                (unsigned long)gps.satellites.value(), gps.hdop.hdop(), gpsGood());
  for (auto& t : tracks) {
    if (!t.used) continue;
    float per = t.received + t.lost > 0 ? 100.0f * t.lost / (t.received + t.lost) : 0;
    Serial.printf("# beacon %08X: rx=%lu perdidos=%.0f%% ancoras=%d bat=%umV visto ha %lus%s\n",
                  t.id, (unsigned long)t.received, per, t.nAnchors, t.batteryMv,
                  (unsigned long)((millis() - t.lastSeenMs) / 1000),
                  (t.lastFlags & FLAG_SOS) ? " SOS" : "");
  }
}

static void handleSerial() {
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') { line += c; continue; }
    line.trim();
    if (line.isEmpty()) continue;
    int sp = line.indexOf(' ');
    String cmd = sp < 0 ? line : line.substring(0, sp);
    String arg = sp < 0 ? "" : line.substring(sp + 1);
    cmd.toLowerCase();
    float v = arg.toFloat();

    if (cmd == "status") printStatus();
    else if (cmd == "reset") {
      for (auto& t : tracks) t.used = false;
      originSet = false;
      Serial.println("# ancoras e beacons apagados");
    } else if (cmd == "n" && v >= 1.5f && v <= 6.0f) {
      model.n = v; prefs.putFloat("n", v); Serial.printf("# n = %.2f\n", v);
    } else if (cmd == "a" && v <= 0 && v >= -120) {
      model.A = v; prefs.putFloat("A", v); Serial.printf("# A = %.1f\n", v);
    } else if (cmd == "raio" && v >= 1 && v <= 100) {
      anchorRadius = v; prefs.putFloat("raio", v); Serial.printf("# raio = %.1f m\n", v);
    } else if (cmd == "estimara") {
      solverOpt.estimateA = arg.toInt() != 0; prefs.putBool("estA", solverOpt.estimateA);
      Serial.printf("# estimarA = %d\n", solverOpt.estimateA);
    } else if (cmd == "cal" && v >= 2) {
      calDistance = v; calRemaining = CAL_SAMPLES;
      Serial.printf("# calibracao: aguardando %d pacotes a %.1f m\n", CAL_SAMPLES, v);
    } else if (cmd == "autocal") {
      autoCal = arg.toInt() != 0; Serial.printf("# autocal = %d\n", autoCal);
    } else if (cmd == "ajustar") fitCalibration();
    else if (cmd == "calreset") {
      cal = {0, 0, 0, 0, 0}; Serial.println("# pontos de calibracao apagados");
    } else {
      Serial.println("# comandos: status | reset | n <v> | a <dBm> | raio <m> | estimarA 0/1");
      Serial.println("#           cal <m> | autocal 0/1 | ajustar | calreset");
    }
    line = "";
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  delay(200);

  prefs.begin("socorrista", false);
  model.A = prefs.getFloat("A", MODEL_A_DBM);
  model.n = prefs.getFloat("n", MODEL_N);
  anchorRadius = prefs.getFloat("raio", ANCHOR_RADIUS_M);
  solverOpt.estimateA = prefs.getBool("estA", true);

  Serial1.setRxBufferSize(2048);   // o solver pode levar ~centenas de ms
  Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

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
  Serial.println("PKT,ms,id,seq,rssi,snr,rssi_eff,freq_err,lat,lon,hdop,sats,ancora,flags,b_lat,b_lon");
  Serial.println("EST,ms,id,lat,lon,raio1s_m,n_ancoras,A,rms_db,geom,dist_m,rumo_deg,erro_real_m");
}

void loop() {
  while (Serial1.available()) gps.encode(Serial1.read());
  handleSerial();

  if (rxFlag) {
    rxFlag = false;
    handlePacket();
  }

  static uint32_t lastStatus = 0;
  if (millis() - lastStatus > STATUS_PERIOD_MS) {
    lastStatus = millis();
    printStatus();
  }
}
