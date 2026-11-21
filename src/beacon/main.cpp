// =====================================================================
//  BEACON (colete da pessoa) - TTGO T-Beam T22 V0.7
//  Transmite periodicamente um pacote LoRa curto com ID, sequência,
//  potência de TX, bateria e (opcionalmente) a posição GPS do beacon,
//  usada apenas como "verdade de campo" para validar/calibrar o sistema.
//
//  Comandos pela serial (115200, terminar com Enter):
//    status              -> mostra configuração
//    intervalo <ms>      -> período de transmissão (500..600000)
//    potencia <dBm>      -> potência de TX (2..17 ou 20)
//    sos                 -> liga/desliga o modo SOS (igual ao botão)
//  As configurações ficam salvas na flash (Preferences).
// =====================================================================
#include <Arduino.h>
#include <Preferences.h>
#include <TinyGPS++.h>
#include "lora_common.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#ifndef BEACON_INTERVAL_MS
#define BEACON_INTERVAL_MS 2000
#endif
#ifndef BEACON_JITTER_MS
#define BEACON_JITTER_MS 400
#endif
#ifndef BEACON_TX_POWER_DBM
#define BEACON_TX_POWER_DBM 14
#endif
#ifndef BEACON_USE_GPS
#define BEACON_USE_GPS 1
#endif

// Pinos I2C da T-Beam
#define SDA_PIN 21
#define SCL_PIN 22

Adafruit_MPU6050 mpu;

// Limite de g's para considerar uma queda ou impacto brusco
const float LIMITE_QUEDA = 8;  // testar e alterar isso

static TinyGPSPlus gps;
static Preferences prefs;

static uint32_t intervalMs = BEACON_INTERVAL_MS;
static int8_t   txPower    = BEACON_TX_POWER_DBM;
static bool     sosMode    = false;
static uint32_t seq        = 0;
static uint32_t beaconId   = 0;
static uint32_t nextTxMs   = 0;

static void blink(int times, int ms) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH); delay(ms);
    digitalWrite(LED_PIN, LOW);  if (i + 1 < times) delay(ms);
  }
}

static void scheduleNext() {
  int32_t jitter = (int32_t)(esp_random() % (2 * BEACON_JITTER_MS + 1)) - BEACON_JITTER_MS;
  nextTxMs = millis() + intervalMs + jitter;
}

static void printStatus() {
  Serial.printf("# BEACON id=%08X  f=%.1f MHz SF%d BW%ld kHz\n",
                beaconId, LORA_FREQ_HZ / 1e6, LORA_SF, LORA_BW_HZ / 1000);
  Serial.printf("# intervalo=%lu ms  potencia=%d dBm  sos=%d  bateria=%u mV\n",
                (unsigned long)intervalMs, txPower, sosMode, batteryMilliVolts());
#if BEACON_USE_GPS
  Serial.printf("# GPS: fix=%d sats=%lu hdop=%.1f\n", gps.location.isValid(),
                (unsigned long)gps.satellites.value(), gps.hdop.hdop());
#endif
}

static void sendBeacon() {
  BeaconPacket p = {};
  p.magic      = PKT_MAGIC;
  p.version    = PKT_VERSION;
  p.beaconId   = beaconId;
  p.seq        = seq++;
  p.txPowerDbm = txPower;
  p.batteryMv  = batteryMilliVolts();
  if (sosMode) p.flags |= FLAG_SOS;
#if BEACON_USE_GPS
  if (gps.location.isValid() && gps.location.age() < 3000) {
    p.flags |= FLAG_GPS_VALID;
    p.latE7 = (int32_t)lround(gps.location.lat() * 1e7);
    p.lonE7 = (int32_t)lround(gps.location.lng() * 1e7);
  }
#endif

  digitalWrite(LED_PIN, HIGH);
  int16_t st = radio.transmit((uint8_t*)&p, sizeof(p));   // bloqueante até terminar
  radio.sleep();             // rádio dorme entre transmissões
  digitalWrite(LED_PIN, LOW);
  if (st != RADIOLIB_ERR_NONE) Serial.printf("# ERRO TX: %d\n", st);

  Serial.printf("TX,%lu,%08X,%lu,%d,%u,%d,%.7f,%.7f\n",
                (unsigned long)millis(), beaconId, (unsigned long)p.seq, txPower,
                p.batteryMv, p.flags, p.latE7 / 1e7, p.lonE7 / 1e7);
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

    if (cmd == "status") {
      printStatus();
    } else if (cmd == "intervalo") {
      long v = arg.toInt();
      if (v >= 500 && v <= 600000) {
        intervalMs = v; prefs.putUInt("interval", intervalMs); scheduleNext();
        Serial.printf("# intervalo = %lu ms\n", (unsigned long)intervalMs);
      } else Serial.println("# uso: intervalo <500..600000>");
    } else if (cmd == "potencia") {
      long v = arg.toInt();
      if (txPowerValid(v) && loraSetTxPower(v) == RADIOLIB_ERR_NONE) {
        txPower = v; prefs.putChar("txpow", txPower);
        Serial.printf("# potencia = %d dBm\n", txPower);
      } else Serial.println("# uso: potencia <2..17 | 20>");
    } else if (cmd == "sos") {
      sosMode = !sosMode; Serial.printf("# sos = %d\n", sosMode);
    } else {
      Serial.println("# comandos: status | intervalo <ms> | potencia <dBm> | sos");
    }
    line = "";
  }
}

static void handleButtonAcel() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Converte a aceleração de m/s² para 'g'
  float ax = a.acceleration.x / 9.81;
  float ay = a.acceleration.y / 9.81;
  float az = a.acceleration.z / 9.81;

  // Calcula a aceleração resultante em 3D
  float aceleracaoTotal = sqrt(ax * ax + ay * ay + az * az);

  if (aceleracaoTotal > LIMITE_QUEDA) {
    Serial.println("\n>>> [ALERTA]: Possível queda ou impacto detectado! <<<");
    sosMode = 1;
    
    delay(2000); // Pausa para evitar múltiplos disparos no mesmo impacto
  }

  static bool last = true;
  static uint32_t lastChange = 0;
  bool now = digitalRead(BUTTON_PIN);           // HIGH = solto
  if (now != last && millis() - lastChange > 50) {
    lastChange = millis();
    last = now;
    if (!now) {                                 // borda de descida: pressionado
      sosMode = !sosMode;
      //Serial.printf("# botao: sos = %d\n", sosMode);
      blink(sosMode ? 5 : 1, 80);
      nextTxMs = millis();                      // transmite já
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  analogSetPinAttenuation(BATTERY_PIN, ADC_11db);
  delay(200);

  beaconId = chipId32();
  prefs.begin("beacon", false);
  intervalMs = prefs.getUInt("interval", BEACON_INTERVAL_MS);
  txPower    = prefs.getChar("txpow", BEACON_TX_POWER_DBM);
  if (!txPowerValid(txPower)) txPower = BEACON_TX_POWER_DBM;  // valor antigo (18/19 dBm)

#if BEACON_USE_GPS
  Serial1.setRxBufferSize(1024);
  Serial1.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
#endif

  int16_t st = loraInit(txPower);
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("# ERRO: falha ao iniciar o SX1276 (RadioLib %d)\n", st);
    while (true) blink(1, 100);
  }
  radio.sleep();

  printStatus();
  Serial.println("TX,ms,id,seq,txpow,bat_mv,flags,lat,lon");
  blink(3, 100);
  // Atraso inicial aleatório: vários beacons ligados juntos não colidem sempre
  nextTxMs = millis() + (esp_random() % intervalMs);

  // Inicializa o barramento I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("Inicializando MPU6050 na T-Beam...");

  if (!mpu.begin()) {
    Serial.println("Falha ao encontrar o MPU6050! Verifique as conexões.");
    while (1) {
      delay(10);
    }
  }
  Serial.println("Sensor MPU6050 conectado com sucesso!");

  // Configura a faixa do acelerômetro para 8g
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

}


void loop() {
#if BEACON_USE_GPS
  while (Serial1.available()) gps.encode(Serial1.read());
#endif
  handleSerial();
  handleButtonAcel();
// mudança aqui
  if ((int32_t)(millis() - nextTxMs) >= 0 && sosMode) {
    sendBeacon();
    scheduleNext();
  }
  delay(5);
}