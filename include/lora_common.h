#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include "board_pins.h"

#ifndef LORA_FREQ_HZ
#define LORA_FREQ_HZ 915000000L
#endif
#ifndef LORA_SF
#define LORA_SF 9
#endif
#ifndef LORA_BW_HZ
#define LORA_BW_HZ 125000L
#endif
#ifndef LORA_CR_DEN
#define LORA_CR_DEN 5
#endif
#ifndef LORA_SYNC_WORD
#define LORA_SYNC_WORD 0x12
#endif
#ifndef LORA_PREAMBLE
#define LORA_PREAMBLE 8
#endif

// Potência de referência usada na calibração do modelo de perda.
// O socorrista normaliza o RSSI pela potência informada no pacote.
#define REF_TX_POWER_DBM 14

// ------------------------------------------------------------------
// Formato do pacote (23 bytes). O CRC é feito pelo próprio LoRa.
// ------------------------------------------------------------------
#define PKT_MAGIC   0xB33C
#define PKT_VERSION 1

#define FLAG_GPS_VALID 0x01   // lat/lon do beacon válidos (usado para validação/calibração)
#define FLAG_SOS       0x02   // pessoa apertou o botão de socorro

#pragma pack(push, 1)
struct BeaconPacket {
  uint16_t magic;
  uint8_t  version;
  uint8_t  flags;
  uint32_t beaconId;
  uint32_t seq;
  int8_t   txPowerDbm;
  uint16_t batteryMv;
  int32_t  latE7;   // graus * 1e7
  int32_t  lonE7;
};
#pragma pack(pop)
static_assert(sizeof(BeaconPacket) == 23, "tamanho do pacote mudou");

// Rádio único do projeto. Este header só é incluído pelo main.cpp de cada
// ambiente, então a instância static não é duplicada.
// Module(CS, DIO0 = IRQ, RST, DIO1 = GPIO)
static SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);

// A RadioLib aceita 2..17 dBm ou 20 dBm no PA_BOOST (18 e 19 são inválidos).
inline bool txPowerValid(int p) { return (p >= 2 && p <= 17) || p == 20; }

// Ajusta a potência e o limite de corrente (OCP) do PA. O padrão da RadioLib
// é 60 mA, o que corta a potência real; usa os mesmos limites da arduino-LoRa
// (100 mA, ou 140 mA em 20 dBm) para manter a calibração do RSSI válida.
inline int16_t loraSetTxPower(int8_t dbm) {
  int16_t st = radio.setCurrentLimit(dbm == 20 ? 140 : 100);
  if (st != RADIOLIB_ERR_NONE) return st;
  return radio.setOutputPower(dbm);
}

// Inicializa o SX1276 com os parâmetros comuns. Retorna o código da RadioLib
// (RADIOLIB_ERR_NONE em caso de sucesso).
inline int16_t loraInit(int8_t txPowerDbm = REF_TX_POWER_DBM) {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  int16_t st = radio.begin(LORA_FREQ_HZ / 1e6, LORA_BW_HZ / 1e3, LORA_SF, LORA_CR_DEN,
                           LORA_SYNC_WORD, txPowerDbm, LORA_PREAMBLE);
  if (st != RADIOLIB_ERR_NONE) return st;
  st = loraSetTxPower(txPowerDbm);
  if (st != RADIOLIB_ERR_NONE) return st;
  return radio.setCRC(true);
}

inline uint32_t chipId32() {
  // Bytes menos genéricos do MAC (b2..b5)
  return (uint32_t)(ESP.getEfuseMac() >> 16);
}

inline uint16_t batteryMilliVolts() {
  return (uint16_t)(analogReadMilliVolts(BATTERY_PIN) * 2);
}
