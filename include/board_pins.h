#pragma once
// Pinagem da TTGO T-Beam T22 V0.7 (versão SEM PMU AXP192).
// Atenção: nas versões V1.x os pinos do GPS/LED/botão são diferentes.

// SX1276 (LoRa)
#define LORA_SCK   5
#define LORA_MISO  19
#define LORA_MOSI  27
#define LORA_CS    18
#define LORA_RST   23
#define LORA_DIO0  26
#define LORA_DIO1  33
#define LORA_DIO2  32

// GPS NEO-6M (UART1)
#define GPS_RX_PIN 12   // RX do ESP32  <- TX do GPS
#define GPS_TX_PIN 15   // TX do ESP32  -> RX do GPS
#define GPS_BAUD   9600

// Periféricos
#define LED_PIN     14  // LED de usuário
#define BUTTON_PIN  39  // botão de usuário (ativo em nível baixo, pull-up externo)
#define BATTERY_PIN 35  // divisor 1:2 da bateria 18650
