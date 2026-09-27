# Localização de pessoas via LoRa — T-Beam T22 V0.7

Um projeto PlatformIO, dois firmwares:

| Ambiente     | Papel | O que faz |
|--------------|-------|-----------|
| `beacon`     | colete da pessoa | transmite um pacote de 23 bytes a cada `intervalo` ms (± jitter) |
| `socorrista` | unidade móvel | mede RSSI/SNR, marca cada pacote com a posição GPS (âncoras virtuais) e estima a posição do beacon por multilateração |

## Como usar no VS Code

1. Instale a extensão **PlatformIO IDE** e abra esta pasta.
2. Grave o beacon: `pio run -e beacon -t upload` (ou pela barra do PlatformIO → *beacon* → *Upload*).
3. Grave o socorrista: `pio run -e socorrista -t upload`.
4. Abra o monitor serial (115200): `pio device monitor -e socorrista`.

Rádio: 915 MHz, SF9, BW 125 kHz, CR 4/5, sync word 0x12, via [RadioLib](https://github.com/jgromes/RadioLib). Altere em `platformio.ini` — os dois ambientes precisam usar os mesmos valores. Potência do beacon: 2–17 dBm ou 20 dBm.

## Estrutura

```
platformio.ini
include/board_pins.h          pinos da T-Beam V0.7
include/lora_common.h         config de rádio + formato do pacote
src/beacon/main.cpp
src/socorrista/main.cpp
src/socorrista/multilateration.{h,cpp}   solver (C++ puro, testável no PC)
tools/sim_multilateracao.cpp  simulação Monte Carlo do solver
```

## Pacote (23 bytes, CRC do LoRa)

`magic(2) versao(1) flags(1) id(4) seq(4) txPower(1) bateria_mV(2) lat_e7(4) lon_e7(4)`

Flags: `0x01` = o beacon mandou a própria posição GPS (verdade de campo, para validação e calibração), `0x02` = SOS (botão GPIO39).

## Como o socorrista estima a posição

1. **RSSI efetivo:** `rssi + snr` quando SNR < 0 (abaixo do piso de ruído o RSSI do pacote superestima o sinal), normalizado pela potência TX do pacote.
2. **Âncoras virtuais:** as recepções feitas dentro de um raio de 8 m (`raio`) são agrupadas. A média do RSSI reduz o desvanecimento rápido. Só entram recepções com GPS bom (HDOP ≤ 5, ≥ 4 satélites).
3. **Modelo:** `RSSI(d) = A − 10·n·log10(d)`.
4. **Solver:** busca em grade com 3 níveis de zoom (evita mínimos locais), depois Levenberg-Marquardt minimizando o erro em dB, com pesos `1/(σ_somb² + σ_desv²/N)`. Com ≥ 5 âncoras, `A` também é estimado, com um prior de ±6 dB.
5. **Saída:** lat/lon estimados, raio de incerteza 1σ, distância e rumo até o alvo, e o **índice de geometria** (0 = âncoras colineares). Abaixo de 0,3 o firmware pede para o socorrista se deslocar perpendicularmente ao trajeto. Com âncoras em linha reta existe uma solução espelhada.

## Calibração (faça antes de ir a campo)

- **Manual:** posicione o socorrista a uma distância conhecida do beacon, digite `cal 10`, espere 10 pacotes e repita em 25, 50, 100 e 200 m. Depois digite `ajustar`.
- **Automática:** com o GPS ligado no beacon, digite `autocal 1` e caminhe variando a distância. Depois digite `ajustar`.
- Os valores de `A` e `n` ficam salvos na flash. Também dá para ajustar à mão com `a -45` e `n 2.7`.

## Simulação no PC

```
cd tools
g++ -O2 -std=c++17 -I../src/socorrista sim_multilateracao.cpp ../src/socorrista/multilateration.cpp -o sim && ./sim
```

Cenário simulado: alvo a cerca de 300 m, trajeto em "L" de 700 m, ruído de 4 dB de sombreamento + 4 dB de desvanecimento, e A do modelo com 3 dB de erro. Resultado: erro mediano de aproximadamente 47 m e p90 de aproximadamente 93 m.

## Próximos passos sugeridos

- Registrar os logs CSV (`PKT`/`EST`) em cartão SD ou enviá-los por BLE para um celular.
- Reduzir o consumo do beacon: light-sleep do ESP32 e modo backup do NEO-6M via UBX. A V0.7 não tem AXP192 para desligar o GPS.
- Usar vários socorristas compartilhando âncoras, o que melhora a geometria.
