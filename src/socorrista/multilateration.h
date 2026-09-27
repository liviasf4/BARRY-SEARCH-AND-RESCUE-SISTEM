#pragma once
// Multilateração por RSSI com âncoras virtuais.
// C++ puro (sem Arduino) para poder ser testado no PC (ver tools/sim_multilateracao.cpp).
//
// Modelo log-distância:  RSSI(d) = A - 10 * n * log10(d / 1 m)
// Solver: busca em grade (evita mínimos locais) + Levenberg-Marquardt
// minimizando o erro em dB (domínio do RSSI), com peso por âncora.
// Opcionalmente estima A junto (x, y, A) quando há âncoras suficientes,
// o que absorve erros de calibração, antena, corpo da pessoa etc.

namespace loc {

constexpr int MAX_SOLVER_ANCHORS = 64;

struct Anchor {
  float x, y;     // posição da âncora no plano local (m, leste/norte)
  float rssi;     // RSSI efetivo médio (dBm), já normalizado pela potência TX
  float weight;   // 1 / variância do RSSI médio (1/dB²)
};

struct PathLossModel {
  float A;        // RSSI a 1 m (dBm)
  float n;        // expoente de perda de percurso
};

struct SolverOptions {
  bool  estimateA      = true;
  int   minAnchorsForA = 5;      // estima A só com >= esse nº de âncoras
  float sigmaAPriorDb  = 6.f;    // prior em A (dB): evita a ambiguidade A x distância
  float maxRangeM      = 3000.f; // limite da região de busca
  int   gridSize       = 41;
  int   maxIter        = 40;
};

struct Estimate {
  bool  valid = false;
  float x = 0, y = 0;         // posição estimada (m)
  float A = 0;                // A usado/estimado
  bool  estimatedA = false;
  float sigmaX = 0, sigmaY = 0, covXY = 0;
  float radius1s = 0;         // sqrt(sx² + sy²), incerteza 1-sigma (m)
  float rmsResidualDb = 0;
  float geometry = 0;         // 0 = âncoras colineares ... 1 = bem espalhadas
  int   nAnchors = 0;
  int   iterations = 0;
};

float rssiToDistance(float rssi, const PathLossModel& m);
float geometryRatio(const Anchor* a, int n);
Estimate multilaterate(const Anchor* a, int n, const PathLossModel& m,
                       const SolverOptions& opt);

}  // namespace loc
