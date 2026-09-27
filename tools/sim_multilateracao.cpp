// Simulação no PC do solver de multilateração (mesmo código do firmware).
// Compilar e rodar:
//   g++ -O2 -std=c++17 -I../src/socorrista sim_multilateracao.cpp ../src/socorrista/multilateration.cpp -o sim && ./sim
//
// Cenário: beacon parado; socorrista caminha em "L" coletando pacotes a cada 5 s.
// RSSI gerado pelo modelo log-distância + sombreamento + desvanecimento gaussianos.
#include <cstdio>
#include <cmath>
#include <random>
#include <vector>
#include <algorithm>
#include "multilateration.h"

struct VA { double x, y, sum; int n; };

int main() {
  const float A_true = -45.f, n_true = 2.7f;
  const float sigShadow = 4.f, sigFade = 4.f;
  const double bx = 180, by = 240;               // posição real do beacon (m)
  loc::PathLossModel model = { -48.f, 2.7f };    // A do modelo com 3 dB de erro de propósito
  loc::SolverOptions opt;

  std::mt19937 rng(42);
  std::normal_distribution<float> N01(0.f, 1.f);
  std::vector<double> errs, radii;

  for (int trial = 0; trial < 300; trial++) {
    std::vector<VA> anchors;
    // sombreamento fixo por âncora (depende do local, não do instante)
    std::vector<float> shadow;
    // trajeto: 0..400 m para leste, depois 0..300 m para norte (1.4 m/s, pacote a cada 5 s)
    for (double s = 0; s < 700; s += 7) {
      double x = s < 400 ? s : 400, y = s < 400 ? 0 : s - 400;
      x += N01(rng) * 2.5; y += N01(rng) * 2.5;   // erro do GPS do socorrista
      double d = std::max(1.0, std::hypot(x - bx, y - by));
      // agrupa em âncoras de raio 8 m
      int k = -1;
      for (size_t i = 0; i < anchors.size(); i++)
        if (std::hypot(anchors[i].x - x, anchors[i].y - y) < 8) { k = (int)i; break; }
      if (k < 0) { anchors.push_back({x, y, 0, 0}); shadow.push_back(N01(rng) * sigShadow); k = (int)anchors.size() - 1; }
      float rssi = A_true - 10 * n_true * log10f((float)d) + shadow[k] + N01(rng) * sigFade;
      if (rssi < -137) continue;                  // abaixo da sensibilidade SF9
      anchors[k].sum += rssi; anchors[k].n++;
    }
    std::vector<loc::Anchor> a;
    for (auto& v : anchors) if (v.n > 0) {
      float var = sigShadow * sigShadow + sigFade * sigFade / v.n;
      a.push_back({ (float)v.x, (float)v.y, (float)(v.sum / v.n), 1.f / var });
    }
    auto e = loc::multilaterate(a.data(), (int)a.size(), model, opt);
    if (!e.valid) continue;
    errs.push_back(std::hypot(e.x - bx, e.y - by));
    radii.push_back(e.radius1s);
    if (trial == 0)
      printf("exemplo: est=(%.1f, %.1f) real=(%.0f, %.0f) raio1s=%.1f m A=%.1f anc=%d geom=%.2f it=%d\n",
             e.x, e.y, bx, by, e.radius1s, e.A, e.nAnchors, e.geometry, e.iterations);
  }
  std::sort(errs.begin(), errs.end());
  std::sort(radii.begin(), radii.end());
  printf("tentativas=%zu  erro mediano=%.1f m  p90=%.1f m  raio1s mediano=%.1f m\n",
         errs.size(), errs[errs.size() / 2], errs[errs.size() * 9 / 10], radii[radii.size() / 2]);
  return 0;
}
