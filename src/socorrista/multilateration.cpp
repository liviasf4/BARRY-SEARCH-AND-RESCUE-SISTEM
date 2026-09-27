#include "multilateration.h"
#include <math.h>
#include <float.h>

namespace loc {

static const float K_LN10 = 2.302585093f;

float rssiToDistance(float rssi, const PathLossModel& m) {
  return powf(10.0f, (m.A - rssi) / (10.0f * m.n));
}

// Razão entre os desvios-padrão do menor e do maior eixo da nuvem de âncoras.
float geometryRatio(const Anchor* a, int n) {
  if (n < 3) return 0.f;
  double mx = 0, my = 0;
  for (int i = 0; i < n; i++) { mx += a[i].x; my += a[i].y; }
  mx /= n; my /= n;
  double sxx = 0, syy = 0, sxy = 0;
  for (int i = 0; i < n; i++) {
    double dx = a[i].x - mx, dy = a[i].y - my;
    sxx += dx * dx; syy += dy * dy; sxy += dx * dy;
  }
  double tr = sxx + syy, det = sxx * syy - sxy * sxy;
  double disc = sqrt(fmax(tr * tr / 4 - det, 0.0));
  double l1 = tr / 2 + disc, l2 = tr / 2 - disc;
  if (l1 <= 1e-9) return 0.f;
  return (float)sqrt(fmax(l2, 0.0) / l1);
}

// Custo ponderado em (x,y). Se estA, o A ótimo tem solução fechada.
static double costAt(const Anchor* a, int n, double x, double y, float nexp,
                     bool estA, double Afixed, float wA, double* Aout) {
  // Em float: o ESP32 tem FPU apenas de precisão simples.
  float g[MAX_SOLVER_ANCHORS];
  float sw = 0, swv = 0;
  const float fx = (float)x, fy = (float)y, k = 5.0f * nexp;
  for (int i = 0; i < n; i++) {
    float dx = fx - a[i].x, dy = fy - a[i].y;
    float d2 = fmaxf(dx * dx + dy * dy, 1.0f);
    g[i] = k * log10f(d2);                    // = 10 n log10(d)
    sw  += a[i].weight;
    swv += a[i].weight * (a[i].rssi + g[i]);
  }
  float A = estA ? (swv + wA * (float)Afixed) / (sw + wA) : (float)Afixed;
  float c = estA ? wA * (A - (float)Afixed) * (A - (float)Afixed) : 0.f;
  for (int i = 0; i < n; i++) {
    float r = a[i].rssi - (A - g[i]);
    c += a[i].weight * r * r;
  }
  if (Aout) *Aout = A;
  return c;
}

// Resolve M·x = b (p <= 3) por eliminação de Gauss com pivotamento.
static bool solve(double M[3][3], double b[3], double x[3], int p) {
  double a[3][4];
  for (int i = 0; i < p; i++) { for (int j = 0; j < p; j++) a[i][j] = M[i][j]; a[i][p] = b[i]; }
  for (int c = 0; c < p; c++) {
    int piv = c;
    for (int r = c + 1; r < p; r++) if (fabs(a[r][c]) > fabs(a[piv][c])) piv = r;
    if (fabs(a[piv][c]) < 1e-12) return false;
    if (piv != c) for (int j = 0; j <= p; j++) { double t = a[c][j]; a[c][j] = a[piv][j]; a[piv][j] = t; }
    for (int r = 0; r < p; r++) {
      if (r == c) continue;
      double f = a[r][c] / a[c][c];
      for (int j = c; j <= p; j++) a[r][j] -= f * a[c][j];
    }
  }
  for (int i = 0; i < p; i++) x[i] = a[i][p] / a[i][i];
  return true;
}

static bool invert(double M[3][3], double Inv[3][3], int p) {
  for (int c = 0; c < p; c++) {
    double e[3] = {0, 0, 0}, col[3];
    e[c] = 1;
    if (!solve(M, e, col, p)) return false;
    for (int r = 0; r < p; r++) Inv[r][c] = col[r];
  }
  return true;
}

// Monta J^T J e J^T r para os parâmetros (x, y[, A]).
static double buildNormal(const Anchor* a, int n, const double th[3], float nexp,
                          int p, double H[3][3], double g[3], double A0, double wA) {
  for (int i = 0; i < 3; i++) { g[i] = 0; for (int j = 0; j < 3; j++) H[i][j] = 0; }
  const double k = 10.0 * nexp / K_LN10;
  double cost = 0;
  for (int i = 0; i < n; i++) {
    double dx = th[0] - a[i].x, dy = th[1] - a[i].y;
    double d2 = fmax(dx * dx + dy * dy, 1.0);
    double sw = sqrt((double)a[i].weight);
    double pred = th[2] - 5.0 * nexp * log10(d2);
    double r = sw * (a[i].rssi - pred);
    double J[3] = { sw * k * dx / d2, sw * k * dy / d2, -sw };
    cost += r * r;
    for (int u = 0; u < p; u++) {
      g[u] += J[u] * r;
      for (int v = 0; v < p; v++) H[u][v] += J[u] * J[v];
    }
  }
  if (p == 3) {                       // pseudo-resíduo do prior em A
    double r = sqrt(wA) * (th[2] - A0);
    H[2][2] += wA; g[2] += sqrt(wA) * r; cost += r * r;
  }
  return cost;
}

static double fullCost(const Anchor* a, int n, const double th[3], float nexp,
                       bool estA, double A0, double wA) {
  double c = estA ? wA * (th[2] - A0) * (th[2] - A0) : 0;
  for (int i = 0; i < n; i++) {
    double dx = th[0] - a[i].x, dy = th[1] - a[i].y;
    double d2 = fmax(dx * dx + dy * dy, 1.0);
    double r = a[i].rssi - (th[2] - 5.0 * nexp * log10(d2));
    c += a[i].weight * r * r;
  }
  return c;
}

Estimate multilaterate(const Anchor* a, int n, const PathLossModel& m,
                       const SolverOptions& opt) {
  Estimate e;
  if (n > MAX_SOLVER_ANCHORS) n = MAX_SOLVER_ANCHORS;
  e.nAnchors = n;
  if (n < 3) return e;

  const bool estA = opt.estimateA && n >= opt.minAnchorsForA;
  const int p = estA ? 3 : 2;
  const float wA = 1.0f / (opt.sigmaAPriorDb * opt.sigmaAPriorDb);
  e.geometry = geometryRatio(a, n);

  // ---- 1) Região de busca: caixa das âncoras + maior distância estimada
  double minx = a[0].x, maxx = a[0].x, miny = a[0].y, maxy = a[0].y, dmax = 0;
  for (int i = 0; i < n; i++) {
    if (a[i].x < minx) minx = a[i].x;
    if (a[i].x > maxx) maxx = a[i].x;
    if (a[i].y < miny) miny = a[i].y;
    if (a[i].y > maxy) maxy = a[i].y;
    double d = rssiToDistance(a[i].rssi, m);
    if (d > dmax) dmax = d;
  }
  double margin = fmin(fmax(dmax * 1.5, 50.0), (double)opt.maxRangeM);
  double cx = (minx + maxx) / 2, cy = (miny + maxy) / 2;
  double half = fmax(maxx - minx, maxy - miny) / 2 + margin;

  // ---- 2) Busca em grade com 3 níveis de zoom
  double bestX = cx, bestY = cy, bestA = m.A, bestC = DBL_MAX;
  const int G = opt.gridSize < 5 ? 5 : opt.gridSize;
  for (int level = 0; level < 3; level++) {
    double step = 2 * half / (G - 1);
    double x0 = cx - half, y0 = cy - half;
    for (int ix = 0; ix < G; ix++) {
      for (int iy = 0; iy < G; iy++) {
        double x = x0 + ix * step, y = y0 + iy * step, A;
        double c = costAt(a, n, x, y, m.n, estA, m.A, wA, &A);
        if (c < bestC) { bestC = c; bestX = x; bestY = y; bestA = A; }
      }
    }
    cx = bestX; cy = bestY; half = 2 * step;
  }

  // ---- 3) Refinamento Levenberg-Marquardt
  double th[3] = { bestX, bestY, bestA };
  double lambda = 1e-3;
  double H[3][3], g[3];
  double cost = buildNormal(a, n, th, m.n, p, H, g, m.A, wA);
  int it = 0;
  for (; it < opt.maxIter; it++) {
    bool improved = false;
    const double prevX = th[0], prevY = th[1];
    while (lambda < 1e8) {
      double M[3][3], rhs[3], d[3] = {0, 0, 0};
      for (int u = 0; u < p; u++) {
        for (int v = 0; v < p; v++) M[u][v] = H[u][v];
        M[u][u] += lambda * (H[u][u] + 1e-9);
        rhs[u] = -g[u];
      }
      if (!solve(M, rhs, d, p)) { lambda *= 10; continue; }
      double t2[3] = { th[0] + d[0], th[1] + d[1], estA ? th[2] + d[2] : th[2] };
      double c2 = fullCost(a, n, t2, m.n, estA, m.A, wA);
      if (c2 < cost) {
        th[0] = t2[0]; th[1] = t2[1]; th[2] = t2[2];
        lambda = fmax(lambda / 10, 1e-9);
        improved = true;
        cost = buildNormal(a, n, th, m.n, p, H, g, m.A, wA);
        break;
      }
      lambda *= 10;
    }
    if (!improved) break;
    if (fabs(th[0] - prevX) + fabs(th[1] - prevY) < 0.01) { it++; break; }  // convergiu
  }

  // ---- 4) Covariância ≈ s² (JᵀJ)⁻¹
  double Inv[3][3];
  if (!invert(H, Inv, p)) {
    // Geometria degenerada: devolve posição, mas com incerteza "infinita"
    Inv[0][0] = Inv[1][1] = 1e8; Inv[0][1] = 0;
  }
  double dof = n - p;
  double s2 = dof > 0 ? fmax(1.0, cost / dof) : 1.0;

  e.valid = true;
  e.x = (float)th[0];
  e.y = (float)th[1];
  e.A = (float)th[2];
  e.estimatedA = estA;
  e.sigmaX = (float)sqrt(fabs(Inv[0][0]) * s2);
  e.sigmaY = (float)sqrt(fabs(Inv[1][1]) * s2);
  e.covXY  = (float)(Inv[0][1] * s2);
  e.radius1s = sqrtf(e.sigmaX * e.sigmaX + e.sigmaY * e.sigmaY);
  double sw = 0;
  for (int i = 0; i < n; i++) sw += a[i].weight;
  e.rmsResidualDb = (float)sqrt(cost / sw);
  e.iterations = it;
  return e;
}

}  // namespace loc
