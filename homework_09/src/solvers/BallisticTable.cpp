#include "solvers/BallisticTable.h"
#include "Logger.h"
#include <algorithm>
#include <fstream>

// ── Інтерполяційні помічники (деталі реалізації) ─────────────────────────────

namespace {

// Нижній індекс на осі та коефіцієнт [0..1] для одного виміру.
struct Interp {
    int   lo;
    float frac;
};

Interp findInterp(float val, const std::vector<float>& axis) {
    if (val <= axis.front()) return {0, 0.0f};
    if (val >= axis.back())  return {static_cast<int>(axis.size()) - 2, 1.0f};

    auto it = std::lower_bound(axis.begin(), axis.end(), val);
    int  i  = static_cast<int>(it - axis.begin()) - 1;
    if (i < 0) i = 0;

    float frac = (val - axis[i]) / (axis[i + 1] - axis[i]);
    return {i, frac};
}

// Лінійна інтерполяція Result (обидва поля паралельно).
BallisticTable::Result lerp(const BallisticTable::Result& a,
                            const BallisticTable::Result& b, float t) {
    return { a.t     + (b.t     - a.t)     * t,
             a.hDist + (b.hDist - a.hDist) * t };
}

} // namespace

// ── Індексація ───────────────────────────────────────────────────────────────

size_t BallisticTable::index(int iz, int iv, int im, int id, int il) const {
    return ((((static_cast<size_t>(iz) * axisV0.size() + iv)
                                        * axisM.size()  + im)
                                        * axisD.size()  + id)
                                        * axisL.size()  + il);
}

const BallisticTable::Result& BallisticTable::at(
    int iz, int iv, int im, int id, int il) const
{
    return data[index(iz, iv, im, id, il)];
}

// ── Завантаження ──────────────────────────────────────────────────────────────

bool BallisticTable::load(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) { LOG("Error opening ballistic table: " << path); return false; }

    int nZ, nV, nM, nD, nL;
    f >> nZ >> nV >> nM >> nD >> nL;

    axisZ0.resize(nZ); for (auto& v : axisZ0) f >> v;
    axisV0.resize(nV); for (auto& v : axisV0) f >> v;
    axisM.resize(nM);  for (auto& v : axisM)  f >> v;
    axisD.resize(nD);  for (auto& v : axisD)  f >> v;
    axisL.resize(nL);  for (auto& v : axisL)  f >> v;

    // Порядок: Z0 → V0 → m → d → l (зовнішній → внутрішній)
    size_t total = static_cast<size_t>(nZ) * nV * nM * nD * nL;
    data.resize(total);
    for (size_t i = 0; i < total; ++i)
        f >> data[i].t >> data[i].hDist;

    if (f.fail()) { LOG("Error: ballistic table is truncated or malformed: " << path); return false; }
    return true;
}

// ── Пошук з інтерполяцією ─────────────────────────────────────────────────────

BallisticTable::Result BallisticTable::lookup(
    float Z0, float V0, float m, float d, float l) const
{
    Interp iz = findInterp(Z0, axisZ0);
    Interp iv = findInterp(V0, axisV0);
    Interp im = findInterp(m,  axisM);
    Interp id = findInterp(d,  axisD);
    Interp il = findInterp(l,  axisL);

    // 2^5 = 32 вершини гіперкуба. Згортаємо по одній осі за раз:
    // 32 → 16 → 8 → 4 → 2 → 1.

    // l: 32 → 16
    Result v[16];
    for (int a = 0; a < 2; ++a)
     for (int b = 0; b < 2; ++b)
      for (int c = 0; c < 2; ++c)
       for (int e = 0; e < 2; ++e) {
           const Result& lo = at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo);
           const Result& hi = at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo + 1);
           v[a*8 + b*4 + c*2 + e] = lerp(lo, hi, il.frac);
       }

    // d: 16 → 8
    Result w[8];
    for (int a = 0; a < 2; ++a)
     for (int b = 0; b < 2; ++b)
      for (int c = 0; c < 2; ++c)
       w[a*4 + b*2 + c] = lerp(v[a*8 + b*4 + c*2], v[a*8 + b*4 + c*2 + 1], id.frac);

    // m: 8 → 4
    Result u[4];
    for (int a = 0; a < 2; ++a)
     for (int b = 0; b < 2; ++b)
      u[a*2 + b] = lerp(w[a*4 + b*2], w[a*4 + b*2 + 1], im.frac);

    // V0: 4 → 2
    Result s[2];
    for (int a = 0; a < 2; ++a)
        s[a] = lerp(u[a*2], u[a*2 + 1], iv.frac);

    // Z0: 2 → 1
    return lerp(s[0], s[1], iz.frac);
}
