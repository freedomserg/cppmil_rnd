#include "solvers/BallisticTable.h"
#include "Logger.h"
#include <algorithm>
#include <array>
#include <fstream>

namespace {

struct Interp {
    int   lo;
    float frac;
};

auto findInterp(float val, const std::vector<float>& axis) -> Interp {
    if (val <= axis.front()) return {0, 0.0f};
    if (val >= axis.back())  return {static_cast<int>(axis.size()) - 2, 1.0f};

    auto it = std::lower_bound(axis.begin(), axis.end(), val);
    int  i  = static_cast<int>(it - axis.begin()) - 1;
    if (i < 0) i = 0;

    float frac = (val - axis[i]) / (axis[i + 1] - axis[i]);
    return {i, frac};
}

auto lerp(const BallisticTable::Result& a, const BallisticTable::Result& b, float t)
    -> BallisticTable::Result {
    return { a.t     + (b.t     - a.t)     * t,
             a.hDist + (b.hDist - a.hDist) * t };
}

} // namespace

auto BallisticTable::index(int iz, int iv, int im, int id, int il) const -> std::size_t {
    return ((((static_cast<std::size_t>(iz) * axisV0.size() + iv)
                                            * axisM.size()  + im)
                                            * axisD.size()  + id)
                                            * axisL.size()  + il);
}

auto BallisticTable::at(int iz, int iv, int im, int id, int il) const -> const Result& {
    return data[index(iz, iv, im, id, il)];
}

auto BallisticTable::load(const std::string& path) -> bool {
    std::ifstream f(path);
    if (!f.is_open()) { LOG("Error opening ballistic table: " << path); return false; }

    int nZ = 0, nV = 0, nM = 0, nD = 0, nL = 0;
    f >> nZ >> nV >> nM >> nD >> nL;

    axisZ0.resize(nZ); for (auto& v : axisZ0) f >> v;
    axisV0.resize(nV); for (auto& v : axisV0) f >> v;
    axisM.resize(nM);  for (auto& v : axisM)  f >> v;
    axisD.resize(nD);  for (auto& v : axisD)  f >> v;
    axisL.resize(nL);  for (auto& v : axisL)  f >> v;

    std::size_t total = static_cast<std::size_t>(nZ) * nV * nM * nD * nL;
    data.resize(total);
    for (std::size_t i = 0; i < total; ++i)
        f >> data[i].t >> data[i].hDist;

    if (f.fail()) { LOG("Error: ballistic table is truncated or malformed: " << path); return false; }
    return true;
}

auto BallisticTable::lookup(float Z0, float V0, float m, float d, float l) const -> Result {
    Interp iz = findInterp(Z0, axisZ0);
    Interp iv = findInterp(V0, axisV0);
    Interp im = findInterp(m,  axisM);
    Interp id = findInterp(d,  axisD);
    Interp il = findInterp(l,  axisL);

    // 2^5 = 32 вершини гіперкуба, згортаємо по одній осі: 32→16→8→4→2→1.
    std::array<Result, 16> v{};
    for (int a = 0; a < 2; ++a)
     for (int b = 0; b < 2; ++b)
      for (int c = 0; c < 2; ++c)
       for (int e = 0; e < 2; ++e) {
           const Result& lo = at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo);
           const Result& hi = at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo + 1);
           v.at(a*8 + b*4 + c*2 + e) = lerp(lo, hi, il.frac);
       }

    std::array<Result, 8> w{};
    for (int a = 0; a < 2; ++a)
     for (int b = 0; b < 2; ++b)
      for (int c = 0; c < 2; ++c)
       w.at(a*4 + b*2 + c) = lerp(v.at(a*8 + b*4 + c*2), v.at(a*8 + b*4 + c*2 + 1), id.frac);

    std::array<Result, 4> u{};
    for (int a = 0; a < 2; ++a)
     for (int b = 0; b < 2; ++b)
      u.at(a*2 + b) = lerp(w.at(a*4 + b*2), w.at(a*4 + b*2 + 1), im.frac);

    std::array<Result, 2> s{};
    for (int a = 0; a < 2; ++a)
        s.at(a) = lerp(u.at(a*2), u.at(a*2 + 1), iv.frac);

    return lerp(s.at(0), s.at(1), iz.frac);
}
