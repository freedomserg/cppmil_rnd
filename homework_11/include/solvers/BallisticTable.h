#pragma once

#include <cstddef>
#include <string>
#include <vector>

// 5-вимірна сітка результатів балістики; між вузлами — лінійна інтерполяція,
// за межами — крайнє значення (clamp).
struct BallisticTable {
    std::vector<float> axisZ0;  // висота
    std::vector<float> axisV0;  // швидкість
    std::vector<float> axisM;   // маса
    std::vector<float> axisD;   // опір
    std::vector<float> axisL;   // підйомна сила

    struct Result {
        float t{};
        float hDist{};
    };

    std::vector<Result> data;   // плоский масив |Z0|*|V0|*|M|*|D|*|L|

    [[nodiscard]] auto index(int iz, int iv, int im, int id, int il) const -> std::size_t;
    [[nodiscard]] auto at(int iz, int iv, int im, int id, int il) const -> const Result&;
    [[nodiscard]] auto load(const std::string& path) -> bool;
    [[nodiscard]] auto lookup(float Z0, float V0, float m, float d, float l) const -> Result;
};
