#include "uart/UartTargetStore.h"

namespace {
constexpr float kVelAlpha = 0.3f;   // EMA-згладжування швидкості цілі
constexpr float kMinDt    = 1e-3f;
}

void UartTargetStore::resize(int nTargets) {
    slots_.assign(nTargets < 0 ? 0 : static_cast<size_t>(nTargets), Slot{});
}

void UartTargetStore::update(uint8_t id, Coord pos, uint32_t clockMs) {
    if (id >= slots_.size()) return;
    Slot& s = slots_[id];
    if (s.seen) {
        const float dt = static_cast<float>(clockMs - s.lastMs) / 1000.0f;
        if (dt > kMinDt) {
            const Coord v = (pos - s.lastPos) / dt;
            s.target.velocity = s.hasVel
                ? s.target.velocity * (1.0f - kVelAlpha) + v * kVelAlpha
                : v;
            s.hasVel = true;
        }
    } else {
        s.seen = true;
    }
    s.target.pos = pos;
    s.lastPos    = pos;
    s.lastMs     = clockMs;
}

auto UartTargetStore::allSeen() const -> bool {
    if (slots_.empty()) return false;
    for (const auto& s : slots_)
        if (!s.seen) return false;
    return true;
}

auto UartTargetStore::getTargetCount() const -> int {
    return static_cast<int>(slots_.size());
}

auto UartTargetStore::getTarget(int idx) const -> Target {
    if (idx < 0 || idx >= static_cast<int>(slots_.size())) return Target{};
    return slots_[static_cast<size_t>(idx)].target;
}
