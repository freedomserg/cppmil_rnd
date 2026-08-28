#pragma once

#include "Types.h"
#include "interfaces/ITargetProvider.h"

#include <cstdint>
#include <vector>

// Сховище цілей, наповнюване з PKT_TARGET. Швидкість — скінченна різниця між
// послідовними позиціями за годинником телеметрії (t_ms), зі згладжуванням EMA.
class UartTargetStore : public ITargetProvider {
public:
    void resize(int nTargets);
    void update(uint8_t id, Coord pos, uint32_t clockMs);
    [[nodiscard]] auto allSeen() const -> bool;

    [[nodiscard]] auto getTargetCount() const -> int override;
    [[nodiscard]] auto getTarget(int idx) const -> Target override;

private:
    struct Slot {
        Target   target{};
        Coord    lastPos{};
        uint32_t lastMs{};
        bool     seen{false};
        bool     hasVel{false};
    };
    std::vector<Slot> slots_;
};
