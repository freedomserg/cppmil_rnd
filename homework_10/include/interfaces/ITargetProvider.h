#pragma once

struct Target;

class ITargetProvider {
public:
    [[nodiscard]] virtual auto getTargetCount() const -> int = 0;
    [[nodiscard]] virtual auto getTarget(int idx) const -> Target = 0;
    virtual ~ITargetProvider() = default;
};
