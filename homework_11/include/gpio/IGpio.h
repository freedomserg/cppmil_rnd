#pragma once

// Абстракція двох ліній GPIO: START (тримається високою) і DROP (одноразовий імпульс).
class IGpio {
public:
    virtual ~IGpio() = default;
    virtual void setStart(bool on) = 0;
    virtual void pulseDrop() = 0;
};
