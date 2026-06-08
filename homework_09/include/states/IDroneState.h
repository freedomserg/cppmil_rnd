#pragma once

#include <string>
#include <memory>    

struct DroneContext;

class IDroneState {
public:
    virtual ~IDroneState() = default;
 
    // Виконати логіку стану, повернути наступний стан.
    // Якщо стан не змінився — повернути nullptr
    // (головний цикл залишить поточний).
    virtual std::unique_ptr<IDroneState>
        execute(DroneContext& ctx) = 0;
 
    virtual const std::string& name() const = 0;
    virtual int stateId() const = 0;
};

