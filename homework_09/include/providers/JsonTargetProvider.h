#pragma once

#include "interfaces/ITargetProvider.h"
#include "Types.h"
#include <string>
#include <vector>

class JsonTargetProvider : public ITargetProvider {
    private:
        std::vector<std::vector<Coord>> vec_;

        std::vector<std::vector<Coord>> loadTargetsCoordinatesVector(const std::string& filename);

    public:
        JsonTargetProvider(const std::string& filename);

        int    getTargetCount()   override;
        Target getTarget(int idx) override;
};
