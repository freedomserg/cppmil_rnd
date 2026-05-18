#include "ballistics.hpp"

#include <iostream>
#include <fstream>

int main (int argc, char** argv) {

    if (argc != 2) {
        std::cerr << "usage: ballistics_cli <input_path>\n";
        return 1;
    }

    float xd {}, yd {}, zd {}, targetX {}, targetY {}, attackSpeed {}, accelerationPath {}, ammoMassKg {}, ammoDrag {}, ammoLift {};
    char ammoName[MAX_AMMO_NAME_LENGTH];
    bool withManeuver = false;

    readInputParameters(argv[1], MAX_AMMO_NAME_LENGTH, xd, yd, zd, targetX, targetY, attackSpeed, accelerationPath, ammoName);

    bool validAttackSpeed, validAccelerationPath, validAmmoName;
    validateInputParameters(attackSpeed, accelerationPath, ammoName, ammoMassKg, ammoDrag, ammoLift, validAttackSpeed, validAccelerationPath, validAmmoName);

    if (!validAttackSpeed) {
        std::cerr << "Error: attack speed must be greater than zero, attackSpeed=" << attackSpeed << std::endl;
        return 1;
    }
    if (!validAccelerationPath) {
        std::cerr << "Error: acceleration path must be greater than zero, accelerationPath=" << accelerationPath << std::endl;
        return 1;
    }
    if (!validAmmoName) {
        std::cerr << "Error: unknown ammo type: " << ammoName << std::endl;
        return 1;
    }

    // calculate the ammo flight time
    float ammoFlightTime = calculateAmmoFlightTime(ammoMassKg, ammoDrag, ammoLift, attackSpeed, zd);
    std::cout << "Calculated ammo flight time: " << ammoFlightTime << " seconds" << std::endl;
    if (ammoFlightTime < 0) {
        std::cerr << "Failed to calculate ammo flight time due to invalid parameters." << std::endl;
        return 1;
    }

    // calculate the horizontal flight distance of the ammo
    float ammoHorizontalDistance = calculateAmmoHorizontalDistance(ammoMassKg, ammoDrag, ammoLift, attackSpeed, ammoFlightTime);
    std::cout << "Calculated ammo horizontal distance: " << ammoHorizontalDistance << " meters" << std::endl;

    float distanceToTarget{}, fireX{}, fireY{};
    if (!prepareDropApproach(xd, yd, targetX, targetY, ammoHorizontalDistance, accelerationPath, distanceToTarget, withManeuver)) {
        return 1;
    }
    calculateDropPointCoordinates(distanceToTarget, ammoHorizontalDistance, xd, yd, targetX, targetY, fireX, fireY);
    std::cout << "Calculated drop point coordinates: fireX = " << fireX << ", fireY = " << fireY << std::endl;

    // log drop point coordinates to the output file
    std::ofstream outputFile("output.txt");
    if (!outputFile) {
        std::cerr << "Error opening output file" << std::endl;
        return 1;
    }

    // if the drone needs to maneuver, log the new drone coordinates along with the drop point coordinates, otherwise log only the drop point coordinates
    if (withManeuver) outputFile << xd << " " << yd << " " << fireX << " " << fireY << std::endl; 
    else outputFile << fireX << " " << fireY << std::endl;

    // close the output files
    outputFile.close();

    return 0;
}