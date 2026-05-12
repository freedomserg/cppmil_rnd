#include "ballistics.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>

int main () {

    // open the input file
    std::ifstream inputFile("input.txt");
    if (!inputFile) {
        std::cerr << "Error opening file" << std::endl;
        return 1;
    }

    // variables to hold input parameters
    float xd {}, yd {}, zd {}, targetX {}, targetY {}, attackSpeed {}, accelerationPath {}, ammoMassKg {}, ammoDrag {}, ammoLift {};
    char ammoName[12]; // given ammo names are 11 chars max + 1 for null terminator
    bool withManeuver = false;

    // read input parameters from the file. 
    // The std::setw is used to prevent buffer overflow when reading ammoName if it comes with more characters than expected.
    if (inputFile >> xd >> yd >> zd >> targetX >> targetY >> attackSpeed >> accelerationPath >> std::setw(sizeof(ammoName)) >> ammoName) {
        std::cout << "Input drone coordinates: " << xd << ", " << yd << ", " << zd << std::endl;
        std::cout << "Input target coordinates: " << targetX << ", " << targetY << std::endl;
        std::cout << "Input attack speed: " << attackSpeed << std::endl;
        std::cout << "Input acceleration path: " << accelerationPath << std::endl;
        std::cout << "Input ammo name: " << ammoName << std::endl;
    } else {
        std::cerr << "Error reading data from file" << std::endl;
        if (inputFile.is_open()) {
           inputFile.close();     
        }
        return 1;
    }

    // validate input parameters
    if (attackSpeed <= 0) {
        std::cerr << "Error: attack speed must be greater than zero, attackSpeed=" << attackSpeed << std::endl;
        return 1;
    }
    if (accelerationPath <= 0) {
        std::cerr << "Error: acceleration path must be greater than zero, accelerationPath=" << accelerationPath << std::endl;
        return 1;
    }

    // determine ammo properties based on the ammoName
    if (std::strcmp(ammoName, "VOG-17") == 0) {
        ammoMassKg = 0.35f;
        ammoDrag = 0.07f;
        ammoLift = 0.0f;
    } else if (std::strcmp(ammoName, "M67") == 0) {
        ammoMassKg = 0.6f;
        ammoDrag = 0.10f;
        ammoLift = 0.0f;
    } else if (std::strcmp(ammoName, "RKG-3") == 0) {
        ammoMassKg = 1.2f;
        ammoDrag = 0.10f;
        ammoLift = 0.0f;
    } else if (std::strcmp(ammoName, "GLIDING-VOG") == 0) {
        ammoMassKg = 0.45f;
        ammoDrag = 0.10f;
        ammoLift = 1.0f;
    } else if (std::strcmp(ammoName, "GLIDING-RKG") == 0) {
        ammoMassKg = 1.4f;
        ammoDrag = 0.10f;
        ammoLift = 1.0f;
    } else {
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

    // calcuate the distance from the drone to the target
    // D = √( (targetX − xd)² + (targetY − yd)² )
    float distanceToTarget = std::sqrt((targetX - xd) * (targetX - xd) + (targetY - yd) * (targetY - yd));
    std::cout << "Calculated distance to target: " << distanceToTarget << " meters" << std::endl;
    if (distanceToTarget < ACCEPTABLE_DEVIATION) {
        std::cerr << "Error: calculated distance to target cannot be zero or too close to zero. calculatedDistance=" << distanceToTarget << std::endl;
        return 1;
    }

    // determine if the drone needs to do a maneuver to get enough distance to accelerate and drop the ammo
    if (ammoHorizontalDistance + accelerationPath > distanceToTarget) {   
        withManeuver = true; 
        // xd' = targetX − (targetX − xd) · (h + accelerationPath) / D
        xd = targetX - (targetX - xd) * (ammoHorizontalDistance + accelerationPath) / distanceToTarget;

        // yd' = targetY − (targetY − yd) · (h + accelerationPath) / D
        yd = targetY - (targetY - yd) * (ammoHorizontalDistance + accelerationPath) / distanceToTarget;

        // Recalculate distance to target from new drone coordinates
        // D = √( (targetX − xd)² + (targetY − yd)² )
        distanceToTarget = std::sqrt((targetX - xd) * (targetX - xd) + (targetY - yd) * (targetY - yd));

        std::cout << "Drone needs to maneuver to new coordinates before dropping ammo: xd = " << xd << ", yd = " << yd << std::endl;
        std::cout << "Recalculated distance to target after maneuver: " << distanceToTarget << " meters" << std::endl;
    }

    // calculate the drop point coordinates
    // ratio = (D − h) / D
    // fireX = xd + (targetX − xd) · ratio
    // fireY = yd + (targetY − yd) · ratio
    float ratio = (distanceToTarget - ammoHorizontalDistance) / distanceToTarget;
    std::cout << "Calculated ratio for drop point: " << ratio << std::endl;
    float fireX = xd + (targetX - xd) * ratio;
    float fireY = yd + (targetY - yd) * ratio;

    // log drop point coordinates to the output file
    std::ofstream outputFile("output.txt");
    if (!outputFile) {
        std::cerr << "Error opening output file" << std::endl;
        return 1;
    }

    // if the drone needs to maneuver, log the new drone coordinates along with the drop point coordinates, otherwise log only the drop point coordinates
    if (withManeuver) outputFile << xd << " " << yd << " " << fireX << " " << fireY << std::endl; 
    else outputFile << fireX << " " << fireY << std::endl;

    // close input and output files
    inputFile.close();
    outputFile.close();

    return 0;
}