#pragma once

// Declares elevation map data access and lookup helpers.

#include <string>

class ElevationMap {
public:
    static constexpr double MIN_LAT = 33.55;
    static constexpr double MAX_LAT = 33.80;
    static constexpr double MIN_LON = 72.95;
    static constexpr double MAX_LON = 73.20;

    double getElevation(double lat, double lon);
    double getElevationCost(double lat1, double lon1, double lat2, double lon2);
    bool loadTile(const std::string& filepath);
    void printInfo();
};