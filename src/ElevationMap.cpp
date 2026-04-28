// Implements elevation map data access and lookup helpers.

#include "ElevationMap.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace {
double clampValue(double value, double minValue, double maxValue) {
    return std::max(minValue, std::min(value, maxValue));
}
}  // namespace

double ElevationMap::getElevation(double lat, double lon) {
    const double clampedLat = clampValue(lat, MIN_LAT, MAX_LAT);
    const double clampedLon = clampValue(lon, MIN_LON, MAX_LON);

    double elevation = 0.0;
    if (clampedLat > 33.72) {
        elevation = 500.0 + (clampedLat - 33.72) * 8000.0 + std::sin(clampedLon * 50.0) * 50.0;
    } else {
        elevation = 490.0 + std::sin(clampedLat * 30.0) * 20.0 + std::sin(clampedLon * 40.0) * 15.0;
    }

    return clampValue(elevation, 400.0, 1100.0);
}

double ElevationMap::getElevationCost(double lat1, double lon1, double lat2, double lon2) {
    const double elevation1 = getElevation(lat1, lon1);
    const double elevation2 = getElevation(lat2, lon2);
    const double diff = elevation2 - elevation1;

    if (diff > 0.0) {
        return diff * 0.01;
    }
    if (diff < 0.0) {
        return std::abs(diff) * 0.003;
    }
    return 0.0;
}

bool ElevationMap::loadTile(const std::string& filepath) {
    (void)filepath;
    std::cout << "Simulated elevation active — no .hgt file needed" << std::endl;
    return true;
}

void ElevationMap::printInfo() {
    std::cout << "Elevation Model: Simulated for Rawalpindi/Islamabad" << std::endl;
    std::cout << "Margalla Hills peak: ~1100m, City center: ~500m" << std::endl;
}