// Implements no-fly zone data structures and checks.

#include "NoFlyZone.h"

#include <cmath>
#include <iostream>

namespace {
constexpr double kEarthRadiusMeters = 6371000.0;
constexpr double kPi = 3.14159265358979323846;

double toRadians(double degrees) {
    return degrees * kPi / 180.0;
}

double haversineDistanceMeters(double lat1, double lon1, double lat2, double lon2) {
    const double lat1Rad = toRadians(lat1);
    const double lat2Rad = toRadians(lat2);
    const double deltaLat = toRadians(lat2 - lat1);
    const double deltaLon = toRadians(lon2 - lon1);

    const double sinHalfLat = std::sin(deltaLat / 2.0);
    const double sinHalfLon = std::sin(deltaLon / 2.0);
    const double a = sinHalfLat * sinHalfLat +
                     std::cos(lat1Rad) * std::cos(lat2Rad) * sinHalfLon * sinHalfLon;
    const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
    return kEarthRadiusMeters * c;
}
}  // namespace

NoFlyZoneManager::NoFlyZoneManager() {
    zones.push_back({
        "Benazir Bhutto International Airport",
        33.6167,
        73.0997,
        5000.0,
        "Active commercial airport"
    });

    zones.push_back({
        "Rawalpindi Cantonment",
        33.5987,
        73.0551,
        2000.0,
        "Military restricted zone"
    });

    zones.push_back({
        "Islamabad Secure Zone",
        33.7294,
        73.0931,
        1500.0,
        "Government high-security area"
    });
}

bool NoFlyZoneManager::isInNoFlyZone(double lat, double lon) const {
    return getViolatedZone(lat, lon) != nullptr;
}

const NoFlyZone* NoFlyZoneManager::getViolatedZone(double lat, double lon) const {
    for (const NoFlyZone& zone : zones) {
        const double distance = haversineDistanceMeters(lat, lon, zone.centerLat, zone.centerLon);
        if (distance <= zone.radiusMeters) {
            return &zone;
        }
    }

    return nullptr;
}

const std::vector<NoFlyZone>& NoFlyZoneManager::getAllZones() const {
    return zones;
}

void NoFlyZoneManager::addZone(const NoFlyZone& zone) {
    zones.push_back(zone);
}

void NoFlyZoneManager::printZones() const {
    for (const NoFlyZone& zone : zones) {
        std::cout << zone.name
                  << " | Center: (" << zone.centerLat << ", " << zone.centerLon << ")"
                  << " | Radius: " << zone.radiusMeters << " meters"
                  << std::endl;
    }
}