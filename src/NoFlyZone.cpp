// Implements no-fly zone data structures and checks.

#include "NoFlyZone.h"

#include <cmath>
#include <iostream>
#include "Utils.h"

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
        const double distance = DroneUtils::haversineDistanceMeters(lat, lon, zone.centerLat, zone.centerLon);
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