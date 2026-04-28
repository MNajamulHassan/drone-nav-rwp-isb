#pragma once

// Declares no-fly zone data structures and checks.

#include <string>
#include <vector>

struct NoFlyZone {
    std::string name;
    double centerLat;
    double centerLon;
    double radiusMeters;
    std::string reason;
};

class NoFlyZoneManager {
public:
    NoFlyZoneManager();

    bool isInNoFlyZone(double lat, double lon) const;
    const NoFlyZone* getViolatedZone(double lat, double lon) const;
    const std::vector<NoFlyZone>& getAllZones() const;
    void addZone(const NoFlyZone& zone);
    void printZones() const;

private:
    std::vector<NoFlyZone> zones;
};