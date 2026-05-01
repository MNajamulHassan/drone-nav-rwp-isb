#pragma once

#include <cmath>

namespace DroneUtils {

constexpr double kEarthRadiusMeters = 6371000.0;
constexpr double kPi = 3.14159265358979323846;

inline double toRadians(double degrees) {
    return degrees * kPi / 180.0;
}

inline double haversineDistanceMeters(double lat1, double lon1, double lat2, double lon2) {
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

}  // namespace DroneUtils
