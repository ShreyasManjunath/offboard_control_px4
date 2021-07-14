#include <cmath>
#include <Eigen/Dense>

#define _USE_MATH_DEFINES

// 	assume WGS84
double wgs84_a = 6378137.0;
double wgs84_f = 1.0 / 298.257223563;
double wgs84_e2 = 2 * wgs84_f - std::pow(wgs84_f,2);

Eigen::Vector3d lla2ecef(Eigen::Vector3d& latLongAlt){
    double lat, lon, alt;
    lat =  (M_PI/180.0) * latLongAlt[0];
    lon = (M_PI/180.0) * latLongAlt[1];
    alt = latLongAlt[2];
    double chi = std::sqrt(1 - wgs84_e2 * std::pow(std::sin(lat),2));
    double q = (wgs84_a / chi + alt) * std::cos(lat);
    return Eigen::Vector3d(q * std::cos(lon), q * std::sin(lon), ((wgs84_a * (1 - wgs84_e2) / chi) + alt) * std::sin(lat));
 }

Eigen::Vector3d ecef2lla(Eigen::Vector3d& ecef){
    double x = ecef[0];
    double y = ecef[1];
    double z = ecef[2];
    double lon = std::atan2(y,x);
    double s = std::sqrt(x*x + y*y);
    int step = 0;
    double lat; double latPrev = 0;
    bool converged = false;
    while(!converged){
        double beta;
        if(step == 0)
            beta = std::atan2(z, (1 - wgs84_f) * s);
        else
        {
            beta = std::atan2((1 - wgs84_f) * std::sin(lat), std::cos(lat));
        }
        lat = std::atan2(z + (wgs84_e2 * (1 - wgs84_f) / (1 - wgs84_e2)) * wgs84_a * std::pow(std::sin(beta), 3), 
                                        s - wgs84_e2 * wgs84_a * std::pow(std::cos(beta), 3));
        if(std::abs(lat - latPrev) < 1e-4)
            converged = true;
        latPrev = lat;
        step++;
    } 
    double N = wgs84_a / std::sqrt(1 - wgs84_e2 * std::pow(std::sin(lat), 2));
    double alt = s * std::cos(lat) + (z + wgs84_e2 * N * std::sin(lat)) * std::sin(lat) - N;
    return Eigen::Vector3d(lat * (180.0/M_PI), lon * (180.0/M_PI), alt);
}

Eigen::Vector3d ecef2enu(Eigen::Vector3d& latlongalt_home, Eigen::Vector3d& ecef){
    double x = ecef[0];
    double y = ecef[1];
    double z = ecef[2];
    Eigen::Vector3d latlongalt_home_ecef = lla2ecef(latlongalt_home);
    double ox = latlongalt_home_ecef[0];
    double oy = latlongalt_home_ecef[1];
    double oz = latlongalt_home_ecef[2];
    double dx = x - ox;
    double dy = y - oy;
    double dz = z - oz;
    double lat = (M_PI/180.0) * latlongalt_home[0];
    double lon = (M_PI/180.0) * latlongalt_home[1];
    double east = -std::sin(lon) * dx + std::cos(lon) * dy;
    double north = -std::sin(lat) * std::cos(lon) * dx - std::sin(lat) * std::sin(lon) * dy + std::cos(lat) * dz;
    double up = std::cos(lat) * std::cos(lon) * dx + std::cos(lat) * std::sin(lon) * dy + std::sin(lat) * dz;

    return Eigen::Vector3d(east, north, up); 
}

Eigen::Vector3d enu2ecef(Eigen::Vector3d& latlongalt_home, Eigen::Vector3d& enu){
    double east = enu[0];
    double north = enu[1];
    double up = enu[2];
    double lat = (M_PI/180.0) * latlongalt_home[0];
    double lon = (M_PI/180.0) * latlongalt_home[1];
    Eigen::Vector3d latlongalt_home_ecef = lla2ecef(latlongalt_home);
    double ox = latlongalt_home_ecef[0];
    double oy = latlongalt_home_ecef[1];
    double oz = latlongalt_home_ecef[2];
    double X_ecef = ox - std::sin(lon) * east - std::cos(lon) * std::sin(lat) * north + std::cos(lon) * std::cos(lat) * up;
    double Y_ecef = oy + std::cos(lon) * east - std::sin(lon) * std::sin(lat) * north + std::cos(lat) * std::sin(lon) * up;
    double Z_ecef = oz + std::cos(lat) * north + std::sin(lat) * up;

    return Eigen::Vector3d(X_ecef, Y_ecef, Z_ecef);

}

Eigen::Vector3d lla2enu(Eigen::Vector3d& latlongalt_home, Eigen::Vector3d& latLonAlt){
    Eigen::Vector3d ecef = lla2ecef(latLonAlt);
    return ecef2enu(latlongalt_home, ecef);
}

Eigen::Vector3d enu2lla(Eigen::Vector3d& latlongalt_home, Eigen::Vector3d& enu){
    Eigen::Vector3d ecef = enu2ecef(latlongalt_home, enu);
    return ecef2lla(ecef);
}