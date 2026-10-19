"""
Flat-earth lat/lon to local ENU (east, north) in meters.
Origin is (lat0_deg, lon0_deg). Suitable for small areas (e.g. race course).
"""

import math

# WGS84 approximate radii (meters)
R_EARTH_EQUATORIAL = 6378137.0
R_EARTH_POLAR = 6356752.314245
# Meridional radius at a latitude: R * (1 - e^2) / (1 - e^2 sin^2(lat))^(3/2); simplified at mid-lat
def _meridional_radius_approx(lat_rad: float) -> float:
    """Approximate meridional radius at given latitude (meters)."""
    e2 = 1.0 - (R_EARTH_POLAR / R_EARTH_EQUATORIAL) ** 2
    sin_lat = math.sin(lat_rad)
    denom = (1.0 - e2 * sin_lat * sin_lat) ** 1.5
    return R_EARTH_EQUATORIAL * (1.0 - e2) / denom


def lat_lon_to_enu(
    lat_deg: float,
    lon_deg: float,
    lat0_deg: float,
    lon0_deg: float,
) -> tuple[float, float]:
    """
    Convert WGS84 (lat_deg, lon_deg) to local ENU (east, north) in meters
    relative to origin (lat0_deg, lon0_deg).
    """
    lat_rad = math.radians(lat_deg)
    lon_rad = math.radians(lon_deg)
    lat0_rad = math.radians(lat0_deg)
    lon0_rad = math.radians(lon0_deg)

    # East: longitude difference * cos(lat) * equatorial radius at this lat
    east = (lon_rad - lon0_rad) * math.cos(lat0_rad) * R_EARTH_EQUATORIAL

    # North: latitude difference * meridional radius
    r_merid = _meridional_radius_approx(lat0_rad)
    north = (lat_rad - lat0_rad) * r_merid

    return (east, north)
