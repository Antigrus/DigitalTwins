# ECEF / GEO / NED coordinate foundation

This module introduces WGS-84 geodetic-to-ECEF conversion and NED/ECEF basis transforms as the foundation for a simulation-wide ECEF migration.

## Coordinate definitions
- GEO input: latitude (north-positive), longitude (east-positive), ellipsoidal altitude in metres.
- ECEF: Earth-centred, Earth-fixed Cartesian axes; X through latitude 0°, longitude 0°, Y through latitude 0°, longitude 90°E, Z through the north pole.
- NED: local North, East, Down right-handed frame at the supplied geodetic latitude/longitude.

## WGS-84 conversion
With semi-major axis a=6378137 m, flattening f=1/298.257223563, eccentricity squared e²=f(2−f), prime-vertical radius N=a/sqrt(1−e²sin²φ):
X=(N+h)cosφcosλ, Y=(N+h)cosφsinλ, Z=(N(1−e²)+h)sinφ.

The matrix returned by ned_to_ecef_matrix() has N/E/D basis vectors as columns. Thus v_ecef = R_ned_to_ecef * v_ned. The inverse is its transpose.

## API
The geodetic/ECEF conversion, NED/ECEF transforms and double-precision vector/matrix helpers are in include/geodesy.h.

## Validation
make test-geodesy builds and runs tests for equator origin, north pole, GEO↔ECEF round-trip and N/E/D basis directions.

## Migration status
This commit adds the coordinate-conversion foundation. Existing trajectory and channel structures still use the legacy local Cartesian representation; they are **not yet migrated to ECEF**. The next stage must convert trajectory inputs and state to double-precision ECEF, transform NED attitude into ECEF, and use ECEF geometry in channel.c. This document records that boundary explicitly.
