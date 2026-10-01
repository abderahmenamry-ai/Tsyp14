// frame_translation.h
// Runs on the outside network node. Converts a Writer robot's local
// (x_coord_cm, y_coord_cm) into real-world GPS coordinates.
//
// Assumes a flat-earth approximation, which is accurate for the small
// scale (tens to a few hundred meters) of a tunnel/mine network.

#ifndef FRAME_TRANSLATION_H
#define FRAME_TRANSLATION_H

#include <math.h>
#include "beacon_protocol.h"

struct GPSCoord {
  double lat;
  double lon;
};

// Meters per degree of latitude is ~constant; meters per degree of
// longitude shrinks with cos(latitude).
static const double METERS_PER_DEG_LAT = 111320.0;

// One-time calibration values, set at deployment (e.g. read from a
// phone GPS + compass at the tunnel entrance before the mission starts).
struct CalibrationData {
  double entryLat;          // GPS latitude at the Writer's local origin
  double entryLon;          // GPS longitude at the Writer's local origin
  double headingOffsetDeg;  // compass bearing of the Writer's local +X axis (forward), clockwise from North
};

// Converts a beacon's local coordinates into a real-world GPS coordinate.
inline GPSCoord translateToGPS(const BeaconMessage &msg, const CalibrationData &cal) {
  double x_m = msg.x_coord_cm / 100.0;
  double y_m = msg.y_coord_cm / 100.0;
  double theta = cal.headingOffsetDeg * M_PI / 180.0;

  // Rotate local (forward=x, right=y) into world (North, East) offsets
  double northOffset_m = x_m * cos(theta) - y_m * sin(theta);
  double eastOffset_m  = x_m * sin(theta) + y_m * cos(theta);

  double deltaLat = northOffset_m / METERS_PER_DEG_LAT;
  double deltaLon = eastOffset_m / (METERS_PER_DEG_LAT * cos(cal.entryLat * M_PI / 180.0));

  GPSCoord result;
  result.lat = cal.entryLat + deltaLat;
  result.lon = cal.entryLon + deltaLon;
  return result;
}

#endif // FRAME_TRANSLATION_H
