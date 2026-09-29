#pragma once
// sun position from date, time and location. ported from suncalc.
// all inputs are utc (matches gps time). outputs in degrees.
// no arduino include here on purpose, so it builds on both esp32 and native.

struct SolarPosition {
  double azimuth;    // clockwise from north, [0, 360)
  double elevation;  // above horizon
};

struct SolarTimes {
  int sunriseHour, sunriseMin;
  int sunsetHour,  sunsetMin;
  int noonHour,    noonMin;
  bool valid;  // false if no sunrise/sunset that day (polar edge case), noon still filled
};

SolarPosition getPosition(int year, int month, int day,
                          int hour, int minute, int second,
                          double lat, double lng);

SolarTimes getTimes(int year, int month, int day, double lat, double lng);
