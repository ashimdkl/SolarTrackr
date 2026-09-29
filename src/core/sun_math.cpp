#include "sun_math.h"
#include <math.h>

// changes from the original desktop/react port
//  - no <Arduino.h>, and no PI macro (clashed with arduino's PI on esp32)
//  - azimuth wrapped into [0, 360)
//  - julian -> HH:MM rounds to the nearest minute instead of truncating
//  - getTimes follows suncalc exactly (anomaly/longitude evaluated at the transit day)
//  - getPanelTarget removed. joint targets now come from kinematics.cpp (tilt-tilt geometry)

namespace {

const double kPi = 3.14159265358979323846;
const double kRad = kPi / 180.0;
const double kJ1970 = 2440588.0;
const double kJ2000 = 2451545.0;
const double kJ0 = 0.0009;
const double kObliquity = kRad * 23.4397;
const double kSunriseAngle = -0.833;  // refraction + solar disc radius

double toJulianDate(int year, int month, int day, int hour, int minute, int second) {
  int a = (14 - month) / 12;
  int y = year + 4800 - a;
  int m = month + 12 * a - 3;
  double jdn = day + (153 * m + 2) / 5 + 365.0 * y + y / 4 - y / 100 + y / 400 - 32045.0;
  double dayFraction = (hour - 12.0) / 24.0 + minute / 1440.0 + second / 86400.0;
  return jdn + dayFraction;
}

double toDays(int year, int month, int day, int hour, int minute, int second) {
  return toJulianDate(year, month, day, hour, minute, second) - kJ2000;
}

double solarMeanAnomaly(double d) { return kRad * (357.5291 + 0.98560028 * d); }

double eclipticLongitude(double M) {
  double C = kRad * (1.9148 * sin(M) + 0.02 * sin(2 * M) + 0.0003 * sin(3 * M));
  return M + C + kRad * 102.9372 + kPi;
}

double declination(double L) { return asin(sin(L) * sin(kObliquity)); }
double rightAscension(double L) { return atan2(sin(L) * cos(kObliquity), cos(L)); }
double siderealTime(double d, double lw) { return kRad * (280.16 + 360.9856235 * d) - lw; }

double azCalc(double H, double phi, double dec) {
  return atan2(sin(H), cos(H) * sin(phi) - tan(dec) * cos(phi));
}
double altCalc(double H, double phi, double dec) {
  return asin(sin(phi) * sin(dec) + cos(phi) * cos(dec) * cos(H));
}

double julianCycle(double d, double lw) { return round(d - kJ0 - lw / (2 * kPi)); }
double approxTransit(double Ht, double lw, double n) { return kJ0 + (Ht + lw) / (2 * kPi) + n; }
double solarTransitJ(double ds, double M, double L) {
  return kJ2000 + ds + 0.0053 * sin(M) - 0.0069 * sin(2 * L);
}

void julianToHHMM(double J, int& h, int& m) {
  double unixSec = (J + 0.5 - kJ1970) * 86400.0;
  double secOfDay = fmod(unixSec, 86400.0);
  if (secOfDay < 0) secOfDay += 86400.0;
  long minutes = (long)floor(secOfDay / 60.0 + 0.5);  // round, not truncate
  if (minutes >= 1440) minutes -= 1440;
  h = (int)(minutes / 60);
  m = (int)(minutes % 60);
}

}  // namespace

SolarPosition getPosition(int year, int month, int day, int hour, int minute, int second,
                          double lat, double lng) {
  double lw = kRad * -lng;
  double phi = kRad * lat;
  double d = toDays(year, month, day, hour, minute, second);

  double M = solarMeanAnomaly(d);
  double L = eclipticLongitude(M);
  double dec = declination(L);
  double ra = rightAscension(L);
  double H = siderealTime(d, lw) - ra;

  SolarPosition pos;
  double az = azCalc(H, phi, dec) / kRad + 180.0;  // suncalc measures from south
  az = fmod(az, 360.0);
  if (az < 0) az += 360.0;
  pos.azimuth = az;
  pos.elevation = altCalc(H, phi, dec) / kRad;
  return pos;
}

SolarTimes getTimes(int year, int month, int day, double lat, double lng) {
  double lw = kRad * -lng;
  double phi = kRad * lat;
  double d = toDays(year, month, day, 12, 0, 0);

  double n = julianCycle(d, lw);
  double ds = approxTransit(0, lw, n);
  double M = solarMeanAnomaly(ds);
  double L = eclipticLongitude(M);
  double dec = declination(L);
  double Jnoon = solarTransitJ(ds, M, L);

  SolarTimes times;
  times.valid = false;
  times.sunriseHour = times.sunriseMin = 0;
  times.sunsetHour = times.sunsetMin = 0;
  julianToHHMM(Jnoon, times.noonHour, times.noonMin);

  double cosW = (sin(kSunriseAngle * kRad) - sin(phi) * sin(dec)) / (cos(phi) * cos(dec));
  if (cosW < -1.0 || cosW > 1.0) return times;  // polar day / night

  double w = acos(cosW);
  double a = approxTransit(w, lw, n);
  double Jset = solarTransitJ(a, M, L);
  double Jrise = Jnoon - (Jset - Jnoon);

  julianToHHMM(Jrise, times.sunriseHour, times.sunriseMin);
  julianToHHMM(Jset, times.sunsetHour, times.sunsetMin);
  times.valid = true;
  return times;
}
