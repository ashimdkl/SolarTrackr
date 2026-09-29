#pragma once
// estimates whether a move pays for itself (hard project requirement).
//   gain = panel watts x clear-sky fraction x (cos after - cos before) x horizon
//   cost = actuator power x predicted move time
// gain is estimated from sun_math instead of measured, since panel output under
// passing clouds is noisy. cost starts from config and is refined from the ina260.
#include "kinematics.h"

struct EnergyConfig {
  double panelRatedW;
  double margin;     // gain must exceed cost * margin
  double horizonS;   // how long the gain is assumed to last (one cycle)
};

struct EnergyDecision {
  double gainWh, costWh;
  bool worthIt;
};

// direct normal irradiance on a clear day as a fraction of 1000 W/m^2 (meinel air-mass model)
double clearSkyFraction(double elevationDeg);

EnergyDecision evaluateMove(const EnergyConfig& cfg, const Vec3& sun, double elevationDeg,
                            double curTiltDeg, double curAzDeg,
                            double tgtTiltDeg, double tgtAzDeg,
                            double moveSeconds, double movePowerW);
