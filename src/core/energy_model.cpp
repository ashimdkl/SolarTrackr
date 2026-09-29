#include "energy_model.h"
#include <math.h>

namespace {
const double kRad = 3.14159265358979323846 / 180.0;
}

double clearSkyFraction(double elevationDeg) {
  if (elevationDeg <= 0.5) return 0;
  double am = 1.0 / sin(elevationDeg * kRad);
  if (am > 38.0) am = 38.0;
  return 1.353 * pow(0.7, pow(am, 0.678));
}

EnergyDecision evaluateMove(const EnergyConfig& cfg, const Vec3& sun, double elevationDeg,
                            double curTiltDeg, double curAzDeg,
                            double tgtTiltDeg, double tgtAzDeg,
                            double moveSeconds, double movePowerW) {
  EnergyDecision d;
  double before = incidenceCos(sun, curTiltDeg, curAzDeg);
  double after = incidenceCos(sun, tgtTiltDeg, tgtAzDeg);
  d.gainWh = cfg.panelRatedW * clearSkyFraction(elevationDeg) * (after - before) * cfg.horizonS / 3600.0;
  d.costWh = movePowerW * moveSeconds / 3600.0;
  d.worthIt = d.gainWh > d.costWh * cfg.margin;
  return d;
}
