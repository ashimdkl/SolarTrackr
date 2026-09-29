#include "time_utils.h"

// civil date <-> day count (howard hinnant's algorithms, proleptic gregorian)
static int64_t daysFromCivil(int y, int m, int d) {
  y -= (m <= 2) ? 1 : 0;
  const int64_t era = (y >= 0 ? y : y - 399) / 400;
  const int64_t yoe = y - era * 400;
  const int64_t mp = (m > 2) ? (m - 3) : (m + 9);
  const int64_t doy = (153 * mp + 2) / 5 + d - 1;
  const int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + doe - 719468;
}

static void civilFromDays(int64_t z, int& y, int& m, int& d) {
  z += 719468;
  const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  const int64_t doe = z - era * 146097;
  const int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const int64_t mp = (5 * doy + 2) / 153;
  d = (int)(doy - (153 * mp + 2) / 5 + 1);
  m = (int)(mp < 10 ? mp + 3 : mp - 9);
  y = (int)(yoe + era * 400 + (m <= 2 ? 1 : 0));
}

int64_t utcToEpoch(const UtcTime& t) {
  return daysFromCivil(t.year, t.month, t.day) * 86400LL +
         t.hour * 3600LL + t.minute * 60LL + t.second;
}

UtcTime epochToUtc(int64_t epoch) {
  int64_t days = epoch / 86400;
  int64_t rem = epoch % 86400;
  if (rem < 0) { rem += 86400; days -= 1; }
  UtcTime t;
  civilFromDays(days, t.year, t.month, t.day);
  t.hour = (int)(rem / 3600);
  t.minute = (int)((rem % 3600) / 60);
  t.second = (int)(rem % 60);
  return t;
}
