#pragma once
#include <stdint.h>
#include "types.h"

// seconds since 1970-01-01 00:00:00 utc
int64_t utcToEpoch(const UtcTime& t);
UtcTime epochToUtc(int64_t epoch);
