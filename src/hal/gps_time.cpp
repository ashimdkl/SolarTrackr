#include "gps_time.h"
#include "core/time_utils.h"

GpsTime::GpsTime()
    : port_(nullptr), synced_(false), syncEpoch_(0), syncMillis_(0), holdoverMs_(0) {}

void GpsTime::begin(HardwareSerial& port, int rxPin, int txPin, uint32_t baud, uint32_t holdoverS) {
  port_ = &port;
  holdoverMs_ = holdoverS * 1000u;
  port_->begin(baud, SERIAL_8N1, rxPin, txPin);
}

void GpsTime::service() {
  if (!port_) return;
  while (port_->available() > 0) {
    gps_.encode((char)port_->read());
    // only trust gps time once there has been a position fix, some modules report
    // garbage time from their own rtc before that
    if (gps_.time.isUpdated() && gps_.date.isValid() && gps_.time.isValid() &&
        gps_.location.isValid() && gps_.date.year() >= 2024 && gps_.date.year() < 2100) {
      UtcTime t;
      t.year = gps_.date.year();
      t.month = gps_.date.month();
      t.day = gps_.date.day();
      t.hour = gps_.time.hour();
      t.minute = gps_.time.minute();
      t.second = gps_.time.second();
      syncEpoch_ = utcToEpoch(t);
      syncMillis_ = millis() - gps_.time.age();
      synced_ = true;
    }
  }
}

bool GpsTime::utcNow(UtcTime& out) {
  if (!synced_) return false;
  uint32_t elapsed = millis() - syncMillis_;
  if (elapsed > holdoverMs_) return false;
  out = epochToUtc(syncEpoch_ + elapsed / 1000);
  return true;
}

bool GpsTime::location(double& lat, double& lng) {
  if (!gps_.location.isValid()) return false;  // tinygps keeps the last valid fix
  lat = gps_.location.lat();
  lng = gps_.location.lng();
  return true;
}
