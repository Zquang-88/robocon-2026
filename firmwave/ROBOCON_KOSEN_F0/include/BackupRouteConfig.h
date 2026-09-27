#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

// Persistent configuration for the optional POINT_C recovery route.  The
// normal A/B route remains the compiled/default route and is not modified by
// these values.  Teensy EEPROM is used so COM14 may be unplugged after setup.
namespace BackupRoute {

constexpr uint32_t CONFIG_MAGIC = 0x31434642UL;  // "BFC1"
constexpr uint16_t CONFIG_VERSION = 1;
constexpr int EEPROM_ADDRESS = 0;

enum class RouteMode : uint8_t { MainAB = 0, BackupC = 1 };

struct MapProfile {
  float startForwardMm;
  float lateralDistanceMm;
  float lateralSpeedMmS;
  float searchSpeedMmS;
  float searchRadiusMm;
  float exitDistanceMm;
  uint8_t useRightGroup;
  uint8_t reserved[3];
};

struct Config {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint8_t routeMode;
  uint8_t reserved[3];
  MapProfile red;
  MapProfile blue;
  uint32_t crc32;
};

inline MapProfile defaultRedProfile() {
  return {300.0f, 2200.0f, 800.0f, 90.0f, 120.0f, 200.0f, 1, {0, 0, 0}};
}

inline MapProfile defaultBlueProfile() {
  return {140.0f, 2200.0f, 800.0f, 90.0f, 120.0f, 200.0f, 0, {0, 0, 0}};
}

inline Config defaults() {
  Config config{};
  config.magic = CONFIG_MAGIC;
  config.version = CONFIG_VERSION;
  config.size = sizeof(Config);
  config.routeMode = static_cast<uint8_t>(RouteMode::MainAB);
  config.red = defaultRedProfile();
  config.blue = defaultBlueProfile();
  return config;
}

inline uint32_t calculateCrc(const Config &config) {
  const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&config);
  const size_t length = offsetof(Config, crc32);
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < length; ++i) {
    crc ^= bytes[i];
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xEDB88320UL &
                          static_cast<uint32_t>(-(static_cast<int32_t>(crc & 1U))));
  }
  return ~crc;
}

inline bool profileValid(const MapProfile &profile) {
  return isfinite(profile.startForwardMm) && profile.startForwardMm >= 50.0f &&
         profile.startForwardMm <= 1000.0f &&
         isfinite(profile.lateralDistanceMm) && profile.lateralDistanceMm >= 200.0f &&
         profile.lateralDistanceMm <= 4000.0f &&
         isfinite(profile.lateralSpeedMmS) && profile.lateralSpeedMmS >= 50.0f &&
         profile.lateralSpeedMmS <= 1200.0f &&
         isfinite(profile.searchSpeedMmS) && profile.searchSpeedMmS >= 30.0f &&
         profile.searchSpeedMmS <= 300.0f &&
         isfinite(profile.searchRadiusMm) && profile.searchRadiusMm >= 30.0f &&
         profile.searchRadiusMm <= 500.0f &&
         isfinite(profile.exitDistanceMm) && profile.exitDistanceMm >= 0.0f &&
         profile.exitDistanceMm <= 1000.0f && profile.useRightGroup <= 1;
}

inline bool valid(const Config &config) {
  return config.magic == CONFIG_MAGIC &&
         config.version == CONFIG_VERSION && config.size == sizeof(Config) &&
         config.routeMode <= static_cast<uint8_t>(RouteMode::BackupC) &&
         profileValid(config.red) && profileValid(config.blue) &&
         config.crc32 == calculateCrc(config);
}

inline bool load(Config &config) {
  EEPROM.get(EEPROM_ADDRESS, config);
  if (valid(config)) return true;
  config = defaults();
  config.crc32 = calculateCrc(config);
  return false;
}

inline bool save(Config &config) {
  if (!profileValid(config.red) || !profileValid(config.blue) ||
      config.routeMode > static_cast<uint8_t>(RouteMode::BackupC))
    return false;
  config.magic = CONFIG_MAGIC;
  config.version = CONFIG_VERSION;
  config.size = sizeof(Config);
  config.crc32 = calculateCrc(config);
  EEPROM.put(EEPROM_ADDRESS, config);
  Config verify{};
  EEPROM.get(EEPROM_ADDRESS, verify);
  return valid(verify) && verify.crc32 == config.crc32;
}

inline void restoreDefaults(Config &config) {
  config = defaults();
  config.crc32 = calculateCrc(config);
}

inline bool backupEnabled(const Config &config) {
  return config.routeMode == static_cast<uint8_t>(RouteMode::BackupC);
}

}  // namespace BackupRoute
