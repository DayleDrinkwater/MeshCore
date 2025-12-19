#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Mesh.h>

// Forward declarations
class CayenneLPP;
struct NodePrefs;

// Centralised power-management for boards using BQ25628E (and optionally STUSB4500).
// This keeps charger / power IC logic out of EnvironmentSensorManager.
class PowerManager {
public:
  // Configure I2C addressable power devices. Call once at boot, after Wire is up.
  static bool begin(TwoWire &wire);

  // Periodic housekeeping (e.g. watchdog kicks). Call from main loop or sensor loop.
  static void loop();

  // Adds telemetry to CayenneLPP using the agreed channel mapping:
  // 1: battery related, 2: input related
  static void addTelemetry(CayenneLPP &lpp);

  // Helpers so board code can inspect init state
  static bool isBQInitialized();
  static bool isSTUSBInitialized();

  // Get battery voltage in millivolts from BQ25628E
  // Returns 0 if BQ is not initialized or read fails
  static uint16_t getBatteryVoltageMilliVolts();

  // Configure BQ25628E with runtime settings
  // Call this to apply new settings from NodePrefs
  static bool configureBQ(uint16_t chg_vreg_mv, uint16_t chg_current_ma,
                          uint16_t prechg_current_ma, uint16_t term_current_ma,
                          uint16_t rechg_threshold_mv, uint16_t input_voltage_mv,
                          uint16_t input_current_ma, uint16_t min_sys_voltage_mv,
                          uint8_t vbat_uvlo);

  // Get charge status string for CLI (returns formatted status)
  // Returns empty string if BQ is not initialized
  static void getChargeStatus(char* status_buffer, size_t buffer_size);

  // Get comprehensive diagnostics including ADC values and register states
  // Returns detailed information for troubleshooting
  static void getDiagnostics(char* diag_buffer, size_t buffer_size);

  // Load and apply BQ settings from NodePrefs
  // Call this after loading prefs at startup to configure BQ with stored settings
  // This also initializes the watchdog with expected values
  static bool loadAndApplyBQSettings(const NodePrefs* prefs);

  // Verify BQ25628E settings match expected values and reconfigure if needed
  // Called automatically by loop() every 10 seconds
  // Returns true if settings are correct or were successfully corrected
  static bool verifyAndCorrectBQSettings();

  // System power reset using BQ25628E BATFET control
  // This cuts power to the MCU via BATFET, then re-enables after ~100ms
  // Use this for a "hard" reset that power-cycles the entire system
  // Falls back to standard MCU reset if BQ is not available
  // This function does not return!
  static void systemPowerReset();
};
