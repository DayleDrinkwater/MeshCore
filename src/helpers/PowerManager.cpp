/**
 * @file PowerManager.cpp
 * @brief Power management for MeshCore using BQ25628E battery charger IC
 * 
 * This module provides:
 * - BQ25628E initialization and configuration
 * - Battery/system voltage and current monitoring via ADC
 * - Charge status reporting and diagnostics
 * - Settings watchdog to detect/correct register corruption
 */

#include "PowerManager.h"

#include <Wire.h>
#include <CayenneLPP.h>
#include <helpers/CommonCLI.h>

// =============================================================================
// Sentinel value for "not configured" prefs
// =============================================================================
// When a pref value is 0, it means the user has not configured it via CLI.
// In that case, we leave the register at its chip default (set by reset).
#define BQ_PREF_NOT_SET 0

// =============================================================================
// BQ25628E Register Definitions
// =============================================================================

#define BQ25628E_I2C_ADDR       0x6A

// Charge parameter registers (16-bit, little-endian)
#define BQ28_REG_ICHG           0x02  // Charge current limit
#define BQ28_REG_VREG           0x04  // Charge voltage limit
#define BQ28_REG_IINDPM         0x06  // Input current limit
#define BQ28_REG_VINDPM         0x08  // Input voltage limit
#define BQ28_REG_VSYSMIN        0x0E  // Minimum system voltage
#define BQ28_REG_PRECHG_CTRL    0x10  // Precharge control
#define BQ28_REG_TERM_CTRL      0x12  // Termination control

// Control registers
#define BQ28_REG_CHARGER_CTRL0  0x16  // Watchdog, HIZ, charge enable
#define BQ28_REG_CHARGER_CTRL1  0x17  // Reset, thermal reg, frequency
#define BQ28_REG_CHARGER_CTRL2  0x18  // BATFET control, JEITA, timers
#define BQ28_REG_CHARGER_CTRL3  0x19  // VBAT_UVLO, peak discharge

// Status registers
#define BQ28_REG_STATUS0        0x1D  // ADC done, regulation status, timers
#define BQ28_REG_STATUS1        0x1E  // VBUS status, charge status
#define BQ28_REG_FAULT0         0x1F  // Fault flags

// ADC registers
#define BQ28_REG_ADC_CTRL       0x26  // ADC enable, mode, resolution
#define BQ28_REG_ADC_FUNC_DIS   0x27  // ADC channel disable flags
#define BQ28_REG_IBUS_ADC       0x28  // Input current ADC
#define BQ28_REG_IBAT_ADC       0x2A  // Battery current ADC
#define BQ28_REG_VBUS_ADC       0x2C  // Input voltage ADC
#define BQ28_REG_VBAT_ADC       0x30  // Battery voltage ADC
#define BQ28_REG_VSYS_ADC       0x32  // System voltage ADC
#define BQ28_REG_TS_ADC         0x34  // Thermistor ADC
#define BQ28_REG_TDIE_ADC       0x36  // Die temperature ADC
#define BQ28_REG_PART_INFO      0x38  // Part ID

// =============================================================================
// Register Encoding Constants
// =============================================================================

// ICHG: 40 mA/LSB, bits 10:5, range 40-2000 mA
#define BQ28_ICHG_STEP_MA       40
#define BQ28_ICHG_MIN_MA        40
#define BQ28_ICHG_MAX_MA        2000

// VREG: 10 mV/LSB, bits 11:3, code = mV/10, range 3500-4800 mV
#define BQ28_VREG_STEP_MV       10
#define BQ28_VREG_MIN_MV        3500
#define BQ28_VREG_MAX_MV        4800

// VINDPM: 40 mV/LSB, bits 13:5, code = mV/40, range 3800-16800 mV
#define BQ28_VINDPM_STEP_MV     40
#define BQ28_VINDPM_MIN_MV      3800
#define BQ28_VINDPM_MAX_MV      16800

// IINDPM: 20 mA/LSB, bits 11:4, code = mA/20, range 100-3200 mA
#define BQ28_IINDPM_STEP_MA     20
#define BQ28_IINDPM_MIN_MA      100
#define BQ28_IINDPM_MAX_MA      3200

// VSYSMIN: 80 mV/LSB, bits 11:6, code = mV/80, range 2560-3840 mV
#define BQ28_VSYSMIN_STEP_MV    80
#define BQ28_VSYSMIN_MIN_MV     2560
#define BQ28_VSYSMIN_MAX_MV     3840

// IPRECHG: 10 mA/LSB, 5 bits shifted by 3 at reg 0x10, range 10-310 mA
#define BQ28_IPRECHG_STEP_MA    10
#define BQ28_IPRECHG_MIN_MA     10
#define BQ28_IPRECHG_MAX_MA     310

// ITERM: 5 mA/LSB, 6 bits shifted by 2 at reg 0x12, range 5-310 mA
#define BQ28_ITERM_STEP_MA      5
#define BQ28_ITERM_MIN_MA       5
#define BQ28_ITERM_MAX_MA       310

// ADC scaling factors (x100 for fixed-point math)
#define BQ28_VADC_STEP_X100     199   // VBAT/VSYS: 1.99 mV/LSB
#define BQ28_VBUS_STEP_X100     397   // VBUS: 3.97 mV/LSB

// =============================================================================
// Thermistor Constants (for NTC temperature calculation)
// =============================================================================

static constexpr float TS_NTC_R25_OHM    = 10000.0f;  // 10k NTC @ 25°C
static constexpr float TS_FIXED_R_BOTTOM = 30000.0f;  // 30k parallel resistor
static constexpr float TS_R_TOP_OHM      = 5230.0f;   // 5.23k bias resistor
static constexpr float TS_BETA_K         = 3435.0f;   // NTC beta coefficient
static constexpr float KELVIN_25C        = 298.15f;   // 25°C in Kelvin

// =============================================================================
// Module State
// =============================================================================

static TwoWire* g_wire = nullptr;
static bool g_bq_initialized = false;
static uint32_t g_last_watchdog_check_ms = 0;
static const NodePrefs* g_node_prefs = nullptr;

// =============================================================================
// Low-Level I2C Primitives
// =============================================================================

static bool bq_write8(uint8_t reg, uint8_t val) {
  if (!g_wire) return false;
  g_wire->beginTransmission(BQ25628E_I2C_ADDR);
  g_wire->write(reg);
  g_wire->write(val);
  return g_wire->endTransmission() == 0;
}

static bool bq_read8(uint8_t reg, uint8_t& val) {
  if (!g_wire) return false;
  g_wire->beginTransmission(BQ25628E_I2C_ADDR);
  g_wire->write(reg);
  if (g_wire->endTransmission(false) != 0) return false;
  if (g_wire->requestFrom(BQ25628E_I2C_ADDR, (uint8_t)1) != 1) return false;
  val = g_wire->read();
  return true;
}

static bool bq_read16(uint8_t reg, uint16_t& val) {
  if (!g_wire) return false;
  g_wire->beginTransmission(BQ25628E_I2C_ADDR);
  g_wire->write(reg);
  if (g_wire->endTransmission(false) != 0) return false;
  if (g_wire->requestFrom(BQ25628E_I2C_ADDR, (uint8_t)2) != 2) return false;
  uint8_t low = g_wire->read();
  uint8_t high = g_wire->read();
  val = (uint16_t)low | ((uint16_t)high << 8);
  return true;
}

static bool bq_write16(uint8_t reg, uint16_t val) {
  // Write both bytes in a single I2C transaction (little-endian)
  if (!g_wire) return false;
  g_wire->beginTransmission(BQ25628E_I2C_ADDR);
  g_wire->write(reg);
  g_wire->write((uint8_t)(val & 0xFF));        // Low byte first
  g_wire->write((uint8_t)((val >> 8) & 0xFF)); // High byte second
  return g_wire->endTransmission() == 0;
}

// =============================================================================
// BQ25628E Register Setters
// =============================================================================

static bool bq_set_ichg(uint16_t ma) {
  if (ma < BQ28_ICHG_MIN_MA) ma = BQ28_ICHG_MIN_MA;
  if (ma > BQ28_ICHG_MAX_MA) ma = BQ28_ICHG_MAX_MA;
  uint8_t code = (uint8_t)(ma / BQ28_ICHG_STEP_MA);
  if (code < 1) code = 1;
  if (code > 50) code = 50;

  // ICHG is bits 10:5 of the 16-bit register at 0x02 (6 bits, shift by 5)
  uint16_t reg_val;
  if (!bq_read16(BQ28_REG_ICHG, reg_val)) return false;
  reg_val = (reg_val & 0xF81F) | ((code & 0x3F) << 5);  // bits 10:5
  return bq_write16(BQ28_REG_ICHG, reg_val);
}

static bool bq_set_vreg(uint16_t mv) {
  if (mv < BQ28_VREG_MIN_MV) mv = BQ28_VREG_MIN_MV;
  if (mv > BQ28_VREG_MAX_MV) mv = BQ28_VREG_MAX_MV;
  uint16_t code = mv / BQ28_VREG_STEP_MV;  // code = mV/10

  uint16_t reg_val;
  if (!bq_read16(BQ28_REG_VREG, reg_val)) return false;
  reg_val = (reg_val & 0xF007) | ((code & 0x01FF) << 3);  // bits 11:3
  return bq_write16(BQ28_REG_VREG, reg_val);
}

static bool bq_set_vindpm(uint16_t mv) {
  if (mv < BQ28_VINDPM_MIN_MV) mv = BQ28_VINDPM_MIN_MV;
  if (mv > BQ28_VINDPM_MAX_MV) mv = BQ28_VINDPM_MAX_MV;
  uint16_t code = mv / BQ28_VINDPM_STEP_MV;  // code = mV/40

  uint16_t reg_val;
  if (!bq_read16(BQ28_REG_VINDPM, reg_val)) return false;
  reg_val = (reg_val & 0xC01F) | ((code & 0x01FF) << 5);  // bits 13:5
  return bq_write16(BQ28_REG_VINDPM, reg_val);
}

static bool bq_set_iindpm(uint16_t ma) {
  if (ma < BQ28_IINDPM_MIN_MA) ma = BQ28_IINDPM_MIN_MA;
  if (ma > BQ28_IINDPM_MAX_MA) ma = BQ28_IINDPM_MAX_MA;
  uint8_t code = (uint8_t)(ma / BQ28_IINDPM_STEP_MA);  // code = mA/20

  uint16_t reg_val;
  if (!bq_read16(BQ28_REG_IINDPM, reg_val)) return false;
  reg_val = (reg_val & 0xF00F) | ((code & 0xFF) << 4);  // bits 11:4
  return bq_write16(BQ28_REG_IINDPM, reg_val);
}

static bool bq_set_vsysmin(uint16_t mv) {
  if (mv < BQ28_VSYSMIN_MIN_MV) mv = BQ28_VSYSMIN_MIN_MV;
  if (mv > BQ28_VSYSMIN_MAX_MV) mv = BQ28_VSYSMIN_MAX_MV;
  uint8_t code = (uint8_t)(mv / BQ28_VSYSMIN_STEP_MV);  // code = mV/80

  uint16_t reg_val;
  if (!bq_read16(BQ28_REG_VSYSMIN, reg_val)) return false;
  reg_val = (reg_val & 0xF03F) | ((code & 0x3F) << 6);  // bits 11:6
  return bq_write16(BQ28_REG_VSYSMIN, reg_val);
}

static bool bq_set_vbat_uvlo(bool low_threshold) {
  uint16_t reg_val;
  if (!bq_read16(BQ28_REG_CHARGER_CTRL3, reg_val)) return false;
  if (low_threshold) {
    reg_val |= (1 << 5);   // 1.8V threshold (LiFePO4)
  } else {
    reg_val &= ~(1 << 5);  // 2.2V threshold (Li-ion)
  }
  return bq_write16(BQ28_REG_CHARGER_CTRL3, reg_val);
}

static bool bq_set_iprechg(uint16_t ma) {
  if (ma < BQ28_IPRECHG_MIN_MA) ma = BQ28_IPRECHG_MIN_MA;
  if (ma > BQ28_IPRECHG_MAX_MA) ma = BQ28_IPRECHG_MAX_MA;
  uint8_t code = (uint8_t)(ma / BQ28_IPRECHG_STEP_MA);  // code = mA/10
  if (code < 1) code = 1;
  if (code > 31) code = 31;

  // IPRECHG is 5 bits shifted by 3 at register 0x10
  uint16_t reg_val;
  if (!bq_read16(BQ28_REG_PRECHG_CTRL, reg_val)) return false;
  reg_val = (reg_val & 0xFF07) | ((code & 0x1F) << 3);  // bits 7:3
  return bq_write16(BQ28_REG_PRECHG_CTRL, reg_val);
}

static bool bq_set_iterm(uint16_t ma) {
  if (ma < BQ28_ITERM_MIN_MA) ma = BQ28_ITERM_MIN_MA;
  if (ma > BQ28_ITERM_MAX_MA) ma = BQ28_ITERM_MAX_MA;
  uint8_t code = (uint8_t)(ma / BQ28_ITERM_STEP_MA);  // code = mA/5
  if (code < 1) code = 1;
  if (code > 62) code = 62;

  // ITERM is 6 bits shifted by 2 at register 0x12
  uint16_t reg_val;
  if (!bq_read16(BQ28_REG_TERM_CTRL, reg_val)) return false;
  reg_val = (reg_val & 0xFF03) | ((code & 0x3F) << 2);  // bits 7:2
  return bq_write16(BQ28_REG_TERM_CTRL, reg_val);
}

static bool bq_set_vrechg(uint16_t mv) {
  // VRECHG_THRESHOLD: 0 = 100mV, 1 = 200mV below VREG
  // Located in CHARGER_CTRL1 (0x17), bit 4
  uint8_t reg_val;
  if (!bq_read8(BQ28_REG_CHARGER_CTRL1, reg_val)) return false;
  if (mv >= 200) {
    reg_val |= (1 << 4);   // 200mV threshold
  } else {
    reg_val &= ~(1 << 4);  // 100mV threshold
  }
  return bq_write8(BQ28_REG_CHARGER_CTRL1, reg_val);
}

// =============================================================================
// BQ25628E Register Getters (for watchdog verification)
// =============================================================================

static bool bq_get_vreg(uint16_t& mv) {
  uint16_t reg_val;
  if (!bq_read16(BQ28_REG_VREG, reg_val)) return false;
  uint16_t code = (reg_val >> 3) & 0x01FF;
  mv = code * BQ28_VREG_STEP_MV;
  return true;
}

static bool bq_get_ichg(uint16_t& ma) {
  // ICHG is bits 10:5 of the 16-bit register at 0x02 (6 bits, shift by 5)
  uint16_t reg_val;
  if (!bq_read16(BQ28_REG_ICHG, reg_val)) return false;
  uint8_t code = (reg_val >> 5) & 0x3F;
  ma = code * BQ28_ICHG_STEP_MA;
  return true;
}

static bool bq_get_vindpm(uint16_t& mv) {
  uint16_t reg_val;
  if (!bq_read16(BQ28_REG_VINDPM, reg_val)) return false;
  uint16_t code = (reg_val >> 5) & 0x01FF;
  mv = code * BQ28_VINDPM_STEP_MV;
  return true;
}

static bool bq_get_iindpm(uint16_t& ma) {
  uint16_t reg_val;
  if (!bq_read16(BQ28_REG_IINDPM, reg_val)) return false;
  uint16_t code = (reg_val >> 4) & 0xFF;
  ma = code * BQ28_IINDPM_STEP_MA;
  return true;
}

static bool bq_get_vsysmin(uint16_t& mv) {
  uint16_t reg_val;
  if (!bq_read16(BQ28_REG_VSYSMIN, reg_val)) return false;
  uint16_t code = (reg_val >> 6) & 0x3F;
  mv = code * BQ28_VSYSMIN_STEP_MV;
  return true;
}

static bool bq_get_vbat_uvlo(uint8_t& uvlo) {
  uint16_t reg_val;
  if (!bq_read16(BQ28_REG_CHARGER_CTRL3, reg_val)) return false;
  uvlo = (reg_val & 0x0020) ? 1 : 0;
  return true;
}

// =============================================================================
// BQ25628E Control Functions
// =============================================================================

/// Reset chip to default register values (clears faults)
static bool bq_reset() {
  uint8_t reg_val;
  if (!bq_read8(BQ28_REG_CHARGER_CTRL1, reg_val)) return false;
  
  reg_val |= 0x80;  // Set REG_RST bit
  if (!bq_write8(BQ28_REG_CHARGER_CTRL1, reg_val)) return false;
  
  // Wait for reset to complete (bit auto-clears)
  uint32_t timeout = millis() + 100;
  while (millis() < timeout) {
    if (!bq_read8(BQ28_REG_CHARGER_CTRL1, reg_val)) return false;
    if ((reg_val & 0x80) == 0) return true;
    delay(1);
  }
  return false;
}

/// Disable I2C watchdog timer
static bool bq_disable_watchdog() {
  uint16_t reg_val;
  if (!bq_read16(BQ28_REG_CHARGER_CTRL0, reg_val)) return false;
  reg_val &= ~0x0003;  // Clear WATCHDOG[1:0]
  return bq_write16(BQ28_REG_CHARGER_CTRL0, reg_val);
}

/// Enable ADC with all channels
static bool bq_enable_adc() {
  if (!bq_write8(BQ28_REG_ADC_FUNC_DIS, 0x00)) return false;  // Enable all channels
  if (!bq_write8(BQ28_REG_ADC_CTRL, 0x00)) return false;      // ADC off initially
  return true;
}

static bool bq_trigger_adc() {
  // One-shot ADC conversion
  // Bit 7: ADC_EN = 1 (enable ADC)
  // Bit 6: ADC_RATE = 1 (one-shot mode)
  
  if (!bq_write8(BQ28_REG_ADC_CTRL, 0xC0)) return false;  // 0b11000000
  
  // Wait for ADC_DONE_STAT bit (bit 6) in STATUS0 register to indicate conversion complete
  // REG0x1D bit 6: ADC_DONE_STAT - 0=not complete, 1=complete (one-shot mode only)
  // Timeout after ~400ms
  uint32_t timeout = millis() + 400;
  uint8_t status0;
  
  while (millis() < timeout) {
    if (!bq_read8(BQ28_REG_STATUS0, status0)) return false;
    if (status0 & 0x40) {  // Bit 6 = ADC_DONE_STAT
      // Conversion complete
      return true;
    }
    delay(5);  // Small delay between polls
  }
  
  // Timeout - conversion did not complete in expected time
  return false;
}

// =============================================================================
// ADC Decoding Functions
// =============================================================================

/// Decode VBAT/VSYS ADC (1.99 mV/LSB, bits 15:1)
static float decode_voltage_v(uint16_t raw) {
  // Extract voltage value from bits 15:1 (shift right by 1)
  uint16_t voltage_value = raw >> 1;
  // Convert to Volts: 1.99mV per step
  return (float)voltage_value * 0.00199f;                      
}

/// Decode VBUS ADC (3.97 mV/LSB, bits 15:2)
static float decode_vbus_v(uint16_t raw) {
  // Extract voltage value from bits 15:2 (shift right by 2)
  uint16_t voltage_value = raw >> 2;
  // Convert to Volts: 3.97mV per step
  return (float)voltage_value * 0.00397f;
}

/// Decode IBAT ADC (4 mA/LSB, signed, bits 15:2)
static float decode_ibat_ma(uint16_t raw) {
  uint16_t val = raw >> 2;
  int16_t signed_val = (val & 0x2000) ? (int16_t)(val | 0xC000) : (int16_t)val;
  return (float)signed_val * 4.0f;
}

/// Decode IBUS ADC (2 mA/LSB, unsigned, bits 15:1)
static float decode_ibus_ma(uint16_t raw) {
  return (float)(raw >> 1) * 2.0f;
}

/// Decode thermistor ADC to temperature (°C)
static float decode_ts_temp(uint16_t raw) {
  uint16_t code = raw & 0x0FFF;
  float percent = (float)code * 0.0961f;
  
  if (percent <= 0.1f || percent >= 98.0f) return NAN;
  
  float v_ratio = percent / 100.0f;
  float r_ntc_par = (v_ratio * TS_R_TOP_OHM) / (1.0f - v_ratio);
  float denom = TS_FIXED_R_BOTTOM - r_ntc_par;
  
  if (denom <= 0.0f) return NAN;
  
  float r_ntc = (r_ntc_par * TS_FIXED_R_BOTTOM) / denom;
  if (r_ntc <= 0.0f) return NAN;
  
  float ln_ratio = logf(r_ntc / TS_NTC_R25_OHM);
  float inv_T = (1.0f / KELVIN_25C) + (ln_ratio / TS_BETA_K);
  return (1.0f / inv_T) - 273.15f;
}

/// Decode die temperature ADC (0.5°C/LSB, signed, bits 11:0)
static float decode_tdie_temp(uint16_t raw) {
  int16_t val = (int16_t)(raw & 0x0FFF);
  if (val & 0x0800) val |= 0xF000;  // Sign extend
  return (float)val * 0.5f;
}

// =============================================================================
// Configuration Functions
// =============================================================================

/// Configure only the settings that have been explicitly set (non-zero)
static bool bq_configure_selective(uint16_t vreg_mv, uint16_t ichg_ma,
                                    uint16_t vindpm_mv, uint16_t iindpm_ma,
                                    uint16_t vsysmin_mv, uint16_t iprechg_ma,
                                    uint16_t iterm_ma, uint16_t vrechg_mv,
                                    uint8_t vbat_uvlo, bool vbat_uvlo_set) {
  // Only apply settings that are non-zero (user-configured)
  if (vreg_mv != BQ_PREF_NOT_SET) {
    if (!bq_set_vreg(vreg_mv)) return false;
    MESH_DEBUG_PRINTLN("PowerManager: VREG=%dmV", vreg_mv);
  }
  if (ichg_ma != BQ_PREF_NOT_SET) {
    if (!bq_set_ichg(ichg_ma)) return false;
    MESH_DEBUG_PRINTLN("PowerManager: ICHG=%dmA", ichg_ma);
  }
  if (vindpm_mv != BQ_PREF_NOT_SET) {
    if (!bq_set_vindpm(vindpm_mv)) return false;
    MESH_DEBUG_PRINTLN("PowerManager: VINDPM=%dmV", vindpm_mv);
  }
  if (iindpm_ma != BQ_PREF_NOT_SET) {
    if (!bq_set_iindpm(iindpm_ma)) return false;
    MESH_DEBUG_PRINTLN("PowerManager: IINDPM=%dmA", iindpm_ma);
  }
  if (vsysmin_mv != BQ_PREF_NOT_SET) {
    if (!bq_set_vsysmin(vsysmin_mv)) return false;
    MESH_DEBUG_PRINTLN("PowerManager: VSYSMIN=%dmV", vsysmin_mv);
  }
  if (iprechg_ma != BQ_PREF_NOT_SET) {
    if (!bq_set_iprechg(iprechg_ma)) return false;
    MESH_DEBUG_PRINTLN("PowerManager: IPRECHG=%dmA", iprechg_ma);
  }
  if (iterm_ma != BQ_PREF_NOT_SET) {
    if (!bq_set_iterm(iterm_ma)) return false;
    MESH_DEBUG_PRINTLN("PowerManager: ITERM=%dmA", iterm_ma);
  }
  if (vrechg_mv != BQ_PREF_NOT_SET) {
    if (!bq_set_vrechg(vrechg_mv)) return false;
    MESH_DEBUG_PRINTLN("PowerManager: VRECHG=%dmV", vrechg_mv);
  }
  // UVLO is tricky - 0 is a valid value (Li-ion), so use separate flag
  if (vbat_uvlo_set) {
    if (!bq_set_vbat_uvlo(vbat_uvlo)) return false;
    MESH_DEBUG_PRINTLN("PowerManager: UVLO=%s", vbat_uvlo ? "LiFePO4" : "Li-ion");
  }
  return true;
}

static bool bq_init() {
  uint8_t part;
  if (!bq_read8(BQ28_REG_PART_INFO, part)) return false;
  MESH_DEBUG_PRINTLN("PowerManager: BQ25628E part=0x%02X", part);

  // Reset to clear any faults and restore chip defaults
  // We keep the reset because:
  // 1. Clears any fault conditions from previous run
  // 2. Ensures chip starts in a known-good state
  // 3. Chip defaults are reasonable (4.2V, conservative currents)
  if (!bq_reset()) {
    MESH_DEBUG_PRINTLN("PowerManager: Reset failed (continuing)");
  }
  delay(10);

  if (!bq_disable_watchdog()) return false;
  
  // Don't apply any configuration here - leave chip at power-on defaults.
  // User-configured settings will be applied via loadAndApplyBQSettings()
  // after prefs are loaded.

  if (!bq_enable_adc()) return false;
  
  return true;
}

// =============================================================================
// Public API Implementation
// =============================================================================

bool PowerManager::begin(TwoWire& wire) {
  g_wire = &wire;
  g_bq_initialized = bq_init();
  MESH_DEBUG_PRINTLN("PowerManager: BQ25628E init %s", g_bq_initialized ? "OK" : "FAIL");
  return g_bq_initialized;
}

void PowerManager::loop() {
  if (!g_node_prefs) return;
  
  uint32_t now = millis();
  if (now - g_last_watchdog_check_ms >= 10000) {
    g_last_watchdog_check_ms = now;
    verifyAndCorrectBQSettings();
  }
}

bool PowerManager::isBQInitialized() {
  return g_bq_initialized;
}

uint16_t PowerManager::getBatteryVoltageMilliVolts() {
  if (!g_bq_initialized) return 0;
  if (!bq_trigger_adc()) return 0;

  uint16_t raw;
  if (!bq_read16(BQ28_REG_VBAT_ADC, raw)) return 0;
  return (uint16_t)(decode_voltage_v(raw) * 1000.0f);
}

void PowerManager::addTelemetry(CayenneLPP& lpp) {
  if (!g_bq_initialized) return;
  if (!bq_trigger_adc()) return;

  uint16_t vbat_raw, vbus_raw, ibat_raw, ibus_raw, ts_raw, tdie_raw;
  if (!bq_read16(BQ28_REG_VBAT_ADC, vbat_raw)) return;
  if (!bq_read16(BQ28_REG_VBUS_ADC, vbus_raw)) return;
  if (!bq_read16(BQ28_REG_IBAT_ADC, ibat_raw)) return;
  if (!bq_read16(BQ28_REG_IBUS_ADC, ibus_raw)) return;
  if (!bq_read16(BQ28_REG_TS_ADC, ts_raw)) return;
  if (!bq_read16(BQ28_REG_TDIE_ADC, tdie_raw)) return;

  float vbat = decode_voltage_v(vbat_raw);
  float vbus = decode_vbus_v(vbus_raw);
  float ibat = decode_ibat_ma(ibat_raw) / 1000.0f;
  float ibus = decode_ibus_ma(ibus_raw) / 1000.0f;
  float ts_temp = decode_ts_temp(ts_raw);
  float tdie_temp = decode_tdie_temp(tdie_raw);

  lpp.addVoltage(1, vbat);
  lpp.addCurrent(1, ibat);
  lpp.addTemperature(1, ts_temp);
  lpp.addVoltage(2, vbus);
  lpp.addCurrent(2, ibus);
  lpp.addTemperature(2, tdie_temp);
}

bool PowerManager::configureBQ(uint16_t chg_vreg_mv, uint16_t chg_current_ma,
                                uint16_t prechg_current_ma, uint16_t term_current_ma,
                                uint16_t rechg_threshold_mv, uint16_t input_voltage_mv,
                                uint16_t input_current_ma, uint16_t min_sys_voltage_mv,
                                uint8_t vbat_uvlo) {
  if (!g_bq_initialized) return false;

  // When called from CLI, apply all provided settings (they are all user-configured)
  bool success = bq_configure_selective(
    chg_vreg_mv, chg_current_ma,
    input_voltage_mv, input_current_ma,
    min_sys_voltage_mv, prechg_current_ma,
    term_current_ma, rechg_threshold_mv,
    vbat_uvlo, true  // UVLO is explicitly set via CLI
  );
  if (success) {
    delay(50);
    verifyAndCorrectBQSettings();
  }
  return success;
}

void PowerManager::getChargeStatus(char* buf, size_t size) {
  if (!buf || size == 0) return;
  
  if (!g_bq_initialized) {
    strncpy(buf, "BQ not initialized", size);
    buf[size - 1] = '\0';
    return;
  }

  uint8_t status0, status1, fault0;
  if (!bq_read8(BQ28_REG_STATUS0, status0) ||
      !bq_read8(BQ28_REG_STATUS1, status1) ||
      !bq_read8(BQ28_REG_FAULT0, fault0)) {
    strncpy(buf, "Read failed", size);
    buf[size - 1] = '\0';
    return;
  }

  const char* vbus_str[] = {"No Power", "0x01", "0x02", "0x03", "Adapter", "0x05", "0x06", "0x07"};
  const char* chg_str[] = {"Not Charging", "Charging", "Taper", "Top-off"};
  const char* ts_str[] = {"Normal", "Cold", "Hot", "Cool", "Warm", "Pre-cool", "Pre-warm", "Bias Fault"};

  size_t n = snprintf(buf, size, "VBUS:%s CHG:%s TS:%s",
                      vbus_str[status1 & 0x07],
                      chg_str[(status1 >> 3) & 0x03],
                      ts_str[fault0 & 0x07]);

  if (fault0 & 0x80) n += snprintf(buf + n, size - n, " VBUS_FAULT");
  if (fault0 & 0x40) n += snprintf(buf + n, size - n, " BAT_FAULT");
  if (fault0 & 0x20) n += snprintf(buf + n, size - n, " SYS_FAULT");
  if (fault0 & 0x08) n += snprintf(buf + n, size - n, " TSHUT");
}

void PowerManager::getDiagnostics(char* buf, size_t size) {
  if (!buf || size == 0) return;
  
  if (!g_bq_initialized) {
    strncpy(buf, "BQ not initialized", size);
    buf[size - 1] = '\0';
    return;
  }

  bq_trigger_adc();

  uint16_t vbat_raw, vsys_raw, vbus_raw, ibat_raw, ibus_raw;
  bq_read16(BQ28_REG_VBAT_ADC, vbat_raw);
  bq_read16(BQ28_REG_VSYS_ADC, vsys_raw);
  bq_read16(BQ28_REG_VBUS_ADC, vbus_raw);
  bq_read16(BQ28_REG_IBAT_ADC, ibat_raw);
  bq_read16(BQ28_REG_IBUS_ADC, ibus_raw);

  uint8_t status0, status1, fault0;
  bq_read8(BQ28_REG_STATUS0, status0);
  bq_read8(BQ28_REG_STATUS1, status1);
  bq_read8(BQ28_REG_FAULT0, fault0);

  snprintf(buf, size,
           "VBAT=%.3fV VSYS=%.3fV VBUS=%.3fV | IBAT=%.0fmA IIN=%.0fmA | REG[1D]=%02X [1E]=%02X [1F]=%02X",
           decode_voltage_v(vbat_raw),
           decode_voltage_v(vsys_raw),
           decode_vbus_v(vbus_raw),
           decode_ibat_ma(ibat_raw),
           decode_ibus_ma(ibus_raw),
           status0, status1, fault0);
}

bool PowerManager::verifyAndCorrectBQSettings() {
  if (!g_bq_initialized || !g_node_prefs) return false;

  bool needs_fix = false;
  uint16_t actual;
  uint8_t actual_uvlo;

  // Only verify settings that are user-configured (non-zero)
  // Skip checking settings the user hasn't set
  if (g_node_prefs->bq_chg_vreg_mv != BQ_PREF_NOT_SET) {
    if (bq_get_vreg(actual) && actual != g_node_prefs->bq_chg_vreg_mv) {
      MESH_DEBUG_PRINTLN("PowerManager: VREG mismatch %d vs %d", actual, g_node_prefs->bq_chg_vreg_mv);
      needs_fix = true;
    }
  }
  if (g_node_prefs->bq_chg_current_ma != BQ_PREF_NOT_SET) {
    if (bq_get_ichg(actual) && actual != g_node_prefs->bq_chg_current_ma) {
      MESH_DEBUG_PRINTLN("PowerManager: ICHG mismatch %d vs %d", actual, g_node_prefs->bq_chg_current_ma);
      needs_fix = true;
    }
  }
  if (g_node_prefs->bq_input_voltage_mv != BQ_PREF_NOT_SET) {
    if (bq_get_vindpm(actual) && actual != g_node_prefs->bq_input_voltage_mv) {
      MESH_DEBUG_PRINTLN("PowerManager: VINDPM mismatch %d vs %d", actual, g_node_prefs->bq_input_voltage_mv);
      needs_fix = true;
    }
  }
  if (g_node_prefs->bq_input_current_ma != BQ_PREF_NOT_SET) {
    if (bq_get_iindpm(actual) && actual != g_node_prefs->bq_input_current_ma) {
      MESH_DEBUG_PRINTLN("PowerManager: IINDPM mismatch %d vs %d", actual, g_node_prefs->bq_input_current_ma);
      needs_fix = true;
    }
  }
  if (g_node_prefs->bq_min_sys_voltage_mv != BQ_PREF_NOT_SET) {
    if (bq_get_vsysmin(actual) && actual != g_node_prefs->bq_min_sys_voltage_mv) {
      MESH_DEBUG_PRINTLN("PowerManager: VSYSMIN mismatch %d vs %d", actual, g_node_prefs->bq_min_sys_voltage_mv);
      needs_fix = true;
    }
  }
  // Note: bq_vbat_uvlo is not checked here because 0 is a valid value
  // and we can't distinguish "not set" from "set to Li-ion"

  if (needs_fix) {
    MESH_DEBUG_PRINTLN("PowerManager: Reconfiguring BQ25628E...");
    return bq_configure_selective(
      g_node_prefs->bq_chg_vreg_mv,
      g_node_prefs->bq_chg_current_ma,
      g_node_prefs->bq_input_voltage_mv,
      g_node_prefs->bq_input_current_ma,
      g_node_prefs->bq_min_sys_voltage_mv,
      g_node_prefs->bq_prechg_current_ma,
      g_node_prefs->bq_term_current_ma,
      g_node_prefs->bq_rechg_threshold_mv,
      g_node_prefs->bq_vbat_uvlo,
      false  // Don't re-apply UVLO during watchdog correction
    );
  }
  return true;
}

bool PowerManager::loadAndApplyBQSettings(const NodePrefs* prefs) {
  if (!prefs || !g_bq_initialized) return false;
  
  g_node_prefs = prefs;
  
  // Check if any BQ settings have been configured by the user
  bool any_set = (prefs->bq_chg_vreg_mv != BQ_PREF_NOT_SET) ||
                 (prefs->bq_chg_current_ma != BQ_PREF_NOT_SET) ||
                 (prefs->bq_input_voltage_mv != BQ_PREF_NOT_SET) ||
                 (prefs->bq_input_current_ma != BQ_PREF_NOT_SET) ||
                 (prefs->bq_min_sys_voltage_mv != BQ_PREF_NOT_SET) ||
                 (prefs->bq_prechg_current_ma != BQ_PREF_NOT_SET) ||
                 (prefs->bq_term_current_ma != BQ_PREF_NOT_SET) ||
                 (prefs->bq_rechg_threshold_mv != BQ_PREF_NOT_SET);
  
  if (!any_set) {
    MESH_DEBUG_PRINTLN("PowerManager: No user BQ settings, using chip defaults");
    return true;
  }
  
  MESH_DEBUG_PRINTLN("PowerManager: Applying user BQ settings");
  
  // Note: For UVLO, we can't tell if 0 means "not set" or "Li-ion",
  // so we only apply it if other settings are configured.
  bool success = bq_configure_selective(
    prefs->bq_chg_vreg_mv,
    prefs->bq_chg_current_ma,
    prefs->bq_input_voltage_mv,
    prefs->bq_input_current_ma,
    prefs->bq_min_sys_voltage_mv,
    prefs->bq_prechg_current_ma,
    prefs->bq_term_current_ma,
    prefs->bq_rechg_threshold_mv,
    prefs->bq_vbat_uvlo,
    any_set  // Apply UVLO if any other setting was configured
  );
  
  if (success) {
    MESH_DEBUG_PRINTLN("PowerManager: Settings applied successfully");
  }
  return success;
}

// =============================================================================
// System Power Reset via BATFET
// =============================================================================

// REG0x18 (Charger_Control_2) bit definitions:
// Bits 7:4: Other settings (JEITA, etc.)
// Bit 3:   BATFET_CTRL_WVBUS - VBUS control behavior
//          0 = BATFET reset only works when VBUS < VVBUS_UVLO (no USB power)
//          1 = BATFET reset works regardless of VBUS presence
// Bit 2:   BATFET_DLY      - Delay before BATFET action (0=no delay, 1=10s delay)
// Bits 1:0: BATFET_CTRL
//          00 = Normal operation
//          01 = Shutdown mode
//          10 = Ship mode  
//          11 = System power reset

#define BATFET_CTRL_MASK     0x03  // Bits 1:0
#define BATFET_DLY           0x04  // Bit 2 - delay before action
#define BATFET_CTRL_WVBUS    0x08  // Bit 3 - set to 1 for reset to work with USB connected

#define BATFET_NORMAL   0b00  // Normal operation
#define BATFET_SHUTDOWN 0b01  // Shutdown mode
#define BATFET_SHIP     0b10  // Ship mode
#define BATFET_RESET    0b11  // System power reset

void PowerManager::systemPowerReset() {
  if (g_bq_initialized && g_wire) {
    MESH_DEBUG_PRINTLN("PowerManager: Initiating system power reset via BATFET");
    
    // Read current CHARGER_CTRL2 register
    uint8_t reg_val;
    if (bq_read8(BQ28_REG_CHARGER_CTRL2, reg_val)) {
      // Per datasheet section 8.3.8.3:
      // When BATFET_CTRL_WVBUS = 1, system power reset proceeds if BATFET_CTRL = 11,
      // regardless of whether VBUS is present or not.
      // We also clear BATFET_DLY (bit 2) to avoid the 10s delay.
      
      // Preserve bits 7:4, set WVBUS=1, DLY=0, CTRL=11
      reg_val = (reg_val & 0xF0) | BATFET_CTRL_WVBUS | BATFET_RESET;
      
      // Write the register - this will cut power to the MCU
      // The BQ25628E will turn off BATFET for tBATFET_RST (~100ms) then re-enable
      bq_write8(BQ28_REG_CHARGER_CTRL2, reg_val);
      
      // If we get here, the BATFET reset didn't work immediately
      // Wait a bit for it to take effect
      delay(500);
    }
    
    MESH_DEBUG_PRINTLN("PowerManager: BATFET reset failed, falling back to MCU reset");
  }
  
  // Fallback: standard MCU reset
  // For ARM Cortex-M: NVIC_SystemReset()
  // For other architectures, this may need to be different
#if defined(__arm__) || defined(__ARM_ARCH)
  NVIC_SystemReset();
#elif defined(ESP32)
  ESP.restart();
#else
  // Generic fallback - endless loop, watchdog should catch this
  while(1) { delay(100); }
#endif
}
