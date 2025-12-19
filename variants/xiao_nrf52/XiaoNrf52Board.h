#pragma once

#include <MeshCore.h>
#include <Arduino.h>
#include <helpers/PowerManager.h>

#ifdef XIAO_NRF52

class XiaoNrf52Board : public mesh::MainBoard {
protected:
  uint8_t startup_reason;

public:
  void begin();
  uint8_t getStartupReason() const override { return startup_reason; }

#if defined(P_LORA_TX_LED)
  void onBeforeTransmit() override {
    digitalWrite(P_LORA_TX_LED, LOW);   // turn TX LED on
  }
  void onAfterTransmit() override {
    digitalWrite(P_LORA_TX_LED, HIGH);   // turn TX LED off
  }
#endif

  uint16_t getBattMilliVolts() override {
    // Use BQ25628E voltage reading if PowerManager is initialized
    if (PowerManager::isBQInitialized()) {
      uint16_t voltage = PowerManager::getBatteryVoltageMilliVolts();
      if (voltage > 0) {
        return voltage;
      }
    }

    // Fallback to direct ADC reading if BQ25628E is not available
    // Please read befor going further ;)
    // https://wiki.seeedstudio.com/XIAO_BLE#q3-what-are-the-considerations-when-using-xiao-nrf52840-sense-for-battery-charging

    // We can't drive VBAT_ENABLE to HIGH as long
    // as we don't know wether we are charging or not ...
    // this is a 3mA loss (4/1500)
    digitalWrite(VBAT_ENABLE, LOW);
    int adcvalue = 0;
    analogReadResolution(12);
    analogReference(AR_INTERNAL_3_0);
    delay(10);
    adcvalue = analogRead(PIN_VBAT);
    return (adcvalue * ADC_MULTIPLIER * AREF_VOLTAGE) / 4.096;
  }

  const char* getManufacturerName() const override {
    return "Seeed Xiao-nrf52";
  }

  void reboot() override {
    // Use BQ25628E BATFET power reset for a full power cycle
    // This cuts power to the MCU via BATFET, then re-enables after ~100ms
    // Falls back to standard MCU reset if BQ is not available
    PowerManager::systemPowerReset();
    // systemPowerReset() doesn't return, but just in case:
    NVIC_SystemReset();
  }

  bool startOTAUpdate(const char* id, char reply[]) override;
};

#endif