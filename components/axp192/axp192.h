#ifndef __AXP192_H__
#define __AXP192_H__

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace axp192 {
  class Axp192Output;

enum AXP192Model {
  AXP192_M5STICKC = 0,
  AXP192_M5CORE2,
  AXP192_M5TOUGH,
};

/ 9.11.1
enum class RegisterLocations : uint8_t {
  POWER_SUPPLY_STATUS = 0x00,
  POWER_SUPPLY_CHARGE_STATUS = 0x01,
  ACIN_VBUS_STATUS = 0x04,
  RTC_DATA_BUFFER = 0x06,
  EXTEN_DCDC2_CONTROL = 0x10,
  DCDC13_LDO23_CONTROL = 0x12,
  DCDC2_VOLTAGE = 0x23,
  DCDC2_VOLTAGE_SLOPE = 0x25,
  DCDC1_VOLTAGE = 0x26,
  DCDC3_VOLTAGE = 0x27,
  LDO23_VOLTAGE = 0x28,
  VBUS_IPSOUT_ACCESS = 0x30,
  VOFF_VOLTAGE = 0x31,
  POWEROFF_BATTERY_CHLED_CONTROL = 0x32,
  CHARGE_CONTROL_REG1 = 0x33,
  CHARGE_CONTROL_REG2 = 0x34,
  BATTERY_BACKUP_CONTROL = 0x35,
  PEK_PARAMETERS = 0x36,
  DCDC_SWITCH_FREQUENCY = 0x37,
  BATTERY_CHARGE_UNDERTEMP_VALUE = 0x38,
  BATTERY_CHARGE_OVERTEMP_VALUE = 0x39,
  APS_UNDERVOLTAGE_UNDERVOLTAGE_LEVEL1 = 0x3A,
  APS_UNDERVOLTAGE_UNDERVOLTAGE_LEVEL2 = 0x3B,
  BATTERY_DISCHARGE_UNDERTEMP_VALUE = 0x3C,
  BATTERY_DISCHARGE_OVERTEMP_VALUE = 0x3D,
  DCDC_MODE = 0x80,
  ADC_ENABLE_REGISTER1 = 0x82,
  ADC_ENABLE_REGISTER2 = 0x83,
  ADC_SAMPLE_FREQUENCY_TS_CONTROL = 0x84,
  GPIO_30_INPUT_RANGE = 0x85,
  TIMER_CONTROL = 0x8A,
  VBUS_MONITOR_SETTINGS = 0x8B,
  OVERTEMP_POWEROFF_CONTROL = 0x8F,
  GPIO_CONTROL = 0x90,
  GPIO_LDO_VOLTAGE = 0x91,
  GPIO1_CONTROL = 0x92,
  GPIO2_CONTROL = 0x93,
  GPIO20_SIGNAL_STATUS = 0x94,
  GPIO43_FUNCTION_CONTROL = 0x95,
  GPIO43_SIGNAL_STATUS = 0x96,
  GPIO20_PULLDOWN = 0x96,
  PWM1_FREQUENCY = 0x98,
  PWM1_DUTY_CYCLE1 = 0x99,
  PWM1_DUTY_CYCLE2 = 0x9A,
  PWM2_FREQUENCY = 0x9B,
  PWM2_DUTY_CYCLE1 = 0x9C,
  PWM2_DUTY_CYCLE2 = 0x9D,
  GPIO5_CONTROL = 0x9E,
  IRQ_ENABLE_REGISTER1 = 0x40,
  IRQ_ENABLE_REGISTER2 = 0x41,
  IRQ_ENABLE_REGISTER3 = 0x42,
  IRQ_ENABLE_REGISTER4 = 0x43,
  IRQ_STATUS_REGISTER1 = 0x44,
  IRQ_STATUS_REGISTER2 = 0x45,
  IRQ_STATUS_REGISTER3 = 0x46,
  IRQ_STATUS_REGISTER4 = 0x47,
  ACIN_VOLTAGE_HIGH8 = 0x56,
  ACIN_VOLTAGE_LOW4 = 0x57,
  ACIN_CURRENT_HIGH8 = 0x58,
  ACIN_CURRENT_LOW4 = 0x59,
  VBUS_VOLTAGE_HIGH8 = 0x5A,
  VBUS_VOLTAGE_LOW4 = 0x5B,
  VBUS_CURRENT_HIGH8 = 0x5C,
  VBUS_CURRENT_LOW4 = 0x5D,
  AXP_TEMP_HIGH8 = 0x5E,
  AXP_TEMP_LOW4 = 0x5F,
  BATTERY_TEMP_HIGH8 = 0x62,
  BATTERY_TEMP_LOW4 = 0x63,
  GPIO0_VOLTAGE_HIGH8 = 0x64,
  GPIO0_VOLTAGE_LOW4 = 0x65,
  GPIO1_VOLTAGE_HIGH8 = 0x66,
  GPIO1_VOLTAGE_LOW4 = 0x67,
  GPIO2_VOLTAGE_HIGH8 = 0x68,
  GPIO2_VOLTAGE_LOW4 = 0x69,
  GPIO3_VOLTAGE_HIGH8 = 0x6A,
  GPIO3_VOLTAGE_LOW4 = 0x6B,
  BATTERY_POWER_HIGH8 = 0x70,
  BATTERY_POWER_MIDDLE8 = 0x71,
  BATTERY_POWER_LOW8 = 0x72,
  BATTERY_VOLTAGE_HIGH8 = 0x78,
  BATTERY_VOLTAGE_LOW4 = 0x79,
  BATTERY_CHARGE_CURRENT_HIGH8 = 0x7A,
  BATTERY_CHARGE_CURRENT_LOW5 = 0x7B,
  BATTERY_DISCHARGE_CURRENT_HIGH8 = 0x7C,
  BATTERY_DISCHARGE_CURRENT_LOW5 = 0x7D,
  APS_VOLTAGE_HIGH8 = 0x7E,
  APS_VOLTAGE_LOW4 = 0x7F,
  BATTERY_CHARGE_COULUMB_COUNTER3 = 0xB0,
  BATTERY_CHARGE_COULUMB_COUNTER2 = 0xB1,
  BATTERY_CHARGE_COULUMB_COUNTER1 = 0xB2,
  BATTERY_CHARGE_COULUMB_COUNTER0 = 0xB3,
  BATTERY_DISCHARGE_COULUMB_COUNTER3 = 0xB4,
  BATTERY_DISCHARGE_COULUMB_COUNTER2 = 0xB5,
  BATTERY_DISCHARGE_COULUMB_COUNTER1 = 0xB6,
  BATTERY_DISCHARGE_COULUMB_COUNTER0 = 0xB7,
  BATTERY_COULUMB_COUTER_CONTROL = 0xB8
};

enum class OutputPin : uint8_t { OUTPUT_LDO2, OUTPUT_LDO3, OUTPUT_DCDC1, OUTPUT_DCDC3, OUTPUT_LDOIO0 };

#define SLEEP_MSEC(us) (((uint64_t)us) * 1000L)
#define SLEEP_SEC(us)  (((uint64_t)us) * 1000000L)
#define SLEEP_MIN(us)  (((uint64_t)us) * 60L * 1000000L)
#define SLEEP_HR(us)   (((uint64_t)us) * 60L * 60L * 1000000L)

#define CURRENT_100MA  (0b0000)
#define CURRENT_190MA  (0b0001)
#define CURRENT_280MA  (0b0010)
#define CURRENT_360MA  (0b0011)
#define CURRENT_450MA  (0b0100)
#define CURRENT_550MA  (0b0101)
#define CURRENT_630MA  (0b0110)
#define CURRENT_700MA  (0b0111)

class AXP192Component : public PollingComponent, public i2c::I2CDevice {
public:
  void set_batterylevel_sensor(sensor::Sensor *batterylevel_sensor) { batterylevel_sensor_ = batterylevel_sensor; }
  void set_charging_sensor(binary_sensor::BinarySensor *charging_sensor) { charging_sensor_ = charging_sensor; }
  void set_brightness(float brightness) { brightness_ = brightness; }
  void set_model(AXP192Model model) { this->model_ = model; }

  void set_voff(VoffVoltage voff);  //
  void set_charge_voltage(ChargeVoltage voltage); //
  void set_charge_current(ChargeCurrent current); //
  void set_vbus_ipsout(VBusIpsout val); //
  void set_vbus_hold_current_limited(VBusHoldCurrentLimited val); //
  void set_vbus_hold_current_limit(VBusHoldCurrentLimit val); //
  void set_vbus_hold_voltage_limited(VBusHoldVoltageLimited val); //
  void set_vbus_hold_voltage_limit(VBusHoldVoltageLimit val);   //
  void set_disable_rtc(bool disable_rtc); //  
  void set_disable_ldo2(bool disable_ldo2); //
  void set_disable_ldo3(bool disable_ldo3); //
  void set_disable_dcdc1(bool disable_dcdc1); //
  void set_disable_dcdc2(bool disable_dcdc2); //
  void set_disable_dcdc3(bool disable_dcdc3); //
  void set_dcdc1_voltage(uint32_t dcdc1_voltage); //
  void set_dcdc2_voltage(uint32_t dcdc2_voltage); //
  void set_dcdc3_voltage(uint32_t dcdc3_voltage); //
  void set_ldo2_voltage(uint32_t ldo2_voltage); //
  void set_ldo3_voltage(uint32_t ldo3_voltage); //
  void set_ldoio0_voltage(uint32_t ldoio0_voltage); //
  void set_ldoio0_mode(LDOio0Control mode); //

  bool configure_ldo2(bool enable); //
  bool configure_ldo3(bool enable); //
  bool configure_dcdc1(bool enable);  //
  bool configure_dcdc2(bool enable);  //
  bool configure_dcdc3(bool enable);  //
  bool configure_ldoio0(bool enable); //
  bool configure_ldo2_voltage(float level); //
  bool configure_ldo3_voltage(float level); //
  bool configure_dcdc1_voltage(float level);  //
  bool configure_dcdc3_voltage(float level);  //
  bool configure_ldoio0_voltage(float level); //
  bool get_ldo2_enabled();  //
  bool get_ldo3_enabled();   //
  bool get_dcdc1_enabled(); //
  bool get_dcdc3_enabled(); //
  bool get_ldoio0_enabled();  //
  float get_ldo2_voltage(); //
  float get_ldo3_voltage(); //
  float get_dcdc1_voltage();   //
  float get_dcdc3_voltage();  //
  float get_ldoio0_voltage();  //

  // ========== INTERNAL METHODS ==========
  // (In most use cases you won't need these)
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;

private:
    static std::string GetStartupReason();

protected:
    sensor::Sensor *batterylevel_sensor_;
    binary_sensor::BinarySensor *charging_sensor_;
    float brightness_{1.0f};
    float curr_brightness_{-1.0f};
    AXP192Model model_;

    /** M5 Stick Values
     * LDO2: Display backlight
     * LDO3: Display Control
     * RTC: Don't set GPIO1 as LDO
     * DCDC1: Main rail. When not set the controller shuts down.
     * DCDC3: Use unknown
     ***********************
     * M5Stack Core2 Values
     * LDO2: ILI9342C PWR (Display)
     * LD03: Vibration Motor
     */

    void  begin(bool disableLDO2 = false, bool disableLDO3 = false, bool disableRTC = false, bool disableDCDC1 = false, bool disableDCDC3 = false);
    void  UpdateBrightness();
    bool  GetBatState();
    bool GetChargingState();
    uint8_t  GetBatData();

    void  EnableCoulombcounter(void);
    void  DisableCoulombcounter(void);
    void  StopCoulombcounter(void);
    void  ClearCoulombcounter(void);
    uint32_t GetCoulombchargeData(void);
    uint32_t GetCoulombdischargeData(void);
    float GetCoulombData(void);

    uint16_t GetVbatData(void) __attribute__((deprecated));
    uint16_t GetIchargeData(void) __attribute__((deprecated));
    uint16_t GetIdischargeData(void) __attribute__((deprecated));
    uint16_t GetTempData(void) __attribute__((deprecated));
    uint32_t GetPowerbatData(void) __attribute__((deprecated));
    uint16_t GetVinData(void) __attribute__((deprecated));
    uint16_t GetIinData(void) __attribute__((deprecated));
    uint16_t GetVusbinData(void) __attribute__((deprecated));
    uint16_t GetIusbinData(void) __attribute__((deprecated));
    uint16_t GetVapsData(void) __attribute__((deprecated));
    uint8_t GetBtnPress(void);

      // -- sleep
    void SetSleep(void);
    void DeepSleep(uint64_t time_in_us = 0);
    void LightSleep(uint64_t time_in_us = 0);

    // void SetChargeVoltage( uint8_t );
    void  SetChargeCurrent( uint8_t );
    float GetBatVoltage();
    float GetBatCurrent();
    float GetVinVoltage();
    float GetVinCurrent();
    float GetVBusVoltage();
    float GetVBusCurrent();
    float GetTempInAXP192();
    float GetBatPower();
    float GetBatChargeCurrent();
    float GetAPSVoltage();
    float GetBatCoulombInput();
    float GetBatCoulombOut();
    uint8_t GetWarningLevel(void);
    void SetCoulombClear();
    void SetLDO2( bool State );
    void SetLDO3( bool State );
    void SetAdcState(bool State);

    void PowerOff();


    void Write1Byte( uint8_t Addr ,  uint8_t Data );
    uint8_t Read8bit( uint8_t Addr );
    uint16_t Read12Bit( uint8_t Addr);
    uint16_t Read13Bit( uint8_t Addr);
    uint16_t Read16bit( uint8_t Addr );
    uint32_t Read24bit( uint8_t Addr );
    uint32_t Read32bit( uint8_t Addr );
    void ReadBuff( uint8_t Addr , uint8_t Size , uint8_t *Buff );
};

}
}

#endif
