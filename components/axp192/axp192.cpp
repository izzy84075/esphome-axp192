#include "axp192.h"
#include "esphome/core/log.h"
#include "esp_sleep.h"
#include <Esp.h>

namespace esphome {
namespace axp192 {

static const char *TAG = "axp192.sensor";
void AXP192Component::setup()
{
  switch (this->model_) {
    case AXP192_M5STICKC:
    {
        begin(false, false, false, false, false);
        break;
    }
    case AXP192_M5CORE2:
    {
        // disable LDO3 Vibration
        begin(false, true, false, false, false);
        break;
    }
    case AXP192_M5TOUGH:
    {
        begin(false, false, false, false, false);

        // If we're waking from a cold boot
        if (GetStartupReason() == "ESP_RST_POWERON")
        {
            ESP_LOGD(TAG, "First power on, restarting ESP...");

            // Reboot the ESP with the axp initialised
            ESP.restart();
        }
        break;
    }
  }
}

bool AXP192Component::configure_ldoio0(bool enable) {
#ifdef USE_SWITCH
  auto location = this->power_control_.find(OutputPin::OUTPUT_LDOIO0);
  this->load_register(RegisterLocations::GPIO_CONTROL);
  this->set_ldoio0_mode(enable ? LDOio0Control::LOWNOISE_LDO : LDOio0Control::FLOATING);
  if (this->save_register(RegisterLocations::GPIO_CONTROL)) {
    this->update_powercontrol(OutputPin::OUTPUT_LDOIO0, this->get_ldoio0_enabled());
    return true;
  }
#endif
  return false;
}

void AXP192Component::set_voff(VoffVoltage voff) {
  this->update_register(RegisterLocations::VOFF_VOLTAGE, detail::to_int(voff), 0b11111000);
}

void AXP192Component::set_charge_voltage(ChargeVoltage voltage) {
  this->update_register(RegisterLocations::CHARGE_CONTROL_REG1, detail::to_int(voltage), 0b10011111);
}

void AXP192Component::set_charge_current(ChargeCurrent current) {
  this->update_register(RegisterLocations::CHARGE_CONTROL_REG1, detail::to_int(current), 0b11110000);
}

void AXP192Component::set_vbus_ipsout(VBusIpsout val) {
  this->update_register(RegisterLocations::VBUS_IPSOUT_ACCESS, detail::to_int(val), 0b01111111);
}

void AXP192Component::set_vbus_hold_current_limited(VBusHoldCurrentLimited val) {
  this->update_register(RegisterLocations::VBUS_IPSOUT_ACCESS, detail::to_int(val), 0b11111101);
}

void AXP192Component::set_vbus_hold_current_limit(VBusHoldCurrentLimit val) {
  this->update_register(RegisterLocations::VBUS_IPSOUT_ACCESS, detail::to_int(val), 0b11111110);
}

void AXP192Component::set_vbus_hold_voltage_limited(VBusHoldVoltageLimited val) {
  this->update_register(RegisterLocations::VBUS_IPSOUT_ACCESS, detail::to_int(val), 0b10111111);
}

void AXP192Component::set_vbus_hold_voltage_limit(VBusHoldVoltageLimit val) {
  this->update_register(RegisterLocations::VBUS_IPSOUT_ACCESS, detail::to_int(val), 0b11000111);
}

void AXP192Component::set_disable_rtc(bool disable_rtc) {
  this->update_register(RegisterLocations::BATTERY_BACKUP_CONTROL, disable_rtc ? 0x0 : 0b10000000, 0b11000111);
}

void AXP192Component::set_disable_ldo2(bool disable_ldo2) {
  this->update_register(RegisterLocations::DCDC13_LDO23_CONTROL, disable_ldo2 ? 0x0 : 0b00000100, 0b11111011);
}

void AXP192Component::set_disable_ldo3(bool disable_ldo3) {
  this->update_register(RegisterLocations::DCDC13_LDO23_CONTROL, disable_ldo3 ? 0x0 : 0b11110111, 0b00001000);
}

void AXP192Component::set_disable_dcdc1(bool disable_dcdc1) {
  this->update_register(RegisterLocations::DCDC13_LDO23_CONTROL, disable_dcdc1 ? 0x0 : 0b00000010, 0b11111101);
}

void AXP192Component::set_disable_dcdc3(bool disable_dcdc3) {
  this->update_register(RegisterLocations::DCDC13_LDO23_CONTROL, disable_dcdc3 ? 0x0 : 0b00000001, 0b11111110);
}

void AXP192Component::set_dcdc1_voltage(uint32_t dcdc1_voltage) {
  this->update_register(RegisterLocations::DCDC1_VOLTAGE,
                        detail::constrained_remap<uint32_t, 700, 3500, 0x0, 0x7F>(dcdc1_voltage), 0b10000000);
}

void AXP192Component::set_dcdc3_voltage(uint32_t dcdc3_voltage) {
  this->update_register(RegisterLocations::DCDC3_VOLTAGE,
                        detail::constrained_remap<uint32_t, 700, 3500, 0x0, 0x7F>(dcdc3_voltage), 0b10000000);
}

void AXP192Component::set_ldo2_voltage(uint32_t ldo2_voltage) {
  this->update_register(RegisterLocations::LDO23_VOLTAGE,
                        (detail::constrained_remap<int, 1800, 3300, 0x0, 0x0F>(ldo2_voltage) << 4), 0b00001111);
}

void AXP192Component::set_ldo3_voltage(uint32_t ldo3_voltage) {
  this->update_register(RegisterLocations::LDO23_VOLTAGE,
                        detail::constrained_remap<uint32_t, 1800, 3300, 0x0, 0x0F>(ldo3_voltage), 0b11110000);
}

void AXP192Component::set_ldoio0_voltage(uint32_t ldoio0_voltage) {
  this->update_register(RegisterLocations::GPIO_LDO_VOLTAGE,
                        (detail::constrained_remap<uint32_t, 1800, 3300, 0x0, 0x0F>(ldoio0_voltage) << 4), 0b00001111);
}

void AXP192Component::set_ldoio0_mode(LDOio0Control mode) {
  this->update_register(RegisterLocations::GPIO_CONTROL, detail::to_int(mode), 0b11111000);
}

bool AXP192Component::configure_ldo2(bool enable) {
  this->debug_log_register_(RegisterLocations::DCDC13_LDO23_CONTROL);
  this->load_register(RegisterLocations::DCDC13_LDO23_CONTROL);
  this->set_disable_ldo2(!enable);
  if (this->save_register(RegisterLocations::DCDC13_LDO23_CONTROL)) {
    this->update_powercontrol(OutputPin::OUTPUT_LDO2, this->get_ldo2_enabled());
    return true;
  }
  return false;
}

bool AXP192Component::configure_ldo3(bool enable) {
  this->load_register(RegisterLocations::DCDC13_LDO23_CONTROL);
  this->set_disable_ldo3(!enable);
  if (this->save_register(RegisterLocations::DCDC13_LDO23_CONTROL)) {
    this->update_powercontrol(OutputPin::OUTPUT_LDO3, this->get_ldo3_enabled());
    return true;
  }
  return false;
}

bool AXP192Component::configure_dcdc1(bool enable) {
  this->load_register(RegisterLocations::DCDC13_LDO23_CONTROL);
  this->set_disable_dcdc1(!enable);
  if (this->save_register(RegisterLocations::DCDC13_LDO23_CONTROL)) {
    this->update_powercontrol(OutputPin::OUTPUT_DCDC1, this->get_dcdc1_enabled());
    return true;
  }
  return false;
}

bool AXP192Component::configure_dcdc3(bool enable) {
  this->load_register(RegisterLocations::DCDC13_LDO23_CONTROL);
  this->set_disable_dcdc3(!enable);
  if (this->save_register(RegisterLocations::DCDC13_LDO23_CONTROL)) {
    this->update_powercontrol(OutputPin::OUTPUT_DCDC3, this->get_dcdc3_enabled());
    return true;
  }
  return false;
}

bool AXP192Component::configure_ldo2_voltage(float level) {
  auto scaled = remap<float, uint32_t>(level, 0.0f, 1.0f, 1800, 3300);
  this->load_register(RegisterLocations::LDO23_VOLTAGE);
  this->set_ldo2_voltage(scaled);
  return this->save_register(RegisterLocations::LDO23_VOLTAGE);
}

bool AXP192Component::configure_ldo3_voltage(float level) {
  auto scaled = remap<float, uint32_t>(level, 0.0f, 1.0f, 1800, 3300);
  this->load_register(RegisterLocations::LDO23_VOLTAGE);
  this->set_ldo3_voltage(scaled);
  return this->save_register(RegisterLocations::LDO23_VOLTAGE);
}

bool AXP192Component::configure_dcdc1_voltage(float level) {
  auto scaled = remap<float, uint32_t>(level, 0.0f, 1.0f, 700, 3500);
  this->load_register(RegisterLocations::DCDC1_VOLTAGE);
  this->set_dcdc1_voltage(scaled);
  return this->save_register(RegisterLocations::DCDC1_VOLTAGE);
}

bool AXP192Component::configure_dcdc3_voltage(float level) {
  auto scaled = remap<float, uint32_t>(level, 0.0f, 1.0f, 700, 3500);
  this->load_register(RegisterLocations::DCDC3_VOLTAGE);
  this->set_dcdc3_voltage(scaled);
  return this->save_register(RegisterLocations::DCDC3_VOLTAGE);
}

bool AXP192Component::configure_ldoio0_voltage(float level) {
  auto scaled = remap<float, uint32_t>(level, 0.0f, 1.0f, 1800, 3300);
  this->load_register(RegisterLocations::GPIO_LDO_VOLTAGE);
  this->set_ldoio0_voltage(scaled);
  return this->save_register(RegisterLocations::GPIO_LDO_VOLTAGE);
}

bool AXP192Component::get_ldo2_enabled() {
  return (this->registers_.at(RegisterLocations::DCDC13_LDO23_CONTROL) & 0b00000100) != 0;
}

bool AXP192Component::get_ldo3_enabled() {
  return (this->registers_.at(RegisterLocations::DCDC13_LDO23_CONTROL) & 0b00001000) != 0;
}

bool AXP192Component::get_dcdc1_enabled() {
  return (this->registers_.at(RegisterLocations::DCDC13_LDO23_CONTROL) & 0b00000001) != 0;
}

bool AXP192Component::get_dcdc3_enabled() {
  return (this->registers_.at(RegisterLocations::DCDC13_LDO23_CONTROL) & 0b00000010) != 0;
}

bool AXP192Component::get_ldoio0_enabled() {
  return (this->registers_.at(RegisterLocations::GPIO_CONTROL) & 0b00000111) == 0b00000010;
}

float AXP192Component::get_ldo2_voltage() {
  auto raw = (this->registers_.at(RegisterLocations::LDO23_VOLTAGE) & 0b11110000) >> 4;
  return remap<float, uint8_t>(raw, 0, 0x0F, 0, 100);
}

float AXP192Component::get_ldo3_voltage() {
  auto raw = (this->registers_.at(RegisterLocations::LDO23_VOLTAGE) & 0b00001111);
  return remap<float, uint8_t>(raw, 0, 0x0F, 0, 100);
}

float AXP192Component::get_dcdc1_voltage() {
  auto raw = (this->registers_.at(RegisterLocations::DCDC1_VOLTAGE) & 0b01111111);
  return remap<float, uint8_t>(raw, 0, 0x7F, 0, 100);
}

float AXP192Component::get_dcdc3_voltage() {
  auto raw = (this->registers_.at(RegisterLocations::DCDC3_VOLTAGE) & 0b01111111);
  return remap<float, uint8_t>(raw, 0, 0x7F, 0, 100);
}

float AXP192Component::get_ldoio0_voltage() {
  auto raw = (this->registers_.at(RegisterLocations::GPIO_LDO_VOLTAGE) & 0b11110000) >> 4;
  return remap<float, uint8_t>(raw, 0, 0x0F, 0, 100);
}

void AXP192Component::set_dcdc2_voltage(uint32_t dcdc2_voltage) {
  this->update_register(RegisterLocations::DCDC2_VOLTAGE,
                        detail::constrained_remap<uint32_t, 700, 2275, 0x0, 0x3F>(dcdc2_voltage), 0b11000000);
}

void AXP192Component::set_disable_dcdc2(bool disable_dcdc2) {
  this->update_register(RegisterLocations::EXTEN_DCDC2_CONTROL, disable_dcdc2 ? 0x0 : 0b00000101, 0b11111010);
}

bool AXP192Component::configure_dcdc2(bool enable) {
  this->load_register(RegisterLocations::EXTEN_DCDC2_CONTROL);
  this->set_disable_dcdc2(!enable);
  return this->save_register(RegisterLocations::EXTEN_DCDC2_CONTROL);
}

void AXP192Component::dump_config() {
  ESP_LOGCONFIG(TAG, "AXP192:");
  LOG_I2C_DEVICE(this);
  LOG_SENSOR("  ", "Battery Level", this->batterylevel_sensor_);
}

float AXP192Component::get_setup_priority() const { return setup_priority::DATA; }

void AXP192Component::update() {

    if (this->batterylevel_sensor_ != nullptr) {
      // To be fixed
      // This is not giving the right value - mostly there to have some sample sensor...
      float vbat = GetBatVoltage();
      float batterylevel = 100.0 * ((vbat - 3.0) / (4.1 - 3.0));

      ESP_LOGD(TAG, "Got Battery Level=%f (%f)", batterylevel, vbat);
      if (batterylevel > 100.) {
        batterylevel = 100;
      }
      this->batterylevel_sensor_->publish_state(batterylevel);
    }
  
    if (this->charging_sensor_ != nullptr) {
      bool battery_state = GetBatState();
      bool charging = GetChargingState();
      ESP_LOGD(TAG, "Got Charging=%d state=%d", charging, battery_state);
      this->charging_sensor_->publish_state(charging);
    }
  
    UpdateBrightness();
}


void AXP192Component::begin(bool disableLDO2, bool disableLDO3, bool disableRTC, bool disableDCDC1, bool disableDCDC3)
{
  switch (this->model_) {
    case AXP192_M5STICKC:
    {
        // Set LDO2 & LDO3(TFT_LED & TFT) 3.0V
        Write1Byte(0x28, 0xcc);
        break;
    }
    case AXP192_M5CORE2:
    {
        // Set DCDC3 (TFT_LED & TFT) 3.0V
        Write1Byte(0x27, 0xcc);
        // Set LDO2 & LDO3(TFT_LED & TFT) 3.0V
        Write1Byte(0x28, 0xcc);
        break;
    }
    case AXP192_M5TOUGH:
    {
        // Set DCDC3 (TFT_LED & TFT) 3.0V
        Write1Byte(0x27, 0xcc);
        // Set LDO2 & LDO3(TFT_LED & TFT) 3.0V
        Write1Byte(0x28, 0xcc);
        break;
    }
  }

    // Set ADC sample rate to 200hz
    Write1Byte(0x84, 0b11110010);

    // Set ADC to All Enable
    Write1Byte(0x82, 0xff);

    // Bat charge voltage to 4.2, Current 100MA
    Write1Byte(0x33, 0xc0);

    // Depending on configuration enable LDO2, LDO3, DCDC1, DCDC3.
    uint8_t buf = (Read8bit(0x12) & 0xef) | 0x4D;
    if(disableLDO3) buf &= ~(1<<3);
    if(disableLDO2) buf &= ~(1<<2);
    if(disableDCDC3) buf &= ~(1<<1);
    if(disableDCDC1) buf &= ~(1<<0);
    Write1Byte(0x12, buf);

    // 128ms power on, 4s power off
    Write1Byte(0x36, 0x0C);

    if(!disableRTC)
    {
        // Set RTC voltage to 3.3V
        Write1Byte(0x91, 0xF0);

        // Set GPIO0 to LDO
        Write1Byte(0x90, 0x02);
    }

    // Disable vbus hold limit
    Write1Byte(0x30, 0x80);

    // Set temperature protection
    Write1Byte(0x39, 0xfc);

    // Enable RTC BAT charge
    Write1Byte(0x35, 0xa2 & (disableRTC ? 0x7F : 0xFF));

    // Enable bat detection
    Write1Byte(0x32, 0x46);

}

void AXP192Component::Write1Byte( uint8_t Addr ,  uint8_t Data )
{
    this->write_byte(Addr, Data);
}

uint8_t AXP192Component::Read8bit( uint8_t Addr )
{
    uint8_t data;
    this->read_byte(Addr, &data);
    return data;
}

uint16_t AXP192Component::Read12Bit( uint8_t Addr)
{
    uint16_t Data = 0;
    uint8_t buf[2];
    ReadBuff(Addr,2,buf);
    Data = ((buf[0] << 4) + buf[1]); //
    return Data;
}

uint16_t AXP192Component::Read13Bit( uint8_t Addr)
{
    uint16_t Data = 0;
    uint8_t buf[2];
    ReadBuff(Addr,2,buf);
    Data = ((buf[0] << 5) + buf[1]); //
    return Data;
}

uint16_t AXP192Component::Read16bit( uint8_t Addr )
{
    uint32_t ReData = 0;
    uint8_t Buff[2];
    this->read_bytes(Addr, Buff, sizeof(Buff));
    for( int i = 0 ; i < sizeof(Buff) ; i++ )
    {
        ReData <<= 8;
        ReData |= Buff[i];
    }
    return ReData;
}

uint32_t AXP192Component::Read24bit( uint8_t Addr )
{
    uint32_t ReData = 0;
    uint8_t Buff[3];
    this->read_bytes(Addr, Buff, sizeof(Buff));
    for( int i = 0 ; i < sizeof(Buff) ; i++ )
    {
        ReData <<= 8;
        ReData |= Buff[i];
    }
    return ReData;
}

uint32_t AXP192Component::Read32bit( uint8_t Addr )
{
    uint32_t ReData = 0;
    uint8_t Buff[4];
    this->read_bytes(Addr, Buff, sizeof(Buff));
    for( int i = 0 ; i < sizeof(Buff) ; i++ )
    {
        ReData <<= 8;
        ReData |= Buff[i];
    }
    return ReData;
}

void AXP192Component::ReadBuff( uint8_t Addr , uint8_t Size , uint8_t *Buff )
{
    this->read_bytes(Addr, Buff, Size);
}

void AXP192Component::UpdateBrightness()
{
    if (brightness_ == curr_brightness_)
    {
        return;
    }

    ESP_LOGD(TAG, "Brightness=%f (Curr: %f)", brightness_, curr_brightness_);
    curr_brightness_ = brightness_;

    const uint8_t c_min = 7;
    const uint8_t c_max = 12;
    auto ubri = c_min + static_cast<uint8_t>(brightness_ * (c_max - c_min));

    if (ubri > c_max)
    {
        ubri = c_max;
    }
    switch (this->model_) {
      case AXP192_M5STICKC:
      {
        uint8_t buf = Read8bit( 0x28 );
        Write1Byte( 0x28 , ((buf & 0x0f) | (ubri << 4)) );
        break;
      }
      case AXP192_M5CORE2:
      {
        uint8_t buf = Read8bit( 0x27 );
        Write1Byte( 0x27 , ((buf & 0x80) | (ubri << 3)) );
        break;
      }
      case AXP192_M5TOUGH:
      {
        uint8_t buf = Read8bit( 0x27 );
        Write1Byte( 0x27 , ((buf & 0x80) | (ubri << 3)) );
        break;
      }
    }
    if (brightness_ == 0) {
      SetLDO2(false);
    } else {
      SetLDO2(true);
    }
}

bool AXP192Component::GetBatState()
{
    if( Read8bit(0x01) | 0x20 )
        return true;
    else
        return false;
}

bool AXP192Component::GetChargingState()
{
    // reading 0x00 bit 6
    if( (Read8bit(0x01) >> 5) & 0x01 )
        return true;
    else
        return false;
}

uint8_t AXP192Component::GetBatData()
{
    return Read8bit(0x75);
}
//---------coulombcounter_from_here---------
//enable: void EnableCoulombcounter(void);
//disable: void DisableCOulombcounter(void);
//stop: void StopCoulombcounter(void);
//clear: void ClearCoulombcounter(void);
//get charge data: uint32_t GetCoulombchargeData(void);
//get discharge data: uint32_t GetCoulombdischargeData(void);
//get coulomb val affter calculation: float GetCoulombData(void);
//------------------------------------------
void  AXP192Component::EnableCoulombcounter(void)
{
    Write1Byte( 0xB8 , 0x80 );
}

void  AXP192Component::DisableCoulombcounter(void)
{
    Write1Byte( 0xB8 , 0x00 );
}

void  AXP192Component::StopCoulombcounter(void)
{
    Write1Byte( 0xB8 , 0xC0 );
}

void  AXP192Component::ClearCoulombcounter(void)
{
    Write1Byte( 0xB8 , 0xA0 );
}

uint32_t AXP192Component::GetCoulombchargeData(void)
{
    return Read32bit(0xB0);
}

uint32_t AXP192Component::GetCoulombdischargeData(void)
{
    return Read32bit(0xB4);
}

float AXP192Component::GetCoulombData(void)
{

  uint32_t coin = 0;
  uint32_t coout = 0;

  coin = GetCoulombchargeData();
  coout = GetCoulombdischargeData();

  //c = 65536 * current_LSB * (coin - coout) / 3600 / ADC rate
  //Adc rate can be read from 84H ,change this variable if you change the ADC reate
  float ccc = 65536 * 0.5 * (coin - coout) / 3600.0 / 25.0;
  return ccc;

}
//----------coulomb_end_at_here----------

uint16_t AXP192Component::GetVbatData(void){

    uint16_t vbat = 0;
    uint8_t buf[2];
    ReadBuff(0x78,2,buf);
    vbat = ((buf[0] << 4) + buf[1]); // V
    return vbat;
}

uint16_t AXP192Component::GetVinData(void)
{
    uint16_t vin = 0;
    uint8_t buf[2];
    ReadBuff(0x56,2,buf);
    vin = ((buf[0] << 4) + buf[1]); // V
    return vin;
}

uint16_t AXP192Component::GetIinData(void)
{
    uint16_t iin = 0;
    uint8_t buf[2];
    ReadBuff(0x58,2,buf);
    iin = ((buf[0] << 4) + buf[1]);
    return iin;
}

uint16_t AXP192Component::GetVusbinData(void)
{
    uint16_t vin = 0;
    uint8_t buf[2];
    ReadBuff(0x5a,2,buf);
    vin = ((buf[0] << 4) + buf[1]); // V
    return vin;
}

uint16_t AXP192Component::GetIusbinData(void)
{
    uint16_t iin = 0;
    uint8_t buf[2];
    ReadBuff(0x5C,2,buf);
    iin = ((buf[0] << 4) + buf[1]);
    return iin;
}

uint16_t AXP192Component::GetIchargeData(void)
{
    uint16_t icharge = 0;
    uint8_t buf[2];
    ReadBuff(0x7A,2,buf);
    icharge = ( buf[0] << 5 ) + buf[1] ;
    return icharge;
}

uint16_t AXP192Component::GetIdischargeData(void)
{
    uint16_t idischarge = 0;
    uint8_t buf[2];
    ReadBuff(0x7C,2,buf);
    idischarge = ( buf[0] << 5 ) + buf[1] ;
    return idischarge;
}

uint16_t AXP192Component::GetTempData(void)
{
    uint16_t temp = 0;
    uint8_t buf[2];
    ReadBuff(0x5e,2,buf);
    temp = ((buf[0] << 4) + buf[1]);
    return temp;
}

uint32_t AXP192Component::GetPowerbatData(void)
{
    uint32_t power = 0;
    uint8_t buf[3];
    ReadBuff(0x70,2,buf);
    power = (buf[0] << 16) + (buf[1] << 8) + buf[2];
    return power;
}

uint16_t AXP192Component::GetVapsData(void)
{
    uint16_t vaps = 0;
    uint8_t buf[2];
    ReadBuff(0x7e,2,buf);
    vaps = ((buf[0] << 4) + buf[1]);
    return vaps;
}

void AXP192Component::SetSleep(void)
{
    Write1Byte(0x31 , Read8bit(0x31) | ( 1 << 3)); // Power off voltag 3.0v
    Write1Byte(0x90 , Read8bit(0x90) | 0x07); // GPIO1 floating
    Write1Byte(0x82, 0x00); // Disable ADCs
    Write1Byte(0x12, Read8bit(0x12) & 0xA1); // Disable all outputs but DCDC1
}

// -- sleep
void AXP192Component::DeepSleep(uint64_t time_in_us)
{
    SetSleep();
    esp_sleep_enable_ext0_wakeup((gpio_num_t)37, 0 /* LOW */);
    if (time_in_us > 0)
    {
        esp_sleep_enable_timer_wakeup(time_in_us);
    }
    else
    {
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    }
    (time_in_us == 0) ? esp_deep_sleep_start() : esp_deep_sleep(time_in_us);
}

void AXP192Component::LightSleep(uint64_t time_in_us)
{
    if (time_in_us > 0)
    {
        esp_sleep_enable_timer_wakeup(time_in_us);
    }
    else
    {
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
    }
    esp_light_sleep_start();
}

// 0 not press, 0x01 long press, 0x02 press
uint8_t AXP192Component::GetBtnPress()
{
    uint8_t state = Read8bit(0x46);
    if(state)
    {
        Write1Byte( 0x46 , 0x03 );
    }
    return state;
}

uint8_t AXP192Component::GetWarningLevel(void)
{
    return Read8bit(0x47) & 0x01;
}

float AXP192Component::GetBatVoltage()
{
    float ADCLSB = 1.1 / 1000.0;
    uint16_t ReData = Read12Bit( 0x78 );
    return ReData * ADCLSB;
}

float AXP192Component::GetBatCurrent()
{
    float ADCLSB = 0.5;
    uint16_t CurrentIn = Read13Bit( 0x7A );
    uint16_t CurrentOut = Read13Bit( 0x7C );
    return ( CurrentIn - CurrentOut ) * ADCLSB;
}

float AXP192Component::GetVinVoltage()
{
    float ADCLSB = 1.7 / 1000.0;
    uint16_t ReData = Read12Bit( 0x56 );
    return ReData * ADCLSB;
}

float AXP192Component::GetVinCurrent()
{
    float ADCLSB = 0.625;
    uint16_t ReData = Read12Bit( 0x58 );
    return ReData * ADCLSB;
}

float AXP192Component::GetVBusVoltage()
{
    float ADCLSB = 1.7 / 1000.0;
    uint16_t ReData = Read12Bit( 0x5A );
    return ReData * ADCLSB;
}

float AXP192Component::GetVBusCurrent()
{
    float ADCLSB = 0.375;
    uint16_t ReData = Read12Bit( 0x5C );
    return ReData * ADCLSB;
}

float AXP192Component::GetTempInAXP192()
{
    float ADCLSB = 0.1;
    const float OFFSET_DEG_C = -144.7;
    uint16_t ReData = Read12Bit( 0x5E );
    return OFFSET_DEG_C + ReData * ADCLSB;
}

float AXP192Component::GetBatPower()
{
    float VoltageLSB = 1.1;
    float CurrentLCS = 0.5;
    uint32_t ReData = Read24bit( 0x70 );
    return  VoltageLSB * CurrentLCS * ReData/ 1000.0;
}

float AXP192Component::GetBatChargeCurrent()
{
    float ADCLSB = 0.5;
    uint16_t ReData = Read13Bit( 0x7A );
    return ReData * ADCLSB;
}

float AXP192Component::GetAPSVoltage()
{
    float ADCLSB = 1.4  / 1000.0;
    uint16_t ReData = Read12Bit( 0x7E );
    return ReData * ADCLSB;
}

float AXP192Component::GetBatCoulombInput()
{
    uint32_t ReData = Read32bit( 0xB0 );
    return ReData * 65536 * 0.5 / 3600 /25.0;
}

float AXP192Component::GetBatCoulombOut()
{
    uint32_t ReData = Read32bit( 0xB4 );
    return ReData * 65536 * 0.5 / 3600 /25.0;
}

void AXP192Component::SetCoulombClear()
{
    Write1Byte(0xB8,0x20);
}

void AXP192Component::SetLDO2( bool State )
{
    uint8_t buf = Read8bit(0x12);
    if( State == true )
    {
        buf = (1<<2) | buf;
    }
    else
    {
        buf = ~(1<<2) & buf;
    }
    Write1Byte( 0x12 , buf );
}

void AXP192Component::SetLDO3(bool State)
{
    uint8_t buf = Read8bit(0x12);
    if( State == true )
    {
        buf = (1<<3) | buf;
    }
    else
    {
        buf = ~(1<<3) & buf;
    }
    Write1Byte( 0x12 , buf );
}

void AXP192Component::SetChargeCurrent(uint8_t current)
{
    uint8_t buf = Read8bit(0x33);
    buf = (buf & 0xf0) | (current & 0x07);
    Write1Byte(0x33, buf);
}

void AXP192Component::PowerOff()
{
    Write1Byte(0x32, Read8bit(0x32) | 0x80);
}

void AXP192Component::SetAdcState(bool state)
{
    Write1Byte(0x82, state ? 0xff : 0x00);
}

std::string AXP192Component::GetStartupReason() {
  esp_reset_reason_t reset_reason = ::esp_reset_reason();
  if (reset_reason == ESP_RST_DEEPSLEEP) {
    esp_sleep_source_t wake_reason = esp_sleep_get_wakeup_cause();
    if (wake_reason == ESP_SLEEP_WAKEUP_EXT0)
      return "ESP_SLEEP_WAKEUP_EXT0";
    if (wake_reason == ESP_SLEEP_WAKEUP_EXT0)
      return "ESP_SLEEP_WAKEUP_EXT0";
    if (wake_reason == ESP_SLEEP_WAKEUP_EXT1)
      return "ESP_SLEEP_WAKEUP_EXT1";
    if (wake_reason == ESP_SLEEP_WAKEUP_TIMER)
      return "ESP_SLEEP_WAKEUP_TIMER";
    if (wake_reason == ESP_SLEEP_WAKEUP_TOUCHPAD)
      return "ESP_SLEEP_WAKEUP_TOUCHPAD";
    if (wake_reason == ESP_SLEEP_WAKEUP_ULP)
      return "ESP_SLEEP_WAKEUP_ULP";
    if (wake_reason == ESP_SLEEP_WAKEUP_GPIO)
      return "ESP_SLEEP_WAKEUP_GPIO";
    if (wake_reason == ESP_SLEEP_WAKEUP_UART)
      return "ESP_SLEEP_WAKEUP_UART";
    return std::string{"WAKEUP_UNKNOWN_REASON"};
  }

  if (reset_reason == ESP_RST_UNKNOWN)
    return "ESP_RST_UNKNOWN";
  if (reset_reason == ESP_RST_POWERON)
    return "ESP_RST_POWERON";
  if (reset_reason == ESP_RST_SW)
    return "ESP_RST_SW";
  if (reset_reason == ESP_RST_PANIC)
    return "ESP_RST_PANIC";
  if (reset_reason == ESP_RST_INT_WDT)
    return "ESP_RST_INT_WDT";
  if (reset_reason == ESP_RST_TASK_WDT)
    return "ESP_RST_TASK_WDT";
  if (reset_reason == ESP_RST_WDT)
    return "ESP_RST_WDT";
  if (reset_reason == ESP_RST_BROWNOUT)
    return "ESP_RST_BROWNOUT";
  if (reset_reason == ESP_RST_SDIO)
    return "ESP_RST_SDIO";
  return std::string{"RESET_UNKNOWN_REASON"};
}

}
}

