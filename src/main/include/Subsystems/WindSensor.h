#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AnalogInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include <cstdint>
#include <fstream>
#include <string>

class WindSensorsubsys : public frc2::SubsystemBase {
public:
  explicit WindSensorsubsys(int windChannel, int tempChannel);

  // raw/voltage accessors
  int16_t GetValue();
  int16_t GetAverageValue();
  double  GetVoltage();
  double  GetAverageVoltage();

  int16_t GetValue2();
  int16_t GetAverageValue2();
  double  GetVoltage2();
  double  GetAverageVoltage2();

  // engineering units
  double  GetWindSpeed();          // m/s (raw)
  double  GetTemperatureC();

  // filtered speed API
  double  GetWindSpeedFiltered() const { return m_speedFilt; }
  void    ResetFilter(double init)     { m_speedFilt = init; m_filtInit = true; }

  // rpm from Robot (encoder)
  void SetRPMValue(double rpm){ m_rpmValue = rpm; }

  // logging helper (PWM value from motor subsystem)
  void SetPWMValue(double pwmCounts) { m_pwmValue = pwmCounts; }

  void Periodic() override;
  ~WindSensorsubsys();

private:
  frc::AnalogInput m_Windsensor;
  frc::AnalogInput m_Tempsensor;

  // rpm for logging
  double m_rpmValue = 0.0;

  // logging state
  bool           m_loggingEnabled = false;
  bool           m_prevRequest    = false;  // edge latch for "Start Logging"
  frc::Timer     m_logTimer;
  std::ofstream  m_logFile;
  int            m_lastFlushSec   = -1;
  double         m_pwmValue       = 0.0;

  // low-pass filter state
  double m_speedFilt = 0.0;
  bool   m_filtInit  = false;
};

