#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/PWM.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include "SlewRateLimiter.h"   // your limiter (Calculate(value, dt))

class MotorPWMsubsys : public frc2::SubsystemBase {
public:
  explicit MotorPWMsubsys(int pwmPort);

  // Accepts "counts" 0..kPwmMaxCounts
  void   SetSpeed(int pwmCounts);
  void   Stop();
  void   Periodic() override;

  int    GetPWMValue() const { return m_pwmValue; }
  double GetSpeed()   const { return m_motor.GetSpeed(); }  // -1..+1 from WPILib

  // Keep this consistent with your dashboard "PWM Max"
  static constexpr int kPwmMaxCounts = 255;  // set 127 if your ESC is half-range

private:
  int      m_pwmValue{0};
  frc::PWM m_motor;
  SlewRateLimiter m_limiter;   // slew on the normalized (-1..+1) signal
};
