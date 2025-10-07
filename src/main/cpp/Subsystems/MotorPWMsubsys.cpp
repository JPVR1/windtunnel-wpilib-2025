#include "subsystems/MotorPWMsubsys.h"
#include <units/time.h>
#include <algorithm>

using namespace units::literals;

MotorPWMsubsys::MotorPWMsubsys(int pwmPort)
  : m_motor(pwmPort), m_limiter(0.5)  // adjust rate as you like
{
  m_motor.SetPeriodMultiplier(frc::PWM::kPeriodMultiplier_4X);
  m_motor.SetBounds(
      2.0_ms,  // max pulse
      1.8_ms,  // upper deadband
      1.5_ms,  // neutral
      1.2_ms,  // lower deadband
      1.0_ms   // min pulse
  );

  // Ensure many ESCs see min pulse at boot (helps arming)
  m_motor.SetSpeed(-1.0);

  // seed manual key (convenience)
  frc::SmartDashboard::SetDefaultNumber("PWM input", 0.0);
}

void MotorPWMsubsys::SetSpeed(int pwmCounts) {
  // clamp and store
  m_pwmValue = std::clamp(pwmCounts, 0, kPwmMaxCounts);

  // SYMMETRIC map: 0..Max -> -1..+1
  // 0 -> -1.0 (min); Max/2 -> 0.0 (neutral); Max -> +1.0
  double normalized = (static_cast<double>(m_pwmValue) / kPwmMaxCounts) * 2.0 - 1.0;

  // no neutral deadband here to preserve low-end authority
  double smoothed = m_limiter.Calculate(normalized, 0.02); // Teleop ~20ms
  m_motor.SetSpeed(smoothed);
}

void MotorPWMsubsys::Stop() {
  m_motor.SetSpeed(0.0);
}

void MotorPWMsubsys::Periodic() {
  frc::SmartDashboard::PutNumber("Motor PWM value", m_pwmValue);
  frc::SmartDashboard::PutNumber("Motor Speed", m_motor.GetSpeed());
}
