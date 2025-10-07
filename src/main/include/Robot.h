// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>
#include "XboxController.h"        // your wrapper (keep if you use it)
#include <frc/Encoder.h>

#include "subsystems/MotorPWMsubsys.h"
#include "subsystems/WindSensor.h"

class Robot : public frc::TimedRobot {
public:
  Robot();
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

  // handy helper
  static int clampi(int x, int lo, int hi) { return std::max(lo, std::min(x, hi)); }

  XboxController m_controller{0};

private:
  // UI seeding
  void SeedUI();
  // drive all ESCs
  void SetAllMotors(int pwmCounts);

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom  = "My Auto";
  std::string m_autoSelected;

  frc::Timer m_armTimer;
  bool m_escArmed{false};

  // --- PID state ---
  double m_iState     = 0.0;  // (m/s)*s
  double m_prevSpeed  = 0.0;  // m/s (for D on measurement)
  double m_prevTime   = 0.0;  // s
  double m_dFilt      = 0.0;  // derivative low-pass state
  int    m_lastMode   = -1;   // detect mode changes
  bool   m_spinning   = false;

  // Turbine encoder
  // DIO2 = A, DIO3 = B; reverse=true (flip if RPM sign is inverted)
  frc::Encoder m_Omron1Encoder{2, 3, true, frc::Encoder::EncodingType::k4X};

  // Simple IIR filter state for steadier readout
  double m_EncoderRPMfilter{0.0};
};
