// Copyright ...
#include "Robot.h"

#include "subsystems/MotorPWMsubsys.h"
#include "subsystems/WindSensor.h"

#include <wpi/print.h>
#include <units/time.h>
#include <algorithm>
#include <cmath>
#include <iostream>

using namespace units::literals;

// ===== Subsystems (adjust ports if needed) =====
MotorPWMsubsys  motorPWMsubsys1{7};
MotorPWMsubsys  motorPWMsubsys2{8};
MotorPWMsubsys  motorPWMsubsys3{9};

WindSensorsubsys Windsensor1{0, 1};

// ---------------- UI seeding ----------------
void Robot::SeedUI() {
  // Modes & basics
  frc::SmartDashboard::SetDefaultNumber("Input Mode", 2);   // 0=Manual, 1=Xbox, 2=PID
  frc::SmartDashboard::SetDefaultNumber("PWM Max", 255);

  // CONTROL TOGGLE (bind your Shuffleboard Toggle Button to THIS key)
  frc::SmartDashboard::SetDefaultBoolean("Start Logging", false);

  // Manual
  frc::SmartDashboard::SetDefaultNumber("PWM input", 0);

  // PID knobs
  frc::SmartDashboard::SetDefaultNumber("Kp", 10.0);
  frc::SmartDashboard::SetDefaultNumber("Ki",  0.0);
  frc::SmartDashboard::SetDefaultNumber("Kd",  0.0);
  frc::SmartDashboard::SetDefaultNumber("I Max (counts)", 80.0);
  frc::SmartDashboard::SetDefaultNumber("Kaw (1/s)", 5.0);
  frc::SmartDashboard::SetDefaultNumber("D Tau (s)", 0.10);

  // Setpoint
  frc::SmartDashboard::SetDefaultNumber("Setpoint(m/s)", 5.0);

  // Feedforward (optional)
  frc::SmartDashboard::SetDefaultBoolean("FF Enable", false);
  frc::SmartDashboard::SetDefaultNumber("FF k (counts per m/s)", 25.0);
  frc::SmartDashboard::SetDefaultNumber("FF bias (counts)", 0.0);

  // Sensor filter already seeded in WindSensor
  frc::SmartDashboard::SetDefaultNumber("Filter Tau (s)", 0.30);

  // Anti-zero / hysteresis and floors (ESC-30A friendly)
  frc::SmartDashboard::SetDefaultNumber("Spin Th On (m/s)", 0.6);
  frc::SmartDashboard::SetDefaultNumber("Spin Th Off (m/s)", 0.3);
  frc::SmartDashboard::SetDefaultNumber("Min Active (counts)", 18);
  frc::SmartDashboard::SetDefaultNumber("SP Active Th (m/s)", 0.10);
  frc::SmartDashboard::SetDefaultNumber("SP Deadband (m/s)", 0.05);

  // Output step limiter
  frc::SmartDashboard::SetDefaultNumber("Cmd Max Step", 8);

  // Spin-up floors
  frc::SmartDashboard::SetDefaultNumber("Start Floor (counts)", 52);
  frc::SmartDashboard::SetDefaultNumber("Hold Floor (counts)", 20);
  frc::SmartDashboard::SetDefaultNumber("Spin Threshold (m/s)", 0.5);

  // Telemetry seed
  frc::SmartDashboard::SetDefaultNumber("Wind Speed (m/s)", 0.0);
  frc::SmartDashboard::SetDefaultNumber("Kp Error (m/s)", 0.0);
  frc::SmartDashboard::SetDefaultNumber("Kp Output (PWM)", 0.0);
  frc::SmartDashboard::PutString("ESC Status", "Arming...");

  // Encoder UI
  frc::SmartDashboard::SetDefaultNumber("Turbine Gear Ratio (enc rev per turb rev)", 1.0);
  frc::SmartDashboard::SetDefaultNumber("RPM Filter Tau (s)", 0.25);
}

Robot::Robot() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom,  kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  SeedUI();

  // Configure turbine encoder: 2000 PPR -> 8000 counts/rev @ 4x
  const double cpr = 2000.0 * 4.0;
  m_Omron1Encoder.SetDistancePerPulse(1.0 / cpr);  // each pulse = 1/8000 rev
  m_Omron1Encoder.SetSamplesToAverage(8);          // mild smoothing at FPGA
  m_Omron1Encoder.SetMaxPeriod(0.2_s);             // >200 ms => stopped
  m_Omron1Encoder.Reset();

  m_armTimer.Start();
}

void Robot::RobotPeriodic() {
  frc::SmartDashboard::PutNumber("Actual Motor speed", motorPWMsubsys1.GetSpeed());
  Windsensor1.Periodic();

  // --- Turbine RPM readout ---
  const double gear = frc::SmartDashboard::GetNumber("Turbine Gear Ratio (enc rev per turb rev)", 1.0);
  const double rev_per_sec_enc = m_Omron1Encoder.GetRate();  // rev/s (distance-per-pulse = rev)
  const double rpm_enc = rev_per_sec_enc * 60.0;
  const double rpm_turbine = (gear > 1e-9) ? (rpm_enc / gear) : 0.0;

  // Simple first-order (IIR) filter for display/logging smoothness
  const double tau = std::clamp(frc::SmartDashboard::GetNumber("RPM Filter Tau (s)", 0.25), 0.05, 1.0);
  const double dt_nom = 0.02; // 20 ms nominal loop
  const double alpha = dt_nom / (tau + dt_nom);
  m_EncoderRPMfilter += alpha * (rpm_turbine - m_EncoderRPMfilter);

  // Telemetry
  frc::SmartDashboard::PutNumber("Turbine RPM (raw)",  rpm_turbine);
  frc::SmartDashboard::PutNumber("Turbine RPM (filt)", m_EncoderRPMfilter);

  Windsensor1.SetRPMValue(m_EncoderRPMfilter);
}

void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  wpi::print("Auto selected: {}\n", m_autoSelected);
}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // reset PID state when enabling teleop
  m_iState = 0.0;
  m_prevSpeed = Windsensor1.GetWindSpeedFiltered();
  m_prevTime  = frc::Timer::GetFPGATimestamp().value();
  m_dFilt     = 0.0;
  m_lastMode  = -1;
  m_spinning  = false;
}

void Robot::SetAllMotors(int cmd) {
  motorPWMsubsys1.SetSpeed(cmd);
  motorPWMsubsys2.SetSpeed(cmd);
  motorPWMsubsys3.SetSpeed(cmd);

  Windsensor1.SetPWMValue(motorPWMsubsys1.GetPWMValue());
}

void Robot::TeleopPeriodic() {
  // --- ESC arming ---
  if (!m_escArmed) {
    if (m_armTimer.Get() < 15.0_s) {
      SetAllMotors(0); // -> -1.0 pulse (min), helps arming
      frc::SmartDashboard::PutString("ESC Status", "Arming...");
      return;
    } else {
      m_escArmed = true;
      frc::SmartDashboard::PutString("ESC Status", "Armed");
    }
  }

  // --- Common UI ---
  int inputMode = static_cast<int>(frc::SmartDashboard::GetNumber("Input Mode", 2));
  int pwmMax    = static_cast<int>(frc::SmartDashboard::GetNumber("PWM Max", 255));
  pwmMax = std::clamp(pwmMax, 1, MotorPWMsubsys::kPwmMaxCounts);

  // detect mode change -> reset PI(D) timing/integ
  if (inputMode != m_lastMode) {
    if (inputMode == 2) { // entering PID
      m_iState    = 0.0;
      m_prevSpeed = Windsensor1.GetWindSpeedFiltered();
      m_prevTime  = frc::Timer::GetFPGATimestamp().value();
      m_dFilt     = 0.0;
      m_spinning  = false;
    }
    m_lastMode = inputMode;
  }

  // --- MODE 1: Xbox ---
  if (inputMode == 1) {
    double y = m_controller.GetLeftY(); // [-1,+1]
    int pwm = static_cast<int>(((y + 1.0) * 0.5) * pwmMax); // 0..pwmMax
    pwm = clampi(pwm, 0, pwmMax);
    SetAllMotors(pwm);
    Windsensor1.SetPWMValue(motorPWMsubsys1.GetPWMValue());
    frc::SmartDashboard::PutNumber("Kp Error (m/s)", 0.0);
    frc::SmartDashboard::PutNumber("Kp Output (PWM)", 0.0);
    return;
  }

  // --- MODE 2: PID (with anti-windup + D-on-meas + optional FF) ---
  if (inputMode == 2) {
    // gains & knobs
    const double Kp    = frc::SmartDashboard::GetNumber("Kp", 10.0);
    const double Ki    = frc::SmartDashboard::GetNumber("Ki", 0.0);
    const double Kd    = frc::SmartDashboard::GetNumber("Kd", 0.0);
    const double Imax  = frc::SmartDashboard::GetNumber("I Max (counts)", 80.0);
    const double Kaw   = frc::SmartDashboard::GetNumber("Kaw (1/s)", 5.0);
    const double dTau  = frc::SmartDashboard::GetNumber("D Tau (s)", 0.10);
    const double sp    = frc::SmartDashboard::GetNumber("Setpoint(m/s)", 0.0);

    const bool   ffEn  = frc::SmartDashboard::GetBoolean("FF Enable", false);
    const double ffk   = frc::SmartDashboard::GetNumber("FF k (counts per m/s)", 25.0);
    const double ffb   = frc::SmartDashboard::GetNumber("FF bias (counts)", 0.0);

    // timing (measured dt with clamp 5..50 ms)
    const double now   = frc::Timer::GetFPGATimestamp().value();
    double dt          = now - m_prevTime;
    dt = std::clamp(dt, 0.005, 0.050);
    m_prevTime = now;

    // measurement
    const double pv    = Windsensor1.GetWindSpeedFiltered();
    frc::SmartDashboard::PutNumber("Wind Speed (m/s)", pv);

    // error
    double e     = sp - pv;
    const double eDead = frc::SmartDashboard::GetNumber("SP Deadband (m/s)", 0.05);
    if (std::abs(e) < eDead) e = 0.0;

    // derivative on measurement (de/dt = -d(pv)/dt), with simple 1st-order filter
    double dmeas = 0.0;
    if (dt > 1e-6) dmeas = (pv - m_prevSpeed) / dt;     // (m/s)/s
    m_prevSpeed = pv;
    double beta = dt / (dTau + dt);                     // 0..1
    m_dFilt += beta * (dmeas - m_dFilt);
    const double dTerm = -Kd * m_dFilt;                 // counts

    // feedforward (optional)
    double u_ff = ffEn ? (ffk * sp + ffb) : 0.0;        // counts

    // --- anti-windup: back-calculation + clamp ---
    double iTerm = Ki * m_iState;
    iTerm = std::clamp(iTerm, -Imax, Imax);

    double u_pre = Kp * e + iTerm + dTerm + u_ff;
    double u_sat = std::clamp(u_pre, 0.0, static_cast<double>(pwmMax));

    if (Ki > 1e-9) {
      m_iState += e * dt + (Kaw * (u_sat - u_pre) / Ki) * dt;
    } else {
      m_iState += e * dt;
    }

    iTerm = Ki * m_iState;
    iTerm = std::clamp(iTerm, -Imax, Imax);

    double u_cmd = Kp * e + iTerm + dTerm + u_ff;   // counts
    int    cmd   = clampi(static_cast<int>(std::lround(u_cmd)), 0, pwmMax);

    // ===== Anti-chatter near zero (ESC-30A) =====
    const int    startFloor = static_cast<int>(frc::SmartDashboard::GetNumber("Start Floor (counts)", 52));
    const int    holdFloor  = static_cast<int>(frc::SmartDashboard::GetNumber("Hold Floor (counts)", 20));
    const int    minActive  = static_cast<int>(frc::SmartDashboard::GetNumber("Min Active (counts)", 18));
    const double spinOn     = frc::SmartDashboard::GetNumber("Spin Th On (m/s)", 0.6);
    const double spinOff    = frc::SmartDashboard::GetNumber("Spin Th Off (m/s)", 0.3);
    const double spActiveTh = frc::SmartDashboard::GetNumber("SP Active Th (m/s)", 0.10);

    if (!m_spinning && pv > spinOn)  m_spinning = true;
    if (m_spinning  && pv < spinOff && (sp < spActiveTh || cmd == 0)) m_spinning = false;

    const bool demandActive = sp >= spActiveTh;
    if (demandActive) {
      if (cmd > 0) {
        if (!m_spinning && cmd < startFloor)      cmd = startFloor; // kick to start
        else if (m_spinning && cmd < minActive)   cmd = minActive;  // keep alive
      } else {
        if (m_spinning) cmd = minActive; // brief overshoot -> don't cut to 0
      }
    }

    // ===== Command step limiter (smooth big jumps) =====
    static int lastCmd = 0;
    const int maxStep = static_cast<int>(frc::SmartDashboard::GetNumber("Cmd Max Step", 8));
    int step = std::clamp(cmd - lastCmd, -maxStep, maxStep);
    cmd = lastCmd + step;
    lastCmd = cmd;

    // drive motors
    SetAllMotors(cmd);
    Windsensor1.SetPWMValue(motorPWMsubsys1.GetPWMValue());

    // telemetry
    frc::SmartDashboard::PutNumber("Kp Error (m/s)", e);
    frc::SmartDashboard::PutNumber("Kp Output (PWM)", cmd);
    return;
  }

  // --- MODE 0: Manual ---
  {
    int pwm = static_cast<int>(frc::SmartDashboard::GetNumber("PWM input", 0));
    pwm = clampi(pwm, 0, pwmMax);
    SetAllMotors(pwm);
    Windsensor1.SetPWMValue(motorPWMsubsys1.GetPWMValue());
    frc::SmartDashboard::PutNumber("Kp Error (m/s)", 0.0);
    frc::SmartDashboard::PutNumber("Kp Output (PWM)", 0.0);
    return;
  }
}

void Robot::DisabledInit() {
  // Force stop logging on disable to avoid dangling files
  frc::SmartDashboard::PutBoolean("Start Logging", false);
}
void Robot::DisabledPeriodic() {}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}
void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
