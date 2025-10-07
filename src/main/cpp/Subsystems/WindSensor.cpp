#include "subsystems/WindSensor.h"

#include <algorithm>
#include <cmath>
#include <ctime>

WindSensorsubsys::WindSensorsubsys(int windChannel, int tempChannel)
  : m_Windsensor{windChannel}, m_Tempsensor{tempChannel}
{
  // If needed later:
  // m_Windsensor.SetAverageBits(4);
  // m_Windsensor.SetOversampleBits(2);
  // m_Tempsensor.SetAverageBits(2);
  // m_Tempsensor.SetOversampleBits(4);
}

int16_t WindSensorsubsys::GetValue()             { return m_Windsensor.GetValue(); }
int16_t WindSensorsubsys::GetAverageValue()      { return m_Windsensor.GetAverageValue(); }
double  WindSensorsubsys::GetVoltage()           { return m_Windsensor.GetVoltage(); }
double  WindSensorsubsys::GetAverageVoltage()    { return m_Windsensor.GetAverageVoltage(); }

int16_t WindSensorsubsys::GetValue2()            { return m_Tempsensor.GetValue(); }
int16_t WindSensorsubsys::GetAverageValue2()     { return m_Tempsensor.GetAverageValue(); }
double  WindSensorsubsys::GetVoltage2()          { return m_Tempsensor.GetVoltage(); }
double  WindSensorsubsys::GetAverageVoltage2()   { return m_Tempsensor.GetAverageVoltage(); }

// calibration from raw -> mph -> m/s
double WindSensorsubsys::GetWindSpeed() {
  int windRaw  = GetAverageValue();
  double offset = 1056.0;
  double scale  = 342.7256;
  double power  = 3.36814;
  if (windRaw < offset) return 0.0;
  double windMPH = std::pow((windRaw - offset) / scale, power);
  return windMPH * 0.44704; // mph -> m/s
}

double WindSensorsubsys::GetTemperatureC() {
  double v = GetAverageVoltage2();
  return ((v - 0.400) / 0.0195) * 0.858715;
}

void WindSensorsubsys::Periodic() {
  // === 1) tunable low-pass on wind speed ===
  static constexpr double kNomDt = 0.02;       // 20 ms nominal loop
  double tau = frc::SmartDashboard::GetNumber("Filter Tau (s)", 0.30);
  tau = std::clamp(tau, 0.05, 1.0);
  const double alpha = kNomDt / (tau + kNomDt);

  const double speed_now = GetWindSpeed();
  if (!m_filtInit) { m_speedFilt = speed_now; m_filtInit = true; }
  else             { m_speedFilt += alpha * (speed_now - m_speedFilt); }

  // === 2) telemetry ===
  frc::SmartDashboard::PutNumber("Wind Sensor Voltage", GetVoltage());
  frc::SmartDashboard::PutNumber("Wind Sensor Raw (12-bit)", GetValue());
  frc::SmartDashboard::PutNumber("Wind Speed (m/s)", speed_now);
  frc::SmartDashboard::PutNumber("Wind Speed filt (m/s)", m_speedFilt);
  frc::SmartDashboard::PutNumber("TMP Sensor Voltage", GetVoltage2());
  frc::SmartDashboard::PutNumber("Temperature (°C)", GetTemperatureC());
  frc::SmartDashboard::PutNumber("Logging Time (s)", m_loggingEnabled ? m_logTimer.Get().value() : 0.0);
  // Read-only status indicator (bind a Boolean/Text widget, not a toggle)
  frc::SmartDashboard::PutBoolean("Logging Active (read-only)", m_loggingEnabled);

  // === 3) logging start/stop with edge latch ===
  const bool requestedLogging = frc::SmartDashboard::GetBoolean("Start Logging", false);

  // Rising edge -> start
  if (requestedLogging && !m_prevRequest && !m_loggingEnabled) {
    auto now = std::time(nullptr);
    char filename[128];
    std::strftime(filename, sizeof(filename), "/home/lvuser/windlog_%Y%m%d_%H%M.csv",
                  std::localtime(&now));
    m_logFile.open(filename, std::ios::out | std::ios::app);
    if (m_logFile.is_open()) {
      m_logFile << "Timestamp_s,WindSpeed_mps,PWM_Value,TurbineRPM,TemperatureC\n";
      m_logTimer.Reset();
      m_logTimer.Start();
      m_lastFlushSec = -1;  // force first flush
      m_loggingEnabled = true;
    } else {
      // failed to open -> push the toggle back off so UI matches reality
      frc::SmartDashboard::PutBoolean("Start Logging", false);
    }
  }

  // Falling edge -> stop
  if (!requestedLogging && m_prevRequest && m_loggingEnabled) {
    if (m_logFile.is_open()) {
      m_logFile.flush();
      m_logFile.close();
    }
    m_logTimer.Stop();
    m_loggingEnabled = false;
  }
  m_prevRequest = requestedLogging;

  // === 4) write a row if logging ===
  if (m_loggingEnabled && m_logFile.is_open()) {
    const double t = m_logTimer.Get().value();
    m_logFile << t << "," << speed_now << "," << m_pwmValue << "," << m_rpmValue << "," << GetTemperatureC() << "\n";

    // periodic flush ~1 Hz to reduce data loss on power cut
    const int sec = static_cast<int>(t);
    if (sec != m_lastFlushSec) {
      m_logFile.flush();
      m_lastFlushSec = sec;
    }
  }
}

WindSensorsubsys::~WindSensorsubsys() {
  if (m_logFile.is_open()) {
    m_logFile.flush();
    m_logFile.close();
  }
}
