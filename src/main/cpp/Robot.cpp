#include "Robot.h"

static double lasttime = 0;
static double preverror = 0;

void Robot::RobotInit() {
  m_motor = new ctre::phoenix6::hardware::TalonFX(7, "Drivebase");
  m_encoder = new ctre::phoenix6::hardware::CANcoder(16, "Drivebase");
  //m_pid = new frc::PIDController(0.022, 0, 0);
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  double currenttimestamp = frc::Timer::GetFPGATimestamp().value();
  double dt = currenttimestamp - lasttime;
  double setpoint = 60;
  double input = m_encoder->GetPosition().GetValue().value();
  double error = setpoint - input;
  double deriv = error - preverror / dt;
  double sum = error * dt;
  double output = 0.001 * error + 0 * sum + 0 * deriv + 0.1 * input;
  m_motor->SetVoltage(units::volt_t{output});

  //m_pid->SetSetpoint(60);
  //m_motor->SetVoltage(units::volt_t{m_pid->Calculate(m_encoder->GetPosition().GetValue().value())});
  lasttime = currenttimestamp;
  preverror = error;
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
