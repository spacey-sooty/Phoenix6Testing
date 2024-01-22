#include "Robot.h"

void Robot::RobotInit() {
  m_motor = new ctre::phoenix6::hardware::TalonFX(7, "Drivebase");
  m_encoder = new ctre::phoenix6::hardware::CANcoder(16, "Drivebase");
  m_pid = new frc::PIDController(0.022, 0, 0);
  m_table = nt::NetworkTableInstance::GetDefault().GetTable("/pid");
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  m_pid->SetSetpoint(60);
  m_table->PutNumber("error", m_pid->GetPositionError());
  m_table->PutNumber("period", m_pid->GetPeriod().value());
  m_table->PutNumber("input", m_encoder->GetPosition().GetValue().value());
  m_table->PutNumber("input", m_encoder->GetPosition().GetValue().value());
  m_table->PutNumber("output", m_pid->Calculate(m_encoder->GetPosition().GetValue().value()));
  m_motor->SetVoltage(units::volt_t{m_pid->Calculate(m_encoder->GetPosition().GetValue().value())});
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
