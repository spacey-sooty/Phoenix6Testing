#include "Robot.h"

void Robot::RobotInit() {
  m_motor = new ctre::phoenix6::hardware::TalonFX(9, "Drivebase");
  m_pid = new frc::PIDController(0.022, 0, 0);
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  m_pid->SetSetpoint(60);
  m_motor->SetVoltage(units::volt_t{m_pid->Calculate(m_motor->GetPosition().GetValue().value())});
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
