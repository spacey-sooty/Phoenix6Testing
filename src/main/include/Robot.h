#pragma once

#include <frc/TimedRobot.h>
#include <frc/controller/PIDController.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <memory>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
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

 private:
  ctre::phoenix6::hardware::TalonFX* m_motor;
  frc::PIDController* m_pid;
  std::shared_ptr<nt::NetworkTable> m_table;
};
