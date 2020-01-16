#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include <hal/cpp/fpga_clock.h>

#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
 public:
  Robot() {};
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

  double GetDeltaTime() { return m_DeltaTime; }

 private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc2::Command* m_autonomousCommand = nullptr;

  double m_DeltaTime = 0; // delta time (time between periodic updates) in seconds
  hal::fpga_clock::time_point m_timePrev = hal::fpga_clock::now();

  RobotContainer m_container;
};
