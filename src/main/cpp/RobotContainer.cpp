#include "RobotContainer.h"

#include <frc2/command/CommandScheduler.h>

RobotContainer::RobotContainer() : m_AutonomousCommand(&m_Drivetrain) {
  // Initialize all of your commands and subsystems here]
  frc2::CommandScheduler::GetInstance().SetDefaultCommand(&m_Drivetrain, m_TeleopDriveCommand);

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_AutonomousCommand;
}
