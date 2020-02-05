#include "RobotContainer.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer::RobotContainer() : m_AutonomousCommand(&m_Drivetrain) {
  // Initialize all of your commands and subsystems here]
  frc2::CommandScheduler::GetInstance().SetDefaultCommand(&m_Drivetrain, m_TeleopDriveCommand);

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings () {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_AutonomousCommand;
}

double c = 3200;
void RobotContainer::ShooterTest () {
  if (m_DriverJoystick.GetXButtonPressed()) {
    c -= 100;
    std::cout << c << std::endl;
  }
  if (m_DriverJoystick.GetBButtonPressed()) {
    c += 100;
    std::cout << c << std::endl;
  }
  if (m_DriverJoystick.GetAButton()) {
    m_Shooter.SetShooterMotorSpeeds(-c, c);
  } else {
    m_Shooter.SetShooterMotorSpeeds(0, 0);
  }
}
