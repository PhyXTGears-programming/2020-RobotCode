#pragma once

#include <frc2/command/Command.h>
#include <frc/XboxController.h>

#include "commands/AutonomousCommand.h"
#include "subsystems/Drivetrain.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  void DrivetrainTest();

 private:
  // Operators' input devices.
  frc::XboxController m_driverJoystick{1};

  // The robot's subsystems and commands are defined here...
  Drivetrain m_drivetrain;
  AutonomousCommand m_autonomousCommand;

  void ConfigureButtonBindings();
};
