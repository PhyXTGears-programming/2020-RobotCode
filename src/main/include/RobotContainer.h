#pragma once

#include <frc2/command/Command.h>
#include <frc/XboxController.h>

#include "subsystems/Drivetrain.h"

#include "commands/AutonomousCommand.h"
#include "commands/IntakeBallsCommand.h"
#include "commands/ExpelIntakeCommand.h"
#include "commands/ExtendIntakeCommand.h"
#include "commands/RetractIntakeCommand.h"

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

  void PollInput();

 private:
  // Operators' input devices.
  // These are 0 indexed!
  frc::XboxController m_DriverJoystick{0};
  frc::XboxController m_OperatorJoystick{1};

  // The robot's subsystems and commands are defined here...
  Drivetrain m_Drivetrain;
  Intake m_Intake;
  
  AutonomousCommand m_AutonomousCommand;
  IntakeBallsCommand m_IntakeBallsCommand {&m_Intake};
  ExpelIntakeCommand m_ExpelIntakeCommand {&m_Intake};
  RetractIntakeCommand m_RetractIntakeCommand {&m_Intake};
  ExtendIntakeCommand m_ExtendIntakeCommand {&m_Intake};

  void ConfigureButtonBindings();
};
