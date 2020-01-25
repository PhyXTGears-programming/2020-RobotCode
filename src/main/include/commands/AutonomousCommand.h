#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivetrain.h"

class AutonomousCommand : public frc2::CommandHelper<frc2::CommandBase, AutonomousCommand> {
 public:
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  explicit AutonomousCommand(Drivetrain* drivetrain);
  void Initialize();
  void Execute();
  bool IsFinished();

 private:
  Drivetrain* m_drivetrain;
};
