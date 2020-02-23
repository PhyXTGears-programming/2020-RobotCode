#include "RobotContainer.h"

enum class Pov : int {
    kRight = 90,
    kLeft = 270,
    kUp = 180,
    kDown = 0
};

RobotContainer::RobotContainer() : m_AutonomousCommand(&m_Drivetrain) {
    // Initialize all of your commands and subsystems here

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

void RobotContainer::PollInput() {
    // This works, but JoystickButton does not.
    if (m_OperatorJoystick.GetYButtonPressed()) {
        m_ExpelIntakeCommand.Schedule();
    } else if (m_OperatorJoystick.GetYButtonReleased()) {
        m_ExpelIntakeCommand.Cancel();
        m_Intake.IntakeStop();
    }
    
    if (m_OperatorJoystick.GetBButtonPressed()) {
        m_IntakeBallsCommand.Schedule();
    } else if (m_OperatorJoystick.GetBButtonReleased()) {
        m_IntakeBallsCommand.Cancel();
        m_Intake.IntakeStop();
    }

    if (static_cast<int>(Pov::kRight) == m_OperatorJoystick.GetPOV()) {
        m_ExtendIntakeCommand.Schedule();
    } else if (static_cast<int>(Pov::kLeft) == m_OperatorJoystick.GetPOV()) {
        m_RetractIntakeCommand.Schedule();
    }
}
