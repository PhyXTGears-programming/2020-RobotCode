#include "RobotContainer.h"

#include <iostream>

#include <frc2/command/CommandScheduler.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/PrintCommand.h>

RobotContainer::RobotContainer() : m_AutonomousCommand(&m_Drivetrain) {
    // Initialize all of your commands and subsystems here]
    frc2::CommandScheduler::GetInstance().SetDefaultCommand(&m_Drivetrain, m_TeleopDriveCommand);

    // Configure the button bindings
    ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
    // frc2::JoystickButton aButton{&m_DriverJoystick, 1};
    // aButton.WhenPressed(m_VisionAimingCommand);
    // aButton.WhenPressed(frc2::PrintCommand("Testing")).WhenReleased(frc2::PrintCommand("Released"));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    // An example command will be run in autonomous
    return &m_AutonomousCommand;
}

void RobotContainer::PollInput() {
    // This works, but JoystickButton does not.

    if (m_DriverJoystick.GetRawButtonPressed(1)) {
        m_VisionAimingCommand.Schedule();
    } else if (m_DriverJoystick.GetRawButtonReleased(1)) {
        m_VisionAimingCommand.Cancel();
    }
}