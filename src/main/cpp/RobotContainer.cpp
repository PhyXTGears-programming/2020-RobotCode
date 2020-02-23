#include "RobotContainer.h"

#include <iostream>

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/PrintCommand.h>
#include <units/units.h>

enum class Pov : int {
    kRight = 90,
    kLeft = 270,
    kUp = 180,
    kDown = 0
};


RobotContainer::RobotContainer() : m_AutonomousCommand(&m_Drivetrain) {
    frc2::CommandScheduler::GetInstance().SetDefaultCommand(&m_Drivetrain, m_TeleopDriveCommand);
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(&m_Shooter);

    // Configure the button bindings
    ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
    // frc2::JoystickButton aButton{&m_DriverJoystick, 1};
    // aButton.WhenPressed(m_VisionAimingCommand);
    // aButton.WhenPressed(frc2::PrintCommand("Testing")).WhenReleased(frc2::PrintCommand("Released"));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return &m_AutonomousCommand;
}

void RobotContainer::PollInput () {
    if (m_OperatorJoystick.GetYButtonPressed()) {
        m_ExpelIntakeCommand.Schedule();
    } else if (m_OperatorJoystick.GetYButtonReleased()) {
        m_ExpelIntakeCommand.Cancel();
    }
    
    if (m_OperatorJoystick.GetBButtonPressed()) {
        m_IntakeBallsCommand.Schedule();
    } else if (m_OperatorJoystick.GetBButtonReleased()) {
        m_IntakeBallsCommand.Cancel();
    }

    if (static_cast<int>(Pov::kRight) == m_OperatorJoystick.GetPOV()) {
        m_ExtendIntakeCommand.Schedule();
    } else if (static_cast<int>(Pov::kLeft) == m_OperatorJoystick.GetPOV()) {
        m_RetractIntakeCommand.Schedule();
    }

    if (m_OperatorJoystick.GetAButtonPressed()) {
        m_Shooter.SetTracking(true);
        std::cout << "tracking" << std::endl;
    } else if (m_OperatorJoystick.GetAButtonReleased()) {
        m_Shooter.SetTracking(false);
    }

    if (m_DriverJoystick.GetAButton()) {
        m_Shooter.SetShooterMotorSpeed(3500_rpm);
    } else {
        m_Shooter.SetShooterMotorSpeed(0_rpm);
    }

    // double speed = m_OperatorJoystick.GetX(frc::XboxController::kLeftHand);
    // speed = fabs(speed) < 0.2 ? 0 : speed;
    // m_Shooter.SetTurretSpeed(speed * 10_rpm);
}
