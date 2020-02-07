#include "RobotContainer.h"

#include <iostream>

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/PrintCommand.h>
#include <units/units.h>

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
        m_Shooter.SetShooterMotorSpeeds(0, 0);
    }
}

void RobotContainer::PollInput () {
    // This works, but JoystickButton does not.

    if (m_DriverJoystick.GetRawButtonPressed(1)) {
        m_VisionAimingCommand.Schedule();
    } else if (m_DriverJoystick.GetRawButtonReleased(1)) {
        m_VisionAimingCommand.Cancel();
    }

    double operatorXInput = m_OperatorJoystick.GetX(frc::XboxController::kLeftHand);
    if (fabs(operatorXInput) > 0.07) {
        units::angular_velocity::revolutions_per_minute_t turretSpeed {operatorXInput * 20};
        m_Shooter.SetTurretSpeed(turretSpeed);
    }
}
