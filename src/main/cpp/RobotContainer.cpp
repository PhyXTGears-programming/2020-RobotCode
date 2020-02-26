#include "RobotContainer.h"

#include <iostream>

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/PrintCommand.h>
#include <units/units.h>

enum class Pov : int {
    Right = 90,
    Left = 270,
    Up = 180,
    Down = 0
};

RobotContainer::RobotContainer() : m_AutonomousCommand(&m_Drivetrain) {
    frc2::CommandScheduler::GetInstance().SetDefaultCommand(&m_Drivetrain, m_TeleopDriveCommand);
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(&m_Shooter);
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(&m_Intake);

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
    // ####################
    // #####  Driver  #####
    // ####################

    // Shooting (A [operator: B])
    if (m_DriverJoystick.GetAButtonPressed() || m_OperatorJoystick.GetBButtonPressed()) {
        m_ShootCommand.Schedule();
    } else if (m_DriverJoystick.GetAButtonReleased() || m_OperatorJoystick.GetBButtonReleased()) {
        m_ShootCommand.Cancel();
    }

    // ####################
    // ##### Operator #####
    // ####################

    // Camera Aiming (A)
    if (m_OperatorJoystick.GetAButtonPressed()) {
        m_Shooter.SetTrackingMode(TrackingMode::CameraTracking);
    } else if (m_OperatorJoystick.GetAButtonReleased()) {
        m_Shooter.SetTrackingMode(TrackingMode::Off);
    }

    // Gyro Aiming (X)
    if (!m_OperatorJoystick.GetAButton()) {
        if (m_OperatorJoystick.GetXButtonPressed()) {
            m_Shooter.SetTrackingMode(TrackingMode::GyroTracking);
        } else if (m_OperatorJoystick.GetXButtonReleased()) {
            m_Shooter.SetTrackingMode(TrackingMode::Off);
        }
    }

    // Manual Aiming (LS)
    if (fabs(m_OperatorJoystick.GetX(frc::GenericHID::JoystickHand::kLeftHand)) > 0.1) {
        m_Shooter.SetTrackingMode(TrackingMode::Off);
        m_Shooter.SetTurretSpeed(std::copysign(1, m_OperatorJoystick.GetX(frc::GenericHID::JoystickHand::kLeftHand)) * 20_rpm);
        m_TurretManualControl = true;
    } else if (m_TurretManualControl) {
        m_Shooter.SetTurretSpeed(0_rpm);
        m_TurretManualControl = false;
    }

    // Intake (RT)
    if (m_OperatorJoystick.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand) > 0.1 && !m_IntakeBallsCommand.IsScheduled()) {
        m_IntakeBallsCommand.Schedule();
    } else if (m_OperatorJoystick.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand) <= 0.1 && m_IntakeBallsCommand.IsScheduled()) {
        m_IntakeBallsCommand.Cancel();
    }

    // Expel Intake (DP Left)
    if (m_OperatorJoystick.GetPOV() == static_cast<int>(Pov::Left) && !m_ExpelIntakeCommand.IsScheduled()) {
        m_ExpelIntakeCommand.Schedule();
    } else if (m_OperatorJoystick.GetPOV() != static_cast<int>(Pov::Left) && m_ExpelIntakeCommand.IsScheduled()) {
        m_ExpelIntakeCommand.Cancel();
    }

    // Deploy/Retract Intake (LB/RB)
    if (m_OperatorJoystick.GetBumperPressed(frc::GenericHID::JoystickHand::kRightHand)) {
        m_ExtendIntakeCommand.Schedule();
    } else if (m_OperatorJoystick.GetBumperPressed(frc::GenericHID::JoystickHand::kLeftHand)) {
        m_RetractIntakeCommand.Schedule();
    }

    // Control Panel (Y)

    // Climb (Back/Start)
}
