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
    // #####   Both   #####
    // ####################

    // Shooting (driver: RB, operator: B)
    if (m_DriverJoystick.GetAButtonPressed() || m_OperatorJoystick.GetBButtonPressed()) {
        m_ShootCommand.Schedule();
    } else if (m_DriverJoystick.GetAButtonReleased() || m_OperatorJoystick.GetBButtonReleased()) {
        m_ShootCommand.Cancel();
    }

    // Intake (driver: LB, operator: RT)
    bool intakeAxis = m_OperatorJoystick.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand)) > 0.1;
    if (intakeAxis && !m_IntakeBallsCommand.IsScheduled()) {
        m_IntakeBallsCommand.Schedule();
    } else if (intakeAxis <= 0.1 && m_IntakeBallsCommand.IsScheduled()) {
        m_IntakeBallsCommand.Cancel();
    }

    // Deploy/Retract Intake (driver: X, operator: LB)
    if (m_DriverJoystick.GetXButtonPressed() || m_OperatorJoystick.GetBumperPressed(frc::GenericHID::JoystickHand::kLeftHand)) {
        if (m_IntakeExtended) {
            m_RetractIntakeCommand.Schedule();
        } else {
            m_ExtendIntakeCommand.Schedule();
        }
        m_IntakeExtended = !m_IntakeExtended;
    }

    // Swap Camera (driver: A, operator: RB)

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

    // Expel Intake (DP Left)
    if (m_OperatorJoystick.GetPOV() == static_cast<int>(Pov::Left) && !m_ExpelIntakeCommand.IsScheduled()) {
        m_ExpelIntakeCommand.Schedule();
    } else if (m_OperatorJoystick.GetPOV() != static_cast<int>(Pov::Left) && m_ExpelIntakeCommand.IsScheduled()) {
        m_ExpelIntakeCommand.Cancel();
    }

    // Reverse Brushes (DP Right)
    if (m_OperatorJoystick.GetPOV() == static_cast<int>(Pov::Right) && !m_ReverseBrushesCommand.IsScheduled()) {
        m_ReverseBrushesCommand.Schedule();
    } else if (m_OperatorJoystick.GetPOV() != static_cast<int>(Pov::Right) && m_ReverseBrushesCommand.IsScheduled()) {
        m_ReverseBrushesCommand.Cancel();
    }

    // Control Panel (Y)

    // Climb (Back/Start)
}
