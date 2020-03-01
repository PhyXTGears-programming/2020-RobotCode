#include "RobotContainer.h"

#include "commands/AimCommand.h"
#include "commands/AimShootCommand.h"
#include "commands/SimpleDriveCommand.h"

#include <iostream>

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
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

    m_SimpleAutoCommand = new frc2::SequentialCommandGroup(
        SimpleDriveCommand{0.25, 0.0, &m_Drivetrain}.WithTimeout(1.0_s),
        AimCommand{&m_Shooter}.WithTimeout(5.0_s),
        AimShootCommand{&m_Shooter, &m_Intake}.WithTimeout(5.0_s)
    );

    // Configure the button bindings
    ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
    // frc2::JoystickButton aButton{&m_DriverJoystick, 1};
    // aButton.WhenPressed(m_VisionAimingCommand);
    // aButton.WhenPressed(frc2::PrintCommand("Testing")).WhenReleased(frc2::PrintCommand("Released"));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return m_SimpleAutoCommand;
}

void RobotContainer::PollInput () {
    using JoystickHand = frc::GenericHID::JoystickHand;

    // ####################
    // #####   Both   #####
    // ####################

    // Shooting (driver: RB, operator: A)
    if (m_DriverJoystick.GetAButtonPressed() || m_OperatorJoystick.GetAButtonPressed()) {
        m_ShootCommand.Schedule();
    } else if (m_DriverJoystick.GetAButtonReleased() || m_OperatorJoystick.GetAButtonReleased()) {
        m_ShootCommand.Cancel();
    }

    // Intake (driver: LB, operator: RT)
    bool intakeAxis = m_OperatorJoystick.GetTriggerAxis(JoystickHand::kRightHand) > 0.1;
    if (intakeAxis && !m_IntakeBallsCommand.IsScheduled()) {
        m_IntakeBallsCommand.Schedule();
    } else if (intakeAxis <= 0.1 && m_IntakeBallsCommand.IsScheduled()) {
        m_IntakeBallsCommand.Cancel();
    }

    // Deploy/Retract Intake (driver: X, operator: RB)
    if (m_DriverJoystick.GetXButtonPressed()
        || (m_OperatorJoystick.GetBumperPressed(JoystickHand::kRightHand) && !m_DriverJoystick.GetXButton())) {
        if (m_IntakeExtended) {
            m_RetractIntakeCommand.Schedule();
        } else {
            m_ExtendIntakeCommand.Schedule();
        }
        m_IntakeExtended = !m_IntakeExtended;
    }

    // Swap Camera (driver: A, operator: Y)

    // ####################
    // ##### Operator #####
    // ####################

    // Camera Aiming (X)
    if (m_OperatorJoystick.GetXButtonPressed()) {
        m_Shooter.SetTrackingMode(TrackingMode::CameraTracking);
    } else if (m_OperatorJoystick.GetXButtonReleased()) {
        m_Shooter.SetTrackingMode(TrackingMode::Off);
    }

    // Gyro Aiming (B)
    if (!m_OperatorJoystick.GetXButton()) {
        if (m_OperatorJoystick.GetBButtonPressed()) {
            m_Shooter.SetTrackingMode(TrackingMode::GyroTracking);
        } else if (m_OperatorJoystick.GetBButtonReleased()) {
            m_Shooter.SetTrackingMode(TrackingMode::Off);
        }
    }

    // Manual Aiming (LS)
    double operatorLeftX = m_OperatorJoystick.GetX(JoystickHand::kLeftHand);
    if (std::abs(operatorLeftX) > 0.1) {
        m_Shooter.SetTrackingMode(TrackingMode::Off);
        m_Shooter.SetTurretSpeed(operatorLeftX * 25_rpm);
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

    // Control Panel (LB, LT)
    // LB to deploy/retract
    // LT to spin wheel

    // Climb (RS)
    // Right stick click to toggle solenoids
    // Right stick X to move with motor
}
