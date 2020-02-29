#include "RobotContainer.h"

#include <iostream>

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/PrintCommand.h>
#include <frc/Relay.h>
#include <units/units.h>

RobotContainer::RobotContainer() {
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

    // Shooting (driver: RB, operator: A)
    if (m_DriverJoystick.GetAButtonPressed() || m_OperatorJoystick.GetAButtonPressed()) {
        m_ShootCommand.Schedule();
    } else if (m_DriverJoystick.GetAButtonReleased() || m_OperatorJoystick.GetAButtonReleased()) {
        m_ShootCommand.Cancel();
    }

    // Intake (driver: LB, operator: RT)
    bool intakeAxis = m_OperatorJoystick.GetTriggerAxis(frc::GenericHID::JoystickHand::kRightHand) > 0.1;
    if (intakeAxis && !m_IntakeBallsCommand.IsScheduled()) {
        m_IntakeBallsCommand.Schedule();
    } else if (intakeAxis <= 0.1 && m_IntakeBallsCommand.IsScheduled()) {
        m_IntakeBallsCommand.Cancel();
    }

    // Deploy/Retract Intake (driver: X, operator: RB)
    if (m_DriverJoystick.GetXButtonPressed() || m_OperatorJoystick.GetBumperPressed(frc::GenericHID::JoystickHand::kRightHand)) {
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
    double operatorLeftX = m_OperatorJoystick.GetX(frc::GenericHID::JoystickHand::kLeftHand);
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
    // if (m_OperatorJoystick.GetPOV() == static_cast<int>(Pov::Right) && !m_ReverseBrushesCommand.IsScheduled()) {
    //     m_ReverseBrushesCommand.Schedule();
    // } else if (m_OperatorJoystick.GetPOV() != static_cast<int>(Pov::Right) && m_ReverseBrushesCommand.IsScheduled()) {
    //     m_ReverseBrushesCommand.Cancel();
    // }

    // Control Panel (LB, LT)
    // LB to deploy/retract
    // LT to spin wheel

    // Climb (RS)
    // Toggle Climb mode
    if (m_OperatorJoystick.GetStickButtonPressed(frc::GenericHID::JoystickHand::kRightHand)) {
        if (m_ClimbMode == ClimbMode::WinchMode) {
            m_ClimbMode = ClimbMode::BarMode;
        } else {
            m_ClimbMode = ClimbMode::WinchMode;
        }
    }
    
    if (m_ClimbMode == ClimbMode::WinchMode) { // Move mechanism up and down
        double operatorRightY = m_OperatorJoystick.GetY(frc::GenericHID::JoystickHand::kRightHand);
        if (std::abs(operatorRightY) > 0.5) {
            if (operatorRightY > 0) {
                m_ExtendClimbCommand.Schedule();
            } else if (operatorRightY < 0) {
                m_RetractClimbCommand.Schedule();
            }
        } else {
            if (m_ExtendClimbCommand.IsScheduled()) m_ExtendClimbCommand.Cancel();
            if (m_RetractClimbCommand.IsScheduled()) m_RetractClimbCommand.Cancel();
        }
    } else if (m_ClimbMode == ClimbMode::BarMode) { // Move robot left and right on the bar
        double operatorRightX = m_OperatorJoystick.GetX(frc::GenericHID::JoystickHand::kRightHand);
        if (std::abs(operatorRightX) > 0.5) {
            // Turn into a command later?
            if (operatorRightX > 0) { // Right
                m_Climb.SetRelay(frc::Relay::Value::kForward);
            } else if (operatorRightX < 0) { // Left
                m_Climb.SetRelay(frc::Relay::Value::kReverse);
            }
        }
    }
}
