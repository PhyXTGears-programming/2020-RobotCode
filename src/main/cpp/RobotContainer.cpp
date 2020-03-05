#include "RobotContainer.h"

#include <iostream>

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/PrintCommand.h>
#include <frc/Relay.h>
#include <units/units.h>

enum Pov {
    POV_RIGHT = 90,
    POV_LEFT = 270,
    POV_UP = 0,
    POV_DOWN = 180,
};

RobotContainer::RobotContainer() : m_AutonomousCommand(&m_Drivetrain) {
    frc2::CommandScheduler::GetInstance().SetDefaultCommand(&m_Drivetrain, m_TeleopDriveCommand);
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(&m_Shooter);
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(&m_Intake);

    m_ControlWinchCommand = new ControlWinchCommand(
        &m_Climb,
        [this] { return m_ClimbJoystick.GetY(frc::GenericHID::JoystickHand::kLeftHand); }
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

    // Expel Intake (DP Left)
    if (POV_LEFT == m_OperatorJoystick.GetPOV() && !m_ExpelIntakeCommand.IsScheduled()) {
        m_ExpelIntakeCommand.Schedule();
    } else if (POV_LEFT != m_OperatorJoystick.GetPOV() && m_ExpelIntakeCommand.IsScheduled()) {
        m_ExpelIntakeCommand.Cancel();
    }

    // Reverse Brushes (DP Right)
    // if (POV_RIGHT == m_OperatorJoystick.GetPOV() && !m_ReverseBrushesCommand.IsScheduled()) {
    //     m_ReverseBrushesCommand.Schedule();
    // } else if (POV_RIGHT != m_OperatorJoystick.GetPOV() && m_ReverseBrushesCommand.IsScheduled()) {
    //     m_ReverseBrushesCommand.Cancel();
    // }

    // Control Panel (LB, LT)
    // LB to deploy/retract
    // LT to spin wheel

    // Climb (RS, Start, Select)
    // Toggle climb piston
    if (m_OperatorJoystick.GetStartButtonPressed()) {
        if (m_Climb.IsPistonExtended()) {
            m_RetractClimbCommand.Schedule();
        } else {
            m_ExtendClimbCommand.Schedule();
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

    // ####################
    // #####  Climb   #####
    // ####################

    // Climb Winch (LS)
    if (std::abs(m_ClimbJoystick.GetY(frc::GenericHID::JoystickHand::kLeftHand)) > 0.2) {
        if (!m_ControlWinchCommand->IsScheduled()) m_ControlWinchCommand->Schedule(true); // interruptible
    } else {
        if (m_ControlWinchCommand->IsScheduled()) m_ControlWinchCommand->Cancel();
    }

    // Climb Roll (RS)
    double climbRoll = m_ClimbJoystick.GetX(frc::GenericHID::JoystickHand::kRightHand);
    if (climbRoll > 0.5) { // Right
        if (!m_RollClimbRightCommand.IsScheduled()) m_RollClimbRightCommand.Schedule();
    } else if (climbRoll < -0.5) { // Left
        if (!m_RollClimbLeftCommand.IsScheduled()) m_RollClimbLeftCommand.Schedule();
    } else {
        m_RollClimbLeftCommand.Cancel();
        m_RollClimbRightCommand.Cancel();
    }

    // Climb Lock Winch (B)
    if (m_ClimbJoystick.GetBButtonPressed()) {
        m_LockWinchCommand.Schedule();
    }

    // Climb Unlock Winch (Y)
    if (m_ClimbJoystick.GetYButtonPressed()) {
        m_UnlockWinchCommand.Schedule();
    }
    
    // Climb Cylinder Extend (A)
    if (m_ClimbJoystick.GetAButtonPressed()) {
        m_ClimbCylinderExtendCommand.Schedule();
    }

    // Climb Cylinder Retract (X)
    if (m_ClimbJoystick.GetXButtonPressed()) {
        m_ClimbCylinderRetractCommand.Schedule();
    }
}
