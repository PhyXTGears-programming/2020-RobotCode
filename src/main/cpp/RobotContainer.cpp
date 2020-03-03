#include "RobotContainer.h"

#include "commands/AimCommand.h"
#include "commands/AimShootCommand.h"
#include "commands/SimpleDriveCommand.h"

#include <iostream>
#include <units/units.h>

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/StartEndCommand.h>

enum class Pov : int {
    Right = 90,
    Left = 270,
    Up = 180,
    Down = 0
};

RobotContainer::RobotContainer () {
    std::shared_ptr<cpptoml::table> toml = LoadConfig("/home/lvuser/deploy/" + ConfigFiles::ConfigFile);

    m_Drivetrain = new Drivetrain();
    m_Intake = new Intake(toml->get_table("intake"));
    m_Shooter = new Shooter(toml->get_table("shooter"));
    m_PowerCellCounter = new PowerCellCounter();

    m_AutonomousCommand     = new AutonomousCommand(m_Drivetrain);
    m_IntakeBallsCommand    = new IntakeBallsCommand(m_Intake, m_PowerCellCounter);
    m_ExpelIntakeCommand    = new ExpelIntakeCommand(m_Intake);
    m_RetractIntakeCommand  = new RetractIntakeCommand(m_Intake);
    m_ExtendIntakeCommand   = new ExtendIntakeCommand(m_Intake);
    m_TeleopDriveCommand    = new TeleopDriveCommand(m_Drivetrain, &m_DriverJoystick);
    m_ShootCommand          = new ShootCommand(m_Shooter, m_Intake);
    m_ReverseBrushesCommand = new ReverseBrushesCommand(m_Intake);

    frc2::CommandScheduler::GetInstance().SetDefaultCommand(m_Drivetrain, *m_TeleopDriveCommand);
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(m_Shooter);
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(m_Intake);
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(m_PowerCellCounter);

    m_ThreeCellAutoCommand = new frc2::SequentialCommandGroup(
        frc2::StartEndCommand {
            [=]() { m_Shooter->SetTurretSpeed(0.8); },
            [=]() { m_Shooter->SetTurretSpeed(0.0); },
            m_Shooter
        }.WithTimeout(0.5_s),
        AimCommand{m_Shooter}.WithTimeout(2.0_s),
        AimShootCommand{m_Shooter, m_Intake}.WithTimeout(3.5_s),
        SimpleDriveCommand{0.25, 0.0, m_Drivetrain}.WithTimeout(1.0_s)
    );

    // Configure the button bindings
    ConfigureButtonBindings();
}

frc2::Command* RobotContainer::GetAutonomousCommand () {
    return m_ThreeCellAutoCommand;
}

void RobotContainer::PollInput () {
    using JoystickHand = frc::GenericHID::JoystickHand;

    // ####################
    // #####  Driver  #####
    // ####################

    // Retract Intake (X)
    if (m_DriverJoystick.GetXButtonPressed()) {
        m_RetractIntakeCommand->Schedule();
        m_IntakeExtended = false;
    }

    // ####################
    // #####   Both   #####
    // ####################

    // Shooting (driver: RB, operator: A)
    if (m_DriverJoystick.GetAButtonPressed() || m_OperatorJoystick.GetAButtonPressed()) {
        m_ShootCommand->Schedule();
    } else if (m_DriverJoystick.GetAButtonReleased() || m_OperatorJoystick.GetAButtonReleased()) {
        m_ShootCommand->Cancel();
    }

    // Intake (driver: LB, operator: RT)
    bool intakeAxis = m_OperatorJoystick.GetTriggerAxis(JoystickHand::kRightHand) > 0.1;
    if (intakeAxis && !m_IntakeBallsCommand->IsScheduled()) {
        m_IntakeBallsCommand->Schedule();
    } else if (intakeAxis <= 0.1 && m_IntakeBallsCommand->IsScheduled()) {
        m_IntakeBallsCommand->Cancel();
    }

    // Swap Camera (driver: A, operator: Y)

    // ####################
    // ##### Operator #####
    // ####################

    // Camera Aiming (X)
    if (m_OperatorJoystick.GetXButtonPressed()) {
        m_Shooter->SetTrackingMode(TrackingMode::CameraTracking);
    } else if (m_OperatorJoystick.GetXButtonReleased()) {
        m_Shooter->SetTrackingMode(TrackingMode::Off);
    }

    // Gyro Aiming (B)
    if (!m_OperatorJoystick.GetXButton()) {
        if (m_OperatorJoystick.GetBButtonPressed()) {
            m_Shooter->SetTrackingMode(TrackingMode::GyroTracking);
        } else if (m_OperatorJoystick.GetBButtonReleased()) {
            m_Shooter->SetTrackingMode(TrackingMode::Off);
        }
    }

    // Manual Aiming (LS)
    double operatorLeftX = m_OperatorJoystick.GetX(JoystickHand::kLeftHand);
    if (std::abs(operatorLeftX) > 0.1) {
        m_Shooter->SetTrackingMode(TrackingMode::Off);
        m_Shooter->SetTurretSpeed(operatorLeftX * 25_rpm);
        m_TurretManualControl = true;
    } else if (m_TurretManualControl) {
        m_Shooter->SetTurretSpeed(0_rpm);
        m_TurretManualControl = false;
    }

    // Deploy/Retract Intake (RB)
    if (m_OperatorJoystick.GetBumperPressed(JoystickHand::kRightHand) && !m_DriverJoystick.GetXButton()) {
        if (m_IntakeExtended) {
            m_RetractIntakeCommand->Schedule();
        } else {
            m_ExtendIntakeCommand->Schedule();
        }
        m_IntakeExtended = !m_IntakeExtended;
    }

    // Expel Intake (DP Left)
    if (m_OperatorJoystick.GetPOV() == static_cast<int>(Pov::Left) && !m_ExpelIntakeCommand->IsScheduled()) {
        m_ExpelIntakeCommand->Schedule();
    } else if (m_OperatorJoystick.GetPOV() != static_cast<int>(Pov::Left) && m_ExpelIntakeCommand->IsScheduled()) {
        m_ExpelIntakeCommand->Cancel();
    }

    // Reverse Brushes (DP Right)
    if (m_OperatorJoystick.GetPOV() == static_cast<int>(Pov::Right) && !m_ReverseBrushesCommand->IsScheduled()) {
        m_ReverseBrushesCommand->Schedule();
    } else if (m_OperatorJoystick.GetPOV() != static_cast<int>(Pov::Right) && m_ReverseBrushesCommand->IsScheduled()) {
        m_ReverseBrushesCommand->Cancel();
    }

    // Control Panel (LB, LT)
    // LB to deploy/retract
    // LT to spin wheel

    // Climb (RS)
    // Right stick click to toggle solenoids
    // Right stick X to move with motor
}

void RobotContainer::ConfigureButtonBindings () {
    // frc2::JoystickButton aButton{&m_DriverJoystick, 1};
    // aButton.WhenPressed(m_VisionAimingCommand);
    // aButton.WhenPressed(frc2::PrintCommand("Testing")).WhenReleased(frc2::PrintCommand("Released"));
}

std::shared_ptr<cpptoml::table> RobotContainer::LoadConfig (std::string path) {
    try {
        return cpptoml::parse_file(path);
    } catch (const cpptoml::parse_exception & ex) {
        std::cerr << "Unable to load config file: " << path << std::endl << ex.what() << std::endl;
        exit(1);
    }
}
