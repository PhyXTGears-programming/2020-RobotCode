#include "RobotContainer.h"

#include <iostream>
#include <units/units.h>

#include <cameraserver/CameraServer.h>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PrintCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/StartEndCommand.h>
#include <frc2/command/WaitUntilCommand.h>

#include "commands/AimCommand.h"
#include "commands/AimShootCommand.h"
#include "commands/SimpleDriveCommand.h"
#include "commands/autonomous/AutonomousRotateTurretCommand.h"
#include "commands/DriveUntilWallCommand.h"

using JoystickHand = frc::GenericHID::JoystickHand;

enum Pov {
    POV_RIGHT = 90,
    POV_LEFT = 270,
    POV_UP = 0,
    POV_DOWN = 180,
};

RobotContainer::RobotContainer () {
    std::shared_ptr<cpptoml::table> toml = LoadConfig("/home/lvuser/deploy/" + ConfigFiles::ConfigFile);

    m_Drivetrain = new Drivetrain();
    m_Intake = new Intake(toml->get_table("intake"));
    m_Climb = new Climb();
    m_Shooter = new Shooter(toml->get_table("shooter"));
    m_ControlPanel = new ControlPanel(toml->get_table("controlPanel"));
    m_PowerCellCounter = new PowerCellCounter();

    m_AutonomousCommand     = new AutonomousCommand(m_Drivetrain);
    m_IntakeBallsCommand    = new IntakeBallsCommand(m_Intake, m_PowerCellCounter);
    m_ExpelIntakeCommand    = new ExpelIntakeCommand(m_Intake);
    m_RetractIntakeCommand  = new RetractIntakeCommand(m_Intake);
    m_ExtendIntakeCommand   = new ExtendIntakeCommand(m_Intake);
    m_TeleopDriveCommand    = new TeleopDriveCommand(m_Drivetrain, &m_DriverJoystick);
    m_TeleopShootCommand    = new ShootCommand(m_Shooter, m_Intake, 4500_rpm);
    m_ReverseBrushesCommand = new ReverseBrushesCommand(m_Intake);

    m_ControlWinchCommand   = new ControlWinchCommand(m_Climb, [=] { return m_ClimbJoystick.GetY(JoystickHand::kLeftHand); });
    m_RetractClimbCommand   = new RetractClimbCommand(m_Climb);
    m_ExtendClimbCommand    = new ExtendClimbCommand(m_Climb);
    m_RollClimbLeftCommand  = new RollClimbLeftCommand(m_Climb);
    m_RollClimbRightCommand = new RollClimbRightCommand(m_Climb);
    m_LockWinchCommand      = new LockWinchCommand(m_Climb);
    m_UnlockWinchCommand    = new UnlockWinchCommand(m_Climb);

    m_ClimbCylinderExtendCommand  = new ClimbCylinderExtendCommand(m_Climb);
    m_ClimbCylinderRetractCommand = new ClimbCylinderRetractCommand(m_Climb);

    frc2::CommandScheduler::GetInstance().SetDefaultCommand(m_Drivetrain, *m_TeleopDriveCommand);
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(m_Shooter);
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(m_Intake);
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(m_PowerCellCounter);

    InitAutonomousChooser();
    frc::SmartDashboard::PutData("Auto Modes", &m_DashboardAutoChooser);

    // Configure the button bindings
    ConfigureButtonBindings();

    auto cameraServer = frc::CameraServer::GetInstance();
    cameraServer->StartAutomaticCapture();
}

frc2::Command* RobotContainer::GetAutonomousCommand () {
    return m_DashboardAutoChooser.GetSelected();
}

void RobotContainer::PollInput () {
    // ####################
    // #####  Driver  #####
    // ####################

    // Retract Intake (X)
    if (m_DriverJoystick.GetXButtonPressed()) {
        m_RetractIntakeCommand->Schedule();
        m_IntakeExtended = false;
    }

    // Brake (RB)
    if (m_DriverJoystick.GetBumperPressed(JoystickHand::kRightHand)) {
        m_Drivetrain->SetBrake(true);
    } else if (m_DriverJoystick.GetBumperReleased(JoystickHand::kRightHand)) {
        m_Drivetrain->SetBrake(false);
    }

    // ####################
    // #####   Both   #####
    // ####################

    // Shooting (driver: RB, operator: A)
    if (m_DriverJoystick.GetAButtonPressed() || m_OperatorJoystick.GetAButtonPressed()) {
        m_TeleopShootCommand->Schedule();
    } else if (m_DriverJoystick.GetAButtonReleased() || m_OperatorJoystick.GetAButtonReleased()) {
        m_TeleopShootCommand->Cancel();
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

    // Control Panel Deploy (LB)
    if (m_OperatorJoystick.GetBumperPressed(JoystickHand::kLeftHand)) {
        m_ControlPanel->Extend();
    } else if (m_OperatorJoystick.GetBumperReleased(JoystickHand::kLeftHand)) {
        m_ControlPanel->Retract();
    }

    // Left Stick
    double operatorLeftX = m_OperatorJoystick.GetX(JoystickHand::kLeftHand);
    if (m_OperatorJoystick.GetBumper(JoystickHand::kLeftHand)) {
        // Control Panel Manual Control
        m_ControlPanel->SetSpeed(operatorLeftX);
    } else {
        // Manual Aiming
        if (std::abs(operatorLeftX) > 0.1) {
            m_Shooter->SetTrackingMode(TrackingMode::Off);
            m_Shooter->SetTurretSpeed(operatorLeftX * 25_rpm);
            m_TurretManualControl = true;
        } else if (m_TurretManualControl) {
            m_Shooter->SetTurretSpeed(0_rpm);
            m_TurretManualControl = false;
        }
    }

    // Deploy/Retract Intake (RB)
    if (m_OperatorJoystick.GetBumperPressed(JoystickHand::kRightHand) && !m_DriverJoystick.GetXButton()) {
        if (m_Intake->IsExtended()) {
            m_RetractIntakeCommand->Schedule();
        } else {
            m_ExtendIntakeCommand->Schedule();
        }
    }

    // Expel Intake (DP Left)
    if (POV_LEFT == m_OperatorJoystick.GetPOV() && !m_ExpelIntakeCommand->IsScheduled()) {
        m_ExpelIntakeCommand->Schedule();
    } else if (POV_LEFT != m_OperatorJoystick.GetPOV() && m_ExpelIntakeCommand->IsScheduled()) {
        m_ExpelIntakeCommand->Cancel();
    }

    // Reverse Brushes (DP Right)
    if (POV_RIGHT == m_OperatorJoystick.GetPOV() && !m_ReverseBrushesCommand->IsScheduled()) {
        m_ReverseBrushesCommand->Schedule();
    } else if (POV_RIGHT != m_OperatorJoystick.GetPOV() && m_ReverseBrushesCommand->IsScheduled()) {
        m_ReverseBrushesCommand->Cancel();
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

    // Climb Roll (Driver Dpad)
    if (m_DriverJoystick.GetPOV() == POV_RIGHT) { // Right
        if (!m_RollClimbRightCommand->IsScheduled()) {
            m_RollClimbRightCommand->Schedule();
        }
    } else if (m_DriverJoystick.GetPOV() == POV_LEFT) { // Left
        if (!m_RollClimbLeftCommand->IsScheduled()) {
            m_RollClimbLeftCommand->Schedule();
        }
    } else {
        m_RollClimbLeftCommand->Cancel();
        m_RollClimbRightCommand->Cancel();
    }

    // Climb Lock Winch (B)
    if (m_ClimbJoystick.GetBButtonPressed()) {
        m_LockWinchCommand->Schedule();
    }

    // Climb Unlock Winch (Y)
    if (m_ClimbJoystick.GetYButtonPressed()) {
        m_UnlockWinchCommand->Schedule();
    }

    // Climb Cylinder Extend (A)
    if (m_ClimbJoystick.GetAButtonPressed()) {
        m_ClimbCylinderExtendCommand->Schedule();
    }

    // Climb Cylinder Retract (X)
    if (m_ClimbJoystick.GetXButtonPressed()) {
        m_ClimbCylinderRetractCommand->Schedule();
    }
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

void RobotContainer::InitAutonomousChooser () {
    frc2::SequentialCommandGroup* threeCellAutoCommand = new frc2::SequentialCommandGroup(
        frc2::StartEndCommand {
            [=]() { m_Shooter->SetTurretSpeed(0.8); },
            [=]() { m_Shooter->SetTurretSpeed(0.0); },
            m_Shooter
        }.WithTimeout(0.5_s),
        AimCommand{m_Shooter}.WithTimeout(2.0_s),
        AimShootCommand{m_Shooter, m_Intake, m_PowerCellCounter}.WithTimeout(3.5_s),
        SimpleDriveCommand{0.25, 0.0, m_Drivetrain}.WithTimeout(1.0_s)
    );

    frc2::SequentialCommandGroup driveThruTrench {
        // Drive thru trench picking up power cells.
        frc2::ParallelCommandGroup{
            frc2::SequentialCommandGroup{
                SimpleDriveCommand{0.6, 0.0, m_Drivetrain}.WithTimeout(1.6_s),
                // Decelerate.
                SimpleDriveCommand{0.4, 0.0, m_Drivetrain}.WithTimeout(0.3_s),
                SimpleDriveCommand{0.2, 0.0, m_Drivetrain}.WithTimeout(0.3_s)
            },
            // Run intake until 3 cells are collected, or timeout expires.
            // frc2::ParallelRaceGroup{
            IntakeBallsCommand{m_Intake, m_PowerCellCounter}.WithTimeout(2.5_s)
            //     frc2::WaitUntilCommand{
            //         [=]() { return 3 == m_PowerCellCounter->GetCount(); }
            //     }
            // }
        },
        // Reverse back to line.
        frc2::ParallelRaceGroup{
            SimpleDriveCommand{-0.6, 0.0, m_Drivetrain}.WithTimeout(1.6_s),
            IntakeBallsCommand{m_Intake, m_PowerCellCounter}
        },
        // Decelerate.
        SimpleDriveCommand{-0.4, 0.0, m_Drivetrain}.WithTimeout(0.3_s),
        SimpleDriveCommand{-0.2, 0.0, m_Drivetrain}.WithTimeout(0.4_s)
    };

    static hal::fpga_clock::time_point startTime;

    frc2::SequentialCommandGroup* sixCellAutoCommand = new frc2::SequentialCommandGroup(
        frc2::InstantCommand{
            [=]() {
                startTime = hal::fpga_clock::now();
                m_Shooter->SetLimelightLight(true);
            }
        },
        ExtendIntakeCommand{m_Intake},
        PreheatShooterCommand{m_Shooter},
        AutonomousRotateTurretCommand{m_Shooter}.WithTimeout(0.3_s),
        AimCommand{m_Shooter}.WithTimeout(1.0_s),
        AimShootCommand{m_Shooter, m_Intake, m_PowerCellCounter}.WithTimeout(4.0_s),
        std::move(driveThruTrench),
        PreheatShooterCommand{m_Shooter},
        AimCommand{m_Shooter}.WithTimeout(0.5_s),
        AimShootCommand{m_Shooter, m_Intake, m_PowerCellCounter}.WithTimeout(4.0_s),
        // RetractIntakeCommand{m_Intake},
        frc2::InstantCommand{
            [=] {
                auto now = hal::fpga_clock::now();
                auto delta = std::chrono::duration_cast<std::chrono::microseconds>(now - startTime).count() / 1.0E6;
                std::cout << "Auto done in " << delta << " seconds" << std::endl;
            }
        }
    );

    frc2::SequentialCommandGroup positionBot {
        SimpleDriveCommand{-0.4, 0.0, m_Drivetrain}.WithTimeout(1.0_s),
        DriveUntilWallCommand{m_Drivetrain},
        SimpleDriveCommand{0.1, 0.0, m_Drivetrain}.WithTimeout(0.1_s)
    };

    frc2::SequentialCommandGroup* closeShotAutoCommand = new frc2::SequentialCommandGroup(
        AutonomousRotateTurretCommand{m_Shooter}.WithTimeout(0.5_s),
        AimCommand{m_Shooter}.WithTimeout(1.0_s),
        PreheatShooterCommand{m_Shooter},
        std::move(positionBot),
        ShootCommand{m_Shooter, m_Intake, 2700_rpm}.WithTimeout(4.0_s)
    );

    m_DashboardAutoChooser.SetDefaultOption("3 cell auto", threeCellAutoCommand);
    m_DashboardAutoChooser.AddOption("6 cell auto", sixCellAutoCommand);
    m_DashboardAutoChooser.AddOption("close auto", closeShotAutoCommand);
}

void RobotContainer::ReportSelectedAuto () {
    std::string name;

    if (m_DashboardAutoChooser.HasSelected()) {
        name = m_DashboardAutoChooser.GetSelectedName();
    } else {
        name = m_DashboardAutoChooser.GetDefaultName();
    }

    frc::SmartDashboard::PutString("Robot sees autonomous", name);
}
