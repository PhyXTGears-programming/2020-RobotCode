#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/XboxController.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/PowerCellCounter.h"
#include "subsystems/Shooter.h"
#include "commands/AutonomousCommand.h"
#include "commands/TeleopDriveCommand.h"
#include "commands/ShootCommand.h"

#include "commands/AutonomousCommand.h"
#include "commands/IntakeBallsCommand.h"
#include "commands/ExpelIntakeCommand.h"
#include "commands/ExtendIntakeCommand.h"
#include "commands/RetractIntakeCommand.h"
#include "commands/ReverseBrushesCommand.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
    public:
        RobotContainer();

        frc2::Command* GetAutonomousCommand();

        void PollInput();

    private:
        void ConfigureButtonBindings();


        // Operators' input devices.
        // These are 0 indexed!
        frc::XboxController m_DriverJoystick{0};
        frc::XboxController m_OperatorJoystick{1};

        // The robot's subsystems and commands are defined here...
        PowerCellCounter m_PowerCellCounter {};
        Drivetrain m_Drivetrain {};
        Intake m_Intake {};
        Shooter m_Shooter {};

        AutonomousCommand m_AutonomousCommand {&m_Drivetrain, &m_Shooter};
        IntakeBallsCommand m_IntakeBallsCommand {&m_Intake, &m_PowerCellCounter};
        ExpelIntakeCommand m_ExpelIntakeCommand {&m_Intake};
        RetractIntakeCommand m_RetractIntakeCommand {&m_Intake};
        ExtendIntakeCommand m_ExtendIntakeCommand {&m_Intake};
        TeleopDriveCommand m_TeleopDriveCommand {&m_Drivetrain, &m_DriverJoystick};
        ShootCommand m_ShootCommand {&m_Shooter, &m_Intake};
        ReverseBrushesCommand m_ReverseBrushesCommand {&m_Intake};

        frc2::SequentialCommandGroup * m_SimpleAutoCommand;

        bool m_TurretManualControl = false; // Currently running manual control
        bool m_IntakeExtended = false;
};
