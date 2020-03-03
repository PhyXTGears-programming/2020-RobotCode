#pragma once

#include <cpptoml.h>

#include <frc2/command/Command.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/XboxController.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/PowerCellCounter.h"
#include "subsystems/Shooter.h"

#include "commands/AutonomousCommand.h"
#include "commands/ExpelIntakeCommand.h"
#include "commands/ExtendIntakeCommand.h"
#include "commands/IntakeBallsCommand.h"
#include "commands/RetractIntakeCommand.h"
#include "commands/ReverseBrushesCommand.h"
#include "commands/ShootCommand.h"
#include "commands/TeleopDriveCommand.h"

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

        std::shared_ptr<cpptoml::table> LoadConfig(std::string path);

        // Operators' input devices.
        // These are 0 indexed!
        frc::XboxController m_DriverJoystick{0};
        frc::XboxController m_OperatorJoystick{1};

        // The robot's subsystems and commands are defined here...
        Drivetrain* m_Drivetrain;
        Intake* m_Intake;
        Shooter* m_Shooter;
        PowerCellCounter* m_PowerCellCounter;

        AutonomousCommand* m_AutonomousCommand;
        IntakeBallsCommand* m_IntakeBallsCommand;
        ExpelIntakeCommand* m_ExpelIntakeCommand;
        RetractIntakeCommand* m_RetractIntakeCommand;
        ExtendIntakeCommand* m_ExtendIntakeCommand;
        TeleopDriveCommand* m_TeleopDriveCommand;
        ShootCommand* m_ShootCommand;
        ReverseBrushesCommand* m_ReverseBrushesCommand;

        frc2::SequentialCommandGroup * m_SimpleAutoCommand;

        bool m_TurretManualControl = false; // Currently running manual control
        bool m_IntakeExtended = false;

    friend class Robot;
};
