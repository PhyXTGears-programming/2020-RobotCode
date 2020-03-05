#pragma once

#include <frc2/command/Command.h>
#include <frc/XboxController.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Shooter.h"
#include "commands/AutonomousCommand.h"
#include "commands/TeleopDriveCommand.h"
#include "commands/ShootCommand.h"

#include "commands/AutonomousCommand.h"
#include "commands/IntakeBallsCommand.h"
#include "commands/ExpelIntakeCommand.h"
#include "commands/ExtendIntakeCommand.h"
#include "commands/ExtendClimbCommand.h"
#include "commands/RetractIntakeCommand.h"
#include "commands/RetractClimbCommand.h"
#include "commands/ReverseBrushesCommand.h"
#include "commands/RollClimbLeftCommand.h"
#include "commands/RollClimbRightCommand.h"
#include "commands/ControlWinchCommand.h"
#include "commands/LockWinchCommand.h"
#include "commands/UnlockWinchCommand.h"
#include "commands/ClimbCylinderExtendCommand.h"
#include "commands/ClimbCylinderRetractCommand.h"

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
        frc::XboxController m_ClimbJoystick{2};

        // The robot's subsystems and commands are defined here...
        Drivetrain m_Drivetrain {};
        Intake m_Intake {};
        Climb m_Climb {};
        Shooter m_Shooter {};

        AutonomousCommand m_AutonomousCommand {&m_Drivetrain};
        IntakeBallsCommand m_IntakeBallsCommand {&m_Intake};
        ExpelIntakeCommand m_ExpelIntakeCommand {&m_Intake};
        RetractIntakeCommand m_RetractIntakeCommand {&m_Intake};
        RetractClimbCommand m_RetractClimbCommand {&m_Climb};
        ExtendIntakeCommand m_ExtendIntakeCommand {&m_Intake};
        ExtendClimbCommand m_ExtendClimbCommand {&m_Climb};
        TeleopDriveCommand m_TeleopDriveCommand {&m_Drivetrain, &m_DriverJoystick};
        ShootCommand m_ShootCommand {&m_Shooter, &m_Intake};
        RollClimbLeftCommand m_RollClimbLeftCommand {&m_Climb};
        RollClimbRightCommand m_RollClimbRightCommand {&m_Climb};
        LockWinchCommand m_LockWinchCommand {&m_Climb};
        UnlockWinchCommand m_UnlockWinchCommand {&m_Climb};
        ClimbCylinderExtendCommand m_ClimbCylinderExtendCommand {&m_Climb};
        ClimbCylinderRetractCommand m_ClimbCylinderRetractCommand {&m_Climb};
        
        ControlWinchCommand* m_ControlWinchCommand;

        bool m_TurretManualControl = false; // Currently running manual control
        bool m_IntakeExtended = false;
};
