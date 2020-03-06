#pragma once

#include <cpptoml.h>

#include <frc2/command/Command.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/XboxController.h>

#include "subsystems/Climb.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/PowerCellCounter.h"
#include "subsystems/Shooter.h"

#include "commands/AutonomousCommand.h"
#include "commands/ExpelIntakeCommand.h"
#include "commands/ExtendIntakeCommand.h"
#include "commands/IntakeBallsCommand.h"
#include "commands/PreheatShooterCommand.h"
#include "commands/RetractIntakeCommand.h"
#include "commands/ReverseBrushesCommand.h"
#include "commands/ShootCommand.h"
#include "commands/TeleopDriveCommand.h"

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

        std::shared_ptr<cpptoml::table> LoadConfig(std::string path);

        void InitAutonomousChooser();

        // Operators' input devices.
        // These are 0 indexed!
        frc::XboxController m_DriverJoystick{0};
        frc::XboxController m_OperatorJoystick{1};
        frc::XboxController m_ClimbJoystick{2};

        // The robot's subsystems and commands are defined here...
        Drivetrain* m_Drivetrain;
        Intake* m_Intake;
        Climb* m_Climb;
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

        RetractClimbCommand* m_RetractClimbCommand;
        ExtendClimbCommand* m_ExtendClimbCommand;
        RollClimbLeftCommand* m_RollClimbLeftCommand;
        RollClimbRightCommand* m_RollClimbRightCommand;
        LockWinchCommand* m_LockWinchCommand;
        UnlockWinchCommand* m_UnlockWinchCommand;
        ClimbCylinderExtendCommand* m_ClimbCylinderExtendCommand;
        ClimbCylinderRetractCommand* m_ClimbCylinderRetractCommand;
        
        ControlWinchCommand* m_ControlWinchCommand;

        frc::SendableChooser<frc2::Command *> m_DashboardAutoChooser;

        bool m_TurretManualControl = false; // Currently running manual control
        bool m_IntakeExtended = false;

    friend class Robot;
};
