#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Shooter.h"
#include "kinematics/GenerateTrajectoryFollower.h"

class AutonomousCommand : public frc2::CommandHelper<frc2::CommandBase, AutonomousCommand> {
    public:
        /**
         * Creates a new ExampleCommand.
         *
         * @param subsystem The subsystem used by this command.
         */
        explicit AutonomousCommand(Drivetrain* drivetrain, Shooter* shooter);
        void Initialize();
        void Execute();
        bool IsFinished();

    private:
        Drivetrain* m_Drivetrain;
        Shooter* m_Shooter;
        GenerateTrajectoryFollower* m_GenerateTrajectoryFollower;
};
