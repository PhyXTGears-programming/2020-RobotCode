#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"

class PreheatShooterCommand : public frc2::CommandHelper<frc2::CommandBase, PreheatShooterCommand> {
    public:
        explicit PreheatShooterCommand(Shooter* shooter);
        void Initialize();
        void Execute();
        void End(bool interrupted);
        bool IsFinished();

    private:
        Shooter* m_Shooter;
};
