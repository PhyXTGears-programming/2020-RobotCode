#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Shooter.h"

class AimCommand : public frc2::CommandHelper<frc2::CommandBase, AimCommand> {
    public:
        explicit AimCommand(Shooter* shooter);
        void Initialize();
        void Execute();
        void End(bool interrupted);
        bool IsFinished();

    private:
        Shooter* m_Shooter;
};
