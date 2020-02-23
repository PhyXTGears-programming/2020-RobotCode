#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Shooter.h"

class ShootCommand : public frc2::CommandHelper<frc2::CommandBase, ShootCommand> {
    public:
        explicit ShootCommand(Shooter* shooter);
        void Initialize();
        void Execute();

    private:
        Shooter* m_Shooter;
};
