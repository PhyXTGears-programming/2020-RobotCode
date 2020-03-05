#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Climb.h"

class RollClimbLeftCommand : public frc2::CommandHelper<frc2::CommandBase, RollClimbLeftCommand> {
    public:
        RollClimbLeftCommand(Climb* Climb);

        void Initialize();
        void Execute();
        void End(bool interrupted);
        bool IsFinished();

    private:
        Climb* m_Climb;
};
