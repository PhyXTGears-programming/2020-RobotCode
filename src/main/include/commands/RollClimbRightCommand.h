#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Climb.h"

class RollClimbRightCommand : public frc2::CommandHelper<frc2::CommandBase, RollClimbRightCommand> {
    public:
        RollClimbRightCommand(Climb* Climb);

        void Initialize();
        void Execute();
        void End(bool interrupted);
        bool IsFinished();

    private:
        Climb* m_Climb;
};
