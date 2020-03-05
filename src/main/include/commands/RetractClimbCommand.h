#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Climb.h"

class RetractClimbCommand : public frc2::CommandHelper<frc2::CommandBase, RetractClimbCommand> {
    public:
        RetractClimbCommand(Climb* Climb);

        void Initialize();
        void Execute();
        void End(bool interrupted);
        bool IsFinished();

    private:
        Climb* m_Climb;
};
