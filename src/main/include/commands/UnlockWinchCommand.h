#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Climb.h"

class UnlockWinchCommand : public frc2::CommandHelper<frc2::CommandBase, UnlockWinchCommand> {
    public:
        explicit UnlockWinchCommand (Climb* climb) { AddRequirements(climb); m_Climb = climb; }
        void Initialize () { m_Climb->WinchUnlock(); }
        bool IsFinished () { return true; }

    private:
        Climb* m_Climb;
};
