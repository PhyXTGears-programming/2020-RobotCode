#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Climb.h"

class LockWinchCommand : public frc2::CommandHelper<frc2::CommandBase, LockWinchCommand> {
    public:
        explicit LockWinchCommand (Climb* climb) { AddRequirements(climb); m_Climb = climb; }
        void Initialize () { m_Climb->WinchLock(); }
        bool IsFinished () { return true; }

    private:
        Climb* m_Climb;
};
