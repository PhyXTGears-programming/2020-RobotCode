#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Climb.h"

class ClimbCylinderExtendCommand : public frc2::CommandHelper<frc2::CommandBase, ClimbCylinderExtendCommand> {
    public:
        explicit ClimbCylinderExtendCommand (Climb* climb) { AddRequirements(climb); m_Climb = climb; }
        void Initialize () { m_Climb->PistonExtend(); }
        bool IsFinished () { return true; }

    private:
        Climb* m_Climb;
};
