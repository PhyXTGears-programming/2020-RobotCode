#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Climb.h"

class ClimbCylinderRetractCommand : public frc2::CommandHelper<frc2::CommandBase, ClimbCylinderRetractCommand> {
    public:
        explicit ClimbCylinderRetractCommand (Climb* climb) { AddRequirements(climb); m_Climb = climb; }
        void Initialize () { m_Climb->PistonRetract(); }
        bool IsFinished () { return true; }

    private:
        Climb* m_Climb;
};
