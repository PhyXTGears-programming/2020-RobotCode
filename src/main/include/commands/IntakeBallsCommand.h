#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Intake.h"
#include "subsystems/PowerCellCounter.h"

class IntakeBallsCommand : public frc2::CommandHelper<frc2::CommandBase, IntakeBallsCommand> {
    public:
        IntakeBallsCommand(Intake* intake, PowerCellCounter* cellCounter);

        void Initialize();
        void Execute();
        void End(bool interrupted);
        bool IsFinished();

    private:
        Intake* m_Intake;
        PowerCellCounter* m_PowerCellCounter;
};
