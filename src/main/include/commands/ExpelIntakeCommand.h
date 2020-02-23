#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Intake.h"

class ExpelIntakeCommand : public frc2::CommandHelper<frc2::CommandBase, ExpelIntakeCommand> {
    public:
        ExpelIntakeCommand(Intake* intake);

        void Initialize();
        void Execute();
        void End();
        bool IsFinished();

    private:
        Intake* m_Intake;
};
