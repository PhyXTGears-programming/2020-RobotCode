#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Intake.h"

class RetractIntakeCommand : public frc2::CommandHelper<frc2::CommandBase, RetractIntakeCommand> {
    public:
        RetractIntakeCommand(Intake* intake);

        void Initialize();
        void Execute();
        void End(bool interrupted);
        bool IsFinished();

    private:
        Intake* m_Intake;
};
