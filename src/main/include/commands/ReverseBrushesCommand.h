#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Intake.h"
#include "subsystems/Drivetrain.h"

class ReverseBrushesCommand : public frc2::CommandHelper<frc2::CommandBase, ReverseBrushesCommand> {
    public:
        ReverseBrushesCommand(Intake* intake);

        void Initialize();
        void Execute();
        void End(bool interrupted);
        bool IsFinished();

    private:
        Intake* m_Intake;
};
