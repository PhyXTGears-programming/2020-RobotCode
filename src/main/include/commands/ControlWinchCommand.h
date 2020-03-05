#pragma once

#include <functional>

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Climb.h"

class ControlWinchCommand : public frc2::CommandHelper<frc2::CommandBase, ControlWinchCommand> {
    public:
        ControlWinchCommand(Climb* Climb, std::function<double(void)> rsLambda);

        void Initialize();
        void Execute();
        void End(bool interrupted);
        bool IsFinished();

    private:
        Climb* m_Climb;

        std::function<double(void)> m_RightStickCheck;
};
