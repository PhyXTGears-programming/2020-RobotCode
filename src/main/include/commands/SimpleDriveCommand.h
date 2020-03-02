#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivetrain.h"

class SimpleDriveCommand : public frc2::CommandHelper<frc2::CommandBase, SimpleDriveCommand> {
    public:
        explicit SimpleDriveCommand(double speed, double turn, Drivetrain * drivetrain);
        void Initialize();
        void Execute();
        void End(bool interrupted);

    private:
        Drivetrain* m_Drivetrain;

        double m_Speed = 0.0;
        double m_Turn = 0.0;
};