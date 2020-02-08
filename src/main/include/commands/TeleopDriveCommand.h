#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/Drivetrain.h"

class TeleopDriveCommand : public frc2::CommandHelper<frc2::CommandBase, TeleopDriveCommand> {
    public:
        explicit TeleopDriveCommand(Drivetrain* drivetrain, frc::XboxController* driverController);
        void Initialize();
        void Execute();

    private:
        Drivetrain* m_Drivetrain;
        frc::XboxController* m_Controller;
};
