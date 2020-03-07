#pragma once

#include <limits>

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivetrain.h"

class DriveUntilWallCommand : public frc2::CommandHelper<frc2::CommandBase, DriveUntilWallCommand> {
    public:
        DriveUntilWallCommand (Drivetrain* drivetrain) {
            m_Drivetrain = drivetrain;
        }

        void Initialize () {
            m_Drivetrain->RadiusDrive(-0.3, std::numeric_limits<double>::infinity());
        }

        bool IsFinished () {
            return m_Drivetrain->GetMotorCurrent() > 7_A;
        }

        void End (bool interrupted) {
            m_Drivetrain->RadiusDrive(0, std::numeric_limits<double>::infinity());
        }

    private:
        Drivetrain* m_Drivetrain;
};
