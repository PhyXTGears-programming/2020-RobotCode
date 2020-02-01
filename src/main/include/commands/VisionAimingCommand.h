#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>
#include <networktables/NetworkTableInstance.h>

#include "subsystems/Drivetrain.h"

#define kP 6.0

class VisionAimingCommand : public frc2::CommandHelper<frc2::CommandBase, VisionAimingCommand> {
    public:
        VisionAimingCommand(Drivetrain* drivetrain);

        void Initialize();
        void Execute();

    private:
        Drivetrain* m_Drivetrain;
        std::shared_ptr<nt::NetworkTable> m_VisionTable;
        frc2::PIDController m_TurnPID {0.0001 * kP, 0, 0};
};
