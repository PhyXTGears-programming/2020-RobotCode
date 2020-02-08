#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>
#include <networktables/NetworkTableInstance.h>

#include "subsystems/Shooter.h"

#define kP 0.01

class VisionAimingCommand : public frc2::CommandHelper<frc2::CommandBase, VisionAimingCommand> {
    public:
        VisionAimingCommand(Shooter* shooter);

        void Initialize();
        void Execute();

    private:
        Shooter* m_Shooter;
        std::shared_ptr<nt::NetworkTable> m_VisionTable;
        frc2::PIDController m_TurretPID {kP, 0, 0};
};
