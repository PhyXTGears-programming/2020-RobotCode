#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Intake.h"
#include "subsystems/PowerCellCounter.h"
#include "subsystems/Shooter.h"

class AimShootCommand : public frc2::CommandHelper<frc2::CommandBase, AimShootCommand> {
    public:
        explicit AimShootCommand(Shooter* shooter, Intake* intake, PowerCellCounter* counter);
        void Initialize();
        void Execute();
        void End(bool interrupted);
        bool IsFinished();

    private:
        Shooter* m_Shooter;
        Intake* m_Intake;
        PowerCellCounter* m_PowerCellCounter;

        struct {
            units::angular_velocity::revolutions_per_minute_t shootSpeed = 4500_rpm;
        } config;

        bool feederActivated = false;
};
