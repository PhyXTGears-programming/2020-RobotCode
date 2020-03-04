#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Shooter.h"
#include "subsystems/Intake.h"

class AimShootCommand : public frc2::CommandHelper<frc2::CommandBase, AimShootCommand> {
    public:
        explicit AimShootCommand(Shooter* shooter, Intake* intake);
        void Initialize();
        void Execute();
        void End(bool interrupted);

    private:
        Shooter* m_Shooter;
        Intake* m_Intake;

        units::angular_velocity::revolutions_per_minute_t m_ShootSpeed = 0_rpm;

        bool feederActivated = false;
};
