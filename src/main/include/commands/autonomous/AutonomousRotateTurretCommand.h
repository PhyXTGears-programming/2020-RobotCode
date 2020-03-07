#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Shooter.h"

class AutonomousRotateTurret : public frc2::CommandHelper<frc2::CommandBase, AutonomousRotateTurret> {
    public:
        explicit AutonomousRotateTurret (Shooter* shooter) { AddRequirements(shooter); m_Shooter = shooter; }

        void Initialize () {
            m_Shooter->SetLimelightLight(true); // Turn on Limelight so that a target can be detected (to end the command)
            m_Shooter->SetTurretSpeed(16_rpm); // Rotate turret at 16 RPM
        }
        void End (bool interrupted) { m_Shooter->SetTurretSpeed(0_rpm); } // Stop turret on completion
        bool IsFinished () { return m_Shooter->GetTargetCount() > 0; } // End when target is detected

    private:
        Shooter* m_Shooter;
};
