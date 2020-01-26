#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include "Constants.h"

class Intake : public frc2::SubsystemBase {
    public:
        Intake();
        void Periodic() override;

        void SetSpeed(double intakeSpeed);

        void IntakeStart();
        void IntakeStop();
        void IntakeReverse();
    
    private:
        ctre::phoenix::motorcontrol::can::TalonSRX m_IntakeMotor {kIntakeMotor};
};
