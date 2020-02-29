#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/Solenoid.h>

#include "Constants.h"

class Climb : public frc2::SubsystemBase {
    public:
        Climb();
        void Periodic() override;

        void PistonExtend();
        void PistonRetract();

        void WinchCableOut(double percentSpeed);
        void WinchCableIn(double percentSpeed);
        void WinchStop();

        bool IsClimbing() { return m_IsClimbing; }

        // TODO movement on bar

    private:
        bool m_IsClimbing = false;

        ctre::phoenix::motorcontrol::can::TalonSRX m_ClimbWinchMotor {kClimbWinchMotor};

        frc::Solenoid m_ClimbExtendSolenoid {kClimbExtendSolenoid};
        frc::Solenoid m_ClimbRetractSolenoid {kClimbRetractSolenoid};

        void SetWinchSpeed(double speed);
};

