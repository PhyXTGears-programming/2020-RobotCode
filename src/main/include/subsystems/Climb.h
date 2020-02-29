#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/Solenoid.h>
#include <frc/Relay.h>

#include "Constants.h"

#define WINCH_SPEED 0.25

class Climb : public frc2::SubsystemBase {
    public:
        Climb();
        void Periodic() override;
        
        void PistonExtend();
        void PistonRetract();

        void SetWinchSpeed(double speed);

        bool IsClimbing() { return m_IsClimbing; }

        // TODO movement on bar 
        void SetRelay(frc::Relay::Value val);
    private:
        bool m_IsClimbing = false;

        ctre::phoenix::motorcontrol::can::TalonSRX m_ClimbWinchMotor {kClimbWinchMotor};

        frc::Relay m_ClimbBarMotor {kClimbBarMotor};

        frc::Solenoid m_ClimbExtendSolenoid {kClimbExtendSolenoid};
        frc::Solenoid m_ClimbRetractSolenoid {kClimbRetractSolenoid};
};

