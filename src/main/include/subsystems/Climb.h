#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/Relay.h>
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

        void WinchLock();
        void WinchUnlock();

        void RollLeft();
        void RollRight();
        void RollStop();

        // Contract: m_IsClimbing = m_IsClimbing || m_IsWinchOut
        bool IsClimbing() { return m_IsClimbing; }
        bool IsPistonExtended() { return m_IsPistonExtended; }
        bool IsWinchCableOut() { return m_IsWinchCableOut; }
        bool IsWinchLocked() { return m_IsWinchLocked; }

        // TODO movement on bar

    private:
        bool m_IsClimbing = false;
        bool m_IsPistonExtended = false;
        bool m_IsWinchCableOut = false;
        bool m_IsWinchLocked = false;

        ctre::phoenix::motorcontrol::can::TalonSRX m_ClimbWinchMotor {kClimbWinchMotor};

        frc::Solenoid m_ClimbExtendSolenoid {kClimbExtendSolenoid};
        frc::Solenoid m_ClimbRetractSolenoid {kClimbRetractSolenoid};

        frc::Solenoid m_Brake {kBrakeSolenoid};

        frc::Relay m_ClimbRoller {kClimbRollerRelay};

        void SetWinchSpeed(double speed);

        void SetPistonFlag();
        void ResetPistonFlag();

        void SetWinchCableOutFlag();
        // No reset winch flag.  Cannot detect when winch is fully reset.
        // Restart robot code if need cleared.

        void SetWinchLockFlag();
        void ResetWinchLockFlag();
};

