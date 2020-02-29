#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/Solenoid.h>

#include "Constants.h"

class Intake : public frc2::SubsystemBase {
    public:
        Intake();
        void Periodic() override;

        void SetIntakeSpeed(double intakeSpeed);
        void SetConveyorSpeed(double conveyorSpeed);

        void IntakeStart();
        void IntakeStop();
        void IntakeReverse();

        void ConveyorStart();
        void ConveyorStop();
        void ConveyorReverse();
    
        int GetNumBalls () {
            return m_NumBalls + m_BallDetected;
        }

        void IntakeExtend () {
            m_IntakeExtendSolenoid.Set(true);
            m_IntakeRetractSolenoid.Set(false);
        }

        void IntakeRetract () {
            m_IntakeExtendSolenoid.Set(false);
            m_IntakeRetractSolenoid.Set(true);
        }

        inline void FeederStart () {
            SetFeeder(true);
        }

        inline void FeederStop () {
            SetFeeder(false);
        }

    private:
        void SetFeeder (bool on) {
            m_FeederMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, on ? -1.0 : 0);
        }

        int m_NumBalls = 0;
        bool m_BallDetected = false;

        ctre::phoenix::motorcontrol::can::TalonSRX m_IntakeMotor {kIntakeMotor};
        ctre::phoenix::motorcontrol::can::TalonSRX m_ConveyorMotor {kConveyorMotor};
        ctre::phoenix::motorcontrol::can::TalonSRX m_FeederMotor {kTurretFeederMotor};

        frc::Solenoid m_IntakeExtendSolenoid {kIntakeExtendSolenoidPin};
        frc::Solenoid m_IntakeRetractSolenoid {kIntakeRetractSolenoidPin};
};

