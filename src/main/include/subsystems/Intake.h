#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/Solenoid.h>

#include "Constants.h"

class Intake : public frc2::SubsystemBase {
    public:
        Intake();
        void Periodic() override;

        void SetSpeed(double intakeSpeed);
        void SetConveyorSpeed(double conveyorSpeed);

        void IntakeStart();
        void IntakeStop();
        void IntakeReverse();
    
        int GetNumBalls () {
            return m_NumBalls + m_BallDetected;
        }

        void ExtendIntake () {
            m_IntakeExtendSolenoid.Set(true);
            m_IntakeRetractSolenoid.Set(false);
        }

        void RetractIntake () {
            m_IntakeExtendSolenoid.Set(false);
            m_IntakeRetractSolenoid.Set(true);
        }

    private:
        int m_NumBalls = 0;
        bool m_BallDetected = false;

        ctre::phoenix::motorcontrol::can::TalonSRX m_IntakeMotor {kIntakeMotor};

        ctre::phoenix::motorcontrol::can::TalonSRX m_ConveyorCornerMotor {kConveyorCornerMotor};
        ctre::phoenix::motorcontrol::can::TalonSRX m_ConveyorAdvanceMotor {kConveyorAdvanceMotor};

        frc::Solenoid m_IntakeExtendSolenoid {kIntakeExtendSolenoidPin};
        frc::Solenoid m_IntakeRetractSolenoid {kIntakeRetractSolenoidPin};
};

