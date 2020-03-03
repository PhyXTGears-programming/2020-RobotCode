#pragma once

#include <cpptoml.h>

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>

#include "Constants.h"

class Intake : public frc2::SubsystemBase {
    public:
        Intake(std::shared_ptr<cpptoml::table> toml);
        void Periodic() override;

        void SetIntakeSpeed(double intakeSpeed);
        void SetConveyorSpeed(double conveyorSpeed);

        void IntakeStart();
        void IntakeStop();
        void IntakeReverse();

        void ConveyorStart();
        void ConveyorStop();
        void ConveyorReverse();

        void IntakeExtend () {
            m_IntakeExtendSolenoid.Set(true);
            m_IntakeRetractSolenoid.Set(false);
        }

        void IntakeRetract () {
            m_IntakeExtendSolenoid.Set(false);
            m_IntakeRetractSolenoid.Set(true);
        }

        void FeedShooterStart ();
        void FeedLoadStart ();
        void FeedStop ();

        bool IsPowerCellInFeeder();

    private:
        void SetFeederSpeed (double percentSpeed);

        ctre::phoenix::motorcontrol::can::TalonSRX m_IntakeMotor {kIntakeMotor};
        ctre::phoenix::motorcontrol::can::TalonSRX m_ConveyorMotor {kConveyorMotor};
        ctre::phoenix::motorcontrol::can::TalonSRX m_FeederMotor {kTurretFeederMotor};

        frc::Solenoid m_IntakeExtendSolenoid {kIntakeExtendSolenoidPin};
        frc::Solenoid m_IntakeRetractSolenoid {kIntakeRetractSolenoidPin};

        frc::DigitalInput m_FeederPowerCellDetector {kBeamPowerCellFeeder};
        
        struct {
            struct {
                double load, shoot;
            } speed;
        } config;
};

